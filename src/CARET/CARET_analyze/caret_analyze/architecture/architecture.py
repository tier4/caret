# Copyright 2021 Research Institute of Systems Planning, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

from typing import Callable, Dict, List, Optional, Tuple

from .architecture_dict import ArchitectureDict
from ..common import Summarizable, Summary, Util
from ..exceptions import InvalidArgumentError, ItemNotFoundError
from ..value_objects import (
    CallbackGroupStructValue,
    CallbackStructValue,
    CommunicationStructValue,
    ExecutorStructValue,
    NodeStructValue,
    PathStructValue,
)


class Architecture(Summarizable):
    def __init__(
        self,
        file_type: str,
        file_path: str,
        sub_yaml_file_path: Optional[str] = None,
    ) -> None:
        from .architecture_reader_factory import ArchitectureReaderFactory
        from .architecture_loaded import ArchitectureLoaded

        # /parameter events and /rosout measurements are not yet supported.
        # ignore_topics: List[str] = IGNORE_TOPICS
        ignore_topics: List[str] = []

        if sub_yaml_file_path is not None:
            reader = ArchitectureReaderFactory.create_instance(
                file_type, file_path, 'yaml', sub_yaml_file_path)
        else:
            reader = ArchitectureReaderFactory.create_instance(file_type, file_path)

        loaded = ArchitectureLoaded(reader, ignore_topics)

        self._nodes: Tuple[NodeStructValue, ...] = loaded.nodes
        self._communications: Tuple[CommunicationStructValue, ...] = loaded.communications
        self._executors: Tuple[ExecutorStructValue, ...] = loaded.executors
        self._path_manager = NamedPathManager(loaded.paths)

    def get_node(self, node_name: str) -> NodeStructValue:
        try:
            return Util.find_one(lambda x: x.node_name == node_name, self.nodes)
        except ItemNotFoundError:
            msg = 'Failed to find node. '
            msg += f'node_name: {node_name}'
            raise ItemNotFoundError(msg)

    def get_executor(self, executor_name: str) -> ExecutorStructValue:
        return Util.find_one(lambda x: x.executor_name == executor_name, self.executors)

    def get_callback_group(self, callback_group_name: str) -> CallbackGroupStructValue:
        return Util.find_one(
            lambda x: x.callback_group_name == callback_group_name, self.callback_groups)

    @property
    def callback_groups(self) -> Tuple[CallbackGroupStructValue, ...]:
        return tuple(Util.flatten([_.callback_groups for _ in self.executors]))

    @property
    def callback_group_names(self) -> Tuple[CallbackGroupStructValue, ...]:
        return tuple(sorted(_.callback_group_name for _ in self.callback_groups))

    @property
    def topic_names(self) -> Tuple[str, ...]:
        return tuple(sorted({_.topic_name for _ in self.communications}))

    def get_callback(self, callback_name: str) -> CallbackStructValue:
        return Util.find_one(lambda x: x.callback_name == callback_name, self.callbacks)

    @property
    def callbacks(self) -> Tuple[CallbackStructValue, ...]:
        return tuple(Util.flatten([_.callbacks for _ in self.callback_groups]))

    def get_communication(
        self,
        publisher_node_name: str,
        subscription_node_name: str,
        topic_name: str
    ) -> CommunicationStructValue:
        def is_target_comm(comm: CommunicationStructValue):
            return comm.publish_node_name == publisher_node_name and \
                comm.subscribe_node_name == subscription_node_name and \
                comm.topic_name == topic_name

        return Util.find_one(is_target_comm, self.communications)

    def get_path(self, path_name: str) -> PathStructValue:
        return self._path_manager.get_named_path(path_name)

    def add_path(self, path_name: str, path_info: PathStructValue) -> None:
        self._path_manager.add_named_path(path_name, path_info)

    def remove_path(self, path_name: str) -> None:
        self._path_manager.remove_named_path(path_name)

    def update_path(self, path_name: str, path: PathStructValue) -> None:
        self._path_manager.update_named_path(path_name, path)

    @property
    def nodes(self) -> Tuple[NodeStructValue, ...]:
        return self._nodes

    @property
    def node_names(self) -> Tuple[str, ...]:
        return tuple(sorted(_.node_name for _ in self._nodes))

    @property
    def executors(self) -> Tuple[ExecutorStructValue, ...]:
        return self._executors

    @property
    def executor_names(self) -> Tuple[ExecutorStructValue, ...]:
        return tuple(sorted(_.executor_name for _ in self._executors))

    @property
    def paths(self) -> Tuple[PathStructValue, ...]:
        return self._path_manager.named_paths

    @property
    def path_names(self) -> Tuple[str, ...]:
        return tuple(sorted(_.path_name for _ in self._path_manager.named_paths))

    @property
    def communications(self) -> Tuple[CommunicationStructValue, ...]:
        return self._communications

    @property
    def summary(self) -> Summary:
        return Summary({
            'nodes': self.node_names
        })

    def export(self, file_path: str, force: bool = False):
        arch_dict = ArchitectureDict(self.nodes, self.executors, self.paths)

        mode = 'w' if force else 'x'
        with open(file_path, mode=mode) as f:
            f.write(str(arch_dict))

    def search_paths(
        self,
        *node_names: str,
        max_node_depth: int = 10,
        node_filter: Optional[Callable[[str], bool]] = None,
        communication_filter: Optional[Callable[[str], bool]] = None,
    ) -> List[PathStructValue]:
        from .graph_search import NodePathSearcher
        assert len(node_names) >= 2

        for node_name in node_names:
            if node_name not in self.node_names:
                raise InvalidArgumentError(f'Failed to find node. {node_name}')

        path_searcher = NodePathSearcher(
            self._nodes, self._communications, node_filter, communication_filter)
        return path_searcher.search(node_names, max_node_depth)
    
    def rename(self, file_path: str, renaming_name: str, renamed_name: str):
        for node in self._nodes:
            if node._node_name == renaming_name:
                print(node.node_name)
                node._node_name = renamed_name

        self.export(file_path, True)


class NamedPathManager():

    def __init__(self, paths: Tuple[PathStructValue, ...]) -> None:
        self._named_paths: Dict[str, PathStructValue] = {}
        for path in paths:
            if path.path_name is None:
                continue
            self._named_paths[path.path_name] = path

    @property
    def named_paths(self) -> Tuple[PathStructValue, ...]:
        return tuple(self._named_paths.values())

    def get_named_path(self, path_name: str) -> PathStructValue:
        if path_name not in self._named_paths.keys():
            raise InvalidArgumentError(f'Failed to get named path. {path_name} not exist.')
        return self._named_paths[path_name]

    def add_named_path(self, path_name: str, path_info: PathStructValue):
        if path_name in self._named_paths.keys():
            raise InvalidArgumentError('Failed to add named path. Duplicate path name.')
        named_path_info = PathStructValue(path_name, path_info.child)
        self._named_paths[path_name] = named_path_info

    def remove_named_path(self, path_name: str):
        if path_name not in self._named_paths.keys():
            raise InvalidArgumentError(f'Failed to remove named path. {path_name} not exist.')
        del self._named_paths[path_name]

    def update_named_path(self, path_name: str, path_info: PathStructValue):
        if path_info.path_name is None:
            raise InvalidArgumentError('path_info.path_name is None')

        self.remove_named_path(path_info.path_name)
        self.add_named_path(path_name, path_info)
