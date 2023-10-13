#!/bin/bash

# echo usage

function show_usage() {
    echo "Warning: This script may break your repositories, please be careful to use $0"
    echo "Usage: $0 [-h or --help"
    echo "          [-d or --dry-run]"
    echo "          [-p or --push]"
    echo "          [-t or --tag tag-id]"
    echo ""
    echo "Options"
    echo "-h or --help:    show usage"
    echo "-d or --dry-run: run without any change. only message is displayed."
    echo "-p or --push:    push branch or tag to origin"
    echo "-t --tag:        tag id, which is to added repository. this is mandatory"

    exit 0
}

# get script directory.
SCRIPT_DIR=$(
    cd "$(dirname "$0")" || exit
    pwd
)
ROOT_DIR=$(
    cd "$(dirname "$0")" || exit
    cd ../ || exit
    pwd
)

# target repositories
# CARET_* repositories
CARET_DIRS_PATH="${ROOT_DIR}/src/CARET"

CARET_REPOS_ARRAY=("caret_trace"
    "caret_analyze"
    "caret_analyze_cpp_impl"
    "ros2caret"
    "caret_common")

# ros-tracing repository
ROS_TRACING_REPOS="ros2_tracing"

# rclcpp repository
ROS_RCLCPP_REPOS="rclcpp"
ROS_RCL_REPOS="rcl"

# cyclonedds
CYCLONEDDS_REPOS="cyclonedds"

# variables
DRY_RUN=
TAG_ID=
PUSH_REMOTE=

# cspell:disable-next-line
OPTIONS=$(getopt -o hdpt: -l help,dry-run,push,tag: -- "$@")

eval set -- "$OPTIONS"

while true; do
    case $1 in
    -h | --help)
        show_usage
        ;;
    -d | --dry-run)
        DRY_RUN='echo'
        shift
        ;;
    -p | --push)
        PUSH_REMOTE=true
        shift
        ;;
    -t | --tag)
        TAG_ID="$2"
        shift 2
        ;;
    --)
        shift
        break
        ;;
    esac
done

# check options and output messages.
if [ -z "${TAG_ID}" ]; then
    echo "tag should be given with option --tag or -t."
    exit 1
fi

if [ "${DRY_RUN}" == "echo" ]; then
    ${DRY_RUN} "This program will run by dry-run mode."
    ${DRY_RUN} "TAG ID: ${TAG_ID}"
fi

# add tags to caret repositories
function add_tag_to_caret_repository() {
    DIR_PATH=${CARET_DIRS_PATH}/${1}
    echo "enter  ${DIR_PATH}..."
    cd "${DIR_PATH}" || exit
    ${DRY_RUN} git checkout main
    ${DRY_RUN} git checkout -b rc/"${2}"
    ${DRY_RUN} git tag "${2}"
    ${DRY_RUN} git remote add github git@github.com:tier4/"${1}".git
    if [ "${PUSH_REMOTE}" == "true" ]; then
        ${DRY_RUN} git push github rc/"${2}"
        ${DRY_RUN} git push github "${2}"
    fi
    cd "${ROOT_DIR}" || exit
    echo "leave ${1} ..."
}

for repos in "${CARET_REPOS_ARRAY[@]}"; do
    add_tag_to_caret_repository "${repos}" "${TAG_ID}"
done

# get tags from repository

function get_hash_from_repository() {
    cd "${1}" || exit
    HASH_ID=$(git rev-parse HEAD)
    cd "${ROOT_DIR}" || exit
    echo "${HASH_ID}"
}

# get hash number from ros-tracing repos
ROS_TRACING_PATH="src/ros-tracing/${ROS_TRACING_REPOS}"
ROS_TRACING_HASH=$(get_hash_from_repository "${ROOT_DIR}"/${ROS_TRACING_PATH})

# get hash number from rclcpp
ROS_RCLCPP_PATH="src/ros2/${ROS_RCLCPP_REPOS}"
ROS_RCLCPP_HASH=$(get_hash_from_repository "${ROOT_DIR}"/${ROS_RCLCPP_PATH})

# get hash number from rcl
ROS_RCL_PATH="src/ros2/${ROS_RCL_REPOS}"
ROS_RCL_HASH=$(get_hash_from_repository "${ROOT_DIR}"/${ROS_RCL_PATH})

# get hash number from cyclonedds
CYCLONEDDS_PATH="src/eclipse-cyclonedds/${CYCLONEDDS_REPOS}"
CYCLONEDDS_HASH=$(get_hash_from_repository "${ROOT_DIR}"/${CYCLONEDDS_PATH})

# checkout caret repository.
${DRY_RUN} git checkout -b rc/"${TAG_ID}"

# copy caret repos and edit as pointing specific tag and hash.
${DRY_RUN} cp "${SCRIPT_DIR}"/template_caret.repos "${ROOT_DIR}"/caret.repos
${DRY_RUN} sed -i -e "s/ROS_TRACING_HASH/${ROS_TRACING_HASH}/g" "${ROOT_DIR}"/caret.repos
${DRY_RUN} sed -i -e "s/RCLCPP_HASH/${ROS_RCLCPP_HASH}/g" "${ROOT_DIR}"/caret.repos
${DRY_RUN} sed -i -e "s/RCL_HASH/${ROS_RCL_HASH}/g" "${ROOT_DIR}"/caret.repos
${DRY_RUN} sed -i -e "s/CYCLONEDDS_HASH/${CYCLONEDDS_HASH}/g" "${ROOT_DIR}"/caret.repos
${DRY_RUN} sed -i -e "s/CARET_TAG/${TAG_ID}/g" "${ROOT_DIR}"/caret.repos

${DRY_RUN} git add "${ROOT_DIR}"/caret.repos
${DRY_RUN} git commit -m "release(caret.repos): change version of sub repositories for ${TAG_ID}"

${DRY_RUN} git tag "${TAG_ID}"

if [ "${PUSH_REMOTE}" == "true" ]; then
    ${DRY_RUN} cd "${ROOT_DIR}" || exit
    ${DRY_RUN} git push origin rc/"${TAG_ID}"
    ${DRY_RUN} git push origin "${TAG_ID}"
    ${DRY_RUN} cd "${SCRIPT_DIR}"
fi

echo "[Info] Completed release script."
