FROM ubuntu:22.04

ARG CARET_VERSION="main"
ARG AUTOWARE_VERSION="main"

RUN apt-get update -y && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        locales \
        && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV TZ=Asia/Tokyo
ENV ROS_DISTRO=humble

# Do not use cache
ADD "https://www.random.org/sequences/?min=1&max=52&col=1&format=plain&rnd=new" /dev/null

RUN echo "===== GET CARET ====="
# RUN git clone https://github.com/tier4/caret.git ros2_caret_ws && \
#     cd ros2_caret_ws && \
#     git checkout "$CARET_VERSION"
COPY ./ /ros2_caret_ws

# cspell: disable
RUN apt update && apt install -y git && \
    apt-get install -y tzdata && \
    ln -fs /usr/share/zoneinfo/Asia/Tokyo /etc/localtime && \
    dpkg-reconfigure --frontend noninteractive tzdata
# cspell: enable

RUN echo "===== Setup Autoware ====="
RUN git clone https://github.com/autowarefoundation/autoware.git && \
    cd autoware && \
    git checkout "$AUTOWARE_VERSION" && \
    DEBIAN_FRONTEND=noninteractive apt install python3.10-venv -y && \
    ./setup-dev-env.sh -y --no-nvidia --no-cuda-drivers && \
    mkdir src && \
    vcs import src < autoware.repos && \
    vcs export --exact src && \
    . /opt/ros/"$ROS_DISTRO"/setup.sh && \
    rosdep update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y ros-humble-pacmod3-msgs=1.0.0-0jammy && \
    DEBIAN_FRONTEND=noninteractive apt-get remove -y python3-gpg && \
    DEBIAN_FRONTEND=noninteractive rosdep install -y --from-paths src --ignore-src --rosdistro "$ROS_DISTRO"
# workarounds:
# install ros-humble-pacmod3-msgs manually because rosdep tries to install ros-galactic-pacmod3-msgs
# remove gpg because build error happens in ad_api_visualizers for some reasons...

# workaround: remove agnocast because CARET doesn't support Agnocast yet and Agnocast is not used by default
RUN rm -rf autoware/src/middleware/external/agnocast
RUN rm -rf autoware/src/universe/autoware_universe/common/autoware_agnocast_wrapper

# https://github.com/ament/ament_cmake/commit/799183ab9bcfd9b66df0de9b644abaf8c9b78e84
RUN echo "===== Modify ament_cmake_auto as workaround ====="
RUN cd /opt/ros/humble/share/ament_cmake_auto/cmake && \
    backup_date="`date +"%Y%m%d_%H%M%S"`" && \
    cp ament_auto_add_executable.cmake ament_auto_add_executable.cmake_"$backup_date" && \
    cp ament_auto_add_library.cmake ament_auto_add_library.cmake_"$backup_date" && \
    sed -i -e 's/SYSTEM//g' ament_auto_add_executable.cmake && \
    sed -i -e 's/SYSTEM//g' ament_auto_add_library.cmake

RUN echo "===== Modify pcl_ros (libtracetools.so) as workaround ====="
RUN cd /opt/ros/humble/share/pcl_ros/cmake && \
    backup_date="`date +"%Y%m%d_%H%M%S"`" && \
    cp export_pcl_rosExport.cmake export_pcl_rosExport.cmake_"$backup_date" && \
    sed -i -e 's/\/opt\/ros\/humble\/lib\/libtracetools.so;//g' export_pcl_rosExport.cmake

RUN echo "===== Modify pcl_ros (rclcpp) as workaround ====="
RUN cd /opt/ros/humble/share/pcl_ros/cmake && \
    backup_date="`date +"%Y%m%d_%H%M%S"`" && \
    cp export_pcl_rosExport.cmake export_pcl_rosExport.cmake_"$backup_date"_2 && \
    sed -i -e 's/\/opt\/ros\/humble\/include\/rclcpp;//g' export_pcl_rosExport.cmake

RUN echo "===== Setup CARET ====="
RUN cd ros2_caret_ws && \
    rm -rf src build log install && \
    mkdir src && \
    vcs import src < caret.repos && \
    . /opt/ros/"$ROS_DISTRO"/setup.sh && \
    ./setup_caret.sh -c

RUN echo "===== Build CARET ====="
RUN cd ros2_caret_ws && \
    . /opt/ros/"$ROS_DISTRO"/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

RUN echo "===== Build Autoware ====="
RUN cd autoware && \
    . /opt/ros/"$ROS_DISTRO"/setup.sh && \
    . /ros2_caret_ws/install/local_setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=Off -DCMAKE_CXX_FLAGS="-w"

RUN echo "===== Verify Build ====="
RUN cd autoware && \
    . /opt/ros/"$ROS_DISTRO"/setup.sh && \
    . /ros2_caret_ws/install/local_setup.sh && \
    ros2 caret check_caret_rclcpp ./
