ARG ROS_DISTRO=humble
ARG CARET_VERSION="main"

FROM ros:${ROS_DISTRO}

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update -y && \
    apt-get install -y --no-install-recommends \
        locales \
        && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV TZ=Asia/Tokyo

# Do not use cache
ADD "https://www.random.org/sequences/?min=1&max=52&col=1&format=plain&rnd=new" /dev/null

RUN git clone https://github.com/tier4/caret.git /ros2_caret_ws && \
    cd /ros2_caret_ws && \
    git checkout ${CARET_VERSION}

# cspell: disable
RUN if [ "$ROS_DISTRO" = "jazzy" ]; then \
      apt-get update && \
      apt-get install -y python3-pip python3-virtualenv && \
      virtualenv -p python3 --system-site-packages $HOME/venv/jazzy ; \
    fi
# cspell: enable

# cspell: disable
RUN apt update && apt install -y git && \
    apt-get install -y tzdata && \
    ln -fs /usr/share/zoneinfo/Asia/Tokyo /etc/localtime && \
    dpkg-reconfigure --frontend noninteractive tzdata
# cspell: enable

RUN echo "===== Setup CARET ====="
RUN cd ros2_caret_ws && \
    mkdir src && \
    if [ "$ROS_DISTRO" = "humble" ]; then \
        REPOS_FILE=caret.repos ; \
    elif [ "$ROS_DISTRO" = "iron" ]; then \
        REPOS_FILE=caret_iron.repos ; \
    elif [ "$ROS_DISTRO" = "jazzy" ]; then \
        REPOS_FILE=caret_jazzy.repos ; \
        export PIP_BREAK_SYSTEM_PACKAGES=1 ; \
    else \
        echo "Unsupported ROS_DISTRO: $ROS_DISTRO" && exit 1 ; \
    fi && \
    vcs import src < $REPOS_FILE && \
    . /opt/ros/"$ROS_DISTRO"/setup.sh && \
    if [ "$ROS_DISTRO" = "jazzy" ]; then \
        . $HOME/venv/jazzy/bin/activate ; \
    fi && \
    ./setup_caret.sh -c -d "$ROS_DISTRO"

RUN echo "===== Build CARET ====="
RUN cd ros2_caret_ws && \
    . /opt/ros/"$ROS_DISTRO"/setup.sh && \
    if [ "$ROS_DISTRO" = "jazzy" ]; then \
      . $HOME/venv/jazzy/bin/activate ; \
    fi && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
