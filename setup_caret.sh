#!/bin/bash

set -Ceu

SCRIPT_DIR=$(readlink -f "$(dirname "$0")")
PIP_BREAK_ENV="PIP_BREAK_SYSTEM_PACKAGES"

function show_usage() {
    echo "${0} executes setup for CARET."
    echo "Options "
    echo "    -h or --help: show help message"
    echo "    -c or --no-interactive"
    echo "    -n or --no-package-install"
    echo "    -p or --no-pip-install  (skip pip installs in ansible playbooks; ansible itself is still installed via pip as a prerequisite)"
    echo "    -d or --ros-distro"
    echo ""
    echo "Required for ROS 2 Jazzy (Ubuntu 24.04+):"
    echo "    export ${PIP_BREAK_ENV}=1"
    echo "    (Allows pip to install packages into the system Python environment / PEP 668)"
    exit 0
}

ALLOWED_ROS_DISTRO=("humble" "jazzy")

function validate_ros_distro() {
    if [[ " ${ALLOWED_ROS_DISTRO[*]} " != *" $1 "* ]]; then
        echo "error: $1 is not supported ROS Distribution." >&2
        echo "Supported ROS Distributions are ${ALLOWED_ROS_DISTRO[*]}" >&2
        exit 1
    fi
}

# parse command options.
OPT=$(getopt -o nphcd: -l no-interactive,no-package-install,no-pip-install,help,ros-distro: -- "$@") # cSpell:ignore nphcd

eval set -- "$OPT"

# Parse args
noninteractive=0
package_install=1
pip_install=1
ros_distro=humble

while true; do
    case $1 in
    -h | --help)
        show_usage
        ;;
    -c | --no-interactive)
        noninteractive=1
        shift
        ;;
    -n | --no-package-install)
        package_install=0
        shift
        ;;
    -p | --no-pip-install)
        pip_install=0
        shift
        ;;
    -d | --ros-distro)
        shift
        ros_distro=$1
        shift
        ;;
    --)
        shift
        break
        ;;
    esac
done

# Add local path
export PATH="$HOME/.local/bin:$PATH"

# Check ROS Distribution
validate_ros_distro "$ros_distro"

# Jazzy Specific Check for PEP 668 ---
if [ "$ros_distro" = "jazzy" ]; then
    env_val="${!PIP_BREAK_ENV:-0}"
    if [ "$env_val" != "1" ]; then
        echo -e "\e[31m[ERROR] Ubuntu 24.04 (Jazzy) detected.\e[0m"
        echo "Starting from this version, pip installation into system packages is restricted by default."
        echo "To proceed with the installation, please run the following command first to acknowledge the risk:"
        echo ""
        echo -e "    \e[36m"export ${PIP_BREAK_ENV}=1"\e[0m"
        echo ""
        echo "Then, run this setup script again."
        exit 1
    fi
fi

# Confirm whether to start installation
if [ $noninteractive -eq 0 ]; then
    # Prevent root execution in interactive mode
    if [ "$(id -u)" -eq 0 ]; then
        echo "Do not run setup as root!" >&2
        exit 1
    fi

    echo "Setting up the build environment take up to 20 minutes."
    read -rp ">  Are you sure you want to run setup? [y/N] " answer

    # Check whether to cancel
    if ! [[ ${answer:0:1} =~ y|Y ]]; then
        echo -e "\e[33mCancelled.\e[0m"
        exit 1
    fi
else
    echo -e "\e[36mRun the setup in non-interactive mode.\e[m"
fi

# Install sudo
if ! (command -v sudo >/dev/null 2>&1); then
    apt-get -y update
    apt-get -y install sudo
fi

# Install pip for ansible
if ! (command -v pip3 >/dev/null 2>&1); then
    sudo apt-get -y update
    sudo apt-get -y install python3-pip
fi

# Run ansible if confirmed
# Install ansible
if [ "$ros_distro" = "jazzy" ]; then
    pip3 install -U ansible
else
    ansible_version=$(pip3 list | grep -oP "^ansible\s+\K([0-9]+)" || true)
    if [ "$ansible_version" != "6" ]; then
        sudo apt-get -y update
        sudo apt-get -y purge ansible
        sudo pip3 install -U "ansible==6.*"
    fi
fi

# Set options
declare -a options=()

if [ $noninteractive -eq 0 ]; then
    options=("--ask-become-pass")
fi

if [ $package_install -eq 0 ]; then
    options+=("--extra-vars" "package_install=n")
elif [ $noninteractive -eq 1 ]; then
    options+=("--extra-vars" "package_install=y")
fi

if [ $pip_install -eq 0 ]; then
    options+=("--extra-vars" "pip_install=n")
elif [ $noninteractive -eq 1 ]; then
    options+=("--extra-vars" "pip_install=y")
fi

# Select playbook
PLAYBOOK="playbook.yml"
if [ "$ros_distro" = "jazzy" ]; then
    PLAYBOOK="playbook_jazzy.yml"
fi

# Run ansible
if ansible-playbook "$SCRIPT_DIR/ansible/$PLAYBOOK" -e WORKSPACE_ROOT="$SCRIPT_DIR" "${options[@]}"; then
    echo -e "\e[32mCompleted.\e[0m"
else
    echo -e "\e[31mFailed.\e[0m"
    exit 1
fi

# Ensure symbol visibility in CycloneDDS by comment out the hidden preset
CHECK_FILE="src/eclipse-cyclonedds/cyclonedds/src/core/CMakeLists.txt"
if [ -f "$CHECK_FILE" ]; then
    sed -i "s/^set_property(TARGET ddsc PROPERTY C_VISIBILITY_PRESET hidden)/\# &/" "$CHECK_FILE"
    echo "CycloneDDS configuration checked: Symbols unhidden successfully."
else
    echo "Error: CycloneDDS source not found. Did you run 'vcs import'?"
    exit 1
fi

# Add PATH
grep -Fxq "export PATH=\$PATH:$HOME/.local/bin" "$HOME/.bashrc" || {
    echo "export PATH=\$PATH:$HOME/.local/bin" >>"$HOME/.bashrc"
}
exit 0
