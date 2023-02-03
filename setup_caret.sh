#!/bin/bash

set -Ceu

SCRIPT_DIR=$(readlink -f "$(dirname "$0")")

function show_usage() {
    echo "${0} executes setup for CARET."
    echo "Options "
    echo "    -h or --help: show help message"
    echo "    -c or --no-interactive"
    echo "    -n or --no-package-install"
    exit 0
}

# parse command options.
OPT=$(getopt -o nch -l no-interactive,no-package-install,help -- "$@")

eval set -- "$OPT"

# Parse args
noninteractive=0
package_install=1

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
    --)
        shift
        break
        ;;
    esac
done

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
# TODO: To update Ansible 6.*
ansible_version=$(pip3 list | grep -oP "^ansible\s+\K([0-9]+)" || true)
if [ "$ansible_version" != "5" ]; then
    sudo apt-get -y update
    sudo apt-get -y purge ansible
    sudo pip3 install -U "ansible==5.*"
fi

# Set options
if [ $noninteractive -eq 0 ]; then
    options=("--ask-become-pass")
fi

if [ $package_install -eq 0 ]; then
    options=("--extra-vars" "package_install=n")
fi

# Run ansible
if ansible-playbook "$SCRIPT_DIR/ansible/playbook.yml" -e WORKSPACE_ROOT="$SCRIPT_DIR" "${options[@]}"; then
    echo -e "\e[32mCompleted.\e[0m"
    exit 0
else
    echo -e "\e[31mFailed.\e[0m"
    exit 1
fi
