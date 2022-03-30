#!/bin/bash

SCRIPT_DIR=$(readlink -f "$(dirname "$0")")

# Parse args
noninteractive=0
while getopts c OPT
do
  case $OPT in
    "c" ) noninteractive=1
      echo "run setup with a noninteractive mode"
      ;;
    "*" )
      ;;
  esac
done

# Confirm to start installation
if [ $noninteractive -eq 0 ]; then
  # Prevent root execution in interactive mode
  if [ "$(id -u)" -eq 0 ]; then
    echo "Do not run setup as root!" >&2
    exit 1
  fi

  echo "Setting up the build environment take up to 30 minutes."
  read -rp ">  Are you sure you want to run setup? [y/N] " answer
else
  answer="y"
fi

# Use "Super Cow Powers"!
if [[ $answer = [cC] ]]; then
  if ! (command -v cowsay  > /dev/null 2>&1); then
    sudo apt-get install -y cowsay
  fi

  answer="y"
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
case $answer in
  [yY]* )
  # Install ansible
  ansible_version=$(pip3 list | grep -oP "^ansible\s+\K([0-9]+)" || true)
  if [ "$ansible_version" != "5" ]; then
    sudo apt-get -y purge ansible
    sudo pip3 install -U "ansible==5.*"
  fi  
  # Set options
  if [ $noninteractive -eq 0 ]; then
    options=("--ask-become-pass")
  else
    options=("--extra-vars" "yn_gpu=y")
  fi

  # Run ansible
  if ansible-playbook "$SCRIPT_DIR/ansible/playbook.yml" -e WORKSPACE_ROOT="$SCRIPT_DIR" "${options[@]}"; then
    echo -e "\e[32mCompleted.\e[0m"
    exit 0
  else
    echo -e "\e[31mFailed.\e[0m"
    exit 1
  fi
  ;;
  * )
  echo -e "\e[33mCancelled.\e[0m"
  exit 1
  ;;
esac
