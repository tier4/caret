#!/bin/bash

# echo usage
function show_usage() {
    echo "Warning: This script may break your repositories, please be careful to use $0"
    echo "Usage: $0 [-h or --help]"
    echo "          [-d or --dry-run]"
    echo "          [-p or --push]"
    echo "          [-t or --tag tag-id]"
    echo ""
    echo "Options"
    echo "-h or --help:    show usage"
    echo "-d or --dry-run: run without any change. only message is displayed."
    echo "-p or --push:    push branch or tag to origin"
    echo "-t --tag:        tag id, which is to added repository. this is mandatory"
    echo "Note:"
    echo "  This script generates .repos files based on the current ROS_DISTRO."
    echo "  Generating repos for older distributions (e.g., humble) in a newer"
    echo "  environment (e.g., jazzy) is disabled, as the required external"
    echo "  repository sources are not present to generate a valid hash."

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

# supported ROS 2 distros
SUPPORTED_DISTROS=("humble" "jazzy")

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

# check the format of version tag
if ! [[ ${TAG_ID} =~ ^v[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    echo "Error: Invalid tag format. Tag must be in the format e.g. v0.4.24"
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

# repos generation loop
for TEMPLATE in "${SCRIPT_DIR}"/template_caret_*.repos; do
    FILENAME=$(basename "$TEMPLATE")

    # get distro name from template file name (ex. template_caret_humble.repos -> humble)
    TEMPLATE_DISTRO=$(echo "$FILENAME" | sed -E 's/template_caret_(.*)\.repos/\1/')
    if [[ $ROS_DISTRO == "jazzy" && $TEMPLATE_DISTRO == "humble" ]]; then
        echo "Skipping ${FILENAME} (Required external packages are missing in this workspace in the ${ROS_DISTRO} environment)."
        continue
    fi
    # check if the distro is supported
    if [[ ! " ${SUPPORTED_DISTROS[*]} " =~ ${TEMPLATE_DISTRO} ]]; then
        continue
    fi

    # select output file name (humble is caret.repos, others are caret_distro.repos)
    if [ "$TEMPLATE_DISTRO" == "humble" ]; then
        OUTPUT="${ROOT_DIR}/caret.repos"
    else
        OUTPUT="${ROOT_DIR}/caret_${TEMPLATE_DISTRO}.repos"
    fi

    # if dry-run, use temp file
    if [ "${DRY_RUN}" == "echo" ]; then
        TARGET_FILE=$(mktemp)
        cp "${TEMPLATE}" "${TARGET_FILE}"
    else
        TARGET_FILE="${OUTPUT}"
        cp "${TEMPLATE}" "${TARGET_FILE}"
    fi

    sed -i -e "s/CARET_TAG/${TAG_ID}/g" "${TARGET_FILE}"

    declare -A MAP=(
        ["ros2/rcl"]="RCL_HASH"
        ["ros2/rclcpp"]="RCLCPP_HASH"
        ["ros-tracing/ros2_tracing"]="ROS_TRACING_HASH"
        ["eclipse-cyclonedds/cyclonedds"]="CYCLONEDDS_HASH"
        ["ros-tooling/topic_tools"]="TOPIC_TOOLS_HASH"
    )

    for path in "${!MAP[@]}"; do
        placeholder="${MAP[$path]}"
        dir=""
        [ -d "${ROOT_DIR}/src/${path}" ] && dir="${ROOT_DIR}/src/${path}"
        [ -z "$dir" ] && [ -d "${ROOT_DIR}/${path}" ] && dir="${ROOT_DIR}/${path}"

        if [ -d "$dir" ]; then
            hash=$(git -C "$dir" rev-parse HEAD)
	    sed -i -e "s/${placeholder}/${hash}/g" "${TARGET_FILE}"
        fi
    done

    if [ "${DRY_RUN}" == "echo" ]; then
        ${DRY_RUN} "--------------------------------------------------------"
	${DRY_RUN} "Dry-run: Previewing generated ${OUTPUT} from ${FILENAME}"
        ${DRY_RUN} "--------------------------------------------------------"
	cat "${TARGET_FILE}"
        rm "${TARGET_FILE}"
    else
        git add "${OUTPUT}"
    fi
done

${DRY_RUN} git commit -m "release(repos): change version of sub repositories for ${TAG_ID}"
${DRY_RUN} git tag "${TAG_ID}"

if [ "${PUSH_REMOTE}" == "true" ]; then
    ${DRY_RUN} git push origin rc/"${TAG_ID}"
    ${DRY_RUN} git push origin "${TAG_ID}"
fi

echo "[Info] Completed release script."
