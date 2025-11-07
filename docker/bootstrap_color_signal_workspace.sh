#!/usr/bin/env bash

set -euo pipefail

usage() {
  cat <<'EOF'
Usage: bootstrap_color_signal_workspace.sh [<humble|jazzy>] [options] [-- docker run args...]

Prepares a ManyMove workspace with the color-signal overlay, then calls
run_manymove_color_signal_container.sh to build and start the docker environment.

Options:
  --workspace PATH         Workspace root (default: ${MANYMOVE_ROS_WS:-~/workspaces/dev_ws})
  --ros-distro DISTRO      ROS 2 distro to target (humble or jazzy, default: jazzy)
  --manymove-branch BRANCH ManyMove git branch to clone (default: main or $MANYMOVE_BRANCH if set)
  --color-signal-branch B  manymove_color_signal git branch (default: main)
  --signal-column-branch B signal_column_msgs git branch (default: main)
  --manymove-repo URL      Override ManyMove git repo URL
  --color-signal-repo URL  Override manymove_color_signal git repo URL
  --signal-column-repo URL Override signal_column_msgs git repo URL
  --pull-latest            Forward to docker runner to refresh base image sources
  --force-rebuild          Force docker rebuild even if caches match
  --build-only             Only build images; do not launch container
  -h, --help               Show this help and exit

Arguments after "--" are forwarded to docker run.
EOF
}

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Required command '$1' not found in PATH." >&2
    exit 1
  fi
}

clone_or_update() {
  local repo_url="$1"
  local branch="$2"
  local dest="$3"
  local description="$4"

  if [[ -d "${dest}/.git" ]]; then
    echo "Updating ${description} in ${dest}..."
    git -C "${dest}" fetch origin "${branch}"
    git -C "${dest}" checkout "${branch}"
    git -C "${dest}" pull --ff-only origin "${branch}"
  else
    echo "Cloning ${description} from ${repo_url} into ${dest}..."
    git clone --branch "${branch}" --depth 1 "${repo_url}" "${dest}"
  fi
}

MANYMOVE_WS_DEFAULT="${MANYMOVE_ROS_WS:-${HOME}/workspaces/dev_ws}"
WORKSPACE="${MANYMOVE_WS_DEFAULT}"
ROS_DISTRO="jazzy"
ROS_DISTRO_SET=false
MANYMOVE_REPO_DEFAULT="https://github.com/pastoriomarco/manymove.git"
MANYMOVE_BRANCH_DEFAULT="${MANYMOVE_BRANCH:-main}"
COLOR_SIGNAL_REPO_DEFAULT="https://github.com/pastoriomarco/manymove_color_signal.git"
COLOR_SIGNAL_BRANCH_DEFAULT="main"
SIGNAL_COLUMN_REPO_DEFAULT="https://github.com/pastoriomarco/signal_column_msgs.git"
SIGNAL_COLUMN_BRANCH_DEFAULT="main"

MANYMOVE_REPO="${MANYMOVE_REPO_DEFAULT}"
MANYMOVE_BRANCH="${MANYMOVE_BRANCH_DEFAULT}"
COLOR_SIGNAL_REPO="${COLOR_SIGNAL_REPO_DEFAULT}"
COLOR_SIGNAL_BRANCH="${COLOR_SIGNAL_BRANCH_DEFAULT}"
SIGNAL_COLUMN_REPO="${SIGNAL_COLUMN_REPO_DEFAULT}"
SIGNAL_COLUMN_BRANCH="${SIGNAL_COLUMN_BRANCH_DEFAULT}"

RUNNER_FLAGS=()
DOCKER_ARGS=()
POSITIONAL_DISTRO=""

if [[ $# -gt 0 ]]; then
  case "$1" in
    humble|jazzy)
      POSITIONAL_DISTRO="$1"
      shift
      ;;
  esac
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --workspace)
      if [[ $# -lt 2 ]]; then
        echo "Missing value for --workspace." >&2
        exit 1
      fi
      WORKSPACE="$2"
      shift 2
      ;;
    --ros-distro)
      if [[ $# -lt 2 ]]; then
        echo "Missing value for --ros-distro." >&2
        exit 1
      fi
      ROS_DISTRO="$2"
      ROS_DISTRO_SET=true
      shift 2
      ;;
    --manymove-branch)
      MANYMOVE_BRANCH="$2"
      shift 2
      ;;
    --color-signal-branch)
      COLOR_SIGNAL_BRANCH="$2"
      shift 2
      ;;
    --signal-column-branch)
      SIGNAL_COLUMN_BRANCH="$2"
      shift 2
      ;;
    --manymove-repo)
      MANYMOVE_REPO="$2"
      shift 2
      ;;
    --color-signal-repo)
      COLOR_SIGNAL_REPO="$2"
      shift 2
      ;;
    --signal-column-repo)
      SIGNAL_COLUMN_REPO="$2"
      shift 2
      ;;
    --pull-latest|--force-rebuild|--build-only)
      RUNNER_FLAGS+=("$1")
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      DOCKER_ARGS+=("$@")
      break
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

case "${ROS_DISTRO}" in
  humble|jazzy)
    ;;
  *)
    echo "Unsupported ROS distro '${ROS_DISTRO}'. Use 'humble' or 'jazzy'." >&2
    exit 1
    ;;
esac

if [[ -n "${POSITIONAL_DISTRO}" ]]; then
  if [[ "${ROS_DISTRO_SET}" == true && "${POSITIONAL_DISTRO}" != "${ROS_DISTRO}" ]]; then
    echo "Conflicting ROS distro selections: positional '${POSITIONAL_DISTRO}' vs --ros-distro '${ROS_DISTRO}'." >&2
    exit 1
  fi
  ROS_DISTRO="${POSITIONAL_DISTRO}"
fi

require_cmd git
require_cmd docker

mkdir -p "${WORKSPACE}"
WORKSPACE="$(cd "${WORKSPACE}" && pwd)"
mkdir -p "${WORKSPACE}/src"

clone_or_update "${MANYMOVE_REPO}" "${MANYMOVE_BRANCH}" "${WORKSPACE}/src/manymove" "ManyMove"
clone_or_update "${COLOR_SIGNAL_REPO}" "${COLOR_SIGNAL_BRANCH}" "${WORKSPACE}/src/manymove_color_signal" "manymove_color_signal"
clone_or_update "${SIGNAL_COLUMN_REPO}" "${SIGNAL_COLUMN_BRANCH}" "${WORKSPACE}/src/signal_column_msgs" "signal_column_msgs"

RUNNER_SCRIPT="${WORKSPACE}/src/manymove_color_signal/docker/run_manymove_color_signal_container.sh"

if [[ ! -x "${RUNNER_SCRIPT}" ]]; then
  echo "Runner script not found or not executable at ${RUNNER_SCRIPT}" >&2
  exit 1
fi

echo "Workspace ready at ${WORKSPACE}."
echo "Launching docker workflow for ROS ${ROS_DISTRO} (ManyMove branch ${MANYMOVE_BRANCH})..."

CMD=("env" "MANYMOVE_BRANCH=${MANYMOVE_BRANCH}" "${RUNNER_SCRIPT}" "${ROS_DISTRO}" "${RUNNER_FLAGS[@]}")
if [[ ${#DOCKER_ARGS[@]} -gt 0 ]]; then
  CMD+=("--")
  CMD+=("${DOCKER_ARGS[@]}")
fi

"${CMD[@]}"
