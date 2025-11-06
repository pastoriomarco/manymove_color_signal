#!/usr/bin/env bash

set -euo pipefail

usage() {
  cat <<'EOF'
Usage: run_manymove_color_signal_container.sh <humble|jazzy> [--pull-latest] [--force-rebuild] [--build-only] [--] [docker run args...]

Builds (if required) the base ManyMove development image and a color-signal
overlay, then optionally starts an interactive container.

Options:
  --pull-latest   Sync the upstream ManyMove repository before building the base image.
  --force-rebuild Rebuild both base and overlay images even if the cached context is unchanged.
  --build-only    Build/refresh the images but do not launch a container.

Arguments after "--" are forwarded to "docker run".
EOF
}

if [[ $# -lt 1 ]]; then
  usage
  exit 1
fi

DISTRO="$1"
shift

case "${DISTRO}" in
  humble|jazzy) ;;
  *)
    echo "Unsupported ROS 2 distribution '${DISTRO}'. Use 'humble' or 'jazzy'." >&2
    exit 1
    ;;
esac

PULL_LATEST=false
FORCE_REBUILD=false
BUILD_ONLY=false
EXTRA_DOCKER_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --pull-latest)
      PULL_LATEST=true
      shift
      ;;
    --force-rebuild)
      FORCE_REBUILD=true
      shift
      ;;
    --build-only)
      BUILD_ONLY=true
      shift
      ;;
    --)
      shift
      EXTRA_DOCKER_ARGS+=("$@")
      break
      ;;
    *)
      EXTRA_DOCKER_ARGS+=("$1")
      shift
      ;;
  esac
done

if ! command -v docker >/dev/null 2>&1; then
  echo "docker CLI not found on PATH." >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
DOCKERFILE="${SCRIPT_DIR}/Dockerfile.manymove_color_signal"
BUILD_CONTEXT="${REPO_ROOT}"
BASE_SCRIPT="${REPO_ROOT}/src/manymove/manymove_bringup/docker/run_manymove_container.sh"

if [[ ! -f "${DOCKERFILE}" ]]; then
  echo "Dockerfile not found at ${DOCKERFILE}" >&2
  exit 1
fi

if [[ ! -x "${BASE_SCRIPT}" ]]; then
  echo "Base ManyMove helper script missing or not executable at ${BASE_SCRIPT}" >&2
  exit 1
fi

ensure_package_dir() {
  local rel_path="$1"
  local abs_path="${REPO_ROOT}/${rel_path}"
  if [[ ! -d "${abs_path}" ]]; then
    echo "Required package directory '${rel_path}' not found under ${REPO_ROOT}" >&2
    exit 1
  fi
}

ensure_package_dir "src/manymove_color_signal"
ensure_package_dir "src/signal_column_msgs"

CONTAINER_USER="${USER:-manymove}"
CONTAINER_GID="$(id -g)"

BASE_IMAGE_TAG="manymove:${DISTRO}"
OVERLAY_IMAGE_TAG="manymove_color_signal:${DISTRO}"
LABEL_KEY="manymove.colorsignal.context.sha"

BASE_BUILD_ARGS=("${DISTRO}" "--build-only")
if [[ "${PULL_LATEST}" == true ]]; then
  BASE_BUILD_ARGS+=("--pull-latest")
fi
if [[ "${FORCE_REBUILD}" == true ]]; then
  BASE_BUILD_ARGS+=("--force-rebuild")
fi

"${BASE_SCRIPT}" "${BASE_BUILD_ARGS[@]}"

BASE_IMAGE_ID="$(docker image inspect "${BASE_IMAGE_TAG}" --format '{{ .Id }}')"

compute_hash() {
  local path_list=("$@")
  {
    printf '%s\n' "${DISTRO}"
    printf '%s\n' "${BASE_IMAGE_ID}"
    cat "${DOCKERFILE}"
    for rel in "${path_list[@]}"; do
      local abs="${REPO_ROOT}/${rel}"
      if [[ -d "${abs}" ]]; then
        find "${abs}" -type f \
          ! -path '*/build/*' \
          ! -path '*/install/*' \
          ! -path '*/log/*' \
          ! -path '*/.git/*' \
          | LC_ALL=C sort \
          | while IFS= read -r file; do
              local rel_path="${file#"${REPO_ROOT}/"}"
              local file_hash
              file_hash="$(sha256sum "${file}" | awk '{print $1}')"
              printf '%s  %s\n' "${file_hash}" "${rel_path}"
            done
      fi
    done
  } | sha256sum | awk '{print $1}'
}

CONTEXT_HASH="$(compute_hash "src/manymove_color_signal" "src/signal_column_msgs")"

IMAGE_PRESENT=false
EXISTING_HASH=""

if docker image inspect "${OVERLAY_IMAGE_TAG}" >/dev/null 2>&1; then
  IMAGE_PRESENT=true
  EXISTING_HASH="$(docker image inspect "${OVERLAY_IMAGE_TAG}" --format "{{ index .Config.Labels \"${LABEL_KEY}\" }}" 2>/dev/null || true)"
fi

NEEDS_BUILD=false
if [[ "${FORCE_REBUILD}" == true ]]; then
  NEEDS_BUILD=true
elif [[ "${IMAGE_PRESENT}" == false ]]; then
  NEEDS_BUILD=true
elif [[ "${EXISTING_HASH}" != "${CONTEXT_HASH}" ]]; then
  NEEDS_BUILD=true
fi

if [[ "${NEEDS_BUILD}" == true ]]; then
  if [[ "${IMAGE_PRESENT}" == true ]]; then
    echo "Rebuilding overlay image '${OVERLAY_IMAGE_TAG}'."
  else
    echo "Building overlay image '${OVERLAY_IMAGE_TAG}'."
  fi

  docker build \
    --build-arg "ROS_DISTRO=${DISTRO}" \
    --build-arg "BASE_IMAGE_TAG=${BASE_IMAGE_TAG}" \
    --build-arg "USERNAME=${CONTAINER_USER}" \
    --build-arg "USER_GID=${CONTAINER_GID}" \
    --label "${LABEL_KEY}=${CONTEXT_HASH}" \
    -f "${DOCKERFILE}" \
    -t "${OVERLAY_IMAGE_TAG}" \
    "${BUILD_CONTEXT}"
else
  echo "Overlay image '${OVERLAY_IMAGE_TAG}' is up to date."
fi

if [[ "${BUILD_ONLY}" == true ]]; then
  exit 0
fi

RUN_ARGS=(
  "--rm"
  "-it"
  "--network" "host"
)

if [[ -n "${DISPLAY:-}" ]]; then
  RUN_ARGS+=("-e" "DISPLAY=${DISPLAY}")
  if [[ -d /tmp/.X11-unix ]]; then
    RUN_ARGS+=("-v" "/tmp/.X11-unix:/tmp/.X11-unix:rw")
  fi
fi

if [[ -n "${XAUTHORITY:-}" && -f "${XAUTHORITY}" ]]; then
  RUN_ARGS+=("-e" "XAUTHORITY=${XAUTHORITY}")
  RUN_ARGS+=("-v" "${XAUTHORITY}:${XAUTHORITY}:rw")
fi

# Optional GPU disable flag
if [[ -z "${MANYMOVE_NO_GPU:-}" ]] && command -v nvidia-smi >/dev/null 2>&1; then
  RUN_ARGS+=("--gpus" "all")
else
  echo "GPU support disabled (MANYMOVE_NO_GPU set or nvidia-smi missing)"
fi

echo "Launching container '${OVERLAY_IMAGE_TAG}'."
docker run "${RUN_ARGS[@]}" "${EXTRA_DOCKER_ARGS[@]}" "${OVERLAY_IMAGE_TAG}" bash
