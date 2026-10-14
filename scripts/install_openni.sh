#!/usr/bin/env bash
# Download the Orbbec OpenNI SDK into the module data directory.
set -euo pipefail

OPENNI_VERSION="v2.3.0.86-beat6"
OPENNI_BASE_URL="https://github.com/orbbec/OpenNI_SDK/releases/download/${OPENNI_VERSION}"

openni_archive_name() {
  case "$(uname -s)-$(uname -m)" in
    Linux-x86_64) printf '%s\n' "OpenNI_2.3.0.86_202210111154_4c8f5aa4_beta6_linux_x64.zip" ;;
    Linux-aarch64) printf '%s\n' "OpenNI_2.3.0.86_202210111155_4c8f5aa4_beta6_arm64.zip" ;;
    Linux-armv7l) printf '%s\n' "OpenNI_2.3.0.86_202210111146_4c8f5aa4_beta6_arm32.zip" ;;
    *)
      echo "OpenNI auto-install supports Linux x86_64, aarch64, and armv7l (this host is $(uname -s) $(uname -m))." >&2
      return 1
      ;;
  esac
}

install_openni() {
  local root="${1:-${VIAM_MODULE_DATA:-.}/openni}"
  local marker="${root}/redist.path"
  local version_file="${root}/version"
  if [[ -f "${marker}" && -f "${version_file}" && "$(cat "${version_file}")" == "${OPENNI_VERSION}" ]]; then
    local existing
    existing="$(cat "${marker}")"
    if [[ -f "${existing}/libOpenNI2.so" ]]; then
      echo "OpenNI SDK ${OPENNI_VERSION} already installed at ${existing}"
      return 0
    fi
  fi

  local archive
  archive="$(openni_archive_name)"
  mkdir -p "${root}"
  local tmp
  tmp="$(mktemp -d)"
  # shellcheck disable=SC2064
  trap "rm -rf '${tmp}'" RETURN

  echo "Downloading Orbbec OpenNI SDK ${OPENNI_VERSION} (${archive})"
  curl -fL --retry 5 --retry-delay 2 -o "${tmp}/sdk.zip" "${OPENNI_BASE_URL}/${archive}"
  rm -rf "${root}/sdk"
  unzip -q "${tmp}/sdk.zip" -d "${root}/sdk"

  local lib drivers redist
  lib="$(find "${root}/sdk" -name libOpenNI2.so -print -quit)"
  if [[ -z "${lib}" ]]; then
    echo "The OpenNI archive did not contain libOpenNI2.so" >&2
    exit 1
  fi
  redist="$(dirname "${lib}")"
  drivers="$(find "${root}/sdk" -type d -path '*/OpenNI2/Drivers' -print -quit)"
  if [[ -z "${drivers}" ]]; then
    echo "The OpenNI archive did not contain OpenNI2/Drivers" >&2
    exit 1
  fi
  if [[ "${drivers}" != "${redist}/OpenNI2/Drivers" ]]; then
    mkdir -p "${redist}/OpenNI2"
    ln -sfn "${drivers}" "${redist}/OpenNI2/Drivers"
  fi
  chmod -R a+rX "${root}/sdk"
  printf '%s\n' "${redist}" > "${marker}"
  printf '%s\n' "${OPENNI_VERSION}" > "${version_file}"
  trap - RETURN
  rm -rf "${tmp}"
  echo "Installed OpenNI SDK at ${redist}"
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  install_openni "${1:-}"
fi
