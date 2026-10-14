#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

PYTHON="${PYTHON:-python3}"
if [[ -n "${VIAM_MODULE_DATA:-}" ]]; then
  VENV="${VIAM_MODULE_DATA}/venv"
  OPENNI_ROOT="${VIAM_MODULE_DATA}/openni"
else
  VENV=".venv"
  OPENNI_ROOT=".openni"
fi

as_root() {
  if [[ "$(id -u)" -eq 0 ]]; then
    "$@"
  elif sudo -n true >/dev/null 2>&1; then
    sudo -n "$@"
  else
    return 1
  fi
}

install_system_packages() {
  if ! command -v apt-get >/dev/null 2>&1; then
    return 0
  fi
  local packages=()
  if ! ldconfig -p 2>/dev/null | grep -q 'libusb-1.0.so'; then
    packages+=(libusb-1.0-0)
  fi
  if ! command -v curl >/dev/null 2>&1; then
    packages+=(curl ca-certificates)
  fi
  if ! command -v unzip >/dev/null 2>&1; then
    packages+=(unzip)
  fi
  if [[ ${#packages[@]} -eq 0 ]]; then
    return 0
  fi
  echo "Installing system packages: ${packages[*]}"
  if ! as_root apt-get update -qq || ! as_root apt-get install -y "${packages[@]}"; then
    echo "Could not install ${packages[*]}. Install them and rerun setup.sh." >&2
    exit 1
  fi
}

install_udev_rules() {
  local src="scripts/99-orbbec-usb.rules"
  local dest="/etc/udev/rules.d/99-orbbec-usb.rules"
  if ! as_root install -m 0644 "${src}" "${dest}"; then
    echo "Could not install ${dest}. The camera will not open until that rule is installed." >&2
    exit 1
  fi
  as_root udevadm control --reload-rules || true
  as_root udevadm trigger || true
  echo "Installed ${dest}. Replug the DaBai Max so the rule applies."
}

"$PYTHON" -m venv "$VENV"
"$VENV/bin/pip" install --upgrade pip
"$VENV/bin/pip" install -r requirements.txt

if [[ "$(uname -s)" == "Linux" ]]; then
  install_system_packages
  install_udev_rules
  # shellcheck disable=SC1091
  source scripts/install_openni.sh
  install_openni "${OPENNI_ROOT}"
else
  echo "Skipping OpenNI SDK and udev install on $(uname -s). Those run on the Linux machine."
fi
