#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-${ROOT}/build-release}"
DIST_DIR="${DIST_DIR:-${ROOT}/dist}"
TARGET="${1:-all}"
VERSION="Alpha-v20260802-1033"

case "${TARGET}" in
  all|deb|appimage) ;;
  *)
    echo "usage: $0 [all|deb|appimage]" >&2
    exit 2
    ;;
esac

mkdir -p "${BUILD_DIR}" "${DIST_DIR}"

# Verify against a test-enabled tree before packaging.
VERIFY_DIR="${ROOT}/build"
cmake -S "${ROOT}" -B "${VERIFY_DIR}" -DBUILD_TESTING=ON -DBUILD_TESTS=ON
cmake --build "${VERIFY_DIR}" --parallel
ctest --test-dir "${VERIFY_DIR}" --output-on-failure

# Package from a release tree with tests disabled.
cmake -S "${ROOT}" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr \
  -DBUILD_TESTING=OFF \
  -DBUILD_TESTS=OFF
cmake --build "${BUILD_DIR}" --parallel

if [[ "${TARGET}" == "all" || "${TARGET}" == "deb" ]]; then
  cpack --config "${BUILD_DIR}/CPackConfig.cmake" -G DEB -B "${DIST_DIR}"
fi

if [[ "${TARGET}" == "all" || "${TARGET}" == "appimage" ]]; then
  ARCH_RAW="$(uname -m)"
  case "${ARCH_RAW}" in
    x86_64) ARCH="x86_64" ;;
    aarch64|arm64) ARCH="aarch64" ;;
    *)
      echo "AppImage packaging is not configured for architecture: ${ARCH_RAW}" >&2
      exit 1
      ;;
  esac

  TOOL_DIR="${BUILD_DIR}/appimage-tools"
  APPDIR="${BUILD_DIR}/AppDir"
  LINUXDEPLOY="${TOOL_DIR}/linuxdeploy-${ARCH}.AppImage"
  QT_PLUGIN="${TOOL_DIR}/linuxdeploy-plugin-qt-${ARCH}.AppImage"
  mkdir -p "${TOOL_DIR}"

  download_tool() {
    local destination="$1"
    local url="$2"
    if [[ ! -x "${destination}" ]]; then
      if ! command -v curl >/dev/null; then
        echo "curl is required to download AppImage packaging tools" >&2
        exit 1
      fi
      curl --fail --location --retry 3 --output "${destination}" "${url}"
      chmod +x "${destination}"
    fi
  }

  download_tool "${LINUXDEPLOY}" \
    "https://github.com/linuxdeploy/linuxdeploy/releases/download/continuous/linuxdeploy-${ARCH}.AppImage"
  download_tool "${QT_PLUGIN}" \
    "https://github.com/linuxdeploy/linuxdeploy-plugin-qt/releases/download/continuous/linuxdeploy-plugin-qt-${ARCH}.AppImage"

  rm -rf "${APPDIR}"
  DESTDIR="${APPDIR}" cmake --install "${BUILD_DIR}"

  # AppImage cannot install host udev rules. Keep a copy for the user to install.
  mkdir -p "${APPDIR}/usr/share/doc/all3dscan"
  cp "${ROOT}/resources/99-all3dscan-scanners.rules" \
    "${APPDIR}/usr/share/doc/all3dscan/99-all3dscan-scanners.rules"

  export PATH="${TOOL_DIR}:${PATH}"
  export APPIMAGE_EXTRACT_AND_RUN=1
  export OUTPUT="${DIST_DIR}/All3DScan-${VERSION}-${ARCH}.AppImage"
  "${LINUXDEPLOY}" \
    --appdir "${APPDIR}" \
    --executable "${APPDIR}/usr/bin/all3dscan" \
    --desktop-file "${APPDIR}/usr/share/applications/io.github.all3dscan.All3DScan.desktop" \
    --icon-file "${APPDIR}/usr/share/icons/hicolor/scalable/apps/io.github.all3dscan.All3DScan.svg" \
    --plugin qt \
    --output appimage
fi

echo "Packages written to ${DIST_DIR}"
