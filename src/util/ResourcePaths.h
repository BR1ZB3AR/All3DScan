#pragma once

#include <optional>
#include <string>
#include <vector>

namespace revoscan::util {

// Resolve packaged/dev data files for .deb, AppImage, and source-tree runs.
// Search order:
//   1. Absolute path (if the caller already provided one)
//   2. ALL3DSCAN_DATA_DIR / ALL3DSCAN_LIB_DIR environment overrides
//   3. AppImage AppDir ($APPDIR)
//   4. Install prefixes next to the executable
//   5. XDG data dirs /usr/share, /usr/local/share
//   6. Source-tree relative paths (development builds)
std::optional<std::string> findDataFile(const std::string& relativePath,
                                        const std::vector<std::string>& extraCandidates = {});

std::optional<std::string> findDevicesJson();
std::optional<std::string> findMeshWorkerScript();
std::optional<std::string> findAppIcon();

// Returns an argv prefix that launches OrcaSlicer (or a compatible slicer),
// e.g. {"orca-slicer"} or {"flatpak","run","com.orcaslicer.OrcaSlicer"}.
// The file to open is appended by the caller. nullopt if none is found.
// Override with ALL3DSCAN_SLICER (path or command).
std::optional<std::vector<std::string>> findSlicerCommand();

} // namespace revoscan::util
