#pragma once

#include <QString>

namespace revoscan::app {

// Dark theme approximating the Revo Scan 6 desktop app look.
QString revoScanStyleSheet();

// Shared palette constants (hex strings) used by custom-painted widgets.
namespace theme {
constexpr const char* kWindowBg = "#1b1d21";
constexpr const char* kPanelBg = "#232629";
constexpr const char* kPanelBgAlt = "#2b2f33";
constexpr const char* kRibbonBg = "#26292e";
constexpr const char* kBorder = "#3a3f45";
constexpr const char* kText = "#d6dae0";
constexpr const char* kTextDim = "#8b9198";
constexpr const char* kAccent = "#12b6b0";       // teal (Excellent)
constexpr const char* kAccentGood = "#5bbf5b";   // green (Good)
constexpr const char* kWarn = "#e0a83e";
constexpr const char* kBad = "#d95c5c";
} // namespace theme

} // namespace revoscan::app
