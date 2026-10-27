#pragma once

#include <cstdint>
#include <memory>
#include <vector>

namespace all3dscan::fusion {

// Builds one point cloud out of a scan's frames: each frame is aligned onto
// the model built so far (point-to-plane ICP, a coarse pass then a fine one,
// seeded with the last frame's pose) and merged into it, thinned to one point
// per voxel. This is the Qt app's tracker (src/processing/Registration),
// driven with the settings its pipeline uses, behind an interface the Rust
// engine can call.
//
// Points come in and go out as x0, y0, z0, x1, ... in millimetres in the
// camera frame, with three colour bytes each, as the engine holds them; the
// registration underneath works in metres.
class FrameFusion {
public:
    // Throws std::invalid_argument unless 0 < voxel_mm <= 50.
    explicit FrameFusion(float voxel_mm);
    ~FrameFusion();
    FrameFusion(const FrameFusion&) = delete;
    FrameFusion& operator=(const FrameFusion&) = delete;

    // Aligns a frame onto the model and merges it, returning whether it was
    // aligned well enough to merge; a frame that was not is left out of the
    // model entirely. `xyz` holds 3 * points floats and `rgb` 3 * points
    // bytes. Plain pointers, so that this header stays free of cxx: the Rust
    // engine passes its slices straight in.
    bool add(const float* xyz, uint64_t points, const uint8_t* rgb);

    // How the last frame sat on the model: the share of its points lying on
    // the model's surface and their RMS distance from it. Both are 0 for the
    // first frame, which becomes the model without being aligned to anything.
    double lastOverlap() const noexcept { return lastOverlap_; }
    double lastRmseMm() const noexcept { return lastRmseMm_; }

    // Where the last merged frame was placed: a row-major 4x4 rigid
    // transform taking the frame's points into the model's axes, with its
    // translation in millimetres. Identity before anything has been merged.
    // The caller applies it to whatever else it keeps per point, such as
    // normals, which the registration underneath does not carry.
    const std::vector<double>& lastPose() const noexcept { return lastPose_; }

    // Points in the model this aligns against. The caller keeps the model
    // it cares about, with whatever it holds per point; this one exists to
    // give ICP something to match against.
    uint64_t modelPoints() const noexcept;
    void clear();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    double lastOverlap_ = 0.0;
    double lastRmseMm_ = 0.0;
    std::vector<double> lastPose_{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
};

std::unique_ptr<FrameFusion> makeFrameFusion(float voxel_mm);

} // namespace all3dscan::fusion
