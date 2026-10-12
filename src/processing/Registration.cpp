#include "Registration.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace revoscan::processing {
namespace {

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

// --- Symmetric 4x4 Jacobi eigen-decomposition -----------------------------
// Cyclic Jacobi (Numerical Recipes style). Fills eigenvalues d[] and
// eigenvectors as columns of v[][]. Input a[][] is destroyed.
void jacobiEigen4(double a[4][4], double v[4][4], double d[4]) {
    const int n = 4;
    double b[4];
    double z[4];
    for (int ip = 0; ip < n; ++ip) {
        for (int iq = 0; iq < n; ++iq) {
            v[ip][iq] = (ip == iq) ? 1.0 : 0.0;
        }
        b[ip] = d[ip] = a[ip][ip];
        z[ip] = 0.0;
    }
    for (int sweep = 0; sweep < 100; ++sweep) {
        double sm = 0.0;
        for (int ip = 0; ip < n - 1; ++ip) {
            for (int iq = ip + 1; iq < n; ++iq) {
                sm += std::fabs(a[ip][iq]);
            }
        }
        if (sm == 0.0) {
            return;
        }
        const double tresh = (sweep < 3) ? 0.2 * sm / (n * n) : 0.0;
        for (int ip = 0; ip < n - 1; ++ip) {
            for (int iq = ip + 1; iq < n; ++iq) {
                const double g = 100.0 * std::fabs(a[ip][iq]);
                if (sweep > 3 && std::fabs(d[ip]) + g == std::fabs(d[ip]) &&
                    std::fabs(d[iq]) + g == std::fabs(d[iq])) {
                    a[ip][iq] = 0.0;
                } else if (std::fabs(a[ip][iq]) > tresh) {
                    double h = d[iq] - d[ip];
                    double t;
                    if (std::fabs(h) + g == std::fabs(h)) {
                        t = a[ip][iq] / h;
                    } else {
                        const double theta = 0.5 * h / a[ip][iq];
                        t = 1.0 / (std::fabs(theta) + std::sqrt(1.0 + theta * theta));
                        if (theta < 0.0) {
                            t = -t;
                        }
                    }
                    const double c = 1.0 / std::sqrt(1.0 + t * t);
                    const double s = t * c;
                    const double tau = s / (1.0 + c);
                    h = t * a[ip][iq];
                    z[ip] -= h;
                    z[iq] += h;
                    d[ip] -= h;
                    d[iq] += h;
                    a[ip][iq] = 0.0;
                    auto rotate = [&](double mat[4][4], int i, int j, int k, int l) {
                        const double gg = mat[i][j];
                        const double hh = mat[k][l];
                        mat[i][j] = gg - s * (hh + gg * tau);
                        mat[k][l] = hh + s * (gg - hh * tau);
                    };
                    for (int j = 0; j < ip; ++j) {
                        rotate(a, j, ip, j, iq);
                    }
                    for (int j = ip + 1; j < iq; ++j) {
                        rotate(a, ip, j, j, iq);
                    }
                    for (int j = iq + 1; j < n; ++j) {
                        rotate(a, ip, j, iq, j);
                    }
                    for (int j = 0; j < n; ++j) {
                        rotate(v, j, ip, j, iq);
                    }
                }
            }
        }
        for (int ip = 0; ip < n; ++ip) {
            b[ip] += z[ip];
            d[ip] = b[ip];
            z[ip] = 0.0;
        }
    }
}

// Horn's absolute-orientation: optimal rotation aligning src -> dst given
// matched, equally weighted correspondences. Returns a quaternion (w,x,y,z).
void hornRotation(const std::vector<Vec3>& src, const std::vector<Vec3>& dst,
                  const Vec3& cs, const Vec3& cd, double quat[4]) {
    // Cross-covariance S[a][b] = sum (src-cs)_a * (dst-cd)_b.
    double S[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    for (size_t i = 0; i < src.size(); ++i) {
        const double sx = src[i].x - cs.x;
        const double sy = src[i].y - cs.y;
        const double sz = src[i].z - cs.z;
        const double dx = dst[i].x - cd.x;
        const double dy = dst[i].y - cd.y;
        const double dz = dst[i].z - cd.z;
        S[0][0] += sx * dx;
        S[0][1] += sx * dy;
        S[0][2] += sx * dz;
        S[1][0] += sy * dx;
        S[1][1] += sy * dy;
        S[1][2] += sy * dz;
        S[2][0] += sz * dx;
        S[2][1] += sz * dy;
        S[2][2] += sz * dz;
    }

    double N[4][4];
    N[0][0] = S[0][0] + S[1][1] + S[2][2];
    N[0][1] = S[1][2] - S[2][1];
    N[0][2] = S[2][0] - S[0][2];
    N[0][3] = S[0][1] - S[1][0];
    N[1][1] = S[0][0] - S[1][1] - S[2][2];
    N[1][2] = S[0][1] + S[1][0];
    N[1][3] = S[2][0] + S[0][2];
    N[2][2] = -S[0][0] + S[1][1] - S[2][2];
    N[2][3] = S[1][2] + S[2][1];
    N[3][3] = -S[0][0] - S[1][1] + S[2][2];
    N[1][0] = N[0][1];
    N[2][0] = N[0][2];
    N[2][1] = N[1][2];
    N[3][0] = N[0][3];
    N[3][1] = N[1][3];
    N[3][2] = N[2][3];

    double eigvec[4][4];
    double eigval[4];
    jacobiEigen4(N, eigvec, eigval);

    int best = 0;
    for (int i = 1; i < 4; ++i) {
        if (eigval[i] > eigval[best]) {
            best = i;
        }
    }
    for (int i = 0; i < 4; ++i) {
        quat[i] = eigvec[i][best];
    }
    double norm = std::sqrt(quat[0] * quat[0] + quat[1] * quat[1] +
                            quat[2] * quat[2] + quat[3] * quat[3]);
    if (norm < 1e-12) {
        quat[0] = 1.0;
        quat[1] = quat[2] = quat[3] = 0.0;
    } else {
        for (int i = 0; i < 4; ++i) {
            quat[i] /= norm;
        }
    }
}

RigidTransform fromQuatTranslation(const double q[4], const Vec3& cs, const Vec3& cd) {
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    double R[3][3];
    R[0][0] = 1 - 2 * (y * y + z * z);
    R[0][1] = 2 * (x * y - w * z);
    R[0][2] = 2 * (x * z + w * y);
    R[1][0] = 2 * (x * y + w * z);
    R[1][1] = 1 - 2 * (x * x + z * z);
    R[1][2] = 2 * (y * z - w * x);
    R[2][0] = 2 * (x * z - w * y);
    R[2][1] = 2 * (y * z + w * x);
    R[2][2] = 1 - 2 * (x * x + y * y);

    // t = cd - R * cs
    const double tx = cd.x - (R[0][0] * cs.x + R[0][1] * cs.y + R[0][2] * cs.z);
    const double ty = cd.y - (R[1][0] * cs.x + R[1][1] * cs.y + R[1][2] * cs.z);
    const double tz = cd.z - (R[2][0] * cs.x + R[2][1] * cs.y + R[2][2] * cs.z);

    RigidTransform out;
    out.m = {R[0][0], R[0][1], R[0][2], tx, R[1][0], R[1][1], R[1][2], ty,
             R[2][0], R[2][1], R[2][2], tz, 0.0,     0.0,     0.0,     1.0};
    return out;
}

// --- Spatial hash for nearest-neighbour correspondence --------------------
struct GridKey {
    int x, y, z;
    bool operator==(const GridKey& o) const { return x == o.x && y == o.y && z == o.z; }
};
struct GridKeyHash {
    size_t operator()(const GridKey& k) const {
        return (static_cast<size_t>(k.x) * 73856093u) ^
               (static_cast<size_t>(k.y) * 19349663u) ^
               (static_cast<size_t>(k.z) * 83492791u);
    }
};

class SpatialHash {
public:
    SpatialHash(const pointcloud::PointCloud& pts, float cell)
        : pts_(&pts), cell_(cell > 0.0f ? cell : 1e-3f) {
        grid_.reserve(pts.points.size());
        for (size_t i = 0; i < pts.points.size(); ++i) {
            grid_[keyOf(pts.points[i])].push_back(i);
        }
    }

    // Nearest point index within maxDist, or -1. Writes squared distance.
    long nearest(double qx, double qy, double qz, double maxDist, double* outSq) const {
        const int cx = cellIndex(qx);
        const int cy = cellIndex(qy);
        const int cz = cellIndex(qz);
        double bestSq = maxDist * maxDist;
        long best = -1;
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    auto it = grid_.find(GridKey{cx + dx, cy + dy, cz + dz});
                    if (it == grid_.end()) {
                        continue;
                    }
                    for (int idx : it->second) {
                        const auto& p = pts_->points[static_cast<size_t>(idx)];
                        const double ex = p.x - qx;
                        const double ey = p.y - qy;
                        const double ez = p.z - qz;
                        const double sq = ex * ex + ey * ey + ez * ez;
                        if (sq < bestSq) {
                            bestSq = sq;
                            best = idx;
                        }
                    }
                }
            }
        }
        if (best >= 0 && outSq) {
            *outSq = bestSq;
        }
        return best;
    }

private:
    int cellIndex(double v) const { return static_cast<int>(std::floor(v / cell_)); }
    GridKey keyOf(const pointcloud::Point& p) const {
        return GridKey{cellIndex(p.x), cellIndex(p.y), cellIndex(p.z)};
    }

    const pointcloud::PointCloud* pts_;
    float cell_;
    std::unordered_map<GridKey, std::vector<int>, GridKeyHash> grid_;
};

} // namespace

RigidTransform RigidTransform::operator*(const RigidTransform& rhs) const {
    RigidTransform out;
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            double sum = 0.0;
            for (int k = 0; k < 4; ++k) {
                sum += m[r * 4 + k] * rhs.m[k * 4 + c];
            }
            out.m[r * 4 + c] = sum;
        }
    }
    return out;
}

double RigidTransform::translationNorm() const {
    return std::sqrt(m[3] * m[3] + m[7] * m[7] + m[11] * m[11]);
}

double RigidTransform::rotationAngle() const {
    const double trace = m[0] + m[5] + m[10];
    const double c = std::clamp((trace - 1.0) * 0.5, -1.0, 1.0);
    return std::acos(c);
}

void transformInPlace(pointcloud::PointCloud& cloud, const RigidTransform& t) {
    const auto& m = t.m;
    for (auto& p : cloud.points) {
        const double x = p.x, y = p.y, z = p.z;
        p.x = static_cast<float>(m[0] * x + m[1] * y + m[2] * z + m[3]);
        p.y = static_cast<float>(m[4] * x + m[5] * y + m[6] * z + m[7]);
        p.z = static_cast<float>(m[8] * x + m[9] * y + m[10] * z + m[11]);
    }
}

pointcloud::PointCloud transformed(const pointcloud::PointCloud& cloud,
                                   const RigidTransform& t) {
    pointcloud::PointCloud out = cloud;
    transformInPlace(out, t);
    return out;
}

pointcloud::PointCloud voxelDownsample(const pointcloud::PointCloud& in, float voxel) {
    if (in.points.empty() || voxel <= 0.0f) {
        return in;
    }
    std::unordered_map<GridKey, pointcloud::Point, GridKeyHash> cells;
    cells.reserve(in.points.size());
    for (const auto& p : in.points) {
        GridKey key{static_cast<int>(std::floor(p.x / voxel)),
                    static_cast<int>(std::floor(p.y / voxel)),
                    static_cast<int>(std::floor(p.z / voxel))};
        cells.emplace(key, p);
    }
    pointcloud::PointCloud out;
    out.points.reserve(cells.size());
    for (const auto& e : cells) {
        out.points.push_back(e.second);
    }
    return out;
}

IcpResult icpPointToPoint(const pointcloud::PointCloud& source,
                          const pointcloud::PointCloud& target,
                          const RigidTransform& init, const IcpOptions& opts) {
    IcpResult result;
    result.transform = init;
    if (source.points.size() < opts.minCorrespondences ||
        target.points.size() < opts.minCorrespondences) {
        return result;
    }

    const SpatialHash hash(target, opts.maxCorrespondenceDistM);
    const double maxDist = opts.maxCorrespondenceDistM;

    pointcloud::PointCloud working = transformed(source, init);

    std::vector<Vec3> srcCorr;
    std::vector<Vec3> dstCorr;
    srcCorr.reserve(source.points.size());
    dstCorr.reserve(source.points.size());

    for (int iter = 0; iter < opts.maxIterations; ++iter) {
        srcCorr.clear();
        dstCorr.clear();
        double sumSq = 0.0;
        for (const auto& wp : working.points) {
            double sq = 0.0;
            const long idx = hash.nearest(wp.x, wp.y, wp.z, maxDist, &sq);
            if (idx < 0) {
                continue;
            }
            const auto& tp = target.points[static_cast<size_t>(idx)];
            srcCorr.push_back(Vec3{wp.x, wp.y, wp.z});
            dstCorr.push_back(Vec3{tp.x, tp.y, tp.z});
            sumSq += sq;
        }

        result.iterations = static_cast<size_t>(iter + 1);
        result.correspondences = srcCorr.size();
        if (srcCorr.size() < opts.minCorrespondences) {
            break;
        }

        Vec3 cs, cd;
        for (size_t i = 0; i < srcCorr.size(); ++i) {
            cs.x += srcCorr[i].x;
            cs.y += srcCorr[i].y;
            cs.z += srcCorr[i].z;
            cd.x += dstCorr[i].x;
            cd.y += dstCorr[i].y;
            cd.z += dstCorr[i].z;
        }
        const double invN = 1.0 / static_cast<double>(srcCorr.size());
        cs.x *= invN;
        cs.y *= invN;
        cs.z *= invN;
        cd.x *= invN;
        cd.y *= invN;
        cd.z *= invN;

        double quat[4];
        hornRotation(srcCorr, dstCorr, cs, cd, quat);
        const RigidTransform delta = fromQuatTranslation(quat, cs, cd);

        result.transform = delta * result.transform;
        working = transformed(source, result.transform);

        result.fitness = static_cast<double>(srcCorr.size()) /
                         static_cast<double>(source.points.size());
        result.rmseM = std::sqrt(sumSq / static_cast<double>(srcCorr.size()));

        if (delta.translationNorm() < opts.convergenceTransEpsM &&
            delta.rotationAngle() < opts.convergenceRotEps) {
            result.converged = true;
            break;
        }
    }

    // Final fitness/RMSE against the converged pose.
    {
        size_t inliers = 0;
        double sumSq = 0.0;
        for (const auto& wp : working.points) {
            double sq = 0.0;
            if (hash.nearest(wp.x, wp.y, wp.z, maxDist, &sq) >= 0) {
                ++inliers;
                sumSq += sq;
            }
        }
        if (!source.points.empty()) {
            result.fitness =
                static_cast<double>(inliers) / static_cast<double>(source.points.size());
        }
        result.correspondences = inliers;
        result.rmseM = inliers > 0 ? std::sqrt(sumSq / static_cast<double>(inliers)) : 0.0;
    }

    return result;
}

} // namespace revoscan::processing
