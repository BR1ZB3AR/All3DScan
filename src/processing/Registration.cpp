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

    // Indices of all points within `radius` (radius must be <= cell size).
    void within(double qx, double qy, double qz, double radius, std::vector<int>& out) const {
        out.clear();
        const int cx = cellIndex(qx);
        const int cy = cellIndex(qy);
        const int cz = cellIndex(qz);
        const double r2 = radius * radius;
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    auto it = grid_.find(GridKey{cx + dx, cy + dy, cz + dz});
                    if (it == grid_.end()) {
                        continue;
                    }
                    for (int idx : it->second) {
                        const auto& p = pts_->points[static_cast<size_t>(idx)];
                        const double ex = p.x - qx, ey = p.y - qy, ez = p.z - qz;
                        if (ex * ex + ey * ey + ez * ez <= r2) {
                            out.push_back(idx);
                        }
                    }
                }
            }
        }
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


// Smallest-eigenvalue eigenvector of a symmetric 3×3 matrix (cyclic Jacobi).
Vec3 smallestEigenvector(double a[3][3], double* smallest, double* largest) {
    double v[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    for (int sweep = 0; sweep < 30; ++sweep) {
        const double off = a[0][1] * a[0][1] + a[0][2] * a[0][2] + a[1][2] * a[1][2];
        if (off < 1e-30) {
            break;
        }
        for (int p = 0; p < 2; ++p) {
            for (int q = p + 1; q < 3; ++q) {
                if (std::fabs(a[p][q]) < 1e-300) {
                    continue;
                }
                const double theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
                const double t = (theta >= 0 ? 1.0 : -1.0) /
                                 (std::fabs(theta) + std::sqrt(theta * theta + 1.0));
                const double c = 1.0 / std::sqrt(t * t + 1.0);
                const double sn = t * c;
                for (int k = 0; k < 3; ++k) {
                    const double akp = a[k][p], akq = a[k][q];
                    a[k][p] = c * akp - sn * akq;
                    a[k][q] = sn * akp + c * akq;
                }
                for (int k = 0; k < 3; ++k) {
                    const double apk = a[p][k], aqk = a[q][k];
                    a[p][k] = c * apk - sn * aqk;
                    a[q][k] = sn * apk + c * aqk;
                }
                for (int k = 0; k < 3; ++k) {
                    const double vkp = v[k][p], vkq = v[k][q];
                    v[k][p] = c * vkp - sn * vkq;
                    v[k][q] = sn * vkp + c * vkq;
                }
            }
        }
    }
    int mi = 0, ma = 0;
    for (int i = 1; i < 3; ++i) {
        mi = a[i][i] < a[mi][mi] ? i : mi;
        ma = a[i][i] > a[ma][ma] ? i : ma;
    }
    *smallest = a[mi][mi];
    *largest = a[ma][ma];
    return Vec3{v[0][mi], v[1][mi], v[2][mi]};
}

// Solves the symmetric positive-definite 6×6 system A x = b (Cholesky).
bool solve6(double A[6][6], const double b[6], double x[6]) {
    double L[6][6] = {};
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j <= i; ++j) {
            double sum = A[i][j];
            for (int k = 0; k < j; ++k) {
                sum -= L[i][k] * L[j][k];
            }
            if (i == j) {
                if (sum <= 1e-18) {
                    return false;
                }
                L[i][i] = std::sqrt(sum);
            } else {
                L[i][j] = sum / L[j][j];
            }
        }
    }
    double y[6];
    for (int i = 0; i < 6; ++i) {
        double sum = b[i];
        for (int k = 0; k < i; ++k) {
            sum -= L[i][k] * y[k];
        }
        y[i] = sum / L[i][i];
    }
    for (int i = 5; i >= 0; --i) {
        double sum = y[i];
        for (int k = i + 1; k < 6; ++k) {
            sum -= L[k][i] * x[k];
        }
        x[i] = sum / L[i][i];
    }
    return true;
}

// Rigid transform from a small rotation vector w (Rodrigues) and translation t.
RigidTransform fromTwist(const double w[3], const double t[3]) {
    const double th = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
    double R[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    if (th > 1e-12) {
        const double kx = w[0] / th, ky = w[1] / th, kz = w[2] / th;
        const double c = std::cos(th), s = std::sin(th), C = 1.0 - c;
        R[0][0] = c + kx * kx * C;      R[0][1] = kx * ky * C - kz * s; R[0][2] = kx * kz * C + ky * s;
        R[1][0] = ky * kx * C + kz * s; R[1][1] = c + ky * ky * C;      R[1][2] = ky * kz * C - kx * s;
        R[2][0] = kz * kx * C - ky * s; R[2][1] = kz * ky * C + kx * s; R[2][2] = c + kz * kz * C;
    }
    RigidTransform out;
    out.m = {R[0][0], R[0][1], R[0][2], t[0], R[1][0], R[1][1], R[1][2], t[1],
             R[2][0], R[2][1], R[2][2], t[2], 0.0, 0.0, 0.0, 1.0};
    return out;
}

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

IcpResult icpPointToPlane(const pointcloud::PointCloud& source,
                          const pointcloud::PointCloud& target,
                          const RigidTransform& init, const PlaneIcpOptions& opts) {
    IcpResult result;
    result.transform = init;
    if (source.points.size() < opts.minCorrespondences ||
        target.points.size() < opts.minCorrespondences) {
        return result;
    }

    // Target normals from local PCA planes.
    std::vector<Vec3> normals(target.points.size());
    std::vector<uint8_t> hasNormal(target.points.size(), 0);
    {
        const SpatialHash nhash(target, opts.normalRadiusM);
        std::vector<int> nb;
        for (size_t i = 0; i < target.points.size(); ++i) {
            const auto& q = target.points[i];
            nhash.within(q.x, q.y, q.z, opts.normalRadiusM, nb);
            if (nb.size() < opts.minNormalNeighbours) {
                continue;
            }
            double mx = 0, my = 0, mz = 0;
            for (int j : nb) {
                const auto& p = target.points[static_cast<size_t>(j)];
                mx += p.x; my += p.y; mz += p.z;
            }
            const double inv = 1.0 / static_cast<double>(nb.size());
            mx *= inv; my *= inv; mz *= inv;
            double c[3][3] = {};
            for (int j : nb) {
                const auto& p = target.points[static_cast<size_t>(j)];
                const double d[3] = {p.x - mx, p.y - my, p.z - mz};
                for (int r = 0; r < 3; ++r) {
                    for (int k = 0; k < 3; ++k) {
                        c[r][k] += d[r] * d[k];
                    }
                }
            }
            double lmin = 0, lmax = 0;
            const Vec3 n = smallestEigenvector(c, &lmin, &lmax);
            if (lmax <= 0.0 || lmin > 0.2 * lmax) {
                continue; // no clear plane (edge, corner, noise)
            }
            normals[i] = n;
            hasNormal[i] = 1;
        }
    }

    const SpatialHash hash(target, opts.maxCorrespondenceDistM);
    const double maxDist = opts.maxCorrespondenceDistM;
    const double huber = std::max(1e-4, maxDist / 4.0);
    pointcloud::PointCloud working = transformed(source, init);

    for (int iter = 0; iter < opts.maxIterations; ++iter) {
        double A[6][6] = {};
        double b[6] = {};
        size_t used = 0;
        for (const auto& wp : working.points) {
            double sq = 0.0;
            const long idx = hash.nearest(wp.x, wp.y, wp.z, maxDist, &sq);
            if (idx < 0 || !hasNormal[static_cast<size_t>(idx)]) {
                continue;
            }
            const auto& q = target.points[static_cast<size_t>(idx)];
            const Vec3& n = normals[static_cast<size_t>(idx)];
            const double r = n.x * (wp.x - q.x) + n.y * (wp.y - q.y) + n.z * (wp.z - q.z);
            // J = [p × n, n]
            const double J[6] = {wp.y * n.z - wp.z * n.y, wp.z * n.x - wp.x * n.z,
                                 wp.x * n.y - wp.y * n.x, n.x, n.y, n.z};
            const double w = std::fabs(r) <= huber ? 1.0 : huber / std::fabs(r);
            for (int i = 0; i < 6; ++i) {
                b[i] -= w * J[i] * r;
                for (int j = 0; j <= i; ++j) {
                    A[i][j] += w * J[i] * J[j];
                }
            }
            ++used;
        }
        result.iterations = static_cast<size_t>(iter + 1);
        result.correspondences = used;
        if (used < opts.minCorrespondences) {
            break;
        }
        for (int i = 0; i < 6; ++i) {
            for (int j = i + 1; j < 6; ++j) {
                A[i][j] = A[j][i];
            }
            A[i][i] += 1e-9; // keep degenerate directions solvable
        }
        double x[6];
        if (!solve6(A, b, x)) {
            break;
        }
        const RigidTransform delta = fromTwist(x, x + 3);
        result.transform = delta * result.transform;
        working = transformed(source, result.transform);
        if (delta.translationNorm() < opts.convergenceTransEpsM &&
            delta.rotationAngle() < opts.convergenceRotEps) {
            result.converged = true;
            break;
        }
    }

    size_t inliers = 0;
    double sumSq = 0.0;
    for (const auto& wp : working.points) {
        double sq = 0.0;
        if (hash.nearest(wp.x, wp.y, wp.z, maxDist, &sq) >= 0) {
            ++inliers;
            sumSq += sq;
        }
    }
    result.fitness = static_cast<double>(inliers) / static_cast<double>(source.points.size());
    result.correspondences = inliers;
    result.rmseM = inliers > 0 ? std::sqrt(sumSq / static_cast<double>(inliers)) : 0.0;
    return result;
}

} // namespace revoscan::processing
