#include "localization/ukf.hpp"

#include <algorithm>
#include <cmath>

namespace loc {
namespace {

constexpr int N = 3;          // state size
constexpr int NS = 2 * N + 1; // sigma points

// Hand-written instead of Eigen: the biggest matrix is 8x8, and this keeps the robot binary
// small with no dependency. Matrices are square arrays of size S, using the top-left n x n.

// Lower-triangular L with L * L^T = A. False if A isn't positive definite.
template <int S> bool cholesky(const double (&A)[S][S], double (&L)[S][S], int n) {
    for (int i = 0; i < n; i++) {
        for (int j = 0; j <= i; j++) {
            double sum = A[i][j];
            for (int k = 0; k < j; k++) sum -= L[i][k] * L[j][k];
            if (i == j) {
                if (!(sum > 0)) return false;
                L[i][i] = std::sqrt(sum);
            } else {
                L[i][j] = sum / L[j][j];
            }
        }
        for (int j = i + 1; j < n; j++) L[i][j] = 0;
    }
    return true;
}

// Symmetrizes A, then adds the smallest diagonal jitter (1e-9, growing 10x) that lets Cholesky
// succeed. Same job as _ensure_positive_definite in the Python, without an eigen solver.
template <int S> void makePositiveDefinite(double (&A)[S][S], int n, double (&L)[S][S]) {
    for (int i = 0; i < n; i++)
        for (int j = i + 1; j < n; j++) A[i][j] = A[j][i] = 0.5 * (A[i][j] + A[j][i]);
    double add = 1e-9;
    for (int tries = 0; tries < 30 && !cholesky(A, L, n); tries++) {
        for (int i = 0; i < n; i++) A[i][i] += add;
        add *= 10;
    }
}

template <int S> void makePositiveDefinite(double (&A)[S][S], int n) {
    double L[S][S];
    makePositiveDefinite(A, n, L);
}

double circularMean(const double* angles, const double* weights, int count) {
    double s = 0, c = 0;
    for (int i = 0; i < count; i++) {
        s += weights[i] * std::sin(angles[i]);
        c += weights[i] * std::cos(angles[i]);
    }
    if (std::fabs(s) < 1e-15 && std::fabs(c) < 1e-15) return angles[0];
    return std::atan2(s, c);
}

} // namespace

Ukf::Ukf(const Params& params) : p_(params) {
    // Scaled unscented transform, floored so n + lambda never gets near zero.
    lambda_ = p_.alpha * p_.alpha * (N + p_.kappa) - N;
    if (N + lambda_ < 1e-3) lambda_ = 1e-3 - N;
    for (int i = 0; i < NS; i++) wm_[i] = wc_[i] = 1.0 / (2.0 * (N + lambda_));
    wm_[0] = lambda_ / (N + lambda_);
    wc_[0] = lambda_ / (N + lambda_) + (1.0 - p_.alpha * p_.alpha + p_.beta);
}

void Ukf::setMounts(const BodyMount* mounts, int count) {
    mountCount_ = std::min(count, kMaxChannels);
    for (int i = 0; i < mountCount_; i++) mounts_[i] = mounts[i];
}

void Ukf::reset(double x, double y, double theta, const double P[3][3]) {
    x_[0] = x;
    x_[1] = y;
    x_[2] = normalizeAngle(theta);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) P_[i][j] = P[i][j];
    makePositiveDefinite(P_, 3);
    clampToField();
}

void Ukf::clampToField() {
    x_[0] = std::clamp(x_[0], -p_.positionBound, p_.positionBound);
    x_[1] = std::clamp(x_[1], -p_.positionBound, p_.positionBound);
}

void Ukf::sigmaPoints(double out[7][3]) const {
    double A[3][3], L[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) A[i][j] = (N + lambda_) * P_[i][j];
    makePositiveDefinite(A, 3, L);
    for (int k = 0; k < 3; k++) out[0][k] = x_[k];
    for (int i = 0; i < N; i++) {
        for (int k = 0; k < 3; k++) {
            out[i + 1][k] = x_[k] + L[k][i];
            out[i + 1 + N][k] = x_[k] - L[k][i];
        }
    }
}

RayHit Ukf::expected(const double pose[3], int mount) const {
    RayHit hit = raycast(pose[0], pose[1], pose[2], mounts_[mount], p_.fieldHalf);
    if (p_.beamHalfAngleRad > 0)
        hit.distance *= std::cos(hit.incidence) / std::cos(std::max(0.0, hit.incidence - p_.beamHalfAngleRad));
    return hit;
}

void Ukf::predict(double forward, double lateral, double dTheta, double sigmaForward, double sigmaLateral,
                  double sigmaTheta) {
    double X[NS][3];
    sigmaPoints(X);

    // Tracking wheels measure arc length in the body frame while it rotates, so the move is an arc,
    // not a straight line along the starting heading. For constant body velocity over the step:
    //   local = [[sin(d)/d, -(1-cos(d))/d], [(1-cos(d))/d, sin(d)/d]] * [forward, lateral]
    double a, b;
    if (std::fabs(dTheta) < 1e-4) {
        a = 1 - dTheta * dTheta / 6;
        b = dTheta / 2;
    } else {
        a = std::sin(dTheta) / dTheta;
        b = (1 - std::cos(dTheta)) / dTheta;
    }
    double localX = a * forward - b * lateral;
    double localY = b * forward + a * lateral;

    double angles[NS];
    for (int i = 0; i < NS; i++) {
        double theta = normalizeAngle(X[i][2]);
        double c = std::cos(theta), s = std::sin(theta);
        X[i][0] += c * localX - s * localY;
        X[i][1] += s * localX + c * localY;
        X[i][2] = normalizeAngle(theta + dTheta);
        angles[i] = X[i][2];
    }

    double mean[3] = {0, 0, circularMean(angles, wm_, NS)};
    for (int i = 0; i < NS; i++) {
        mean[0] += wm_[i] * X[i][0];
        mean[1] += wm_[i] * X[i][1];
    }

    double P[3][3] = {};
    for (int i = 0; i < NS; i++) {
        double d[3] = {X[i][0] - mean[0], X[i][1] - mean[1], normalizeAngle(X[i][2] - mean[2])};
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++) P[r][c] += wc_[i] * d[r] * d[c];
    }

    // Process noise is set in the body frame (forward and sideways errors differ) and rotated
    // into the world at the heading the move started from.
    double c = std::cos(x_[2]), s = std::sin(x_[2]);
    double vf = std::max(sigmaForward * sigmaForward, 1e-12);
    double vl = std::max(sigmaLateral * sigmaLateral, 1e-12);
    P[0][0] += c * c * vf + s * s * vl;
    P[1][1] += s * s * vf + c * c * vl;
    P[0][1] += c * s * (vf - vl);
    P[1][0] += c * s * (vf - vl);
    P[2][2] += std::max(sigmaTheta * sigmaTheta, 1e-12);
    makePositiveDefinite(P, 3);

    for (int k = 0; k < 3; k++) x_[k] = mean[k];
    for (int r = 0; r < 3; r++)
        for (int c2 = 0; c2 < 3; c2++) P_[r][c2] = P[r][c2];
    clampToField();
}

UpdateStats Ukf::update(const Measurement* z, int count) {
    UpdateStats stats;
    count = std::min(count, kMaxChannels);
    stats.offered = count;
    if (count == 0) return stats;

    // Steep beams: the V5 sensor's cone reads short when it hits a wall at an angle, so skip
    // those. Decided at the current estimate, before the expensive part.
    int active[kMaxChannels];
    int m = 0;
    for (int j = 0; j < count; j++) {
        RayHit hit = expected(x_, z[j].mount);
        stats.predicted[j] = hit.distance;
        if (hit.incidence > p_.maxIncidenceRad) {
            stats.verdict[j] = Verdict::Steep;
            stats.steep++;
        } else {
            active[m++] = j;
        }
    }
    if (m == 0) return stats;

    double X[NS][3];
    sigmaPoints(X);

    double Z[NS][kMaxChannels];
    for (int i = 0; i < NS; i++)
        for (int k = 0; k < m; k++) Z[i][k] = expected(X[i], z[active[k]].mount).distance;

    double zMean[kMaxChannels] = {};
    for (int i = 0; i < NS; i++)
        for (int k = 0; k < m; k++) zMean[k] += wm_[i] * Z[i][k];

    double Pzz[kMaxChannels][kMaxChannels] = {};
    double Pxz[3][kMaxChannels] = {};
    for (int i = 0; i < NS; i++) {
        double dx[3] = {X[i][0] - x_[0], X[i][1] - x_[1], normalizeAngle(X[i][2] - x_[2])};
        double dz[kMaxChannels];
        for (int k = 0; k < m; k++) dz[k] = Z[i][k] - zMean[k];
        for (int r = 0; r < m; r++) {
            for (int c = 0; c < m; c++) Pzz[r][c] += wc_[i] * dz[r] * dz[c];
        }
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < m; c++) Pxz[r][c] += wc_[i] * dx[r] * dz[c];
    }
    for (int k = 0; k < m; k++) {
        double sigma = z[active[k]].sigma;
        Pzz[k][k] += std::max(sigma * sigma, 1e-4);
    }
    makePositiveDefinite(Pzz, m);

    // Gate each reading on its own. The statistics above don't depend on which other readings
    // are included, so dropping one just means using the matching rows and columns.
    double innovation[kMaxChannels];
    int keep[kMaxChannels];
    int k2 = 0;
    for (int k = 0; k < m; k++) {
        int j = active[k];
        innovation[k] = z[j].range - zMean[k];
        stats.nis[j] = innovation[k] * innovation[k] / Pzz[k][k];
        if (p_.gateSigmas > 0 && stats.nis[j] > p_.gateSigmas * p_.gateSigmas) {
            stats.verdict[j] = Verdict::Gated;
            stats.gated++;
        } else {
            stats.verdict[j] = Verdict::Used;
            stats.used++;
            keep[k2++] = k;
        }
    }
    if (k2 == 0) return stats;

    double S[kMaxChannels][kMaxChannels], L[kMaxChannels][kMaxChannels];
    for (int r = 0; r < k2; r++)
        for (int c = 0; c < k2; c++) S[r][c] = Pzz[keep[r]][keep[c]];
    makePositiveDefinite(S, k2, L);

    // K = Pxz * S^-1, solved with the Cholesky factor (never invert S). Row by row: S * k^T = pxz^T.
    double K[3][kMaxChannels];
    for (int r = 0; r < 3; r++) {
        double y[kMaxChannels];
        for (int i = 0; i < k2; i++) {
            double sum = Pxz[r][keep[i]];
            for (int j = 0; j < i; j++) sum -= L[i][j] * y[j];
            y[i] = sum / L[i][i];
        }
        for (int i = k2 - 1; i >= 0; i--) {
            double sum = y[i];
            for (int j = i + 1; j < k2; j++) sum -= L[j][i] * K[r][j];
            K[r][i] = sum / L[i][i];
        }
    }

    for (int r = 0; r < 3; r++)
        for (int i = 0; i < k2; i++) x_[r] += K[r][i] * innovation[keep[i]];
    x_[2] = normalizeAngle(x_[2]);
    clampToField();

    // P -= K S K^T
    double KS[3][kMaxChannels] = {};
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < k2; c++)
            for (int i = 0; i < k2; i++) KS[r][c] += K[r][i] * S[i][c];
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            for (int i = 0; i < k2; i++) P_[r][c] -= KS[r][i] * K[c][i];
    makePositiveDefinite(P_, 3);
    return stats;
}

void Ukf::inflate(double positionFactor, double headingFactor) {
    double f[3] = {std::sqrt(positionFactor), std::sqrt(positionFactor), std::sqrt(headingFactor)};
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++) P_[r][c] *= f[r] * f[c];
}

} // namespace loc
