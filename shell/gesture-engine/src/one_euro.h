// one_euro.h — One-Euro filter (Casiez et al. 2012) for joint positions.
// Internal header; not part of the public API.
//
// Applied per joint axis before feature extraction so ±10 mm tracker jitter
// stops flapping trigger/release thresholds, while the speed-adaptive cutoff
// keeps intentional motion (0.5–2 m/s pinch approaches) near-lagless.

#pragma once

#include <cmath>

namespace ge {

struct OneEuroParams {
    float min_cutoff_hz = 1.5f;  // cutoff at rest — governs jitter rejection
    float beta = 30.0f;          // cutoff gain per m/s — governs motion lag
    float d_cutoff_hz = 1.0f;    // derivative smoothing cutoff
};

class OneEuroAxis {
   public:
    float update(float x, float dt, const OneEuroParams &p) {
        if (dt <= 0.0f) return has_ ? x_ : x;
        if (!has_) {
            has_ = true;
            x_ = x;
            dx_ = 0.0f;
            return x;
        }
        float raw_dx = (x - x_) / dt;
        dx_ += smoothing_alpha(dt, p.d_cutoff_hz) * (raw_dx - dx_);
        float cutoff = p.min_cutoff_hz + p.beta * std::fabs(dx_);
        x_ += smoothing_alpha(dt, cutoff) * (x - x_);
        return x_;
    }

    void reset() { has_ = false; }

   private:
    static float smoothing_alpha(float dt, float cutoff_hz) {
        float tau = 1.0f / (2.0f * static_cast<float>(M_PI) * cutoff_hz);
        return 1.0f / (1.0f + tau / dt);
    }

    bool has_ = false;
    float x_ = 0.0f;
    float dx_ = 0.0f;
};

}  // namespace ge
