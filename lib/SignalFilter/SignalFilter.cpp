// ============================================================================
// SignalFilter.cpp — Alpha-Beta Filter Implementation
// Cursr-V Antenna Tracker
// ============================================================================
#include "SignalFilter.h"

AlphaBetaFilter::AlphaBetaFilter(float alpha, float beta, float dt)
    : _alpha(alpha)
    , _beta(beta)
    , _dt(dt)
    , _xk(0.0f)
    , _vk(0.0f)
    , _initialised(false)
{}

void AlphaBetaFilter::reset() {
    _xk = 0.0f;
    _vk = 0.0f;
    _initialised = false;
}

float AlphaBetaFilter::update(float measurement) {
    if (!_initialised) {
        // First sample — accept as ground truth, velocity unknown.
        _xk = measurement;
        _vk = 0.0f;
        _initialised = true;
        return _xk;
    }

    // ---- Prediction step ----
    float xp = _xk + _vk * _dt;         // predicted position
    // velocity prediction is unchanged (constant-velocity model)

    // ---- Correction step ----
    float residual = measurement - xp;   // innovation / prediction error

    _xk = xp + _alpha * residual;        // corrected position
    _vk = _vk + (_beta / _dt) * residual; // corrected velocity

    return _xk;
}
