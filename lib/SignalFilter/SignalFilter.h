// ============================================================================
// SignalFilter.h — Alpha-Beta Filter for Altitude Smoothing
// Cursr-V Antenna Tracker
//
// A lightweight predictor-corrector filter suitable for 50-100 Hz telemetry.
// Provides smoothed position and estimated velocity with O(1) computation.
// ============================================================================
#ifndef SIGNAL_FILTER_H
#define SIGNAL_FILTER_H

class AlphaBetaFilter {
public:
    /// @param alpha  Position smoothing gain (0 < alpha ≤ 1). Higher = more responsive.
    /// @param beta   Velocity smoothing gain (0 < beta  ≤ 1). Higher = faster velocity adaptation.
    /// @param dt     Expected time step between samples (seconds), e.g. 0.01 for 100 Hz.
    AlphaBetaFilter(float alpha = 0.85f, float beta = 0.005f, float dt = 0.01f);

    /// Reset internal state. Next update() will re-initialise from the measurement.
    void reset();

    /// Feed a new raw measurement and return the smoothed estimate.
    float update(float measurement);

    /// Most recent smoothed position estimate.
    float getPosition() const;

    /// Most recent estimated velocity (units/s).
    float getVelocity() const;

private:
    float _alpha;
    float _beta;
    float _dt;

    float _xk;   // smoothed position
    float _vk;   // estimated velocity
    bool  _initialised;
};

#endif // SIGNAL_FILTER_H
