#include "OneEuroFilter.h"

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/// <summary>
/// Constructor
/// </summary>
OneEuroFilter::OneEuroFilter(float minCutoff, float beta, float dCutoff) :
	m_minCutoff(minCutoff),
	m_beta(beta),
	m_dCutoff(dCutoff),
	m_xPrev(0.0f),
	m_dxPrev(0.0f),
	m_hasPrev(false)
{
}

/// <summary>
/// Sets the filter parameters
/// </summary>
void OneEuroFilter::setParameters(float minCutoff, float beta, float dCutoff)
{
	m_minCutoff = minCutoff;
	m_beta = beta;
	m_dCutoff = dCutoff;
	reset();
}

/// <summary>
/// Resets the internal state of the filter
/// </summary>
void OneEuroFilter::reset()
{
	m_hasPrev = false;
	m_xPrev = 0.0f;
	m_dxPrev = 0.0f;
}

/// <summary>
/// Computes the smoothing factor for a given cutoff frequency
/// </summary>
float OneEuroFilter::alpha(float cutoff, float dt)
{
	float tau = 1.0f / (2.0f * float(M_PI) * cutoff);
	return 1.0f / (1.0f + tau / dt);
}

/// <summary>
/// Simple exponential low pass filter
/// </summary>
float OneEuroFilter::lowpass(float value, float alpha)
{
	return alpha * value + (1.0f - alpha) * m_xPrev;
}

/// <summary>
/// Filters a value. dt is expressed in seconds
/// </summary>
float OneEuroFilter::filter(float value, float dt)
{
	if (dt <= 0.0f)
		dt = 0.001f;

	// Estimate the derivate
	float dx = (m_hasPrev) ? (value - m_xPrev) / dt : 0.0f;

	// Cutoff adapts to the speed of the movement
	float edx = m_hasPrev ? lowpass(dx, alpha(m_dCutoff, dt)) : dx;
	float cutoff = m_minCutoff + m_beta * std::fabs(edx);

	// Smooth the value
	float xOut = m_hasPrev ? lowpass(value, alpha(cutoff, dt)) : value;

	m_xPrev = xOut;
	m_dxPrev = edx;
	m_hasPrev = true;

	return xOut;
}

/// <summary>
/// Constructor
/// </summary>
KBodySmoother::KBodySmoother() :
	m_tick(0),
	m_strength(0.0f),
	m_minCutoff(90.0f)
{
}

float KBodySmoother::strength() const
{
	std::lock_guard<std::recursive_mutex> lk(m_stateMutex);
	return m_strength;
}

/// <summary>
/// Sets the smoothing strength in the 0..100 range.
/// </summary>
void KBodySmoother::setStrength(float strength)
{
	if (strength < 0.0f) strength = 0.0f;
	if (strength > 100.0f) strength = 100.0f;

	{
		std::lock_guard<std::recursive_mutex> lk(m_stateMutex);
		m_strength = strength;

		// 0  -> minCutoff 90   (almost no smoothing)
		// 100 -> minCutoff 0.5 (very smooth, adds latency)
		float t = strength / 100.0f;
		m_minCutoff = 90.0f + (0.5f - 90.0f) * t;
	}

	reset();
}

/// <summary>
/// Forgets everything, including every tracked body state
/// </summary>
void KBodySmoother::reset()
{
	std::lock_guard<std::recursive_mutex> lk(m_stateMutex);
	m_bodyStates.clear();
}

/// <summary>
/// Smooths the given body joints. The arrays are modified in place.
/// </summary>
void KBodySmoother::smooth(UINT64 trackingId, Joint *joints, float dt)
{
	std::lock_guard<std::recursive_mutex> lk(m_stateMutex);

	BodyState &state = m_bodyStates[trackingId];
	state.lastSeenTick = ++m_tick;
	pruneStaleStates();

	for (int j = 0; j < JointType_Count; ++j)
	{
		Joint &jnt = joints[j];

		if (jnt.TrackingState == TrackingState_NotTracked)
		{
			// Do not smooth lost joints, reset their filter state
			for (int c = 0; c < 3; ++c)
				state.posFilters[j][c].reset();
			continue;
		}

		float x = state.posFilters[j][0].filter(jnt.Position.X, dt);
		float y = state.posFilters[j][1].filter(jnt.Position.Y, dt);
		float z = state.posFilters[j][2].filter(jnt.Position.Z, dt);

		jnt.Position.X = x;
		jnt.Position.Y = y;
		jnt.Position.Z = z;
	}
}

/// <summary>
/// Drops the least recently seen body state once the map outgrows the number
/// of bodies Kinect can track at once. Tracking ids are fresh values every
/// time a body is lost and found again, so without this the map would keep a
/// full set of filters (95 per body) alive for the whole life of the process.
/// </summary>
void KBodySmoother::pruneStaleStates()
{
	const size_t kMaxStates = 16;
	if (m_bodyStates.size() <= kMaxStates)
		return;

	auto oldest = m_bodyStates.end();
	for (auto it = m_bodyStates.begin(); it != m_bodyStates.end(); ++it)
	{
		if (oldest == m_bodyStates.end() || it->second.lastSeenTick < oldest->second.lastSeenTick)
			oldest = it;
	}
	if (oldest != m_bodyStates.end())
		m_bodyStates.erase(oldest);
}

/// <summary>
/// Smooths the given body orientations. The arrays are modified in place.
/// Quaternions are smoothed per component. The current quaternion is first
/// moved to the same hemisphere as the previous one to keep the
/// interpolation path continuous.
/// </summary>
void KBodySmoother::smoothOrientations(UINT64 trackingId, JointOrientation *orientations, float dt)
{
	std::lock_guard<std::recursive_mutex> lk(m_stateMutex);

	BodyState &state = m_bodyStates[trackingId];
	state.lastSeenTick = ++m_tick;
	pruneStaleStates();

	for (int j = 0; j < JointType_Count; ++j)
	{
		JointOrientation &ori = orientations[j];

		bool isNull = ori.Orientation.x == 0.0f &&
			ori.Orientation.y == 0.0f &&
			ori.Orientation.z == 0.0f &&
			ori.Orientation.w == 0.0f;

		if (isNull)
		{
			for (int c = 0; c < 4; ++c)
				state.oriFilters[j][c].reset();
			continue;
		}

		float comp[4] = { ori.Orientation.x, ori.Orientation.y, ori.Orientation.z, ori.Orientation.w };

		// Keep the quaternion on the same hemisphere as the previous frame
		if (state.oriFilters[j][0].hasValue())
		{
			float dot = 0.0f;
			for (int c = 0; c < 4; ++c)
				dot += comp[c] * state.oriFilters[j][c].peek();

			if (dot < 0.0f)
			{
				for (int c = 0; c < 4; ++c)
					comp[c] = -comp[c];
			}
		}

		for (int c = 0; c < 4; ++c)
			comp[c] = state.oriFilters[j][c].filter(comp[c], dt);

		// Renormalise — per-component filtering breaks unit length
		float len2 = comp[0]*comp[0] + comp[1]*comp[1] + comp[2]*comp[2] + comp[3]*comp[3];
		if (len2 > 1e-8f) { float inv = 1.0f / sqrtf(len2); for (int c = 0; c < 4; ++c) comp[c] *= inv; }

		ori.Orientation.x = comp[0];
		ori.Orientation.y = comp[1];
		ori.Orientation.z = comp[2];
		ori.Orientation.w = comp[3];
	}
}
