#pragma once

#include "..\common\stdafx.h"
#include <Kinect.h>
#include <map>
#include <mutex>

/*
 One Euro Filter - adaptive, low latency smoothing filter.
 It removes the jitter of the Kinect stream while keeping the
 motion responsive (the cutoff frequency adapts to the speed of
 the movement). See "One Euro Filter" - G. Casiez, N. Roussel, D. Vogel.
*/
class OneEuroFilter
{
public:
	OneEuroFilter(float minCutoff = 1.0f, float beta = 0.0f, float dCutoff = 1.0f);

	/// <summary>
	/// Sets the filter parameters
	/// </summary>
	void setParameters(float minCutoff, float beta, float dCutoff);

	/// <summary>
	/// Resets the internal state of the filter
	/// </summary>
	void reset();

	/// <summary>
	/// Filters a value. dt is expressed in seconds
	/// </summary>
	float filter(float value, float dt);

	/// <summary>
	/// Returns the last filtered value (or 0 if the filter has never been fed)
	/// </summary>
	float peek() const { return m_xPrev; }

	/// <summary>
	/// Returns true if the filter has already received a value
	/// </summary>
	bool hasValue() const { return m_hasPrev; }

private:
	float lowpass(float value, float alpha);
	float alpha(float cutoff, float dt);

	float m_minCutoff;
	float m_beta;
	float m_dCutoff;

	float m_xPrev;
	float m_dxPrev;
	bool  m_hasPrev;
};

/*
 Smooths the whole skeleton (joint positions + orientations).
 One state per Kinect body tracking id, so that several people
 can be tracked at the same time without mixing their history.
*/
class KBodySmoother
{
public:
	/// <summary>
	/// Constructor
	/// </summary>
	KBodySmoother();

	/// <summary>
	/// Sets the smoothing strength in the 0..100 range.
	/// 0 = raw stream, 100 = very smooth (with more latency)
	/// </summary>
	void setStrength(float strength);

	/// <summary>
	/// Current smoothing strength (0..100)
	/// </summary>
	float strength() const;

	/// <summary>
	/// Forgets everything, including every tracked body state
	/// </summary>
	void reset();

	/// <summary>
	/// Smooths the given body joints. The arrays are modified in place.
	/// </summary>
	/// <param name="trackingId">Kinect body tracking id</param>
	/// <param name="joints">In/out joint array</param>
	/// <param name="dt">Elapsed time since last processed frame, in seconds</param>
	void smooth(UINT64 trackingId, Joint *joints, float dt);

	/// <summary>
	/// Smooths the given body orientations. The arrays are modified in place.
	/// </summary>
	/// <param name="trackingId">Kinect body tracking id</param>
	/// <param name="orientations">In/out joint orientation array</param>
	/// <param name="dt">Elapsed time since last processed frame, in seconds</param>
	void smoothOrientations(UINT64 trackingId, JointOrientation *orientations, float dt);

private:
	// Drops the least recently seen body state once the map outgrows the
	// number of bodies Kinect can track. Must be called with m_stateMutex held.
	void pruneStaleStates();

	struct BodyState
	{
		OneEuroFilter posFilters[JointType_Count][3];
		OneEuroFilter oriFilters[JointType_Count][4];
		UINT64 lastSeenTick = 0;
	};

	// One state per body. reset() runs on the UI thread while smooth() runs
	// on the capture thread, so every access is serialised by m_stateMutex.
	std::map<UINT64, BodyState> m_bodyStates;
	mutable std::recursive_mutex m_stateMutex;
	UINT64 m_tick;

	float m_strength;
	float m_minCutoff;
};
