#pragma once

#include "..\common\stdafx.h"
#include <Kinect.h>
#include <vector>
#include <algorithm>

// A single sampled joint (camera space, meters)
struct KSampleJoint
{
	float position[3];        // X, Y, Z in meters (Kinect camera space)
	float orientation[4];     // Quaternion (X, Y, Z, W)
	int   trackingState;      // TrackingState enum
};

// A sampled body inside a recorded frame
struct KSampleBody
{
	UINT64 trackingId;
	KSampleJoint joints[JointType_Count];
	bool isTracked;
};

// A recorded frame, containing every tracked body at that moment
struct KSampleFrame
{
	INT64 timeMS;                         // Relative time, in milliseconds
	std::vector<KSampleBody> bodies;      // Tracked bodies
};

/*
 In-memory buffer that stores everything recorded so far.
 It feeds the timeline, the playback and the FBX exporter.
*/
class KRecording
{
public:
	// Floor reference captured at recording start — used as world origin for
	// bake/export so the animation stays valid even if the Kinect moves later.
	bool  hasFloor = false;
	float floorPlane[4] = { 0.0f, 1.0f, 0.0f, 0.0f }; // n·p + w = 0, normalized

	/// <summary>
	/// Clears the recording
	/// </summary>
	void clear() { frames.clear(); hasFloor = false; floorPlane[0]=0; floorPlane[1]=1; floorPlane[2]=0; floorPlane[3]=0; }

	/// <summary>
	/// Returns true if nothing has been recorded
	/// </summary>
	bool empty() const { return frames.empty(); }

	/// <summary>
	/// Number of recorded frames
	/// </summary>
	size_t size() const { return frames.size(); }

	/// <summary>
	/// Start time of the take, in milliseconds
	/// </summary>
	INT64 startTimeMS() const { return frames.empty() ? 0 : frames.front().timeMS; }

	/// <summary>
	/// Total duration of the take, in milliseconds
	/// </summary>
	INT64 durationMS() const
	{
		return frames.empty() ? 0 : (frames.back().timeMS - frames.front().timeMS);
	}

	/// <summary>
	/// Returns the index of the frame returned by frameAt() (-1 when empty).
	/// Needed by the node graph (temporal Smooth window) to locate neighbours.
	/// </summary>
	int frameIndexAt(INT64 relTimeMS) const
	{
		if (frames.empty())
			return -1;

		INT64 base = startTimeMS();

		if (relTimeMS < 0) relTimeMS = 0;
		if (relTimeMS > durationMS()) relTimeMS = durationMS();

		INT64 target = base + relTimeMS;

		// Binary search for the last frame with time <= target
		size_t lo = 0, hi = frames.size();
		while (lo < hi)
		{
			size_t mid = lo + (hi - lo) / 2;
			if (frames[mid].timeMS <= target)
				lo = mid + 1;
			else
				hi = mid;
		}

		if (lo == 0)
			return 0;

		return (int)(lo - 1);
	}

	/// <summary>
	/// Returns the frame closest to the given relative time (in ms).
	/// The frame at or right before the requested time is returned.
	/// Returns nullptr when out of range.
	/// </summary>
	const KSampleFrame* frameAt(INT64 relTimeMS) const
	{
		int i = frameIndexAt(relTimeMS);
		if (i < 0)
			return nullptr;
		return &frames[i];
	}

	// All recorded frames, in chronological order
	std::vector<KSampleFrame> frames;
};
