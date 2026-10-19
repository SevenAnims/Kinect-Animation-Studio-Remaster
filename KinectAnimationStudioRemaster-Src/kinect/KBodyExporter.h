#pragma once


#include "..\common\stdafx.h"
#include "KBodyReader.h"
#include "KRecording.h"
#include "OneEuroFilter.h"

#include <atomic>
#include <mutex>

class KBodyExporter : public KBodyReader {
public:

	/// <summary>
	/// Constructor
	/// </summary>
	KBodyExporter(IKinectSensor *kSensor = NULL, FbxManager *fManager = NULL);

	/// <summary>
	/// Destructor
	/// </summary>
	~KBodyExporter();


	/// <summary>
	/// Sets SDK Manager
	/// </summary>
	/// <param name="fManager">SDK manager to be set</param>
	void initFBXSDKManager(FbxManager *fManager) {
		m_pFBXManager = fManager;
	}

	/// <summary>
	/// Starts recording Skeleton Data. Everything recorded is kept in memory
	/// so the user can preview it before exporting.
	/// </summary>
	void startRecording();

	/// <summary>
	/// Stops recording Skeleton Data. The recorded take stays in memory.
	/// </summary>
	void stopRecording();

	/// <summary>
	/// Returns whether we are currently recording the skeletons
	/// </summary>
	bool recordingStatus()  { return m_pIsRecording; };

	/// <summary>
	/// Notify class about a frame that arrived
	/// </summary>
	/// <param name="bFrame">Incoming frame</param>
	/// <param name="frameTime">Frame timestamp</param>
	virtual void notify(IBodyFrame *bFrame, INT64 frameTime);

	/// <summary>
	/// Exports the whole recorded take to the given file.
	/// Returns true on success.
	/// </summary>
	/// <param name="fileName">Output file name</param>
	bool exportTake(const char *fileName);

	/// <summary>
	/// Exports only the frames inside [inMS, outMS] (relative ms) to the given file.
	/// Returns true on success.
	/// </summary>
	/// <param name="fileName">Output file name</param>
	/// <param name="inMS">Start of the exported range, in relative ms</param>
	/// <param name="outMS">End of the exported range, in relative ms</param>
	bool exportTake(const char *fileName, INT64 inMS, INT64 outMS);

	/// <summary>
	/// Removes the last 'ms' milliseconds from the recorded take. Used after a
	/// gesture stop to drop the frames polluted by the countdown stop pose.
	/// </summary>
	/// <param name="ms">Number of milliseconds to remove from the end</param>
	void trimRecordingEnd(INT64 ms);

	/// <summary>
	/// Clears the recorded take
	/// </summary>
	void clearTake();

	/// <summary>
	/// UI thread only: moves the frames captured on the recording thread into
	/// the take. m_recording is read by every paint (timeline ticks, duration
	/// label, viewport), so it must only ever be written from the thread that
	/// reads it - appending from the capture thread could reallocate the frame
	/// array right in the middle of such a read.
	/// Called from the 33 ms UI timer, from stopRecording(), and by every
	/// accessor below so no caller can ever see a half-transferred take.
	/// </summary>
	void flushPendingFrames() const;

	/// <summary>
	/// Returns a reference to the current recording buffer
	/// </summary>
	const KRecording& recording() const { flushPendingFrames(); return m_recording; }

	/// <summary>
	/// Replaces the current recording buffer (used when a project is loaded)
	/// </summary>
	void setRecording(const KRecording &rec) {
		{ std::lock_guard<std::mutex> lk(m_recMutex); m_pendingFrames.clear(); }
		m_recording = rec;
		m_hasFloorRef = rec.hasFloor;
		if (rec.hasFloor) { m_floorRefPlane[0]=rec.floorPlane[0]; m_floorRefPlane[1]=rec.floorPlane[1]; m_floorRefPlane[2]=rec.floorPlane[2]; m_floorRefPlane[3]=rec.floorPlane[3]; }
		else { m_hasFloorRef=false; m_floorRefPlane[0]=0; m_floorRefPlane[1]=1; m_floorRefPlane[2]=0; m_floorRefPlane[3]=0; }
	}

	/// <summary>
	/// Thread-safe copy of the frame closest to the given relative time (in ms).
	/// Returns false when the take is empty.
	/// </summary>
	bool getFrameCopy(INT64 relTimeMS, KSampleFrame &out) const
	{
		flushPendingFrames();
		if (m_recording.empty())
			return false;

		const KSampleFrame *f = m_recording.frameAt(relTimeMS);
		if (!f)
			return false;

		out = *f;
		return true;
	}

	/// <summary>
	/// Duration of the recorded take, in milliseconds
	/// </summary>
	INT64 takeDurationMS() const { flushPendingFrames(); return m_recording.durationMS(); }

	/// <summary>
	/// Number of recorded frames
	/// </summary>
	size_t recordedFrameCount() const { flushPendingFrames(); return m_recording.size(); }

	/// <summary>
	/// Sets the smoothing strength (0..100) applied while recording
	/// </summary>
	void setSmoothing(float strength) { m_smoother.setStrength(strength); }

	/// <summary>
	/// Returns true when at least one frame has been recorded
	/// </summary>
	bool hasTake() const { flushPendingFrames(); return !m_recording.empty(); }

private:

	// Constants
	const char *c_FBXBinaryFileDesc = "FBX binary (*.fbx)";

	// Variables

	std::atomic_bool m_pIsRecording;

	// FBX SDK Manager
	FbxManager *m_pFBXManager;

	// Recording buffer. Written from the UI thread only (see
	// flushPendingFrames); the capture thread stages into m_pendingFrames.
	// mutable because the read accessors flush before handing it out.
	mutable KRecording            m_recording;

	// Frames staged by the recording thread, and the lock that serialises
	// them against the UI thread's flush.
	mutable std::mutex                 m_recMutex;
	mutable std::vector<KSampleFrame>  m_pendingFrames;

	// Smoother applied to incoming frames
	KBodySmoother m_smoother;

	// Absolute time (ms) at which the current take started
	INT64 m_takeStartAbsoluteMS;

	// Absolute time (ms) of the last processed frame, used for the dt of the filter
	INT64 m_lastFrameTimeMS;

	// Floor reference for this take — captured from the first valid FloorClipPlane
	bool  m_hasFloorRef = false;
	float m_floorRefPlane[4] = { 0.0f, 1.0f, 0.0f, 0.0f };

	/// <summary>
	/// Builds an FBX scene from the recorded buffer and saves it.
	/// </summary>
	/// <param name="inMS">Start of the exported range, -1 = whole take</param>
	/// <param name="outMS">End of the exported range, -1 = whole take</param>
	bool exportTakeToFile(const char *fileName, INT64 inMS = -1, INT64 outMS = -1);

};
