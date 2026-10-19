#pragma once

#include "..\common\stdafx.h"

#include <atomic>

/*
 Global application session state, shared between the UI thread and the
 rendering code. Only written on the UI thread.

 livePaused and gestureRecord are also read by the capture thread (from
 KBodyVisualizer::notify), so they are atomic to avoid a torn/stale read.
*/
struct AppSession
{
	std::atomic<bool> livePaused;  // Freeze the live preview
	bool  playing;         // Playback of the recorded take is running
	bool  loop;            // Loop the playback
	bool  ghost;           // Show a ghost reference pose
	bool  ghostSymmetry;   // Mirror ghost-pose edits across the sagittal plane
	bool  recording;       // Currently recording (UI state only)
	float smoothStrength;  // 0..100
	INT64 playheadMS;      // Playback cursor, relative to the take
	INT64 takeDurationMS;  // Duration of the recorded take
	INT64 playInMS;        // Trim start (relative ms), valid when hasRange
	INT64 playOutMS;       // Trim end (relative ms), valid when hasRange
	bool  hasRange;        // In/out trim is active
	bool  depthView;       // Show the depth/IR point cloud in the viewport
	bool  cameraView;      // Show the Kinect as a camera frustum
	bool  floorAlign;      // Align the grid to the detected floor
	std::atomic<bool> gestureRecord; // Start/stop recording with both-fists gesture
	int   workspace;       // 0 = Motion Capture, 1 = Retarget
	bool  exportBVH;       // Export the retargeted take as BVH instead of FBX
	bool  showRetargetLive; // Show retarget mesh in Motion Capture live view
	bool  pipetteActive;   // Eyedropper picking active
	int   pipetteJoint;    // Joint selected for pipette (-1 none)
	int   pipetteBone;     // Bone selected for pipette (-1 none)
};

// Global session state (defined in UI.cpp)
extern AppSession g_session;

// Global pointer to the recording exporter (defined in UI.cpp)
class KBodyExporter;
extern KBodyExporter *g_pExporter;

// Global pointer to the retarget engine (defined in UI.cpp)
class KRetargeter;
extern KRetargeter *g_retargeter;
