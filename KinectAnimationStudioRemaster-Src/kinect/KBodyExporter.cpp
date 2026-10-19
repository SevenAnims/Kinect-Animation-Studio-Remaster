#include "KBodyExporter.h"
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


/// <summary>
/// Constructor
/// </summary>
KBodyExporter::KBodyExporter(IKinectSensor *kSensor, FbxManager *fManager) :
m_pIsRecording(false),
m_pFBXManager(NULL),
m_takeStartAbsoluteMS(0),
m_lastFrameTimeMS(-1),
KBodyReader(kSensor)
{
	if (fManager) {
		initFBXSDKManager(fManager);
	}
}

/// <summary>
/// Destructor
/// </summary>
KBodyExporter::~KBodyExporter() {
	m_pIsRecording = false;
}

/// <summary>
/// Starts recording Skeleton Data.
/// </summary>
void KBodyExporter::startRecording() {
	// Start with a fresh take
	{
		std::lock_guard<std::mutex> lk(m_recMutex);
		m_pendingFrames.clear();
	}
	m_recording.clear();

	m_pIsRecording = true;
	m_smoother.reset();
	m_takeStartAbsoluteMS = 0;
	m_lastFrameTimeMS = -1;
	m_hasFloorRef = false;
	m_floorRefPlane[0]=0; m_floorRefPlane[1]=1; m_floorRefPlane[2]=0; m_floorRefPlane[3]=0;

	UI_Printf("Recording started.");
}

/// <summary>
/// Stops recording Skeleton Data.
/// </summary>
void KBodyExporter::stopRecording() {
	m_pIsRecording = false;
	// Keep whatever the capture thread staged before the flag went down.
	flushPendingFrames();
	if (m_hasFloorRef) {
		m_recording.hasFloor = true;
		m_recording.floorPlane[0]=m_floorRefPlane[0];
		m_recording.floorPlane[1]=m_floorRefPlane[1];
		m_recording.floorPlane[2]=m_floorRefPlane[2];
		m_recording.floorPlane[3]=m_floorRefPlane[3];
		UI_Printf("Floor reference saved (n=%.2f %.2f %.2f w=%.2f).",
			m_floorRefPlane[0], m_floorRefPlane[1], m_floorRefPlane[2], m_floorRefPlane[3]);
	}
	UI_Printf("Recording stopped. %d frame(s), %.2f s recorded.",
		int(m_recording.size()),
		double(m_recording.durationMS()) / 1000.0);
}

// Helpers: map floor plane to Y=0 (floor as origin) — rotation + translation
static void floorAimRotation(const float from[3], const float to[3], float R[3][3]) {
	R[0][0]=1; R[0][1]=0; R[0][2]=0; R[1][0]=0; R[1][1]=1; R[1][2]=0; R[2][0]=0; R[2][1]=0; R[2][2]=1;
	float dot = from[0]*to[0]+from[1]*to[1]+from[2]*to[2];
	if (dot > 0.99999f) return;
	float ax[3]={ from[1]*to[2]-from[2]*to[1], from[2]*to[0]-from[0]*to[2], from[0]*to[1]-from[1]*to[0] };
	float angle;
	if (dot < -0.99999f) { angle=(float)M_PI; ax[0]=1; ax[1]=0; ax[2]=0; float d=ax[0]*from[0]+ax[1]*from[1]+ax[2]*from[2]; ax[0]-=d*from[0]; ax[1]-=d*from[1]; ax[2]-=d*from[2]; } else angle=acosf(dot);
	float len=sqrtf(ax[0]*ax[0]+ax[1]*ax[1]+ax[2]*ax[2]); if(len<1e-6f) return; ax[0]/=len; ax[1]/=len; ax[2]/=len;
	float s=sinf(angle*0.5f), qw=cosf(angle*0.5f), qx=ax[0]*s, qy=ax[1]*s, qz=ax[2]*s;
	R[0][0]=1-2*(qy*qy+qz*qz); R[0][1]=2*(qx*qy-qz*qw); R[0][2]=2*(qx*qz+qy*qw);
	R[1][0]=2*(qx*qy+qz*qw); R[1][1]=1-2*(qx*qx+qz*qz); R[1][2]=2*(qy*qz-qx*qw);
	R[2][0]=2*(qx*qz-qy*qw); R[2][1]=2*(qy*qz+qx*qw); R[2][2]=1-2*(qx*qx+qy*qy);
}
static void floorAlignTransform(const float p[3], const float plane[4], float out[3]) {
	float n[3]={plane[0],plane[1],plane[2]}; float w=plane[3];
	float p0[3]={ -n[0]*w, -n[1]*w, -n[2]*w }; // closest point on plane
	float tmp[3]={ p[0]-p0[0], p[1]-p0[1], p[2]-p0[2] };
	float yAxis[3]={0,1,0};
	float R[3][3]; floorAimRotation(n, yAxis, R);
	out[0]=R[0][0]*tmp[0]+R[0][1]*tmp[1]+R[0][2]*tmp[2];
	out[1]=R[1][0]*tmp[0]+R[1][1]*tmp[1]+R[1][2]*tmp[2];
	out[2]=R[2][0]*tmp[0]+R[2][1]*tmp[1]+R[2][2]*tmp[2];
}

/// <summary>
/// Notify class about a frame that arrived
/// </summary>
/// <param name="bFrame">Incoming frame</param>
/// <param name="frameTime">Frame timestamp</param>
void KBodyExporter::notify(IBodyFrame *bFrame, INT64 frameTime) {
	// Not recording, ignore frame
	if (!m_pIsRecording)
		return;

	// Capture floor reference on first valid plane (locked as origin for this take)
	// Ensure normal points up (Y+) — Kinect may return (0,-1,0) which would bake a 180° X flip.
	if (!m_hasFloorRef) {
		Vector4 fp;
		if (SUCCEEDED(bFrame->get_FloorClipPlane(&fp))) {
			float len = sqrtf(fp.x*fp.x + fp.y*fp.y + fp.z*fp.z);
			if (len > 0.5f) {
				float inv = 1.0f/len;
				float nx=fp.x*inv, ny=fp.y*inv, nz=fp.z*inv, w=fp.w*inv;
				if (ny < 0) { nx=-nx; ny=-ny; nz=-nz; w=-w; } // force Y up → no 180° X
				m_floorRefPlane[0]=nx; m_floorRefPlane[1]=ny; m_floorRefPlane[2]=nz; m_floorRefPlane[3]=w;
				m_hasFloorRef = true;
				m_recording.hasFloor = true;
				m_recording.floorPlane[0]=m_floorRefPlane[0]; m_recording.floorPlane[1]=m_floorRefPlane[1];
				m_recording.floorPlane[2]=m_floorRefPlane[2]; m_recording.floorPlane[3]=m_floorRefPlane[3];
			}
		}
	}

	// Use the general notifier first, it will save the bodies of the current frame
	KBodyReader::notify(bFrame, frameTime);

	// If the frame could not be read, skip it
	if (!m_pBodyReadStatus)
		return;

	// Kinect clock works in increments of 100ns
	INT64 timeMS = frameTime / 10000;

	if (m_lastFrameTimeMS < 0)
		m_lastFrameTimeMS = timeMS;

	float dt = (float)((timeMS - m_lastFrameTimeMS) / 1000.0);
	if (dt < 0.0f) dt = 0.0f;
	m_lastFrameTimeMS = timeMS;

	{
		std::lock_guard<std::mutex> lk(m_recMutex);
		if (m_recording.frames.empty() && m_pendingFrames.empty())
			m_takeStartAbsoluteMS = timeMS;
	}

	// Build the frame sample
	KSampleFrame sample;
	sample.timeMS = timeMS - m_takeStartAbsoluteMS;

	for (int i = 0; i < BODY_COUNT; ++i)
	{
		IBody* pBody = m_ppBodies[i];
		if (!pBody)
			continue;

		BOOLEAN isTracked = FALSE;
		if (FAILED(pBody->get_IsTracked(&isTracked)) || !isTracked)
			continue;

		UINT64 trackingId = 0;
		pBody->get_TrackingId(&trackingId);

		Joint joints[JointType_Count];
		JointOrientation orientations[JointType_Count];

		if (FAILED(pBody->GetJoints(_countof(joints), joints)))
			continue;
		if (FAILED(pBody->GetJointOrientations(_countof(orientations), orientations)))
			continue;

		// Smooth the raw data to avoid jitter
		m_smoother.smooth(trackingId, joints, dt);
		m_smoother.smoothOrientations(trackingId, orientations, dt);

		// Bake with floor as origin when available (floor becomes y=0)
		bool useFloor = m_hasFloorRef;
		float floorR[3][3]; float floorQ[4]={0,0,0,1};
		if (useFloor) {
			float n[3]={m_floorRefPlane[0],m_floorRefPlane[1],m_floorRefPlane[2]};
			float yAxis[3]={0,1,0}; floorAimRotation(n, yAxis, floorR);
			float tr=floorR[0][0]+floorR[1][1]+floorR[2][2];
			if (tr>0){ float s=sqrtf(tr+1)*2; floorQ[3]=0.25f*s; floorQ[0]=(floorR[2][1]-floorR[1][2])/s; floorQ[1]=(floorR[0][2]-floorR[2][0])/s; floorQ[2]=(floorR[1][0]-floorR[0][1])/s; }
			else if (floorR[0][0]>floorR[1][1]&&floorR[0][0]>floorR[2][2]){ float s=sqrtf(1+floorR[0][0]-floorR[1][1]-floorR[2][2])*2; floorQ[0]=0.25f*s; floorQ[1]=(floorR[0][1]+floorR[1][0])/s; floorQ[2]=(floorR[0][2]+floorR[2][0])/s; floorQ[3]=(floorR[2][1]-floorR[1][2])/s; }
			else if (floorR[1][1]>floorR[2][2]){ float s=sqrtf(1+floorR[1][1]-floorR[0][0]-floorR[2][2])*2; floorQ[0]=(floorR[0][1]+floorR[1][0])/s; floorQ[1]=0.25f*s; floorQ[2]=(floorR[1][2]+floorR[2][1])/s; floorQ[3]=(floorR[0][2]-floorR[2][0])/s; }
			else { float s=sqrtf(1+floorR[2][2]-floorR[0][0]-floorR[1][1])*2; floorQ[0]=(floorR[0][2]+floorR[2][0])/s; floorQ[1]=(floorR[1][2]+floorR[2][1])/s; floorQ[2]=0.25f*s; floorQ[3]=(floorR[1][0]-floorR[0][1])/s; }
		}

		KSampleBody bodySample;
		bodySample.trackingId = trackingId;
		bodySample.isTracked = true;

		for (int j = 0; j < JointType_Count; ++j)
		{
			KSampleJoint &sj = bodySample.joints[j];
			float p[3]={joints[j].Position.X, joints[j].Position.Y, joints[j].Position.Z};
			if (useFloor) { float q[3]; floorAlignTransform(p, m_floorRefPlane, q); p[0]=q[0]; p[1]=q[1]; p[2]=q[2]; }
			sj.position[0]=p[0]; sj.position[1]=p[1]; sj.position[2]=p[2];
			float ox=orientations[j].Orientation.x, oy=orientations[j].Orientation.y, oz=orientations[j].Orientation.z, ow=orientations[j].Orientation.w;
			if (useFloor) {
				// q' = q_floor * q
				float x= floorQ[0]*ow + floorQ[3]*ox + floorQ[1]*oz - floorQ[2]*oy;
				float y= floorQ[1]*ow + floorQ[3]*oy + floorQ[2]*ox - floorQ[0]*oz;
				float z= floorQ[2]*ow + floorQ[3]*oz + floorQ[0]*oy - floorQ[1]*ox;
				float w= floorQ[3]*ow - floorQ[0]*ox - floorQ[1]*oy - floorQ[2]*oz;
				ox=x; oy=y; oz=z; ow=w;
			}
			sj.orientation[0]=ox; sj.orientation[1]=oy; sj.orientation[2]=oz; sj.orientation[3]=ow;
			sj.trackingState = joints[j].TrackingState;
		}

		sample.bodies.push_back(bodySample);
	}

	// Only keep frames that contain at least one tracked body
	if (!sample.bodies.empty())
	{
		// Staged here, appended to the take by flushPendingFrames() on the UI
		// thread: the take is read (and iterated) by every paint.
		std::lock_guard<std::mutex> lk(m_recMutex);
		m_pendingFrames.push_back(sample);
	}
}

void KBodyExporter::flushPendingFrames() const
{
	std::lock_guard<std::mutex> lk(m_recMutex);
	if (m_pendingFrames.empty())
		return;
	m_recording.frames.insert(m_recording.frames.end(),
		m_pendingFrames.begin(), m_pendingFrames.end());
	m_pendingFrames.clear();
}

/// <summary>
/// Removes the last 'ms' milliseconds from the recorded take
/// </summary>
void KBodyExporter::trimRecordingEnd(INT64 ms) {
	flushPendingFrames();
	if (ms <= 0 || m_recording.frames.empty())
		return;

	INT64 cutoff = m_recording.frames.back().timeMS - ms;
	while (!m_recording.frames.empty() &&
		m_recording.frames.back().timeMS > cutoff)
	{
		m_recording.frames.pop_back();
	}

	UI_Printf("Trimmed last %.2f s. %d frame(s) left.",
		double(ms) / 1000.0, int(m_recording.size()));
}

/// <summary>
/// Clears the recorded take
/// </summary>
void KBodyExporter::clearTake() {
	{
		std::lock_guard<std::mutex> lk(m_recMutex);
		m_pendingFrames.clear();
	}
	m_recording.clear();
	m_smoother.reset();
	m_takeStartAbsoluteMS = 0;
	m_lastFrameTimeMS = -1;
	m_hasFloorRef=false; m_floorRefPlane[0]=0; m_floorRefPlane[1]=1; m_floorRefPlane[2]=0; m_floorRefPlane[3]=0;
	UI_Printf("Take cleared.");
}

/// <summary>
/// Exports the whole recorded take to the given file.
/// Returns true on success.
/// </summary>
bool KBodyExporter::exportTake(const char *fileName) {
	return exportTakeToFile(fileName, -1, -1);
}

/// <summary>
/// Exports only the frames inside [inMS, outMS] (relative ms) to the given file.
/// Returns true on success.
/// </summary>
bool KBodyExporter::exportTake(const char *fileName, INT64 inMS, INT64 outMS) {
	return exportTakeToFile(fileName, inMS, outMS);
}

/// <summary>
/// Builds an FBX scene from the recorded buffer and saves it.
/// </summary>
bool KBodyExporter::exportTakeToFile(const char *fileName, INT64 inMS, INT64 outMS) {

	if (!m_pFBXManager) {
		UI_Printf("FBX SDK manager has not been initialized.");
		return false;
	}

	if (m_recording.frames.empty()) {
		UI_Printf("Nothing recorded yet. Press Record before exporting.");
		return false;
	}

	// Normalize the export range
	INT64 takeStart = m_recording.startTimeMS();
	INT64 takeEnd = m_recording.frames.back().timeMS;
	if (inMS < 0) inMS = takeStart;
	if (outMS < 0 || outMS > takeEnd) outMS = takeEnd;
	if (inMS > outMS) { INT64 t = inMS; inMS = outMS; outMS = t; }

	// Create a scene
	FbxScene* lScene = FbxScene::Create(m_pFBXManager, "");

	// Create Animation Stack
	FbxAnimStack* lAnimStack = FbxAnimStack::Create(lScene, "Base animation");
	FbxAnimLayer* lAnimLayer = FbxAnimLayer::Create(lScene, "Base Layer");
	lAnimStack->AddMember(lAnimLayer);

	// Set the global time mode to 30 fps (Kinect frame rate)
	FbxGlobalSettings& gs = lScene->GetGlobalSettings();
	gs.SetTimeMode(FbxTime::eFrames30);

	// The exported clip starts at frame 1 of the trimmed range
	INT64 baseTime = inMS;

	size_t exportedCount = 0;

	for (size_t i = 0; i < m_recording.frames.size(); ++i)
	{
		const KSampleFrame &frame = m_recording.frames[i];

		if (frame.timeMS < inMS || frame.timeMS > outMS)
			continue;

		INT64 frameTime = frame.timeMS - baseTime + 1;

		for (size_t b = 0; b < frame.bodies.size(); ++b)
		{
			const KSampleBody &body = frame.bodies[b];
			if (!body.isTracked)
				continue;

			Joint joints[JointType_Count];
			JointOrientation orientations[JointType_Count];

			for (int j = 0; j < JointType_Count; ++j)
			{
				const KSampleJoint &sj = body.joints[j];

				joints[j].Position.X = sj.position[0];
				joints[j].Position.Y = sj.position[1];
				joints[j].Position.Z = sj.position[2];
				joints[j].TrackingState = (TrackingState)sj.trackingState;

				orientations[j].Orientation.x = sj.orientation[0];
				orientations[j].Orientation.y = sj.orientation[1];
				orientations[j].Orientation.z = sj.orientation[2];
				orientations[j].Orientation.w = sj.orientation[3];
			}

			KinectSkeletonMapper::map(lScene, frameTime, body.trackingId, joints, orientations);
		}

		exportedCount++;
	}

	// Apply post processing filters (unroll rotation keys, ...)
	KinectSkeletonMapper::applyPostProcessingFilters(lScene);

	// Get File Format (FBX binary)
	int lFileFormat = -1;
	int wFormat = m_pFBXManager->GetIOPluginRegistry()->FindWriterIDByDescription(c_FBXBinaryFileDesc);
	if (wFormat >= 0)
		lFileFormat = wFormat;
	else
		lFileFormat = m_pFBXManager->GetIOPluginRegistry()->FindReaderIDByDescription(c_FBXBinaryFileDesc);

	bool saved = SaveScene(m_pFBXManager, lScene, fileName, lFileFormat, false);

	// Clean up the scene
	lScene->Destroy();
	lScene = NULL;

	if (saved) {
		UI_Printf("Take exported to %s (%d frames, %.2f s).",
			fileName, int(exportedCount), double(outMS - inMS) / 1000.0);
	} else {
		UI_Printf("Failed to export take to %s.", fileName);
	}

	return saved;
}
