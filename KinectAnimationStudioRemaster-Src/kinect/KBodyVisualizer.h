#pragma once


#include "..\common\stdafx.h"
#include "..\UI\resource.h"
#include "KBodyReader.h"
#include "KRecording.h"
#include "OneEuroFilter.h"
#include "Session.h"

#include <dwrite.h>
#include <d3d11.h>
#include <d3dcompiler.h>

// Custom message posted to the main window when the fist-gesture countdown
// finishes. wParam: 1 = start recording, 0 = stop recording.
#define WM_GESTURE_RECORD (WM_APP + 1)

// Global consts
static const WCHAR msc_fontName[] = L"Segoe UI";
static const FLOAT msc_fontSize = 12;

// Forward declarations to access the recorded take and the depth stream
class KBodyExporter;
class KDepthReader;
class KRetargeter;
struct RetargetPose;

/*
 Real-time 3D skeleton viewer. Draws the Kinect bodies into a Direct2D
 render target using a software 3D projection (orbit / zoom / pan camera),
 so the user gets a proper 3D viewport without needing a GPU pipeline.
*/
class KBodyVisualizer : public KBodyReader {

public:
	/// <summary>
	/// Constructor
	/// </summary>
	/// <param name="kSensor">Default Kinect Sensor</param>
	KBodyVisualizer(IKinectSensor *kSensor = NULL);

	/// <summary>
	/// Destructor
	/// </summary>
	~KBodyVisualizer();

	/// <summary>
	/// Initializes the desired window and begins processing
	/// </summary>
	HRESULT   attach(HWND hWnd);

	/// <summary>
	/// Detaches the visualizer from the window
	/// </summary>
	void detach();

	/// <summary>
	/// Main update function, will redraw anything
	/// </summary>
	bool update();

	/// <summary>
	/// Notify class about a frame that arrived
	/// </summary>
	/// <param name="bFrame">Incoming frame</param>
	/// <param name="frameTime">Frame timestamp</param>
	virtual void notify(IBodyFrame *bFrame, INT64 frameTime);

	/// <summary>
	/// Checks if visualizer is attached to window
	/// </summary>
	bool is_attached() { return m_hWnd != NULL; };

	/// <summary>
	/// Sets the recording source used for playback
	/// </summary>
	void setExporter(KBodyExporter *exporter) { m_pExporter = exporter; }

	/// <summary>
	/// Sets the depth stream reader used for the depth/IR point cloud
	/// </summary>
	void setDepthReader(KDepthReader *reader) { m_pDepthReader = reader; }

	/// <summary>
	/// Sets the retarget engine used to preview the imported character
	/// </summary>
	void setRetargeter(KRetargeter *retargeter) { m_pRetargeter = retargeter; }

	/// <summary>
	/// True when the Kinect reports a valid floor clip plane
	/// </summary>
	bool hasFloor() const { return m_hasFloor; }

	/// <summary>
	/// Height of the sensor above the detected floor, in metres
	/// </summary>
	float floorHeight() const { return m_floorHeight; }

	/// <summary>
	/// Sets the smoothing strength (0..100) applied to the live preview
	/// </summary>
	void setSmoothing(float strength) { m_smoother.setStrength(strength); }

	/// <summary>
	/// Camera state — persisted per take (static during mocap)
	/// </summary>
	void getCamera(float &yaw, float &pitch, float &dist, float &panX, float &panY, float &fov) const { yaw=m_yaw; pitch=m_pitch; dist=m_dist; panX=m_panX; panY=m_panY; fov=m_fov; }
	void setCamera(float yaw, float pitch, float dist, float panX, float panY, float fov) { m_yaw=yaw; m_pitch=pitch; m_dist=dist; m_panX=panX; m_panY=panY; m_fov=fov; }

	/// <summary>
	/// Handles window resize
	/// </summary>
	void resize(int width, int height);

	// Camera / mouse interaction (called from the view window procedure)

	void mouseDown(float x, float y, int button, bool shift = false, bool ctrl = false);
	void mouseMove(float x, float y);
	void mouseUp();
	void mouseWheel(float delta);
	void resetCamera();

	int pickRetargetBonePublic(float sx, float sy) { return pickRetargetBone(sx, sy); }
	int pickGhostJointPublic(float sx, float sy) { return pickGhostJoint(sx, sy); }

	/// <summary>
	/// Applies a quick view preset (Blender-like): 0 front, 1 right, 2 top, 3 Kinect
	/// </summary>
	void setViewPreset(int preset);

	/// <summary>
	/// Applies the current D2D color scheme
	/// </summary>
	void ensureBrushes();

	// Ghost reference pose editing (retarget workspace)
	/// <summary>
	/// Highlights the Kinect ghost joint (green) and the character bone (orange)
	/// selected in the mapping panel, and enables the rotation gizmo on the ghost.
	/// Pass -1 to clear one side.
	/// </summary>
	void setSelection(int joint, int bone) { m_selJoint = joint; m_selBone = bone; }
	int  selectionJoint() const { return m_selJoint; }
	int  selectionBone() const { return m_selBone; }
	/// <summary>
	/// Restores the ghost reference pose to the default T-pose.
	/// </summary>
	void resetGhostPose();

	/// <summary>
	/// Finds the retarget bone whose projected joint is closest to a screen point
	/// </summary>
	int pickRetargetBone(float sx, float sy);

private:

	// Bone connectivity (Kinect JointType index pairs)
	static const int c_BoneCount = 22;
	static const int c_Bones[c_BoneCount][2];

	// Camera
	float m_yaw;
	float m_pitch;
	float m_dist;
	float m_panX;
	float m_panY;
	float m_fov;

	bool m_draggingRotate;
	bool m_draggingPan;
	bool m_draggingDolly;
	float m_lastMouseX;
	float m_lastMouseY;

	// Selection / ghost-pose editing
	int   m_selJoint;     // Selected Kinect joint (green ghost highlight), -1 none
	int   m_selBone;      // Selected character bone (orange highlight), -1 none
	bool  m_gizmoDrag;    // Rotating the ghost with the gizmo
	bool  m_gizmoShift;   // Shift held during the gizmo drag (twist)
	int   m_gizmoAxis;    // Axis grabbed on the rotation gizmo (0=X,1=Y,2=Z), -1 none
	float m_gizmoStartAngle;  // Grab angle around the pivot (atan2)
	float m_gizmoCenterX;     // Projected pivot at grab time
	float m_gizmoCenterY;
	float m_lastGizmoX;
	float m_lastGizmoY;

	// Floor detection (from the Kinect body frame)
	std::atomic<bool>  m_hasFloor;
	std::atomic<float> m_floorHeight;
	float m_floorPlane[4];          // X, Y, Z, W of the floor clip plane (n·p + w = 0, normalized)
	std::mutex         m_floorMutex;

	// Live preview data (filled on the Kinect thread, read on the UI thread)
	struct LiveBody
	{
		UINT64 trackingId;
		float positions[JointType_Count][3];
		int   states[JointType_Count];
		int   leftHand;    // HandState (open / closed / ...)
		int   rightHand;   // HandState
		float fistDistLeft;   // dist(Hand,HandTip) + dist(Hand,Thumb), small = fist
		float fistDistRight;  // same for the right hand
		bool  fingersLeftTracked;   // hand tip/thumb joints tracked (geometric check valid)
		bool  fingersRightTracked;
		bool  isTracked;
	};

	std::vector<LiveBody> m_liveBodies;
	std::timed_mutex m_liveMutex;

	// Fist-gesture recording countdown (written on the Kinect thread,
	// read on the UI thread). Active while an actor holds both fists closed.
	std::atomic<bool>       m_gestureActive;
	std::atomic<bool>       m_gestureForStart;   // true: countdown to start recording
	std::atomic<ULONGLONG>  m_gestureStartMS;    // steady_clock milliseconds when the gesture began

	// Kinect smoothing for the live preview
	KBodySmoother m_smoother;

	// Recording source for playback
	KBodyExporter *m_pExporter;

	// Retarget engine (owned elsewhere, e.g. UI.cpp)
	KRetargeter *m_pRetargeter;

	// Depth stream reader (owned elsewhere, e.g. UI.cpp)
	KDepthReader *m_pDepthReader;

	// Window
	HWND m_hWnd;

	// Performance counters
	INT64 m_nLastCounter;
	DWORD m_nFramesSinceUpdate;
	double m_fFreq;
	float m_fps;

	// D2D objects
	ID2D1Factory* m_pD2DFactory;
	IDWriteFactory* m_pD2DWriteFactory;
	ID2D1HwndRenderTarget* m_pRT;
	IDWriteTextFormat* m_pTextFormat;
	IDWriteTextFormat* m_pTextFormatSmall;
	IDWriteTextFormat* m_pTextFormatBig;

	ID2D1SolidColorBrush* m_bgBrush;
	ID2D1SolidColorBrush* m_gridBrush = nullptr;
	ID2D1SolidColorBrush* m_axisBrush = nullptr;
	ID2D1SolidColorBrush* m_floorFillBrush = nullptr;
	ID2D1SolidColorBrush* m_floorHatchBrush = nullptr;
	ID2D1SolidColorBrush* m_boneTracked;
	ID2D1SolidColorBrush* m_boneInferred;
	ID2D1SolidColorBrush* m_jointTracked;
	ID2D1SolidColorBrush* m_jointInferred;
	ID2D1SolidColorBrush* m_ghostBrush;
	ID2D1SolidColorBrush* m_textBrush;
	ID2D1SolidColorBrush* m_dimTextBrush;
	ID2D1SolidColorBrush* m_accentBrush;
	ID2D1SolidColorBrush* m_redBrush;
	ID2D1SolidColorBrush* m_greenBrush;
	ID2D1SolidColorBrush* m_orangeBrush;
	ID2D1SolidColorBrush* m_yellowBrush;
	ID2D1SolidColorBrush* m_blueBrush;
	ID2D1SolidColorBrush* m_meshFillBrush;
	ID2D1SolidColorBrush* m_meshWireBrush;

	// Depth point-cloud palette (near -> far gradient)
	ID2D1SolidColorBrush* m_depthPalette[16];

	// Scratch copy of the latest depth frame, reused by every paint so the
	// point cloud does not allocate ~430 KB per frame.
	std::vector<UINT16> m_depthScratch;

	int m_width;
	int m_height;

	// D3D11 GPU mesh renderer (retarget character preview, skinned on the GPU)
	ID3D11Device*            m_pD3DDevice;
	ID3D11DeviceContext*     m_pD3DCtx;
	ID3D11VertexShader*      m_pVS;
	ID3D11PixelShader*       m_pPS;
	ID3D11InputLayout*       m_pIL;
	ID3D11Buffer*            m_pVB;
	ID3D11Buffer*            m_pIB;
	ID3D11Buffer*            m_pPerFrame;
	ID3D11Buffer*            m_pPerBone;
	ID3D11RasterizerState*   m_pRSSolid;
	ID3D11RasterizerState*   m_pRSWire;
	ID3D11DepthStencilState* m_pDSState;
	ID3D11DepthStencilState* m_pDSWireState;
	ID3D11Texture2D*         m_pMeshRT;
	ID3D11RenderTargetView*  m_pMeshRTV;
	ID3D11Texture2D*         m_pMeshDS;
	ID3D11DepthStencilView*  m_pMeshDSV;
	ID3D11Texture2D*         m_pMeshStaging;
	ID2D1Bitmap*             m_pMeshBitmap;
	int                      m_gpuMeshVersion;
	int                      m_gpuRTWidth;
	int                      m_gpuRTHeight;
	bool                     m_gpuReady;
	bool                     m_wasRetargetPlaying; // Edge detection for playback start

	/// <summary>
	/// Reads the latest body data from the sensor and stores it
	/// </summary>
	void receiveBodies();

	/// <summary>
	/// Projects a 3D camera-space point to screen space
	/// </summary>
	D2D1_POINT_2F project(const float p[3]);

	/// <summary>
	/// Draws the ground grid and the axis helpers at the given height
	/// </summary>
	void drawGrid(float floorY);
	void drawGridOriented();
	void drawFloorHatched(const float n[3], float w, const float p0[3], const float u[3], const float v[3], float extent);

	/// <summary>
	/// Draws the depth/IR point cloud from the Kinect depth stream
	/// </summary>
	void drawDepthCloud();

	/// <summary>
	/// Draws the Kinect as a camera frustum at the origin
	/// </summary>
	void drawKinectCamera();

	/// <summary>
	/// Creates the depth point-cloud color palette
	/// </summary>
	void createDepthPalette();

	/// <summary>
	/// Draws a skeleton given its joint positions and tracking states
	/// </summary>
	void drawSkeleton(const float positions[][3], const int states[], bool ghost);

	/// <summary>
	/// Draws a single bone
	/// </summary>
	void drawBone(const float positions[][3], const int states[], int joint0, int joint1, bool ghost);

	/// <summary>
	/// Draws the overlay texts (mode, fps, recording indicator, hints)
	/// </summary>
	void drawOverlays();

	/// <summary>
	/// Draws the live pose or the playback pose
	/// </summary>
	void drawScene();

	/// <summary>
	/// Draws the imported character (retarget preview): skeleton + skinned mesh
	/// </summary>
	void drawRetarget();

	/// <summary>
	/// Creates the D3D11 device, shaders and input layout for GPU mesh rendering
	/// </summary>
	bool gpuInit();

	/// <summary>
	/// Releases every D3D11 resource owned by the visualizer
	/// </summary>
	void gpuRelease();

	/// <summary>
	/// Releases the D3D render-target / staging textures and the D2D bitmap
	/// </summary>
	void gpuReleaseRenderTargets();

	/// <summary>
	/// (Re)creates the offscreen D3D render target matching the window size
	/// </summary>
	bool gpuEnsureRenderTargets();

	/// <summary>
	/// Uploads the current retarget mesh (rest pose + skinning data) to the GPU
	/// </summary>
	bool gpuEnsureMesh();

	/// <summary>
	/// Skin and rasterize the character mesh into the offscreen render target
	/// </summary>
	bool gpuRenderMesh();

	/// <summary>
	/// Draws the retarget skeleton bones as D2D lines
	/// </summary>
	void drawRetargetSkeleton(const RetargetPose &pose);

	/// <summary>
	/// Finds the ghost joint whose projected position is closest to a screen point
	/// </summary>
	int pickGhostJoint(float sx, float sy);

	/// <summary>
	/// Draws the green ghost-joint / orange character-bone selection highlight,
	/// the connector line and the rotation gizmo
	/// </summary>
	void drawSelectionOverlay(const float ghostPos[][3]);

	/// <summary>
	/// Draws the classic 3-axis rotation gizmo around a world-space pivot
	/// </summary>
	void drawGizmo(const float pivot[3]);

	/// <summary>
	/// Screen radius (px) used to draw / hit-test the rotation gizmo
	/// </summary>
	float gizmoRadius(const float pivot[3]) const;

	/// <summary>
	/// Returns the gizmo axis (0=X,1=Y,2=Z) whose rotation circle is nearest
	/// to the screen point, or -1
	/// </summary>
	int gizmoPickAxis(float sx, float sy);

	/// <summary>
	/// Rotates a ghost joint's subtree (and its mirrored partner when the
	/// symmetry flag is on)
	/// </summary>
	void applyGhostRotation(int joint, const float axis[3], float angle);

	/// <summary>
	/// Formats a millisecond value as mm:ss.cc
	/// </summary>
	void formatTime(INT64 ms, WCHAR *out, int outLen);

};
