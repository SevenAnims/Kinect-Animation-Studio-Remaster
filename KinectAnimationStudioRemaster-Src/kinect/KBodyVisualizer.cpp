#include "KBodyVisualizer.h"
#include "KBodyExporter.h"
#include "KDepthReader.h"
#include "KRetargeter.h"
#include "..\common\stdafx.h"
#include <strsafe.h>
#include <cmath>
#include <cstring>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ---------------------------------------------------------------------------
// GPU mesh renderer (retarget preview)
// ---------------------------------------------------------------------------

// Interleaved GPU vertex: rest position + rest normal + up-to-4 bone influences
struct GpuMeshVertex
{
	float pos[3];
	float nrm[3];
	unsigned int bones[4];
	float weights[4];
};

// Multiplies two column-major 4x4 matrices (data[c*4+r] layout): out = a * b
static void mulMat4Col(float *out, const float *a, const float *b)
{
	for (int r = 0; r < 4; ++r)
		for (int c = 0; c < 4; ++c)
		{
			float s = 0.0f;
			for (int k = 0; k < 4; ++k)
				s += a[k * 4 + r] * b[c * 4 + k];
			out[c * 4 + r] = s;
		}
}

// Normalizes a 3D vector in place
static void normalize3(float v[3])
{
	float l = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
	if (l > 1e-6f) { v[0] /= l; v[1] /= l; v[2] /= l; }
}

// out = a x b (3D cross product)
static void cross3(const float a[3], const float b[3], float out[3])
{
	out[0] = a[1] * b[2] - a[2] * b[1];
	out[1] = a[2] * b[0] - a[0] * b[2];
	out[2] = a[0] * b[1] - a[1] * b[0];
}

// Mirrored Kinect joint across the sagittal plane (X axis flip). Center
// joints map to themselves; a limb maps to its left/right counterpart.
static int mirrorJoint(int j)
{
	static const int mp[JointType_Count] = {
		0, 1, 2, 3,                       // SpineBase, SpineMid, Neck, Head
		8, 9, 10, 11,                     // ShoulderLeft..HandLeft -> Right
		4, 5, 6, 7,                       // ShoulderRight..HandRight -> Left
		16, 17, 18, 19,                   // HipLeft..FootLeft -> Right
		12, 13, 14, 15,                   // HipRight..FootRight -> Left
		20,                               // SpineShoulder
		23, 24, 21, 22,                   // HandTipLeft/ThumbLeft <-> Right
	};
	return (j >= 0 && j < JointType_Count) ? mp[j] : j;
}

static const char *c_vsRetarget =
"cbuffer cbPerFrame : register(b0)\n"
"{\n"
"    float4x4 g_viewProj;\n"
"    float4   g_pan;   // xy: pan offset normalized to the viewport half-size\n"
"    float4   g_pad;\n"
"};\n"
"cbuffer cbPerBone : register(b1)\n"
"{\n"
"    float4x4 g_bones[256];\n"
"};\n"
"struct VSIn\n"
"{\n"
"    float3 pos    : POSITION;\n"
"    float3 nrm    : NORMAL;\n"
"    uint4  bones  : BONEINDEX;\n"
"    float4 weights: BONEWEIGHT;\n"
"};\n"
"struct VSOut\n"
"{\n"
"    float4 pos : SV_POSITION;\n"
"    float3 col : COLOR;\n"
"};\n"
"VSOut main(VSIn i)\n"
"{\n"
"    float3 p = float3(0, 0, 0);\n"
"    float3 n = float3(0, 0, 0);\n"
"    float sum = 0;\n"
"    for (uint k = 0; k < 4; ++k)\n"
"    {\n"
"        float w = i.weights[k];\n"
"        if (w <= 0.0) break;\n"
"        float4x4 S = g_bones[i.bones[k]];\n"
"        p += w * mul(S, float4(i.pos, 1.0)).xyz;\n"
"        n += w * mul((float3x3)S, i.nrm);\n"
"        sum += w;\n"
"    }\n"
"    if (sum <= 0.0001)\n"
"    {\n"
"        p = i.pos;\n"
"        n = i.nrm;\n"
"    }\n"
"    n = normalize(n);\n"
"    float3 L = normalize(float3(0.35, 0.70, 0.55));\n"
"    float3 R = normalize(float3(-0.40, 0.55, -0.60));\n"
"    float3 base = float3(0.18, 0.30, 0.52);\n"
"    float3 col = base * (0.38 + 0.62 * saturate(dot(n, L)));\n"
"    col += float3(0.30, 0.42, 0.68) * pow(saturate(dot(n, R)), 6.0) * 0.35;\n"
"    col = saturate(col);\n"
"    float4 clip = mul(g_viewProj, float4(p, 1.0));\n"
"    clip.x += g_pan.x * clip.w;\n"
"    clip.y -= g_pan.y * clip.w;\n"
"    VSOut o;\n"
"    o.pos = clip;\n"
"    o.col = col;\n"
"    return o;\n"
"}\n";

static const char *c_psRetarget =
"struct VSOut\n"
"{\n"
"    float4 pos : SV_POSITION;\n"
"    float3 col : COLOR;\n"
"};\n"
"float4 main(VSOut i) : SV_Target\n"
"{\n"
"    return float4(i.col, 1.0);\n"
"}\n";

// Bone connectivity (Kinect JointType index pairs)
const int KBodyVisualizer::c_Bones[c_BoneCount][2] =
{
	{ JointType_SpineBase,   JointType_SpineMid },
	{ JointType_SpineMid,    JointType_SpineShoulder },
	{ JointType_SpineShoulder, JointType_Neck },
	{ JointType_Neck,        JointType_Head },

	{ JointType_SpineShoulder, JointType_ShoulderRight },
	{ JointType_ShoulderRight, JointType_ElbowRight },
	{ JointType_ElbowRight,    JointType_WristRight },
	{ JointType_WristRight,    JointType_HandRight },
	{ JointType_WristRight,    JointType_ThumbRight },

	{ JointType_SpineShoulder, JointType_ShoulderLeft },
	{ JointType_ShoulderLeft,  JointType_ElbowLeft },
	{ JointType_ElbowLeft,     JointType_WristLeft },
	{ JointType_WristLeft,     JointType_HandLeft },
	{ JointType_WristLeft,     JointType_ThumbLeft },

	{ JointType_SpineBase,  JointType_HipRight },
	{ JointType_HipRight,   JointType_KneeRight },
	{ JointType_KneeRight,  JointType_AnkleRight },
	{ JointType_AnkleRight, JointType_FootRight },

	{ JointType_SpineBase,  JointType_HipLeft },
	{ JointType_HipLeft,    JointType_KneeLeft },
	{ JointType_KneeLeft,   JointType_AnkleLeft },
	{ JointType_AnkleLeft,  JointType_FootLeft },
};

/// <summary>
/// Constructor
/// </summary>
KBodyVisualizer::KBodyVisualizer(IKinectSensor *kSensor) :
m_hWnd(NULL),
m_nLastCounter(0),
m_nFramesSinceUpdate(0),
m_fFreq(0),
m_fps(0.0f),
m_yaw(0.0f),
m_pitch(-0.15f),
m_dist(3.8f),
m_panX(0.0f),
m_panY(0.0f),
	m_fov(55.0f * float(M_PI) / 180.0f),
	m_draggingRotate(false),
	m_draggingPan(false),
	m_draggingDolly(false),
	m_lastMouseX(0.0f),
	m_lastMouseY(0.0f),
	m_selJoint(-1),
	m_selBone(-1),
	m_gizmoDrag(false),
	m_gizmoShift(false),
	m_gizmoAxis(-1),
	m_gizmoStartAngle(0.0f),
	m_gizmoCenterX(0.0f),
	m_gizmoCenterY(0.0f),
	m_lastGizmoX(0.0f),
	m_lastGizmoY(0.0f),
	m_pExporter(NULL),
	m_pRetargeter(NULL),
	m_pDepthReader(NULL),
	m_hasFloor(false),
	m_floorHeight(0.0f),
	m_pD2DFactory(NULL),
	m_pD2DWriteFactory(NULL),
	m_pRT(NULL),
	m_pTextFormat(NULL),
	m_pTextFormatSmall(NULL),
	m_pTextFormatBig(NULL),
	m_bgBrush(NULL),
	m_gridBrush(NULL),
	m_axisBrush(NULL),
	m_boneTracked(NULL),
	m_boneInferred(NULL),
	m_jointTracked(NULL),
	m_jointInferred(NULL),
	m_ghostBrush(NULL),
	m_textBrush(NULL),
	m_dimTextBrush(NULL),
	m_accentBrush(NULL),
	m_redBrush(NULL),
	m_greenBrush(NULL),
	m_orangeBrush(NULL),
	m_yellowBrush(NULL),
	m_blueBrush(NULL),
	m_width(0),
	m_height(0),
	m_pD3DDevice(NULL),
	m_pD3DCtx(NULL),
	m_pVS(NULL),
	m_pPS(NULL),
	m_pIL(NULL),
	m_pVB(NULL),
	m_pIB(NULL),
	m_pPerFrame(NULL),
	m_pPerBone(NULL),
	m_pRSSolid(NULL),
	m_pRSWire(NULL),
	m_pDSState(NULL),
	m_pDSWireState(NULL),
	m_pMeshRT(NULL),
	m_pMeshRTV(NULL),
	m_pMeshDS(NULL),
	m_pMeshDSV(NULL),
	m_pMeshStaging(NULL),
	m_pMeshBitmap(NULL),
	m_gpuMeshVersion(-1),
	m_gpuRTWidth(0),
	m_gpuRTHeight(0),
	m_gpuReady(false),
	m_wasRetargetPlaying(false),
	KBodyReader(kSensor)
{
	m_gestureActive = false;
	m_gestureForStart = false;
	m_gestureStartMS = 0;

	m_floorPlane[0] = 0.0f;
	m_floorPlane[1] = 1.0f;
	m_floorPlane[2] = 0.0f;
	m_floorPlane[3] = 0.0f;
	for (int i = 0; i < 16; ++i)
		m_depthPalette[i] = NULL;

	HRESULT hr = D2D1CreateFactory(
		D2D1_FACTORY_TYPE_SINGLE_THREADED,
		&m_pD2DFactory
		);

	if (SUCCEEDED(hr)) {
		hr = DWriteCreateFactory(
			DWRITE_FACTORY_TYPE_SHARED,
			__uuidof(m_pD2DWriteFactory),
			reinterpret_cast<IUnknown **>(&m_pD2DWriteFactory)
			);
	}

	if (SUCCEEDED(hr)) {
		// Query performance results
		LARGE_INTEGER qpf = { 0 };
		if (QueryPerformanceFrequency(&qpf))
		{
			m_fFreq = double(qpf.QuadPart);
		}
	}

	m_smoother.setStrength(35.0f);
}

/// <summary>
/// Destructor
/// </summary>
KBodyVisualizer::~KBodyVisualizer()
{
	gpuRelease();
	detach();

	SafeRelease(m_pTextFormatBig);
	SafeRelease(m_pTextFormatSmall);
	SafeRelease(m_pTextFormat);
	SafeRelease(m_pD2DWriteFactory);
	SafeRelease(m_pD2DFactory);
}

/// <summary>
/// Attaches body visualizer to the desired window and initializes D2D structures
/// </summary>
HRESULT KBodyVisualizer::attach(HWND hWnd)
{
	m_hWnd = hWnd;

	if (!m_gpuReady)
		gpuInit();

	RECT rc;
	GetClientRect(m_hWnd, &rc);
	resize(rc.right - rc.left, rc.bottom - rc.top);

	return S_OK;
}

/// <summary>
/// Detaches the visualizer from the window
/// </summary>
void KBodyVisualizer::detach()
{
	m_hWnd = NULL;

	gpuRelease();

	SafeRelease(m_bgBrush);
	SafeRelease(m_gridBrush);
	SafeRelease(m_axisBrush);
	SafeRelease(m_floorFillBrush);
	SafeRelease(m_floorHatchBrush);
	SafeRelease(m_boneTracked);
	SafeRelease(m_boneInferred);
	SafeRelease(m_jointTracked);
	SafeRelease(m_jointInferred);
	SafeRelease(m_ghostBrush);
	SafeRelease(m_textBrush);
	SafeRelease(m_dimTextBrush);
	SafeRelease(m_accentBrush);
	SafeRelease(m_redBrush);
	SafeRelease(m_greenBrush);
	SafeRelease(m_orangeBrush);
	SafeRelease(m_yellowBrush);
	SafeRelease(m_blueBrush);
	for (int i = 0; i < 16; ++i)
	{
		SafeRelease(m_depthPalette[i]);
		m_depthPalette[i] = NULL;
	}
	SafeRelease(m_pRT);
}

/// <summary>
/// Recreates the render target and the brushes after a resize
/// </summary>
void KBodyVisualizer::resize(int width, int height)
{
	if (width < 10) width = 10;
	if (height < 10) height = 10;

	m_width = width;
	m_height = height;

	if (!m_hWnd || !m_pD2DFactory)
		return;

	SafeRelease(m_bgBrush);
	SafeRelease(m_gridBrush);
	SafeRelease(m_axisBrush);
	SafeRelease(m_floorFillBrush);
	SafeRelease(m_floorHatchBrush);
	SafeRelease(m_boneTracked);
	SafeRelease(m_boneInferred);
	SafeRelease(m_jointTracked);
	SafeRelease(m_jointInferred);
	SafeRelease(m_ghostBrush);
	SafeRelease(m_textBrush);
	SafeRelease(m_dimTextBrush);
	SafeRelease(m_accentBrush);
	SafeRelease(m_redBrush);
	SafeRelease(m_greenBrush);
	SafeRelease(m_orangeBrush);
	SafeRelease(m_yellowBrush);
	SafeRelease(m_blueBrush);
	for (int i = 0; i < 16; ++i)
	{
		SafeRelease(m_depthPalette[i]);
		m_depthPalette[i] = NULL;
	}
	SafeRelease(m_pRT);

	HRESULT hr = m_pD2DFactory->CreateHwndRenderTarget(
		D2D1::RenderTargetProperties(D2D1_RENDER_TARGET_TYPE_DEFAULT,
			D2D1::PixelFormat(DXGI_FORMAT_B8G8R8A8_UNORM, D2D1_ALPHA_MODE_IGNORE),
			0.0f, 0.0f, D2D1_RENDER_TARGET_USAGE_NONE, D2D1_FEATURE_LEVEL_DEFAULT),
		D2D1::HwndRenderTargetProperties(
			m_hWnd,
			D2D1::SizeU(UINT32(width), UINT32(height))),
		&m_pRT
		);

	if (FAILED(hr))
		return;

	ensureBrushes();

	// Text formats
	SafeRelease(m_pTextFormat);
	SafeRelease(m_pTextFormatSmall);
	SafeRelease(m_pTextFormatBig);

	if (m_pD2DWriteFactory)
	{
		m_pD2DWriteFactory->CreateTextFormat(
			msc_fontName, NULL,
			DWRITE_FONT_WEIGHT_SEMI_BOLD,
			DWRITE_FONT_STYLE_NORMAL,
			DWRITE_FONT_STRETCH_NORMAL,
			14.0f, L"", &m_pTextFormat);

		m_pD2DWriteFactory->CreateTextFormat(
			msc_fontName, NULL,
			DWRITE_FONT_WEIGHT_NORMAL,
			DWRITE_FONT_STYLE_NORMAL,
			DWRITE_FONT_STRETCH_NORMAL,
			11.0f, L"", &m_pTextFormatSmall);

		m_pD2DWriteFactory->CreateTextFormat(
			msc_fontName, NULL,
			DWRITE_FONT_WEIGHT_BOLD,
			DWRITE_FONT_STYLE_NORMAL,
			DWRITE_FONT_STRETCH_NORMAL,
			42.0f, L"", &m_pTextFormatBig);
	}
}

/// <summary>
/// Applies the current D2D color scheme
/// </summary>
void KBodyVisualizer::ensureBrushes()
{
	if (!m_pRT)
		return;

	m_pRT->CreateSolidColorBrush(D2D1::ColorF(0.102f, 0.110f, 0.137f), &m_bgBrush);
	m_pRT->CreateSolidColorBrush(D2D1::ColorF(0.22f, 0.24f, 0.30f), &m_gridBrush);
	m_pRT->CreateSolidColorBrush(D2D1::ColorF(0.32f, 0.36f, 0.45f), &m_axisBrush);
	m_pRT->CreateSolidColorBrush(D2D1::ColorF(0.14f, 0.18f, 0.26f, 0.72f), &m_floorFillBrush);
	m_pRT->CreateSolidColorBrush(D2D1::ColorF(0.42f, 0.52f, 0.68f, 0.35f), &m_floorHatchBrush);
	m_pRT->CreateSolidColorBrush(D2D1::ColorF(0.42f, 0.88f, 0.50f), &m_boneTracked);
	m_pRT->CreateSolidColorBrush(D2D1::ColorF(0.45f, 0.48f, 0.55f), &m_boneInferred);
	m_pRT->CreateSolidColorBrush(D2D1::ColorF(0.90f, 0.95f, 1.00f), &m_jointTracked);
	m_pRT->CreateSolidColorBrush(D2D1::ColorF(1.00f, 0.86f, 0.30f), &m_jointInferred);
	m_pRT->CreateSolidColorBrush(D2D1::ColorF(0.35f, 0.38f, 0.45f), &m_ghostBrush);
	m_pRT->CreateSolidColorBrush(D2D1::ColorF(0.86f, 0.88f, 0.92f), &m_textBrush);
	m_pRT->CreateSolidColorBrush(D2D1::ColorF(0.55f, 0.58f, 0.66f), &m_dimTextBrush);
	m_pRT->CreateSolidColorBrush(D2D1::ColorF(0.48f, 0.64f, 0.97f), &m_accentBrush);
	m_pRT->CreateSolidColorBrush(D2D1::ColorF(0.90f, 0.28f, 0.30f), &m_redBrush);
	m_pRT->CreateSolidColorBrush(D2D1::ColorF(0.31f, 0.86f, 0.51f), &m_greenBrush);
	m_pRT->CreateSolidColorBrush(D2D1::ColorF(1.00f, 0.55f, 0.00f), &m_orangeBrush);
	m_pRT->CreateSolidColorBrush(D2D1::ColorF(0.98f, 0.85f, 0.30f), &m_yellowBrush);
	m_pRT->CreateSolidColorBrush(D2D1::ColorF(0.30f, 0.55f, 1.00f), &m_blueBrush);
	m_pRT->CreateSolidColorBrush(D2D1::ColorF(0.16f, 0.30f, 0.55f, 0.85f), &m_meshFillBrush);
	m_pRT->CreateSolidColorBrush(D2D1::ColorF(0.55f, 0.72f, 1.00f), &m_meshWireBrush);

	createDepthPalette();
}

/// <summary>
/// Creates the depth point-cloud color palette (near = warm, far = cool)
/// </summary>
void KBodyVisualizer::createDepthPalette()
{
	if (!m_pRT)
		return;

	for (int i = 0; i < 16; ++i)
	{
		SafeRelease(m_depthPalette[i]);
		m_depthPalette[i] = NULL;
	}

	for (int i = 0; i < 16; ++i)
	{
		float t = float(i) / 15.0f;
		// Red (near) -> orange -> yellow -> cyan -> blue (far)
		D2D1::ColorF c(
			0.95f - t * 0.75f,
			0.25f + t * 0.40f,
			0.25f + t * 0.75f);
		m_pRT->CreateSolidColorBrush(c, &m_depthPalette[i]);
	}
}

/// <summary>
/// Redraw window
/// </summary>
bool KBodyVisualizer::update()
{
	if (!m_hWnd || !m_pRT)
		return false;

	PAINTSTRUCT ps;
	BeginPaint(m_hWnd, &ps);

	m_pRT->BeginDraw();

	// Background
	m_pRT->Clear(D2D1::ColorF(0.102f, 0.110f, 0.137f));

	drawScene();
	drawOverlays();

	HRESULT hr = m_pRT->EndDraw();

	// Device lost, recreate the render target
	if (hr == D2DERR_RECREATE_TARGET)
	{
		RECT rc;
		GetClientRect(m_hWnd, &rc);
		resize(rc.right - rc.left, rc.bottom - rc.top);
	}

	EndPaint(m_hWnd, &ps);

	// Update FPS counter
	LARGE_INTEGER qpcNow = { 0 };
	if (m_fFreq && QueryPerformanceCounter(&qpcNow))
	{
		m_nFramesSinceUpdate++;
		if (m_nLastCounter)
		{
			double elapsed = double(qpcNow.QuadPart - m_nLastCounter) / m_fFreq;
			if (elapsed > 0.25)
			{
				m_fps = float(m_nFramesSinceUpdate / elapsed);
				m_nFramesSinceUpdate = 0;
				m_nLastCounter = qpcNow.QuadPart;
			}
		}
		else
		{
			m_nLastCounter = qpcNow.QuadPart;
		}
	}

	return true;
}

/// <summary>
/// Notify class about a frame that arrived
/// </summary>
void KBodyVisualizer::notify(IBodyFrame *bFrame, INT64 frameTime)
{
	// If visualizer is not attached to a window, just return
	if (!m_hWnd)
		return;

	// If the live view is paused, keep the last displayed pose
	if (g_session.livePaused)
		return;

	// Use the general notifier first, it will save the bodies of the current frame
	KBodyReader::notify(bFrame, frameTime);

	// The body frame carries the floor clip plane (if the sensor can see the floor)
	// Plane equation: n·p + w = 0  (Kinect SDK, normalized) — force Y up to avoid 180° X flip
	Vector4 floorPlane;
	if (SUCCEEDED(bFrame->get_FloorClipPlane(&floorPlane)))
	{
		float nx = floorPlane.x, ny = floorPlane.y, nz = floorPlane.z;
		float len = sqrtf(nx * nx + ny * ny + nz * nz);
		if (len > 0.5f)
		{
			float inv = 1.0f / len;
			nx*=inv; ny*=inv; nz*=inv; float w= floorPlane.w * inv;
			if (ny < 0) { nx=-nx; ny=-ny; nz=-nz; w=-w; }
			{
				std::lock_guard<std::mutex> lk(m_floorMutex);
				m_floorPlane[0] = nx;
				m_floorPlane[1] = ny;
				m_floorPlane[2] = nz;
				m_floorPlane[3] = w;
			}
			// Distance from the sensor (origin) to the floor plane
			m_floorHeight = fabsf(w);
			m_hasFloor = true;
		}
		else
		{
			m_hasFloor = false;
		}
	}

	if (!m_pBodyReadStatus)
		return;

	receiveBodies();

	// Ask the UI thread to repaint
	InvalidateRect(m_hWnd, NULL, FALSE);
}

/// <summary>
/// Reads the latest body data from the sensor and stores it
/// </summary>
void KBodyVisualizer::receiveBodies()
{
	// Lock mutex when running this method
	std::unique_lock<std::timed_mutex> lockB(*_m_pbodyUpdateMutex, std::defer_lock);
	if (!lockB.try_lock_for(std::chrono::milliseconds(300)))
		return;

	std::unique_lock<std::timed_mutex> lockLive(m_liveMutex, std::defer_lock);
	if (!lockLive.try_lock_for(std::chrono::milliseconds(100)))
		return;

	static INT64 s_lastFrameTime = -1;

	float dt = 0.0f;
	if (s_lastFrameTime > 0 && m_tlatestFrameTime > s_lastFrameTime)
		dt = float((m_tlatestFrameTime - s_lastFrameTime) / 1e7);
	if (dt > 1.0f) dt = 1.0f;   // clamp long gaps
	s_lastFrameTime = m_tlatestFrameTime;

	m_liveBodies.clear();

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
		if (FAILED(pBody->GetJoints(_countof(joints), joints)))
			continue;

		// Smooth the raw stream to avoid jitter
		m_smoother.smooth(trackingId, joints, dt);

		LiveBody lb;
		lb.trackingId = trackingId;
		lb.isTracked = true;

		HandState handLeft = HandState_NotTracked;
		HandState handRight = HandState_NotTracked;
		pBody->get_HandLeftState(&handLeft);
		pBody->get_HandRightState(&handRight);
		lb.leftHand = (int)handLeft;
		lb.rightHand = (int)handRight;

		// Geometric fist detection: a closed fist tucks the finger tips near the
		// palm. Kinect v2 HandState is unreliable, so we also measure the distance
		// from the hand joint to the hand tip and thumb joints (small = fist).
		auto jointDistance = [&joints](JointType a, JointType b) -> float
		{
			const CameraSpacePoint &pa = joints[a].Position;
			const CameraSpacePoint &pb = joints[b].Position;
			float dx = pa.X - pb.X, dy = pa.Y - pb.Y, dz = pa.Z - pb.Z;
			return sqrtf(dx * dx + dy * dy + dz * dz);
		};
		lb.fistDistLeft = jointDistance(JointType_HandLeft, JointType_HandTipLeft)
			+ jointDistance(JointType_HandLeft, JointType_ThumbLeft);
		lb.fistDistRight = jointDistance(JointType_HandRight, JointType_HandTipRight)
			+ jointDistance(JointType_HandRight, JointType_ThumbRight);
		lb.fingersLeftTracked =
			joints[JointType_HandLeft].TrackingState != TrackingState_NotTracked &&
			joints[JointType_HandTipLeft].TrackingState != TrackingState_NotTracked &&
			joints[JointType_ThumbLeft].TrackingState != TrackingState_NotTracked;
		lb.fingersRightTracked =
			joints[JointType_HandRight].TrackingState != TrackingState_NotTracked &&
			joints[JointType_HandTipRight].TrackingState != TrackingState_NotTracked &&
			joints[JointType_ThumbRight].TrackingState != TrackingState_NotTracked;

		for (int j = 0; j < JointType_Count; ++j)
		{
			lb.positions[j][0] = joints[j].Position.X;
			lb.positions[j][1] = joints[j].Position.Y;
			lb.positions[j][2] = joints[j].Position.Z;
			lb.states[j] = joints[j].TrackingState;
		}

		m_liveBodies.push_back(lb);
	}

	// Fist-gesture countdown: when any tracked body closes both fists and holds
	// them closed for 5 seconds, toggle recording (start when idle, stop when
	// recording). Releasing a fist cancels the countdown.
	static const ULONGLONG c_GestureCountdownMS = 5000;
	// A hand counts as a fist when the Kinect reports HandState_Closed. As a
	// fallback (only when the finger joints are tracked), the finger tips must
	// be tucked very close to the palm (sum of hand->tip and hand->thumb, m).
	static const float c_FistGeoDist = 0.10f;

	bool bothFistsClosed = false;
	for (size_t b = 0; b < m_liveBodies.size(); ++b)
	{
		const LiveBody &lb = m_liveBodies[b];
		if (!lb.isTracked)
			continue;

		bool fistLeft = (lb.leftHand == (int)HandState_Closed) ||
			(lb.fingersLeftTracked && lb.fistDistLeft < c_FistGeoDist);
		bool fistRight = (lb.rightHand == (int)HandState_Closed) ||
			(lb.fingersRightTracked && lb.fistDistRight < c_FistGeoDist);

		if (fistLeft && fistRight)
		{
			bothFistsClosed = true;
			break;
		}
	}

	ULONGLONG nowMS = (ULONGLONG)std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::steady_clock::now().time_since_epoch()).count();
	bool gestureEnabled = g_session.gestureRecord;
	if (bothFistsClosed && gestureEnabled)
	{
		if (!m_gestureActive.load())
		{
			// A new gesture starts on the Kinect thread, the UI thread handles
			// the actual recording toggle once the countdown completes.
			m_gestureActive = true;
			m_gestureForStart = !(m_pExporter && m_pExporter->recordingStatus());
			m_gestureStartMS = nowMS;
		}
		else if (nowMS - m_gestureStartMS.load() >= c_GestureCountdownMS)
		{
			m_gestureActive = false;
			// The visualizer is attached to the view window; the recording toggle
			// is handled by the main window, so post to the top-level ancestor.
			HWND hMain = GetAncestor(m_hWnd, GA_ROOT);
			if (!hMain)
				hMain = m_hWnd;
			PostMessage(hMain, WM_GESTURE_RECORD, m_gestureForStart.load() ? 1 : 0, 0);
		}
	}
	else
	{
		m_gestureActive = false;
	}
}

/// <summary>
/// Projects a 3D camera-space point to screen space
/// </summary>
D2D1_POINT_2F KBodyVisualizer::project(const float p[3])
{
	// Rotate around the Y axis (yaw)
	float cy = cosf(m_yaw), sy = sinf(m_yaw);
	float x1 = p[0] * cy + p[2] * sy;
	float z1 = -p[0] * sy + p[2] * cy;

	// Rotate around the X axis (pitch)
	float cp = cosf(m_pitch), sp = sinf(m_pitch);
	float y2 = p[1] * cp - z1 * sp;
	float z2 = p[1] * sp + z1 * cp;

	float depth = z2 + m_dist;
	if (depth < 0.05f) depth = 0.05f;

	float focal = (m_height * 0.5f) / tanf(m_fov * 0.5f);

	float sx = m_width * 0.5f + (x1 * focal / depth) + m_panX;
	float screenY = m_height * 0.5f - (y2 * focal / depth) + m_panY;

	return D2D1::Point2F(sx, screenY);
}

void KBodyVisualizer::drawFloorHatched(const float n[3], float w, const float p0[3], const float u[3], const float v[3], float extent)
{
	if (!m_pRT || !m_floorFillBrush || !m_floorHatchBrush)
		return;

	// Fill the 4x4m floor quad (centered on p0, spanning u/v)
	float c00[3] = { p0[0] - u[0]*extent - v[0]*extent, p0[1] - u[1]*extent - v[1]*extent, p0[2] - u[2]*extent - v[2]*extent };
	float c10[3] = { p0[0] + u[0]*extent - v[0]*extent, p0[1] + u[1]*extent - v[1]*extent, p0[2] + u[2]*extent - v[2]*extent };
	float c11[3] = { p0[0] + u[0]*extent + v[0]*extent, p0[1] + u[1]*extent + v[1]*extent, p0[2] + u[2]*extent + v[2]*extent };
	float c01[3] = { p0[0] - u[0]*extent + v[0]*extent, p0[1] - u[1]*extent + v[1]*extent, p0[2] - u[2]*extent + v[2]*extent };

	ID2D1PathGeometry *path = nullptr;
	if (SUCCEEDED(m_pD2DFactory->CreatePathGeometry(&path)) && path)
	{
		ID2D1GeometrySink *sink = nullptr;
		if (SUCCEEDED(path->Open(&sink)) && sink)
		{
			sink->SetFillMode(D2D1_FILL_MODE_WINDING);
			sink->BeginFigure(project(c00), D2D1_FIGURE_BEGIN_FILLED);
			sink->AddLine(project(c10));
			sink->AddLine(project(c11));
			sink->AddLine(project(c01));
			sink->EndFigure(D2D1_FIGURE_END_CLOSED);
			sink->Close();
			m_pRT->FillGeometry(path, m_floorFillBrush);
			sink->Release();
		}
		path->Release();
	}

	// Diagonal hatching: lines along (u+v) direction, spaced 0.25m
	const float hatchStep = 0.28f;
	for (float d = -extent * 1.35f; d <= extent * 1.35f; d += hatchStep)
	{
		float a[3] = { p0[0] + u[0]*d + v[0]*(-extent*1.2f), p0[1] + u[1]*d + v[1]*(-extent*1.2f), p0[2] + u[2]*d + v[2]*(-extent*1.2f) };
		float b[3] = { p0[0] + u[0]*d + v[0]*( extent*1.2f), p0[1] + u[1]*d + v[1]*( extent*1.2f), p0[2] + u[2]*d + v[2]*( extent*1.2f) };
		// shift to get diagonal (add perpendicular)
		// Convert to (u+v) diagonal by shearing v along u
		m_pRT->DrawLine(project(a), project(b), m_floorHatchBrush, 1.0f);
	}
}

void KBodyVisualizer::drawGridOriented()
{
	if (!m_pRT)
		return;

	// Floor handling:
	// - Live (camera space): grid at p0 = -n*w, skeleton at camera positions → aligned.
	// - Playback (floor-baked): skeleton already transformed to floor at y=0, so grid must stay at y=0.
	//   The baked floor is the stable origin and must NOT move with the live Kinect.
	bool useRecordingFloor = false;
	float recN[3]={0,1,0}, recW=0;
	if (g_session.playing && m_pExporter && m_pExporter->hasTake() && m_pExporter->recording().hasFloor) {
		const KRecording &rec = m_pExporter->recording();
		recN[0]=rec.floorPlane[0]; recN[1]=rec.floorPlane[1]; recN[2]=rec.floorPlane[2]; recW=rec.floorPlane[3];
		useRecordingFloor = true;
	}
	bool hasFloor = (useRecordingFloor || (m_hasFloor.load() && g_session.floorAlign));
	float n[3] = { 0,1,0 }, w = 0, p0[3] = { 0,0,0 };
	if (useRecordingFloor) {
		// Playback of a floor-baked take: floor is at y=0 (origin), skeleton already aligned.
		// Keep grid horizontal at origin — do NOT displace to p0, otherwise skeleton (y~0.1) and grid (y=-1) misalign.
		n[0]=0; n[1]=1; n[2]=0; w=0; p0[0]=0; p0[1]=0; p0[2]=0;
	} else if (hasFloor)
	{
		{
			std::lock_guard<std::mutex> lk(m_floorMutex);
			n[0]=m_floorPlane[0]; n[1]=m_floorPlane[1]; n[2]=m_floorPlane[2]; w=m_floorPlane[3];
		}
		// Live floor: grid at its camera-space position p0 = -n*w
		p0[0]= -n[0]*w; p0[1]= -n[1]*w; p0[2]= -n[2]*w;
	}
	else
	{
		// Fallback: horizontal at y=0
		n[0]=0; n[1]=1; n[2]=0; w=0; p0[0]=0; p0[1]=0; p0[2]=0;
	}

	// Build orthonormal basis u/v spanning the floor plane
	float helper[3] = { 1,0,0 };
	if (fabsf(n[0]) > 0.9f) { helper[0]=0; helper[1]=0; helper[2]=1; }
	float u[3], v[3];
	cross3(n, helper, u); normalize3(u);
	cross3(n, u, v); normalize3(v);

	const float extent = 2.4f;
	const float step = 0.5f;

	// Hatched fill first (behind grid lines)
	if (hasFloor)
		drawFloorHatched(n, w, p0, u, v, extent);

	// Grid lines on the plane
	for (float d = -extent; d <= extent + 0.001f; d += step)
	{
		float a[3] = { p0[0] + u[0]*d + v[0]*(-extent), p0[1] + u[1]*d + v[1]*(-extent), p0[2] + u[2]*d + v[2]*(-extent) };
		float b[3] = { p0[0] + u[0]*d + v[0]*( extent), p0[1] + u[1]*d + v[1]*( extent), p0[2] + u[2]*d + v[2]*( extent) };
		m_pRT->DrawLine(project(a), project(b), m_gridBrush, 1.0f);
	}
	for (float d = -extent; d <= extent + 0.001f; d += step)
	{
		float a[3] = { p0[0] + v[0]*d + u[0]*(-extent), p0[1] + v[1]*d + u[1]*(-extent), p0[2] + v[2]*d + u[2]*(-extent) };
		float b[3] = { p0[0] + v[0]*d + u[0]*( extent), p0[1] + v[1]*d + u[1]*( extent), p0[2] + v[2]*d + u[2]*( extent) };
		m_pRT->DrawLine(project(a), project(b), m_gridBrush, 1.0f);
	}

	// Axis helpers anchored on the floor plane (projected)
	float xo[3] = { p0[0] - u[0]*extent, p0[1] - u[1]*extent, p0[2] - u[2]*extent };
	float xp[3] = { p0[0] + u[0]*extent, p0[1] + u[1]*extent, p0[2] + u[2]*extent };
	m_pRT->DrawLine(project(xo), project(xp), m_axisBrush, 1.5f);
	float zo[3] = { p0[0] - v[0]*extent, p0[1] - v[1]*extent, p0[2] - v[2]*extent };
	float zp[3] = { p0[0] + v[0]*extent, p0[1] + v[1]*extent, p0[2] + v[2]*extent };
	m_pRT->DrawLine(project(zo), project(zp), m_axisBrush, 1.5f);
	float yo[3] = { p0[0], p0[1], p0[2] };
	float yp[3] = { p0[0] + n[0]*1.8f, p0[1] + n[1]*1.8f, p0[2] + n[2]*1.8f };
	m_pRT->DrawLine(project(yo), project(yp), m_axisBrush, 1.5f);
}

/// <summary>
/// Draws the ground grid and the axis helpers at the given height
/// </summary>
void KBodyVisualizer::drawGrid(float floorY)
{
	// Legacy entry — delegate to oriented version.
	// floorY is ignored when a real floor plane is available and alignment is on;
	// otherwise it draws the horizontal grid at floorY (used as fallback).
	if (g_session.floorAlign && m_hasFloor.load())
	{
		drawGridOriented();
		return;
	}

	if (!m_pRT)
		return;

	const float extent = 2.0f;
	const float step = 0.5f;

	// Ground grid (XZ plane at floorY)
	for (float x = -extent; x <= extent + 0.001f; x += step)
	{
		float a[3] = { x, floorY, -extent };
		float b[3] = { x, floorY, extent };
		m_pRT->DrawLine(project(a), project(b), m_gridBrush, 1.0f);
	}
	for (float z = -extent; z <= extent + 0.001f; z += step)
	{
		float a[3] = { -extent, floorY, z };
		float b[3] = { extent, floorY, z };
		m_pRT->DrawLine(project(a), project(b), m_gridBrush, 1.0f);
	}

	// Axis helpers
	float xo[3] = { -extent, floorY, 0.0f };
	float xp[3] = { extent, floorY, 0.0f };
	m_pRT->DrawLine(project(xo), project(xp), m_axisBrush, 1.5f);

	float zo[3] = { 0.0f, floorY, -extent };
	float zp[3] = { 0.0f, floorY, extent };
	m_pRT->DrawLine(project(zo), project(zp), m_axisBrush, 1.5f);

	float yo[3] = { 0.0f, floorY, 0.0f };
	float yp[3] = { 0.0f, floorY + 1.8f, 0.0f };
	m_pRT->DrawLine(project(yo), project(yp), m_axisBrush, 1.5f);
}

/// <summary>
/// Draws a skeleton given its joint positions and tracking states
/// </summary>
void KBodyVisualizer::drawSkeleton(const float positions[][3], const int states[], bool ghost)
{
	if (!m_pRT)
		return;

	// Bones first (so joints are drawn on top)
	for (int i = 0; i < c_BoneCount; ++i)
	{
		drawBone(positions, states, c_Bones[i][0], c_Bones[i][1], ghost);
	}

	// Joints on top
	for (int j = 0; j < JointType_Count; ++j)
	{
		if (states[j] == TrackingState_NotTracked)
			continue;

		D2D1_POINT_2F pt = project(positions[j]);

		float r = ghost ? 3.5f : 4.5f;
		D2D1_ELLIPSE ellipse = D2D1::Ellipse(pt, r, r);

		if (ghost)
		{
			m_pRT->FillEllipse(ellipse, m_ghostBrush);
		}
		else if (states[j] == TrackingState_Inferred)
		{
			m_pRT->FillEllipse(ellipse, m_jointInferred);
		}
		else
		{
			m_pRT->FillEllipse(ellipse, m_jointTracked);
		}
	}
}

/// <summary>
/// Draws a single bone
/// </summary>
void KBodyVisualizer::drawBone(const float positions[][3], const int states[], int joint0, int joint1, bool ghost)
{
	int state0 = states[joint0];
	int state1 = states[joint1];

	// If we can't find either of these joints, exit
	if ((state0 == TrackingState_NotTracked) || (state1 == TrackingState_NotTracked))
		return;

	// Don't draw if both points are inferred
	if ((state0 == TrackingState_Inferred) && (state1 == TrackingState_Inferred))
		return;

	D2D1_POINT_2F p0 = project(positions[joint0]);
	D2D1_POINT_2F p1 = project(positions[joint1]);

	if (ghost)
	{
		// Dashed line for the ghost reference
		m_pRT->DrawLine(p0, p1, m_ghostBrush, 2.0f);
		return;
	}

	if ((state0 == TrackingState_Tracked) && (state1 == TrackingState_Tracked))
	{
		m_pRT->DrawLine(p0, p1, m_boneTracked, 6.0f);
	}
	else
	{
		m_pRT->DrawLine(p0, p1, m_boneInferred, 2.5f);
	}
}

/// <summary>
/// Draws the live pose or the playback pose
/// </summary>
void KBodyVisualizer::drawScene()
{
	if (!m_pRT)
		return;

	// If floor alignment is enabled, place the grid on the detected floor
	float floorY = 0.0f;
	if (g_session.floorAlign && m_hasFloor)
		floorY = -m_floorHeight;

	drawGrid(floorY);

	// Depth / IR point cloud (behind the skeleton)
	if (g_session.depthView)
		drawDepthCloud();

	// Kinect camera frustum
	if (g_session.cameraView)
		drawKinectCamera();

	bool anythingDrawn = false;

	// Reference ghost pose (the editable pose used by the position mapping)
	const float (*ghostPos)[3] = NULL;
	if (g_session.ghost)
	{
		ghostPos = (m_pRetargeter)
			? (const float (*)[3])m_pRetargeter->ghostPositions()
			: g_ghostJointPositions;

		int ghostStates[JointType_Count];
		for (int j = 0; j < JointType_Count; ++j)
			ghostStates[j] = TrackingState_Tracked;

		drawSkeleton(ghostPos, ghostStates, true);
	}

	// Playback frame
	if (g_session.playing && m_pExporter && m_pExporter->hasTake())
	{
		KSampleFrame frame;
		if (m_pExporter->getFrameCopy(g_session.playheadMS, frame))
		{
			for (size_t b = 0; b < frame.bodies.size(); ++b)
			{
				const KSampleBody &body = frame.bodies[b];
				if (!body.isTracked)
					continue;

				float positions[JointType_Count][3];
				int states[JointType_Count];

				for (int j = 0; j < JointType_Count; ++j)
				{
					positions[j][0] = body.joints[j].position[0];
					positions[j][1] = body.joints[j].position[1];
					positions[j][2] = body.joints[j].position[2];
					states[j] = body.joints[j].trackingState;
				}

				drawSkeleton(positions, states, false);
				anythingDrawn = true;
			}
		}
	}
	else
	{
		// Live pose
		std::unique_lock<std::timed_mutex> lockLive(m_liveMutex, std::defer_lock);
		if (lockLive.try_lock_for(std::chrono::milliseconds(50)))
		{
			for (size_t b = 0; b < m_liveBodies.size(); ++b)
			{
				const LiveBody &lb = m_liveBodies[b];
				if (!lb.isTracked)
					continue;

				drawSkeleton(lb.positions, lb.states, false);
				anythingDrawn = true;
			}
		}
	}

	// Retarget preview: in Retarget workspace always, in Mocap when "Show Retarget Live" is checked
	if ((g_session.workspace == 1 || g_session.showRetargetLive) && m_pRetargeter && m_pRetargeter->hasCharacter())
	{
		drawRetarget();
		anythingDrawn = true;
	}

	// Selection highlight + ghost rotation gizmo (drawn on top of everything)
	if (g_session.workspace == 1 && g_session.ghost && m_selJoint >= 0
		&& m_pRetargeter && m_pRetargeter->hasCharacter())
	{
		if (!ghostPos)
			ghostPos = (const float (*)[3])m_pRetargeter->ghostPositions();
		drawSelectionOverlay(ghostPos);
		anythingDrawn = true;
	}

	if (!anythingDrawn)
	{
		// Placeholder message
		WCHAR msg[128];
		if (g_session.playing)
			StringCchCopy(msg, _countof(msg), L"No data in the recorded take");
		else
			StringCchCopy(msg, _countof(msg), L"Waiting for a body...\nStand in front of the Kinect");

		if (m_pTextFormat && m_pTextFormatSmall)
		{
			m_pRT->DrawText(msg, (UINT32)wcslen(msg), m_pTextFormatSmall,
				D2D1::RectF(0, m_height * 0.45f, FLOAT(m_width), m_height * 0.6f),
				m_dimTextBrush, D2D1_DRAW_TEXT_OPTIONS_NONE, DWRITE_MEASURING_MODE_NATURAL);
		}
	}
}

/// <summary>
/// Draws the depth/IR point cloud from the Kinect depth stream.
/// Every sampled depth pixel is unprojected into camera space and
/// drawn as a small colored dot (color = distance to the sensor).
/// </summary>
void KBodyVisualizer::drawDepthCloud()
{
	if (!m_pRT || !m_pDepthReader)
		return;

	UINT dw = 0, dh = 0;
	std::vector<UINT16> &depth = m_depthScratch;
	if (!m_pDepthReader->getDepth(dw, dh, depth))
		return;
	if (dw == 0 || dh == 0)
		return;

	// Kinect v2 depth camera intrinsic parameters (512x424)
	const float cx = 254.878f;
	const float cy = 205.395f;
	const float fx = 365.456f;
	const float fy = 365.456f;

	const int stride = 3;         // draw every 3rd pixel
	const float minDist = 0.35f;  // metres
	const float maxDist = 4.0f;   // metres
	const float halfW = 1.6f;     // half dot size in pixels

	for (UINT r = 0; r < dh; r += stride)
	{
		for (UINT c = 0; c < dw; c += stride)
		{
			UINT16 raw = depth[r * dw + c];
			if (raw == 0 || raw >= 8000)
				continue;

			float z = float(raw) * 0.001f;   // millimetres -> metres
			if (z < minDist || z > maxDist)
				continue;

			float x = (float(c) - cx) * z / fx;
			float y = (float(cy) - float(r)) * z / fy;

			float p[3] = { x, y, z };
			D2D1_POINT_2F sp = project(p);
			if (sp.x < -50.0f || sp.x > m_width + 50.0f ||
				sp.y < -50.0f || sp.y > m_height + 50.0f)
				continue;

			int bucket = int((z - minDist) / (maxDist - minDist) * 15.0f);
			if (bucket < 0) bucket = 0;
			if (bucket > 15) bucket = 15;

			m_pRT->FillRectangle(
				D2D1::RectF(sp.x - halfW, sp.y - halfW, sp.x + halfW, sp.y + halfW),
				m_depthPalette[bucket]);
		}
	}
}

/// <summary>
/// Draws the Kinect as a camera frustum at the scene origin.
/// The Kinect camera space already has its origin at the sensor.
/// </summary>
void KBodyVisualizer::drawKinectCamera()
{
	if (!m_pRT)
		return;

	// When playing a floor-baked take the floor is at y=0. The sensor
	// was at height |w| above it at capture time — show it there instead
	// of at the origin, so the frustum sits correctly above the hatched plane.
	float camOff[3]={0,0,0};
	if (g_session.playing && m_pExporter && m_pExporter->hasTake() && m_pExporter->recording().hasFloor) {
		const KRecording &rec = m_pExporter->recording();
		float h = fabsf(rec.floorPlane[3]);
		// After floor alignment the floor normal is Y, sensor height is h
		camOff[0]=0; camOff[1]=h; camOff[2]=0;
	}

	// Kinect v2 field of view
	const float hfov = 70.6f * float(M_PI) / 180.0f;
	const float vfov = 60.0f * float(M_PI) / 180.0f;

	const float nearD = 0.25f;
	const float farD = 3.5f;

	float hn = tanf(hfov * 0.5f) * nearD;
	float vn = tanf(vfov * 0.5f) * nearD;
	float hf = tanf(hfov * 0.5f) * farD;
	float vf = tanf(vfov * 0.5f) * farD;

	// Far plane corners (relative to sensor)
	float fc[4][3] = {
		{ -hf,  vf, farD }, { hf,  vf, farD },
		{  hf, -vf, farD }, { -hf, -vf, farD },
	};

	// Near plane corners
	float nc[4][3] = {
		{ -hn,  vn, nearD }, { hn,  vn, nearD },
		{  hn, -vn, nearD }, { -hn, -vn, nearD },
	};
	for (int i=0;i<4;++i){ fc[i][0]+=camOff[0]; fc[i][1]+=camOff[1]; fc[i][2]+=camOff[2]; nc[i][0]+=camOff[0]; nc[i][1]+=camOff[1]; nc[i][2]+=camOff[2]; }

	// Edge lines from the apex (sensor) to the far corners
	for (int i = 0; i < 4; ++i)
	{
		float origin[3] = { camOff[0], camOff[1], camOff[2] };
		m_pRT->DrawLine(project(origin), project(fc[i]), m_accentBrush, 1.5f);
	}

	// Far rectangle
	for (int i = 0; i < 4; ++i)
		m_pRT->DrawLine(project(fc[i]), project(fc[(i + 1) % 4]), m_accentBrush, 2.0f);

	// Near rectangle
	for (int i = 0; i < 4; ++i)
		m_pRT->DrawLine(project(nc[i]), project(nc[(i + 1) % 4]), m_accentBrush, 1.5f);

	// Camera body (small box at the sensor)
	float b = 0.045f;
	float box[8][3] = {
		{ -b+camOff[0], -b+camOff[1], -0.05f+camOff[2] }, { b+camOff[0], -b+camOff[1], -0.05f+camOff[2] }, { b+camOff[0],  b+camOff[1], -0.05f+camOff[2] }, { -b+camOff[0],  b+camOff[1], -0.05f+camOff[2] },
		{ -b+camOff[0], -b+camOff[1],  0.00f+camOff[2] }, { b+camOff[0], -b+camOff[1],  0.00f+camOff[2] }, { b+camOff[0],  b+camOff[1],  0.00f+camOff[2] }, { -b+camOff[0],  b+camOff[1],  0.00f+camOff[2] },
	};
	int edges[12][2] = {
		{ 0,1 },{ 1,2 },{ 2,3 },{ 3,0 },
		{ 4,5 },{ 5,6 },{ 6,7 },{ 7,4 },
		{ 0,4 },{ 1,5 },{ 2,6 },{ 3,7 }
	};
	for (int i = 0; i < 12; ++i)
		m_pRT->DrawLine(project(box[edges[i][0]]), project(box[edges[i][1]]), m_redBrush, 2.0f);

	// Label near the sensor
	if (m_pTextFormatSmall)
	{
		D2D1_POINT_2F pos = project(fc[0]);
		WCHAR label[] = L"Kinect";
		m_pRT->DrawText(label, (UINT32)wcslen(label), m_pTextFormatSmall,
			D2D1::RectF(pos.x - 30, pos.y - 16, pos.x + 60, pos.y + 4),
			m_accentBrush, D2D1_DRAW_TEXT_OPTIONS_NONE, DWRITE_MEASURING_MODE_NATURAL);
	}
}

/// <summary>
/// Formats a millisecond value as mm:ss.cc into a static buffer
/// </summary>
static const WCHAR* formatTimeShort(INT64 ms)
{
	static WCHAR buf[32];
	INT64 total = ms / 1000;
	INT64 min = total / 60;
	INT64 sec = total % 60;
	INT64 cent = (ms % 1000) / 10;
	StringCchPrintf(buf, 32, L"%02lld:%02lld.%02lld", min, sec, cent);
	return buf;
}

/// <summary>
/// Draws the overlay texts (mode, fps, recording indicator, hints)
/// </summary>
void KBodyVisualizer::drawOverlays()
{
	if (!m_pRT)
		return;

	WCHAR mode[96];
	ID2D1SolidColorBrush *modeBrush = m_textBrush;

	if (g_session.recording)
	{
		StringCchPrintf(mode, _countof(mode), L"\x25CF  REC  %s", formatTimeShort(g_session.takeDurationMS));
		modeBrush = m_redBrush;
	}
	else if (g_session.playing)
	{
		StringCchPrintf(mode, _countof(mode), L"\x25B6 PLAYBACK  %s", formatTimeShort(g_session.playheadMS));
		modeBrush = m_accentBrush;
	}
	else if (g_session.livePaused)
	{
		StringCchPrintf(mode, _countof(mode), L"LIVE (paused)");
		modeBrush = m_dimTextBrush;
	}
	else
	{
		StringCchPrintf(mode, _countof(mode), L"LIVE");
		modeBrush = m_greenBrush;
	}

	if (m_pTextFormat)
	{
		m_pRT->DrawText(mode, (UINT32)wcslen(mode), m_pTextFormat,
			D2D1::RectF(12, 10, FLOAT(m_width), 40), modeBrush);

		// FPS top right
		WCHAR fps[64];
		StringCchPrintf(fps, _countof(fps), L"FPS: %.0f", m_fps);
		m_pRT->DrawText(fps, (UINT32)wcslen(fps), m_pTextFormatSmall,
			D2D1::RectF(FLOAT(m_width - 140), 12, FLOAT(m_width - 12), 30), m_dimTextBrush,
			D2D1_DRAW_TEXT_OPTIONS_NONE, DWRITE_MEASURING_MODE_NATURAL);

		// Tracked bodies top right (below fps)
		WCHAR bodies[64];
		size_t trackedCount = 0;
		int dbgLeft = -1, dbgRight = -1;
		float dbgFistL = -1.0f, dbgFistR = -1.0f;
		bool fingersL = false, fingersR = false;
		{
			std::unique_lock<std::timed_mutex> lockLive(m_liveMutex, std::defer_lock);
			if (lockLive.try_lock_for(std::chrono::milliseconds(20)))
			{
				for (size_t b = 0; b < m_liveBodies.size(); ++b)
				{
					if (m_liveBodies[b].isTracked)
					{
						if (trackedCount == 0)
						{
							dbgLeft = m_liveBodies[b].leftHand;
							dbgRight = m_liveBodies[b].rightHand;
							dbgFistL = m_liveBodies[b].fistDistLeft;
							dbgFistR = m_liveBodies[b].fistDistRight;
							fingersL = m_liveBodies[b].fingersLeftTracked;
							fingersR = m_liveBodies[b].fingersRightTracked;
						}
						trackedCount++;
					}
				}
			}
		}
		StringCchPrintf(bodies, _countof(bodies), L"Bodies: %d / 6", int(trackedCount));
		m_pRT->DrawText(bodies, (UINT32)wcslen(bodies), m_pTextFormatSmall,
			D2D1::RectF(FLOAT(m_width) - 140, 30, FLOAT(m_width) - 12, 48), m_dimTextBrush,
			D2D1_DRAW_TEXT_OPTIONS_NONE, DWRITE_MEASURING_MODE_NATURAL);

		// Fist-gesture debug status (hand states + geometric distance)
		if (dbgLeft >= 0)
		{
			WCHAR gs[160];
			const WCHAR *nl = (dbgLeft == (int)HandState_Closed) ? L"Closed" :
				(dbgLeft == (int)HandState_Open) ? L"Open" :
				(dbgLeft == (int)HandState_NotTracked) ? L"NoTrk" : L"Unknown";
			const WCHAR *nr = (dbgRight == (int)HandState_Closed) ? L"Closed" :
				(dbgRight == (int)HandState_Open) ? L"Open" :
				(dbgRight == (int)HandState_NotTracked) ? L"NoTrk" : L"Unknown";
			bool fL = (dbgLeft == (int)HandState_Closed) ||
				(fingersL && dbgFistL < 0.10f);
			bool fR = (dbgRight == (int)HandState_Closed) ||
				(fingersR && dbgFistR < 0.10f);
			StringCchPrintf(gs, _countof(gs), L"L:%s d=%.2f F=%s   R:%s d=%.2f F=%s   Fists:%s   Gesture:%s",
				nl, dbgFistL, fingersL ? L"y" : L"n",
				nr, dbgFistR, fingersR ? L"y" : L"n",
				(fL && fR) ? L"DETECTED" : L"no",
				g_session.gestureRecord ? L"ON" : L"OFF");
			m_pRT->DrawText(gs, (UINT32)wcslen(gs), m_pTextFormatSmall,
				D2D1::RectF(12, FLOAT(m_height) - 48, FLOAT(m_width) - 12, FLOAT(m_height) - 28), m_dimTextBrush,
				D2D1_DRAW_TEXT_OPTIONS_NONE, DWRITE_MEASURING_MODE_NATURAL);
		}

		// Hint at the bottom
		WCHAR hint[] = L"MMB: orbit   |   Shift+MMB: pan   |   Ctrl+MMB: zoom   |   Wheel: zoom   |   Home: reset   |   1/3/7: views";
		m_pRT->DrawText(hint, (UINT32)wcslen(hint), m_pTextFormatSmall,
			D2D1::RectF(0, FLOAT(m_height - 30), FLOAT(m_width), FLOAT(m_height - 10)),
			m_dimTextBrush,
			D2D1_DRAW_TEXT_OPTIONS_NONE, DWRITE_MEASURING_MODE_NATURAL);

		// Floor / depth status (below the bodies counter)
		WCHAR info[96];
		if (m_hasFloor)
			StringCchPrintf(info, _countof(info), L"Floor: %.2f m", m_floorHeight.load());
		else
			StringCchPrintf(info, _countof(info), L"Floor: --");
		m_pRT->DrawText(info, (UINT32)wcslen(info), m_pTextFormatSmall,
			D2D1::RectF(FLOAT(m_width) - 140, 48, FLOAT(m_width) - 12, 66), m_dimTextBrush,
			D2D1_DRAW_TEXT_OPTIONS_NONE, DWRITE_MEASURING_MODE_NATURAL);
	}

	// Fist-gesture countdown (big centered text)
	if (m_gestureActive.load() && m_pTextFormatBig)
	{
		static const ULONGLONG c_GestureCountdownMS = 5000;
		ULONGLONG nowMS = (ULONGLONG)std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::steady_clock::now().time_since_epoch()).count();
		ULONGLONG elapsed = nowMS - m_gestureStartMS.load();
		ULONGLONG remaining = (elapsed < c_GestureCountdownMS) ? (c_GestureCountdownMS - elapsed) : 0;
		int seconds = int((remaining + 999) / 1000);

		WCHAR count[96];
		if (m_gestureForStart.load())
			StringCchPrintf(count, _countof(count), L"RECORDING IN  %d", seconds);
		else
			StringCchPrintf(count, _countof(count), L"STOPPING IN  %d", seconds);

		ID2D1SolidColorBrush *countBrush = m_gestureForStart.load() ? m_greenBrush : m_redBrush;
		m_pRT->DrawText(count, (UINT32)wcslen(count), m_pTextFormatBig,
			D2D1::RectF(0, FLOAT(m_height) * 0.32f, FLOAT(m_width), FLOAT(m_height) * 0.75f),
			countBrush, D2D1_DRAW_TEXT_OPTIONS_NONE, DWRITE_MEASURING_MODE_NATURAL);
	}
}

/// <summary>
/// Formats a millisecond value as mm:ss.cc
/// </summary>
void KBodyVisualizer::formatTime(INT64 ms, WCHAR *out, int outLen)
{
	INT64 total = ms / 1000;
	INT64 min = total / 60;
	INT64 sec = total % 60;
	INT64 cent = (ms % 1000) / 10;
	StringCchPrintf(out, outLen, L"%02lld:%02lld.%02lld", min, sec, cent);
}

/// <summary>
/// Camera / mouse interaction.
///
/// Blender key bindings:
///   MMB drag            -> Orbit
///   Shift+MMB drag      -> Pan
///   Ctrl+MMB drag       -> Zoom (dolly)
///   Shift+Ctrl+MMB drag -> Dolly
///   Mouse wheel         -> Zoom
///   LMB drag            -> Orbit   (convenience, this app has no selection)
///   RMB drag            -> Pan     (convenience)
///
/// Orbit direction is intentionally OPPOSITE to the mouse movement:
///   - drag right -> the camera orbits left  (you see the left side)
///   - drag up    -> the camera pitches down (you see the underside)
/// This matches "invert all camera rotation axes".
///
/// Reason (see project()): increasing m_yaw moves a point in front of the
/// camera to the RIGHT on screen, which corresponds to the camera orbiting
/// LEFT around the scene; increasing m_pitch drops that point on screen,
/// which corresponds to the camera tilting UP (looking at the underside).
/// </summary>
void KBodyVisualizer::mouseDown(float x, float y, int button, bool shift, bool ctrl)
{
	m_draggingRotate = false;
	m_draggingPan = false;
	m_draggingDolly = false;

	// Ghost-pose editing (retarget workspace): click a ghost joint to select it,
	// or grab the gizmo around the selected joint's pivot to bend the limb.
	bool ghostEditing = g_session.workspace == 1 && g_session.ghost
		&& m_pRetargeter && m_pRetargeter->hasCharacter();

	if (button == 0 && ghostEditing)
	{
		// 1) Grab the 3-axis rotation gizmo (or Shift = twist around the limb)
		if (m_selJoint >= 0)
		{
			int par = m_pRetargeter->ghostParent(m_selJoint);
			const float *g = m_pRetargeter->ghostPositions();
			int pivot = (par >= 0) ? par : m_selJoint;
			float pv[3] = { g[3 * pivot + 0], g[3 * pivot + 1], g[3 * pivot + 2] };
			D2D1_POINT_2F sp = project(pv);

			int a = gizmoPickAxis(x, y);
			float dx = x - sp.x, dy = y - sp.y;
			bool nearCenter = (dx * dx + dy * dy < 70.0f * 70.0f);
			if (a >= 0 || (shift && nearCenter))
			{
				m_gizmoDrag = true;
				m_gizmoShift = shift;
				m_gizmoAxis = a;
				m_gizmoCenterX = sp.x;
				m_gizmoCenterY = sp.y;
				m_gizmoStartAngle = atan2f(y - sp.y, x - sp.x);
				m_lastGizmoX = x;
				m_lastGizmoY = y;
				return;
			}
		}

		// 2) Pick a ghost joint to select it (and its mapped character bone)
		int j = pickGhostJoint(x, y);
		if (j >= 0)
		{
			m_selJoint = j;
			m_selBone = m_pRetargeter->getMapping(j);
			InvalidateRect(m_hWnd, NULL, FALSE);
			return;
		}
	}

	if (button == 1)   // middle mouse
	{
		if (shift && ctrl)
			m_draggingDolly = true;       // Shift+Ctrl+MMB = Dolly
		else if (shift)
			m_draggingPan = true;         // Shift+MMB = Pan
		else if (ctrl)
			m_draggingDolly = true;       // Ctrl+MMB = Zoom (dolly)
		else
			m_draggingRotate = true;      // MMB = Orbit
	}
	else if (button == 0)   // left mouse
	{
		m_draggingRotate = true;          // LMB = Orbit
	}
	else   // right mouse
	{
		m_draggingPan = true;             // RMB = Pan
	}

	m_lastMouseX = x;
	m_lastMouseY = y;
}

void KBodyVisualizer::mouseMove(float x, float y)
{
	float dx = x - m_lastMouseX;
	float dy = y - m_lastMouseY;
	m_lastMouseX = x;
	m_lastMouseY = y;

	// Ghost rotation gizmo: drag -> rotate the selected joint's subtree
	if (m_gizmoDrag && m_pRetargeter && m_selJoint >= 0)
	{
		// Rotate angle = the mouse's angular motion around the pivot
		float ang = atan2f(y - m_gizmoCenterY, x - m_gizmoCenterX);
		float delta = ang - m_gizmoStartAngle;
		m_gizmoStartAngle = ang;

		if (m_gizmoShift)
		{
			// Twist / roll around the bone axis (joint -> pivot)
			int par = m_pRetargeter->ghostParent(m_selJoint);
			const float *g = m_pRetargeter->ghostPositions();
			int pivot = (par >= 0) ? par : m_selJoint;
			float axis[3] = {
				g[3 * m_selJoint + 0] - g[3 * pivot + 0],
				g[3 * m_selJoint + 1] - g[3 * pivot + 1],
				g[3 * m_selJoint + 2] - g[3 * pivot + 2],
			};
			applyGhostRotation(m_selJoint, axis, delta);
		}
		else if (m_gizmoAxis >= 0)
		{
			static const float axes[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
			const float *A = axes[m_gizmoAxis];
			// Sign so counter-clockwise screen motion rotates around the axis
			// when it points toward the viewer (camera-forward frame)
			float cy = cosf(m_yaw), sy = sinf(m_yaw);
			float cp = cosf(m_pitch), sp = sinf(m_pitch);
			float fwd[3] = { -sy * cp, sp, cy * cp };
			float sign = (A[0] * fwd[0] + A[1] * fwd[1] + A[2] * fwd[2] >= 0.0f)
				? 1.0f : -1.0f;
			applyGhostRotation(m_selJoint, A, delta * sign);
		}
		InvalidateRect(m_hWnd, NULL, FALSE);
		return;
	}

	if (m_draggingRotate)
	{
		// Camera is OPPOSITE to the mouse (inverted axes)
		m_yaw -= dx * 0.008f;
		m_pitch -= dy * 0.008f;
		if (m_pitch > 1.5f) m_pitch = 1.5f;
		if (m_pitch < -1.5f) m_pitch = -1.5f;
		InvalidateRect(m_hWnd, NULL, FALSE);
	}
	else if (m_draggingPan)
	{
		m_panX += dx;
		m_panY += dy;
		InvalidateRect(m_hWnd, NULL, FALSE);
	}
	else if (m_draggingDolly)
	{
		m_dist *= (1.0f + dy * 0.01f);
		if (m_dist < 0.3f) m_dist = 0.3f;
		if (m_dist > 20.0f) m_dist = 20.0f;
		InvalidateRect(m_hWnd, NULL, FALSE);
	}
}

void KBodyVisualizer::mouseUp()
{
	m_gizmoDrag = false;
	m_gizmoAxis = -1;
	m_draggingRotate = false;
	m_draggingPan = false;
	m_draggingDolly = false;
}

void KBodyVisualizer::mouseWheel(float delta)
{
	m_dist *= (delta > 0.0f) ? 0.90f : 1.12f;
	if (m_dist < 0.3f) m_dist = 0.3f;
	if (m_dist > 20.0f) m_dist = 20.0f;
	InvalidateRect(m_hWnd, NULL, FALSE);
}

void KBodyVisualizer::resetCamera()
{
	m_yaw = 0.0f;
	m_pitch = -0.15f;
	m_dist = 3.8f;
	m_panX = 0.0f;
	m_panY = 0.0f;
	InvalidateRect(m_hWnd, NULL, FALSE);
}

/// <summary>
/// Quick view presets (Blender-like): 0 front, 1 right, 2 top, 3 Kinect view
/// </summary>
void KBodyVisualizer::setViewPreset(int preset)
{
	switch (preset)
	{
	case 0:   // front (+Z)
		m_yaw = 0.0f;
		m_pitch = 0.0f;
		m_dist = 4.0f;
		break;
	case 1:   // right (looking along -X)
		m_yaw = -1.5708f;
		m_pitch = 0.0f;
		m_dist = 4.0f;
		break;
	case 2:   // top (looking down)
		m_yaw = 0.0f;
		m_pitch = -1.5708f;
		m_dist = 4.5f;
		break;
	case 3:   // from the Kinect sensor, looking down its optical axis
	default:
		m_yaw = 0.0f;
		m_pitch = 0.0f;
		m_dist = 0.8f;
		break;
	}
	m_panX = 0.0f;
	m_panY = 0.0f;
	InvalidateRect(m_hWnd, NULL, FALSE);
}

/// <summary>
/// Draws the imported character (retarget preview): skeleton + skinned mesh
/// </summary>
void KBodyVisualizer::drawRetarget()
{
	if (!m_pRT || !m_pRetargeter)
		return;

	// Current pose: from the recorded take when playing, rest pose otherwise.
	// When "Show Retarget Live" is enabled, show the live Kinect-driven retarget even in Mocap workspace.
	bool retargetPlaying = g_session.playing && m_pExporter && m_pExporter->hasTake();
	bool showLive = g_session.showRetargetLive && !retargetPlaying;
	if (retargetPlaying)
	{
		// Re-capture the motion anchor every time a new playback starts so the
		// motion is added relative to the take, not to an old reference.
		if (!m_wasRetargetPlaying && m_pRetargeter)
			m_pRetargeter->resetMotionAnchor();
		m_wasRetargetPlaying = true;

		KSampleFrame frame;
		if (m_pExporter->getFrameCopy(g_session.playheadMS, frame)) {
			// Final (reworked) pose through the node motion graph. An unlinked
			// Input node yields the T-pose; edits in the graph repaint live.
			const KRecording &rec = m_pExporter->recording();
			m_pRetargeter->poseFinal(rec, rec.frameIndexAt(g_session.playheadMS), 0);
		}
	}
	else if (showLive)
	{
		m_wasRetargetPlaying = false;
		// Live retarget: drive the character directly from the live bodies
		bool posed = false;
		{
			std::unique_lock<std::timed_mutex> lock(m_liveMutex, std::defer_lock);
			if (lock.try_lock_for(std::chrono::milliseconds(20))) {
				for (auto &lb : m_liveBodies) if (lb.isTracked) {
					KSampleFrame frame; frame.timeMS = 0;
					KSampleBody body; body.trackingId = lb.trackingId; body.isTracked = true;
					for (int j=0;j<JointType_Count;++j) {
						float p[3]={lb.positions[j][0], lb.positions[j][1], lb.positions[j][2]};
						// Floor-align live positions when floor is available (like recorded bake)
						if (g_session.floorAlign && m_hasFloor.load()) {
							float n[3], w, p0[3], tmp[3];
							{
								std::lock_guard<std::mutex> lk(m_floorMutex);
								n[0]=m_floorPlane[0]; n[1]=m_floorPlane[1]; n[2]=m_floorPlane[2]; w=m_floorPlane[3];
							}
							if (n[1] < 0) { n[0]=-n[0]; n[1]=-n[1]; n[2]=-n[2]; w=-w; }
							p0[0]= -n[0]*w; p0[1]= -n[1]*w; p0[2]= -n[2]*w;
							tmp[0]=p[0]-p0[0]; tmp[1]=p[1]-p0[1]; tmp[2]=p[2]-p0[2];
							float yAxis[3]={0,1,0};
							float R[3][3]; R[0][0]=1; R[0][1]=0; R[0][2]=0; R[1][0]=0; R[1][1]=1; R[1][2]=0; R[2][0]=0; R[2][1]=0; R[2][2]=1;
							float dot=n[0]*yAxis[0]+n[1]*yAxis[1]+n[2]*yAxis[2];
							if (dot < 0.99999f) {
								float ax[3]={n[1]*yAxis[2]-n[2]*yAxis[1], n[2]*yAxis[0]-n[0]*yAxis[2], n[0]*yAxis[1]-n[1]*yAxis[0]};
								float ang = (dot < -0.99999f) ? (float)M_PI : acosf(dot);
								float len=sqrtf(ax[0]*ax[0]+ax[1]*ax[1]+ax[2]*ax[2]); if(len>1e-6f){ax[0]/=len;ax[1]/=len;ax[2]/=len; float s=sinf(ang*0.5f), qw=cosf(ang*0.5f), qx=ax[0]*s, qy=ax[1]*s, qz=ax[2]*s;
								R[0][0]=1-2*(qy*qy+qz*qz); R[0][1]=2*(qx*qy-qz*qw); R[0][2]=2*(qx*qz+qy*qw); R[1][0]=2*(qx*qy+qz*qw); R[1][1]=1-2*(qx*qx+qz*qz); R[1][2]=2*(qy*qz-qx*qw); R[2][0]=2*(qx*qz-qy*qw); R[2][1]=2*(qy*qz+qx*qw); R[2][2]=1-2*(qx*qx+qy*qy);}
							}
							p[0]=R[0][0]*tmp[0]+R[0][1]*tmp[1]+R[0][2]*tmp[2];
							p[1]=R[1][0]*tmp[0]+R[1][1]*tmp[1]+R[1][2]*tmp[2];
							p[2]=R[2][0]*tmp[0]+R[2][1]*tmp[1]+R[2][2]*tmp[2];
						}
						body.joints[j].position[0]=p[0]; body.joints[j].position[1]=p[1]; body.joints[j].position[2]=p[2];
						body.joints[j].orientation[0]=0; body.joints[j].orientation[1]=0; body.joints[j].orientation[2]=0; body.joints[j].orientation[3]=1;
						body.joints[j].trackingState=lb.states[j];
					}
				frame.bodies.push_back(body);
				m_pRetargeter->poseFinalLive(body);
				posed = true;
					break;
				}
			}
		}
		if (!posed) m_pRetargeter->poseRest();
	}
	else
	{
		m_wasRetargetPlaying = false;
		m_pRetargeter->poseRest();
	}

	const RetargetPose &pose = m_pRetargeter->pose();
	if (!pose.valid)
		return;

	// Skinned mesh: prefer the D3D11 GPU path (skinning done on the GPU),
	// fall back to the CPU / D2D path when no D3D device is available.
	if (gpuRenderMesh() && m_pMeshBitmap)
	{
		m_pRT->DrawBitmap(m_pMeshBitmap,
			D2D1::RectF(0, 0, (float)m_width, (float)m_height),
			1.0f, D2D1_BITMAP_INTERPOLATION_MODE_LINEAR);
	}
	else
	{
		// CPU fallback: build a D2D path geometry and fill / stroke it
		const RetargetMesh &mesh = m_pRetargeter->mesh();
		if (mesh.valid && pose.skinnedPos.size() >= 3)
		{
			const std::vector<unsigned int> &idx = mesh.indices;
			const std::vector<float> &pos = pose.skinnedPos;
			size_t nTris = idx.size() / 3;

			ID2D1PathGeometry *path = NULL;
			if (SUCCEEDED(m_pD2DFactory->CreatePathGeometry(&path)) && path)
			{
				ID2D1GeometrySink *sink = NULL;
				if (SUCCEEDED(path->Open(&sink)) && sink)
				{
					sink->SetFillMode(D2D1_FILL_MODE_WINDING);
					for (size_t t = 0; t < nTris; ++t)
					{
						unsigned int i0 = idx[t * 3 + 0];
						unsigned int i1 = idx[t * 3 + 1];
						unsigned int i2 = idx[t * 3 + 2];

						float v0[3] = { pos[3 * i0 + 0], pos[3 * i0 + 1], pos[3 * i0 + 2] };
						float v1[3] = { pos[3 * i1 + 0], pos[3 * i1 + 1], pos[3 * i1 + 2] };
						float v2[3] = { pos[3 * i2 + 0], pos[3 * i2 + 1], pos[3 * i2 + 2] };

						// Skip triangles that pass behind the camera
						float sy = sinf(m_yaw), cy = cosf(m_yaw);
						float sp = sinf(m_pitch), cp = cosf(m_pitch);
						float depthOf[3];
						for (int k = 0; k < 3; ++k)
						{
							const float *p = (k == 0) ? v0 : (k == 1) ? v1 : v2;
							float z1 = -p[0] * sy + p[2] * cy;
							depthOf[k] = p[1] * sp + z1 * cp + m_dist;
						}
						if (depthOf[0] < 0.15f || depthOf[1] < 0.15f || depthOf[2] < 0.15f)
							continue;

						D2D1_POINT_2F a = project(v0);
						D2D1_POINT_2F b = project(v1);
						D2D1_POINT_2F c = project(v2);

						sink->BeginFigure(a, D2D1_FIGURE_BEGIN_FILLED);
						sink->AddLine(b);
						sink->AddLine(c);
						sink->EndFigure(D2D1_FIGURE_END_CLOSED);
					}
					sink->Close();
					m_pRT->FillGeometry(path, m_meshFillBrush);
					m_pRT->DrawGeometry(path, m_meshWireBrush, 1.0f);
					sink->Release();
				}
				path->Release();
			}
		}
	}

	// Skeleton lines on top of the mesh
	drawRetargetSkeleton(pose);
}

/// <summary>
/// Draws the retarget skeleton bones as D2D lines (shared by all paths)
/// </summary>
void KBodyVisualizer::drawRetargetSkeleton(const RetargetPose &pose)
{
	int nBones = m_pRetargeter->boneCount();
	for (int i = 0; i < nBones; ++i)
	{
		const RetargetBone &b = m_pRetargeter->bone(i);
		if (b.parent < 0)
			continue;
		D2D1_POINT_2F a = project(&pose.bonePos[3 * i]);
		D2D1_POINT_2F c = project(&pose.bonePos[3 * b.parent]);
		if (i == m_selBone)
		{
			m_pRT->DrawLine(a, c, m_orangeBrush, 5.0f);
			D2D1_ELLIPSE dot = D2D1::Ellipse(a, 6.0f, 6.0f);
			m_pRT->FillEllipse(dot, m_orangeBrush);
		}
		else
		{
			m_pRT->DrawLine(a, c, m_meshWireBrush, 2.0f);
		}
	}
	// Pipette logo next to hovered bone when eyedropper is active
	if (g_session.pipetteActive)
	{
		int hovered = pickRetargetBone(m_lastMouseX, m_lastMouseY);
		if (hovered >= 0 && hovered < nBones)
		{
			const float *p = &pose.bonePos[3 * hovered];
			D2D1_POINT_2F sp = project(p);
			// Draw pipette icon: outer accent circle + inner bg + drop tip
			D2D1_ELLIPSE outer = D2D1::Ellipse(D2D1::Point2F(sp.x + 14, sp.y - 14), 9, 9);
			m_pRT->FillEllipse(outer, m_accentBrush);
			D2D1_ELLIPSE inner = D2D1::Ellipse(D2D1::Point2F(sp.x + 14, sp.y - 14), 5, 5);
			m_pRT->FillEllipse(inner, m_bgBrush);
			// Drop tip line
			D2D1_POINT_2F tipA = D2D1::Point2F(sp.x + 14, sp.y - 5);
			D2D1_POINT_2F tipB = D2D1::Point2F(sp.x + 7, sp.y + 2);
			m_pRT->DrawLine(tipA, tipB, m_accentBrush, 2.0f);
		}
	}
}

/// <summary>
/// Finds the ghost joint whose projected position is closest to a screen point
/// </summary>
int KBodyVisualizer::pickGhostJoint(float sx, float sy)
{
	if (!m_pRetargeter)
		return -1;

	const float *g = m_pRetargeter->ghostPositions();
	const float threshold = 18.0f;
	int best = -1;
	float bestD = threshold * threshold;
	for (int j = 0; j < JointType_Count; ++j)
	{
		float p[3] = { g[3 * j + 0], g[3 * j + 1], g[3 * j + 2] };
		D2D1_POINT_2F sp = project(p);
		float dx = sp.x - sx, dy = sp.y - sy;
		float d = dx * dx + dy * dy;
		if (d < bestD) { bestD = d; best = j; }
	}
	return best;
}

/// <summary>
/// Finds the retarget bone whose projected joint is closest to a screen point
/// </summary>
int KBodyVisualizer::pickRetargetBone(float sx, float sy)
{
	if (!m_pRetargeter || !m_pRetargeter->hasCharacter())
		return -1;
	const RetargetPose &pose = m_pRetargeter->pose();
	if (!pose.valid)
		return -1;
	const float threshold = 22.0f;
	int best = -1;
	float bestD = threshold * threshold;
	int n = m_pRetargeter->boneCount();
	for (int i = 0; i < n; ++i)
	{
		const float *p = &pose.bonePos[3 * i];
		D2D1_POINT_2F sp = project(p);
		float dx = sp.x - sx, dy = sp.y - sy;
		float d = dx*dx + dy*dy;
		if (d < bestD) { bestD = d; best = i; }
	}
	return best;
}

/// <summary>
/// Draws the green ghost-joint / orange character-bone selection highlight,
/// the connector line and the rotation gizmo
/// </summary>
void KBodyVisualizer::drawSelectionOverlay(const float ghostPos[][3])
{
	if (!m_pRT || !m_pRetargeter)
		return;

	if (m_selJoint < 0)
		return;

	// Green: the selected ghost joint + the ghost bone that leads to it
	const float *gj = ghostPos[m_selJoint];
	D2D1_POINT_2F a = project(gj);

	int par = m_pRetargeter->ghostParent(m_selJoint);
	if (par >= 0)
	{
		D2D1_POINT_2F c = project(ghostPos[par]);
		m_pRT->DrawLine(a, c, m_greenBrush, 5.0f);
	}
	D2D1_ELLIPSE dot = D2D1::Ellipse(a, 8.0f, 8.0f);
	m_pRT->FillEllipse(dot, m_greenBrush);

	// Connector to the mapped character bone (yellow dashed)
	if (m_selBone >= 0 && m_selBone < m_pRetargeter->boneCount()
		&& m_pRetargeter->pose().valid)
	{
		const RetargetPose &pose = m_pRetargeter->pose();
		D2D1_POINT_2F b = project(&pose.bonePos[3 * m_selBone]);
		float dxx = b.x - a.x, dyy = b.y - a.y;
		float len = sqrtf(dxx * dxx + dyy * dyy);
		if (len > 1.0f)
		{
			float steps = len / 12.0f;
			float sx = dxx / steps, sy = dyy / steps;
			float cx = a.x, cy = a.y;
			bool on = true;
			for (int i = 0; i < (int)steps; ++i)
			{
				if (on)
					m_pRT->DrawLine(D2D1::Point2F(cx, cy),
						D2D1::Point2F(cx + sx, cy + sy), m_yellowBrush, 1.5f);
				cx += sx;
				cy += sy;
				on = !on;
			}
		}
	}

	// Rotation gizmo around the pivot (parent joint, or the joint itself when root)
	int pv = (par >= 0) ? par : m_selJoint;
	drawGizmo(ghostPos[pv]);
}

/// <summary>
/// Screen radius (px) used to draw / hit-test the rotation gizmo
/// </summary>
float KBodyVisualizer::gizmoRadius(const float pivot[3]) const
{
	float cy = cosf(m_yaw), sy = sinf(m_yaw);
	float cp = cosf(m_pitch), sp = sinf(m_pitch);
	float z1 = -pivot[0] * sy + pivot[2] * cy;
	float z2 = pivot[1] * sp + z1 * cp;
	float depth = z2 + m_dist;
	if (depth < 0.05f) depth = 0.05f;
	float focal = (m_height * 0.5f) / tanf(m_fov * 0.5f);
	return 70.0f * depth / focal;
}

/// <summary>
/// Draws the classic 3-axis rotation gizmo around a world-space pivot:
/// red/green/blue rotation circles (X/Y/Z world axes) with matching axis
/// guide lines. The active (grabbed) axis is drawn thicker.
/// </summary>
void KBodyVisualizer::drawGizmo(const float pivot[3])
{
	if (!m_pRT)
		return;

	D2D1_POINT_2F c = project(pivot);
	const float R = gizmoRadius(pivot);

	static const float axes[3][3] = {
		{ 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 }
	};
	ID2D1SolidColorBrush *brushes[3] = { m_redBrush, m_greenBrush, m_blueBrush };
	bool active[3] = { false, false, false };
	if (m_gizmoDrag && m_gizmoAxis >= 0)
		active[m_gizmoAxis] = true;

	// Guide lines along each world axis
	for (int a = 0; a < 3; ++a)
	{
		float p[3] = {
			pivot[0] + axes[a][0] * R * 0.85f,
			pivot[1] + axes[a][1] * R * 0.85f,
			pivot[2] + axes[a][2] * R * 0.85f,
		};
		m_pRT->DrawLine(c, project(p), brushes[a], active[a] ? 3.0f : 1.5f);
	}

	// Rotation circles (one per world axis)
	for (int a = 0; a < 3; ++a)
	{
		// Orthonormal basis spanning the plane perpendicular to axis a
		float t[3] = { 1, 0, 0 };
		if (fabsf(axes[a][0]) > 0.9f) { t[0] = 0; t[1] = 1; }
		float U[3], V[3];
		cross3(axes[a], t, U); normalize3(U);
		cross3(axes[a], U, V); normalize3(V);

		const int n = 64;
		float prevX = 0, prevY = 0;
		for (int i = 0; i <= n; ++i)
		{
			float th = float(i) / n * (2.0f * 3.14159265f);
			float co = cosf(th), si = sinf(th);
			float p[3] = {
				pivot[0] + R * (U[0] * co + V[0] * si),
				pivot[1] + R * (U[1] * co + V[1] * si),
				pivot[2] + R * (U[2] * co + V[2] * si),
			};
			D2D1_POINT_2F sp = project(p);
			if (i > 0)
				m_pRT->DrawLine(D2D1::Point2F(prevX, prevY), sp,
					brushes[a], active[a] ? 3.0f : 2.0f);
			prevX = sp.x;
			prevY = sp.y;
		}
	}

	// Center cross (the pivot itself)
	m_pRT->DrawLine(D2D1::Point2F(c.x - 5, c.y), D2D1::Point2F(c.x + 5, c.y), m_textBrush, 1.5f);
	m_pRT->DrawLine(D2D1::Point2F(c.x, c.y - 5), D2D1::Point2F(c.x, c.y + 5), m_textBrush, 1.5f);
}

/// <summary>
/// Returns the gizmo axis (0=X,1=Y,2=Z) whose rotation circle is nearest
/// to the screen point, or -1 when none is within reach
/// </summary>
int KBodyVisualizer::gizmoPickAxis(float sx, float sy)
{
	if (!m_pRetargeter || m_selJoint < 0)
		return -1;

	int par = m_pRetargeter->ghostParent(m_selJoint);
	const float *g = m_pRetargeter->ghostPositions();
	int pivot = (par >= 0) ? par : m_selJoint;
	float pv[3] = { g[3 * pivot + 0], g[3 * pivot + 1], g[3 * pivot + 2] };
	const float R = gizmoRadius(pv);

	static const float axes[3][3] = {
		{ 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 }
	};
	const float threshold = 16.0f;
	int best = -1;
	float bestD = threshold * threshold;

	for (int a = 0; a < 3; ++a)
	{
		float t[3] = { 1, 0, 0 };
		if (fabsf(axes[a][0]) > 0.9f) { t[0] = 0; t[1] = 1; }
		float U[3], V[3];
		cross3(axes[a], t, U); normalize3(U);
		cross3(axes[a], U, V); normalize3(V);

		const int n = 64;
		float px = 0, py = 0;
		bool first = true;
		for (int i = 0; i <= n; ++i)
		{
			float th = float(i) / n * (2.0f * 3.14159265f);
			float co = cosf(th), si = sinf(th);
			float p[3] = {
				pv[0] + R * (U[0] * co + V[0] * si),
				pv[1] + R * (U[1] * co + V[1] * si),
				pv[2] + R * (U[2] * co + V[2] * si),
			};
			D2D1_POINT_2F sp = project(p);
			if (!first)
			{
				// Point-to-segment distance
				float dx = sp.x - px, dy = sp.y - py;
				float len2 = dx * dx + dy * dy;
				float q = (len2 > 0.0f) ? ((sx - px) * dx + (sy - py) * dy) / len2 : 0.0f;
				if (q < 0.0f) q = 0.0f;
				if (q > 1.0f) q = 1.0f;
				float cx = px + q * dx, cy = py + q * dy;
				float ex = sx - cx, ey = sy - cy;
				float d = ex * ex + ey * ey;
				if (d < bestD) { bestD = d; best = a; }
			}
			px = sp.x;
			py = sp.y;
			first = false;
		}
	}
	return best;
}

/// <summary>
/// Rotates a ghost joint's subtree, mirroring the rotation to the opposite
/// limb when the session symmetry flag is on
/// </summary>
void KBodyVisualizer::applyGhostRotation(int joint, const float axis[3], float angle)
{
	if (!m_pRetargeter)
		return;

	m_pRetargeter->rotateGhost(joint, axis, angle);

	if (g_session.ghostSymmetry)
	{
		int mj = mirrorJoint(joint);
		if (mj != joint)
		{
			float ma[3] = { -axis[0], axis[1], axis[2] };
			m_pRetargeter->rotateGhost(mj, ma, -angle);
		}
	}
}

/// <summary>
/// Restores the ghost reference pose to the default T-pose
/// </summary>
void KBodyVisualizer::resetGhostPose()
{
	if (m_pRetargeter)
		m_pRetargeter->resetGhost();
	InvalidateRect(m_hWnd, NULL, FALSE);
}

/// <summary>
/// Creates the D3D11 device, shaders and input layout for GPU mesh rendering
/// </summary>
bool KBodyVisualizer::gpuInit()
{
	if (m_gpuReady)
		return true;

	HRESULT hr = D3D11CreateDevice(
		NULL, D3D_DRIVER_TYPE_HARDWARE, NULL,
		D3D11_CREATE_DEVICE_BGRA_SUPPORT | D3D11_CREATE_DEVICE_SINGLETHREADED,
		NULL, 0, D3D11_SDK_VERSION,
		&m_pD3DDevice, NULL, &m_pD3DCtx);
	if (FAILED(hr))
	{
		// No hardware device: fall back to WARP so the feature still works
		hr = D3D11CreateDevice(
			NULL, D3D_DRIVER_TYPE_WARP, NULL,
			D3D11_CREATE_DEVICE_BGRA_SUPPORT | D3D11_CREATE_DEVICE_SINGLETHREADED,
			NULL, 0, D3D11_SDK_VERSION,
			&m_pD3DDevice, NULL, &m_pD3DCtx);
	}
	if (FAILED(hr))
		return false;

	ID3DBlob *vsBlob = NULL, *psBlob = NULL, *errBlob = NULL;

	auto cleanup = [&]() {
		SafeRelease(errBlob);
		SafeRelease(vsBlob);
		SafeRelease(psBlob);
		gpuRelease();
	};

	hr = D3DCompile(c_vsRetarget, (SIZE_T)strlen(c_vsRetarget), NULL, NULL, NULL,
		"main", "vs_5_0", 0, 0, &vsBlob, &errBlob);
	if (FAILED(hr) || !vsBlob)
	{
		cleanup();
		return false;
	}

	hr = D3DCompile(c_psRetarget, (SIZE_T)strlen(c_psRetarget), NULL, NULL, NULL,
		"main", "ps_5_0", 0, 0, &psBlob, &errBlob);
	if (FAILED(hr) || !psBlob)
	{
		cleanup();
		return false;
	}

	hr = m_pD3DDevice->CreateVertexShader(vsBlob->GetBufferPointer(), vsBlob->GetBufferSize(), NULL, &m_pVS);
	if (FAILED(hr))
	{
		cleanup();
		return false;
	}
	hr = m_pD3DDevice->CreatePixelShader(psBlob->GetBufferPointer(), psBlob->GetBufferSize(), NULL, &m_pPS);
	if (FAILED(hr))
	{
		cleanup();
		return false;
	}

	D3D11_INPUT_ELEMENT_DESC layout[] =
	{
		{ "POSITION",   0, DXGI_FORMAT_R32G32B32_FLOAT,    0,  0, D3D11_INPUT_PER_VERTEX_DATA, 0 },
		{ "NORMAL",     0, DXGI_FORMAT_R32G32B32_FLOAT,    0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0 },
		{ "BONEINDEX",  0, DXGI_FORMAT_R32G32B32A32_UINT,  0, 24, D3D11_INPUT_PER_VERTEX_DATA, 0 },
		{ "BONEWEIGHT", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 40, D3D11_INPUT_PER_VERTEX_DATA, 0 },
	};
	hr = m_pD3DDevice->CreateInputLayout(layout, 4,
		vsBlob->GetBufferPointer(), vsBlob->GetBufferSize(), &m_pIL);
	if (FAILED(hr))
	{
		cleanup();
		return false;
	}

	D3D11_BUFFER_DESC bd = {};
	bd.Usage = D3D11_USAGE_DEFAULT;
	bd.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
	bd.ByteWidth = 96;
	hr = m_pD3DDevice->CreateBuffer(&bd, NULL, &m_pPerFrame);
	if (FAILED(hr))
	{
		cleanup();
		return false;
	}

	bd.ByteWidth = 256 * 64;
	hr = m_pD3DDevice->CreateBuffer(&bd, NULL, &m_pPerBone);
	if (FAILED(hr))
	{
		cleanup();
		return false;
	}

	D3D11_RASTERIZER_DESC rd = {};
	rd.FillMode = D3D11_FILL_SOLID;
	rd.CullMode = D3D11_CULL_NONE;
	rd.DepthClipEnable = TRUE;
	hr = m_pD3DDevice->CreateRasterizerState(&rd, &m_pRSSolid);
	if (FAILED(hr))
	{
		cleanup();
		return false;
	}

	rd.FillMode = D3D11_FILL_WIREFRAME;
	hr = m_pD3DDevice->CreateRasterizerState(&rd, &m_pRSWire);
	if (FAILED(hr))
	{
		cleanup();
		return false;
	}

	D3D11_DEPTH_STENCIL_DESC dd = {};
	dd.DepthEnable = TRUE;
	dd.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ALL;
	dd.DepthFunc = D3D11_COMPARISON_LESS;
	hr = m_pD3DDevice->CreateDepthStencilState(&dd, &m_pDSState);
	if (FAILED(hr))
	{
		cleanup();
		return false;
	}

	dd.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ZERO;
	dd.DepthFunc = D3D11_COMPARISON_LESS_EQUAL;
	hr = m_pD3DDevice->CreateDepthStencilState(&dd, &m_pDSWireState);
	if (FAILED(hr))
	{
		cleanup();
		return false;
	}

	SafeRelease(vsBlob);
	SafeRelease(psBlob);
	SafeRelease(errBlob);
	m_gpuReady = true;
	return true;
}

/// <summary>
/// Releases the D3D render-target / staging textures and the D2D bitmap
/// </summary>
void KBodyVisualizer::gpuReleaseRenderTargets()
{
	SafeRelease(m_pMeshBitmap);
	SafeRelease(m_pMeshStaging);
	SafeRelease(m_pMeshDSV);
	SafeRelease(m_pMeshDS);
	SafeRelease(m_pMeshRTV);
	SafeRelease(m_pMeshRT);
	m_gpuRTWidth = 0;
	m_gpuRTHeight = 0;
}

/// <summary>
/// Releases every D3D11 resource owned by the visualizer
/// </summary>
void KBodyVisualizer::gpuRelease()
{
	gpuReleaseRenderTargets();
	SafeRelease(m_pDSWireState);
	SafeRelease(m_pDSState);
	SafeRelease(m_pRSWire);
	SafeRelease(m_pRSSolid);
	SafeRelease(m_pPerBone);
	SafeRelease(m_pPerFrame);
	SafeRelease(m_pIB);
	SafeRelease(m_pVB);
	SafeRelease(m_pIL);
	SafeRelease(m_pPS);
	SafeRelease(m_pVS);
	SafeRelease(m_pD3DCtx);
	SafeRelease(m_pD3DDevice);
	m_gpuMeshVersion = -1;
	m_gpuReady = false;
}

/// <summary>
/// (Re)creates the offscreen D3D render target matching the window size
/// </summary>
bool KBodyVisualizer::gpuEnsureRenderTargets()
{
	if (!m_gpuReady || !m_pRT)
		return false;
	if (m_width < 10 || m_height < 10)
		return false;
	if (m_gpuRTWidth == m_width && m_gpuRTHeight == m_height && m_pMeshRT && m_pMeshBitmap)
		return true;

	gpuReleaseRenderTargets();

	D3D11_TEXTURE2D_DESC td = {};
	td.Width = (UINT)m_width;
	td.Height = (UINT)m_height;
	td.MipLevels = 1;
	td.ArraySize = 1;
	td.Format = DXGI_FORMAT_B8G8R8A8_UNORM;
	td.SampleDesc.Count = 1;
	td.Usage = D3D11_USAGE_DEFAULT;
	td.BindFlags = D3D11_BIND_RENDER_TARGET;
	if (FAILED(m_pD3DDevice->CreateTexture2D(&td, NULL, &m_pMeshRT)))
		return false;
	if (FAILED(m_pD3DDevice->CreateRenderTargetView(m_pMeshRT, NULL, &m_pMeshRTV)))
		return false;

	td.Format = DXGI_FORMAT_D24_UNORM_S8_UINT;
	td.BindFlags = D3D11_BIND_DEPTH_STENCIL;
	if (FAILED(m_pD3DDevice->CreateTexture2D(&td, NULL, &m_pMeshDS)))
		return false;
	if (FAILED(m_pD3DDevice->CreateDepthStencilView(m_pMeshDS, NULL, &m_pMeshDSV)))
		return false;

	td.Format = DXGI_FORMAT_B8G8R8A8_UNORM;
	td.Usage = D3D11_USAGE_STAGING;
	td.BindFlags = 0;
	td.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
	if (FAILED(m_pD3DDevice->CreateTexture2D(&td, NULL, &m_pMeshStaging)))
		return false;

	m_pRT->CreateBitmap(D2D1::SizeU((UINT32)m_width, (UINT32)m_height), NULL, 0,
		D2D1::BitmapProperties(D2D1::PixelFormat(DXGI_FORMAT_B8G8R8A8_UNORM, D2D1_ALPHA_MODE_PREMULTIPLIED)),
		&m_pMeshBitmap);

	m_gpuRTWidth = m_width;
	m_gpuRTHeight = m_height;
	return m_pMeshRT != NULL && m_pMeshBitmap != NULL;
}

/// <summary>
/// Uploads the current retarget mesh (rest pose + skinning data) to the GPU
/// </summary>
bool KBodyVisualizer::gpuEnsureMesh()
{
	if (!m_gpuReady || !m_pRetargeter)
		return false;

	const RetargetMesh &mesh = m_pRetargeter->mesh();
	int version = m_pRetargeter->meshVersion();
	if (m_gpuMeshVersion == version && m_pVB && m_pIB)
		return true;

	m_gpuMeshVersion = -1;
	SafeRelease(m_pVB);
	SafeRelease(m_pIB);

	if (!mesh.valid || mesh.restPositions.empty() || mesh.indices.empty())
		return false;

	int nVerts = (int)(mesh.restPositions.size() / 3);
	int nIndices = (int)mesh.indices.size();

	// Rest normals from the topology (area-weighted), computed once per mesh
	std::vector<float> norms(3 * nVerts, 0.0f);
	for (int t = 0; t + 2 < nIndices; t += 3)
	{
		unsigned int i0 = mesh.indices[t + 0];
		unsigned int i1 = mesh.indices[t + 1];
		unsigned int i2 = mesh.indices[t + 2];
		if (i0 >= (unsigned int)nVerts || i1 >= (unsigned int)nVerts || i2 >= (unsigned int)nVerts)
			continue;
		float ax = mesh.restPositions[3 * i0 + 0], ay = mesh.restPositions[3 * i0 + 1], az = mesh.restPositions[3 * i0 + 2];
		float bx = mesh.restPositions[3 * i1 + 0], by = mesh.restPositions[3 * i1 + 1], bz = mesh.restPositions[3 * i1 + 2];
		float cx = mesh.restPositions[3 * i2 + 0], cy_ = mesh.restPositions[3 * i2 + 1], cz = mesh.restPositions[3 * i2 + 2];
		float ex0 = bx - ax, ey0 = by - ay, ez0 = bz - az;
		float ex1 = cx - ax, ey1 = cy_ - ay, ez1 = cz - az;
		float nx = ey0 * ez1 - ez0 * ey1;
		float ny = ez0 * ex1 - ex0 * ez1;
		float nz = ex0 * ey1 - ey0 * ex1;
		if (nx * nx + ny * ny + nz * nz < 1e-12f)
			continue;
		norms[3 * i0 + 0] += nx; norms[3 * i0 + 1] += ny; norms[3 * i0 + 2] += nz;
		norms[3 * i1 + 0] += nx; norms[3 * i1 + 1] += ny; norms[3 * i1 + 2] += nz;
		norms[3 * i2 + 0] += nx; norms[3 * i2 + 1] += ny; norms[3 * i2 + 2] += nz;
	}

	std::vector<GpuMeshVertex> verts(nVerts);
	for (int v = 0; v < nVerts; ++v)
	{
		GpuMeshVertex &gv = verts[v];
		gv.pos[0] = mesh.restPositions[3 * v + 0];
		gv.pos[1] = mesh.restPositions[3 * v + 1];
		gv.pos[2] = mesh.restPositions[3 * v + 2];

		float nl = sqrtf(norms[3 * v + 0] * norms[3 * v + 0] +
			norms[3 * v + 1] * norms[3 * v + 1] +
			norms[3 * v + 2] * norms[3 * v + 2]);
		if (nl > 1e-8f)
		{
			gv.nrm[0] = norms[3 * v + 0] / nl;
			gv.nrm[1] = norms[3 * v + 1] / nl;
			gv.nrm[2] = norms[3 * v + 2] / nl;
		}
		else
		{
			gv.nrm[0] = 0.0f; gv.nrm[1] = 1.0f; gv.nrm[2] = 0.0f;
		}

		gv.bones[0] = gv.bones[1] = gv.bones[2] = gv.bones[3] = 0;
		gv.weights[0] = gv.weights[1] = gv.weights[2] = gv.weights[3] = 0.0f;

		// Gather the influences into a scratch buffer first (the vertex layout
		// only holds 4 slots; writing >4 influences straight into gv would
		// overflow into the neighbouring vertex and corrupt its position).
		// Then keep the top-4 by weight, matching the CPU skinning.
		const std::vector<std::pair<int, float> > &wl = mesh.weights[v];
		unsigned int infBone[8];
		float infW[8];
		int nInf = 0;
		for (size_t k = 0; k < wl.size() && nInf < 8; ++k)
		{
			if (wl[k].second > 0.0f)
			{
				infBone[nInf] = (unsigned int)wl[k].first;
				infW[nInf] = wl[k].second;
				++nInf;
			}
		}
		for (int a = 1; a < nInf; ++a)
		{
			unsigned int bi = infBone[a];
			float bw = infW[a];
			int b = a - 1;
			while (b >= 0 && infW[b] < bw)
			{
				infBone[b + 1] = infBone[b];
				infW[b + 1] = infW[b];
				--b;
			}
			infBone[b + 1] = bi;
			infW[b + 1] = bw;
		}
		int n = (nInf < 4) ? nInf : 4;
		for (int a = 0; a < n; ++a)
		{
			gv.bones[a] = infBone[a];
			gv.weights[a] = infW[a];
		}
		float sum = 0.0f;
		for (int a = 0; a < n; ++a) sum += gv.weights[a];
		if (sum > 1e-6f)
		{
			for (int a = 0; a < n; ++a) gv.weights[a] /= sum;
		}
		else
		{
			// Unweighted vertex: the shader keeps the rest position
			for (int a = 0; a < 4; ++a) gv.weights[a] = 0.0f;
		}
	}

	D3D11_BUFFER_DESC bd = {};
	D3D11_SUBRESOURCE_DATA sd = {};

	bd.Usage = D3D11_USAGE_DEFAULT;
	bd.BindFlags = D3D11_BIND_VERTEX_BUFFER;
	bd.ByteWidth = nVerts * (int)sizeof(GpuMeshVertex);
	sd.pSysMem = verts.data();
	sd.SysMemPitch = bd.ByteWidth;
	if (FAILED(m_pD3DDevice->CreateBuffer(&bd, &sd, &m_pVB)))
		return false;

	bd.BindFlags = D3D11_BIND_INDEX_BUFFER;
	bd.ByteWidth = nIndices * 4;
	sd.pSysMem = mesh.indices.data();
	sd.SysMemPitch = bd.ByteWidth;
	if (FAILED(m_pD3DDevice->CreateBuffer(&bd, &sd, &m_pIB)))
	{
		SafeRelease(m_pVB);
		return false;
	}

	m_gpuMeshVersion = version;
	return true;
}

/// <summary>
/// Skin and rasterize the character mesh into the offscreen render target
/// </summary>
bool KBodyVisualizer::gpuRenderMesh()
{
	if (!m_gpuReady || !m_pRT || !m_pRetargeter)
		return false;
	if (!gpuEnsureRenderTargets())
		return false;
	if (!gpuEnsureMesh())
		return false;

	const RetargetMesh &mesh = m_pRetargeter->mesh();
	const RetargetPose &pose = m_pRetargeter->pose();
	if (!mesh.valid || !pose.valid)
		return false;

	int nBones = m_pRetargeter->boneCount();
	if (nBones < 1 || (int)pose.boneMat.size() < 16 * nBones)
		return false;

	// Bone skinning matrices, transposed into HLSL column-major slots.
	// Slot layout: g_bones[256] (unused slots stay identity for safety).
	std::vector<float> boneData(16 * 256, 0.0f);
	for (int i = 0; i < 256; ++i)
		boneData[16 * i + 0] = boneData[16 * i + 5] = boneData[16 * i + 10] = boneData[16 * i + 15] = 1.0f;
	int boneSlots = (nBones < 256) ? nBones : 256;
	for (int b = 0; b < boneSlots; ++b)
	{
		const float *S = &pose.boneMat[16 * b]; // row-major S[r*4+c]
		for (int r = 0; r < 4; ++r)
			for (int c = 0; c < 4; ++c)
				boneData[16 * b + 4 * c + r] = S[4 * r + c];
	}
	m_pD3DCtx->UpdateSubresource(m_pPerBone, 0, NULL, boneData.data(), 0, 0);

	// Camera matrices reproducing KBodyVisualizer::project() exactly
	float focal = (m_height * 0.5f) / tanf(m_fov * 0.5f);
	float fcx = 2.0f * focal / (float)m_width;
	float fcy = 2.0f * focal / (float)m_height;
	float cy = cosf(m_yaw), sy = sinf(m_yaw);
	float cp = cosf(m_pitch), sp = sinf(m_pitch);
	const float zn = 0.15f, zf = 100.0f;

	float V[16]; // column-major view: data[j*4+i] = coefficient of input j for output i
	V[0] = cy;        V[1] = sy * sp;   V[2] = sy * cp;    V[3] = 0.0f;
	V[4] = 0.0f;      V[5] = cp;        V[6] = -sp;        V[7] = 0.0f;
	V[8] = sy;        V[9] = -cy * sp;  V[10] = -cy * cp;  V[11] = 0.0f;
	V[12] = 0.0f;     V[13] = 0.0f;     V[14] = -m_dist;   V[15] = 1.0f;

	float P[16]; // column-major projection
	P[0] = fcx;   P[1] = 0.0f;  P[2] = 0.0f;            P[3] = 0.0f;
	P[4] = 0.0f;  P[5] = fcy;   P[6] = 0.0f;            P[7] = 0.0f;
	P[8] = 0.0f;  P[9] = 0.0f;  P[10] = zf / (zn - zf); P[11] = -1.0f;
	P[12] = 0.0f; P[13] = 0.0f; P[14] = zn * zf / (zn - zf); P[15] = 0.0f;

	float VP[16];
	mulMat4Col(VP, P, V);

	float cb[24] = { 0.0f };
	memcpy(cb, VP, sizeof(VP));
	cb[16] = m_panX / (m_width * 0.5f);
	cb[17] = m_panY / (m_height * 0.5f);
	m_pD3DCtx->UpdateSubresource(m_pPerFrame, 0, NULL, cb, 0, 0);

	// Viewport + targets
	D3D11_VIEWPORT vp = { 0, 0, (float)m_width, (float)m_height, 0, 1 };
	m_pD3DCtx->RSSetViewports(1, &vp);
	m_pD3DCtx->OMSetRenderTargets(1, &m_pMeshRTV, m_pMeshDSV);

	float clear[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
	m_pD3DCtx->ClearRenderTargetView(m_pMeshRTV, clear);
	m_pD3DCtx->ClearDepthStencilView(m_pMeshDSV, D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0f, 0);

	UINT stride = (UINT)sizeof(GpuMeshVertex), offset = 0;
	UINT nIndices = (UINT)mesh.indices.size();

	m_pD3DCtx->IASetInputLayout(m_pIL);
	m_pD3DCtx->IASetVertexBuffers(0, 1, &m_pVB, &stride, &offset);
	m_pD3DCtx->IASetIndexBuffer(m_pIB, DXGI_FORMAT_R32_UINT, 0);
	m_pD3DCtx->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
	m_pD3DCtx->VSSetShader(m_pVS, NULL, 0);
	m_pD3DCtx->PSSetShader(m_pPS, NULL, 0);
	m_pD3DCtx->VSSetConstantBuffers(0, 1, &m_pPerFrame);
	m_pD3DCtx->VSSetConstantBuffers(1, 1, &m_pPerBone);

	// Pass 1: solid fill
	m_pD3DCtx->RSSetState(m_pRSSolid);
	m_pD3DCtx->OMSetDepthStencilState(m_pDSState, 0);
	m_pD3DCtx->DrawIndexed(nIndices, 0, 0);

	// Pass 2: wireframe edges on top of the fill
	m_pD3DCtx->RSSetState(m_pRSWire);
	m_pD3DCtx->OMSetDepthStencilState(m_pDSWireState, 0);
	m_pD3DCtx->DrawIndexed(nIndices, 0, 0);

	// Read the offscreen target back into the D2D bitmap
	m_pD3DCtx->CopyResource(m_pMeshStaging, m_pMeshRT);
	D3D11_MAPPED_SUBRESOURCE map;
	if (SUCCEEDED(m_pD3DCtx->Map(m_pMeshStaging, 0, D3D11_MAP_READ, 0, &map)))
	{
		if (m_pMeshBitmap)
			m_pMeshBitmap->CopyFromMemory(NULL, map.pData, map.RowPitch);
		m_pD3DCtx->Unmap(m_pMeshStaging, 0);
		return true;
	}
	return false;
}
