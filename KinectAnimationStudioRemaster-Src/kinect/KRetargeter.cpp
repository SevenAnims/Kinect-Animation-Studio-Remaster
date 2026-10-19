#include "KRetargeter.h"
#include "KBodyExporter.h"
#include <map>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <float.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================================================================
// Mappable Kinect joints exposed by the mapping window
// ============================================================================

const RetargetJointOption g_retargetJoints[] = {
	{ JointType_SpineBase,     "Spine Base"     },
	{ JointType_SpineMid,      "Spine Mid"      },
	{ JointType_SpineShoulder, "Spine Shoulder" },
	{ JointType_Neck,          "Neck"           },
	{ JointType_Head,          "Head"           },
	{ JointType_ShoulderLeft,  "Shoulder Left"  },
	{ JointType_ElbowLeft,     "Elbow Left"     },
	{ JointType_WristLeft,     "Wrist Left"     },
	{ JointType_HandLeft,      "Hand Left"      },
	{ JointType_ShoulderRight, "Shoulder Right" },
	{ JointType_ElbowRight,    "Elbow Right"    },
	{ JointType_WristRight,    "Wrist Right"    },
	{ JointType_HandRight,     "Hand Right"     },
	{ JointType_HipLeft,       "Hip Left"       },
	{ JointType_KneeLeft,      "Knee Left"      },
	{ JointType_AnkleLeft,     "Ankle Left"     },
	{ JointType_FootLeft,      "Foot Left"      },
	{ JointType_HipRight,      "Hip Right"      },
	{ JointType_KneeRight,     "Knee Right"     },
	{ JointType_AnkleRight,    "Ankle Right"    },
	{ JointType_FootRight,     "Foot Right"     },
};
const int g_retargetJointCount = _countof(g_retargetJoints);

// Reference skeleton (the "ghost") joint positions in scene space (metres).
// This is the same pose the visualizer draws when the ghost is enabled.
const float g_ghostJointPositions[JointType_Count][3] = {
	{ 0.00f, 0.90f, 0.10f },  // SpineBase
	{ 0.00f, 1.10f, 0.10f },  // SpineMid
	{ 0.00f, 1.30f, 0.10f },  // Neck
	{ 0.00f, 1.55f, 0.10f },  // Head
	{ 0.20f, 1.30f, 0.10f },  // ShoulderLeft
	{ 0.47f, 1.25f, 0.10f },  // ElbowLeft
	{ 0.72f, 1.27f, 0.10f },  // WristLeft
	{ 0.80f, 1.28f, 0.10f },  // HandLeft
	{ -0.20f, 1.30f, 0.10f }, // ShoulderRight
	{ -0.47f, 1.25f, 0.10f }, // ElbowRight
	{ -0.72f, 1.27f, 0.10f }, // WristRight
	{ -0.80f, 1.28f, 0.10f }, // HandRight
	{ 0.09f, 0.90f, 0.10f },  // HipLeft
	{ 0.12f, 0.55f, 0.10f },  // KneeLeft
	{ 0.10f, 0.12f, 0.10f },  // AnkleLeft
	{ 0.10f, 0.04f, 0.10f },  // FootLeft
	{ -0.09f, 0.90f, 0.10f }, // HipRight
	{ -0.12f, 0.55f, 0.10f }, // KneeRight
	{ -0.10f, 0.12f, 0.10f }, // AnkleRight
	{ -0.10f, 0.04f, 0.10f }, // FootRight
	{ 0.00f, 1.30f, 0.10f },  // SpineShoulder
	{ 0.60f, 1.23f, 0.10f },  // HandTipLeft
	{ 0.66f, 1.32f, 0.10f },  // ThumbLeft
	{ -0.60f, 1.23f, 0.10f }, // HandTipRight
	{ -0.66f, 1.32f, 0.10f }, // ThumbRight
};

// Canonical Kinect joint hierarchy used for the angle transfer. For every joint
// this is the next joint "down" the chain that is driven by the joint's bone:
// the bone mapped to joint j is aimed along  (actor[j] -> actor[g_kinChild[j]]).
// A leaf joint (-1) keeps the orientation of its parent.
static const int g_kinChild[JointType_Count] = {
	JointType_SpineMid,        // SpineBase    (0)
	JointType_SpineShoulder,   // SpineMid     (1)
	JointType_Head,            // Neck         (2)
	-1,                        // Head         (3)
	JointType_ElbowLeft,       // ShoulderLeft (4)
	JointType_WristLeft,       // ElbowLeft    (5)
	JointType_HandLeft,        // WristLeft    (6)
	-1,                        // HandLeft     (7)
	JointType_ElbowRight,      // ShoulderRight(8)
	JointType_WristRight,      // ElbowRight   (9)
	JointType_HandRight,       // WristRight   (10)
	-1,                        // HandRight    (11)
	JointType_KneeLeft,        // HipLeft      (12)
	JointType_AnkleLeft,       // KneeLeft     (13)
	JointType_FootLeft,        // AnkleLeft    (14)
	-1,                        // FootLeft     (15)
	JointType_KneeRight,       // HipRight     (16)
	JointType_AnkleRight,      // KneeRight    (17)
	JointType_FootRight,       // AnkleRight   (18)
	-1,                        // FootRight    (19)
	JointType_Neck,            // SpineShoulder(20)
	-1,                        // HandTipLeft  (21)
	-1,                        // ThumbLeft    (22)
	-1,                        // HandTipRight (23)
	-1,                        // ThumbRight   (24)
};

// Parent of each joint in the canonical Kinect hierarchy (mirror of
// g_kinChild). Used to follow the actor segment that ENDS at a joint, which
// is the one a character clavicle / upper arm / forearm really corresponds to.
static const int g_kinParent[JointType_Count] = {
	-1,                        // SpineBase    (0)
	JointType_SpineBase,       // SpineMid     (1)
	JointType_SpineShoulder,   // Neck         (2)
	JointType_Neck,            // Head         (3)
	JointType_SpineShoulder,   // ShoulderLeft (4)
	JointType_ShoulderLeft,    // ElbowLeft    (5)
	JointType_ElbowLeft,       // WristLeft    (6)
	JointType_WristLeft,       // HandLeft     (7)
	JointType_SpineShoulder,   // ShoulderRight(8)
	JointType_ShoulderRight,   // ElbowRight   (9)
	JointType_ElbowRight,      // WristRight   (10)
	JointType_WristRight,      // HandRight    (11)
	JointType_SpineBase,       // HipLeft      (12)
	JointType_HipLeft,         // KneeLeft     (13)
	JointType_KneeLeft,        // AnkleLeft    (14)
	JointType_AnkleLeft,       // FootLeft     (15)
	JointType_SpineBase,       // HipRight     (16)
	JointType_HipRight,        // KneeRight    (17)
	JointType_KneeRight,       // AnkleRight   (18)
	JointType_AnkleRight,      // FootRight    (19)
	JointType_SpineBase,       // SpineShoulder(20)
	-1,                        // HandTipLeft  (21)
	-1,                        // ThumbLeft    (22)
	-1,                        // HandTipRight (23)
	-1,                        // ThumbRight   (24)
};

// ============================================================================
// Implementation
// ============================================================================

KRetargeter::KRetargeter() :
m_scale(1.0f),
m_rootBone(-1),
m_scaleReady(false),
m_meshVersion(0),
m_importScale(1.0f),
	m_importRotX(0.0f),
	m_importRotY(0.0f),
	m_importRotZ(0.0f),
	m_liveTick(0)
{
	m_anchor[0] = m_anchor[1] = m_anchor[2] = 0.0f;
	m_motionAnchor[0] = m_motionAnchor[1] = m_motionAnchor[2] = 0.0f;
	m_motionAnchorSet = false;
	memset(m_mapping, -1, sizeof(m_mapping));
	memcpy(m_ghostPos, g_ghostJointPositions, sizeof(m_ghostPos));
	memset(m_importRotMat, 0, sizeof(m_importRotMat));
	memset(m_importRotMatT, 0, sizeof(m_importRotMatT));
	m_importRotMat[0][0] = m_importRotMat[1][1] = m_importRotMat[2][2] = 1.0f;
	m_importRotMatT[0][0] = m_importRotMatT[1][1] = m_importRotMatT[2][2] = 1.0f;
}

KRetargeter::~KRetargeter() {
}

// ----------------------------------------------------------------------------
// Editable ghost reference pose
// ----------------------------------------------------------------------------

void KRetargeter::resetGhost() {
	memcpy(m_ghostPos, g_ghostJointPositions, sizeof(m_ghostPos));
}

int KRetargeter::ghostParent(int joint) const {
	return (joint >= 0 && joint < JointType_Count) ? g_kinParent[joint] : -1;
}

// Collects every joint whose parent chain leads to `joint` (joint included).
static void ghostSubtree(int joint, std::vector<int> &out) {
	out.clear();
	out.push_back(joint);
	for (int j = 0; j < JointType_Count; ++j) {
		if (j == joint)
			continue;
		int cur = g_kinParent[j];
		while (cur >= 0) {
			if (cur == joint) {
				out.push_back(j);
				break;
			}
			cur = g_kinParent[cur];
		}
	}
}

void KRetargeter::rotateGhost(int joint, const float axis[3], float angleRad) {
	if (joint < 0 || joint >= JointType_Count)
		return;
	float ax = axis[0], ay = axis[1], az = axis[2];
	float len = sqrtf(ax * ax + ay * ay + az * az);
	if (len < 1e-6f)
		return;
	ax /= len; ay /= len; az /= len;

	float c = cosf(angleRad), s = sinf(angleRad), t = 1.0f - c;
	float R[3][3] = {
		{ t * ax * ax + c,     t * ax * ay - s * az, t * ax * az + s * ay },
		{ t * ax * ay + s * az, t * ay * ay + c,    t * ay * az - s * ax },
		{ t * ax * az - s * ay, t * ay * az + s * ax, t * az * az + c     },
	};

	// Pivot: the current position of the parent joint (itself when root)
	int par = g_kinParent[joint];
	const float *pv = (par >= 0) ? m_ghostPos[par] : m_ghostPos[joint];

	std::vector<int> sub;
	ghostSubtree(joint, sub);
	for (size_t k = 0; k < sub.size(); ++k) {
		int j = sub[k];
		float dx = m_ghostPos[j][0] - pv[0];
		float dy = m_ghostPos[j][1] - pv[1];
		float dz = m_ghostPos[j][2] - pv[2];
		m_ghostPos[j][0] = pv[0] + R[0][0] * dx + R[0][1] * dy + R[0][2] * dz;
		m_ghostPos[j][1] = pv[1] + R[1][0] * dx + R[1][1] * dy + R[1][2] * dz;
		m_ghostPos[j][2] = pv[2] + R[2][0] * dx + R[2][1] * dy + R[2][2] * dz;
	}
}


// ----------------------------------------------------------------------------
// Matrix helpers
// ----------------------------------------------------------------------------

void KRetargeter::aimRotation(float R[3][3], const float from[3], const float to[3]) {
	R[0][0] = 1; R[0][1] = 0; R[0][2] = 0;
	R[1][0] = 0; R[1][1] = 1; R[1][2] = 0;
	R[2][0] = 0; R[2][1] = 0; R[2][2] = 1;

	float dot = from[0] * to[0] + from[1] * to[1] + from[2] * to[2];
	if (dot > 0.99999f)
		return;

	float ax[3];
	ax[0] = from[1] * to[2] - from[2] * to[1];
	ax[1] = from[2] * to[0] - from[0] * to[2];
	ax[2] = from[0] * to[1] - from[1] * to[0];

	float angle;
	if (dot < -0.99999f) {
		angle = 3.14159265358979f;
		if (fabs(from[0]) < 0.9f) { ax[0] = 1; ax[1] = 0; ax[2] = 0; }
		else { ax[0] = 0; ax[1] = 1; ax[2] = 0; }
		float d = ax[0] * from[0] + ax[1] * from[1] + ax[2] * from[2];
		ax[0] -= d * from[0]; ax[1] -= d * from[1]; ax[2] -= d * from[2];
	} else {
		angle = acos(dot);
	}

	float len = sqrtf(ax[0] * ax[0] + ax[1] * ax[1] + ax[2] * ax[2]);
	if (len < 1e-6f)
		return;
	ax[0] /= len; ax[1] /= len; ax[2] /= len;

	float s = sinf(angle * 0.5f);
	float qw = cosf(angle * 0.5f);
	float qx = ax[0] * s;
	float qy = ax[1] * s;
	float qz = ax[2] * s;

	R[0][0] = 1 - 2 * (qy*qy + qz*qz);
	R[0][1] =     2 * (qx*qy - qz*qw);
	R[0][2] =     2 * (qx*qz + qy*qw);
	R[1][0] =     2 * (qx*qy + qz*qw);
	R[1][1] = 1 - 2 * (qx*qx + qz*qz);
	R[1][2] =     2 * (qy*qz - qx*qw);
	R[2][0] =     2 * (qx*qz - qy*qw);
	R[2][1] =     2 * (qy*qz + qx*qw);
	R[2][2] = 1 - 2 * (qx*qx + qy*qy);
}

void KRetargeter::transpose3(const float in[3][3], float out[3][3]) {
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
			out[j][i] = in[i][j];
}

void KRetargeter::mulMat3(const float a[3][3], const float b[3][3], float out[3][3]) {
	float t[3][3];
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
			t[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
	memcpy(out, t, sizeof(t));
}

void KRetargeter::mulMat4(const float a[16], const float b[16], float out[16]) {
	float t[16];
	for (int r = 0; r < 4; ++r)
		for (int c = 0; c < 4; ++c)
			t[r * 4 + c] =
				a[r * 4 + 0] * b[0 * 4 + c] +
				a[r * 4 + 1] * b[1 * 4 + c] +
				a[r * 4 + 2] * b[2 * 4 + c] +
				a[r * 4 + 3] * b[3 * 4 + c];
	memcpy(out, t, sizeof(t));
}

bool KRetargeter::invert4(const float m[16], float out[16]) {
	float inv[16];
	float det;
	inv[0]  =  m[5] * m[10] * m[15] - m[5] * m[11] * m[14] - m[9] * m[6] * m[15] + m[9] * m[7] * m[14] + m[13] * m[6] * m[11] - m[13] * m[7] * m[10];
	inv[4]  = -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] + m[8] * m[6] * m[15] - m[8] * m[7] * m[14] - m[12] * m[6] * m[11] + m[12] * m[7] * m[10];
	inv[8]  =  m[4] * m[9]  * m[15] - m[4] * m[11] * m[13] - m[8] * m[5] * m[15] + m[8] * m[7] * m[13] + m[12] * m[5] * m[11] - m[12] * m[7] * m[9];
	inv[12] = -m[4] * m[9]  * m[14] + m[4] * m[10] * m[13] + m[8] * m[5] * m[14] - m[8] * m[6] * m[13] - m[12] * m[5] * m[10] + m[12] * m[6] * m[9];
	inv[1]  = -m[1] * m[10] * m[15] + m[1] * m[11] * m[14] + m[9] * m[2] * m[15] - m[9] * m[3] * m[14] - m[13] * m[2] * m[11] + m[13] * m[3] * m[10];
	inv[5]  =  m[0] * m[10] * m[15] - m[0] * m[11] * m[14] - m[8] * m[2] * m[15] + m[8] * m[3] * m[14] + m[12] * m[2] * m[11] - m[12] * m[3] * m[10];
	inv[9]  = -m[0] * m[9]  * m[15] + m[0] * m[11] * m[13] + m[8] * m[1] * m[15] - m[8] * m[3] * m[13] - m[12] * m[1] * m[11] + m[12] * m[3] * m[9];
	inv[13] =  m[0] * m[9]  * m[14] - m[0] * m[10] * m[13] - m[8] * m[1] * m[14] + m[8] * m[2] * m[13] + m[12] * m[1] * m[10] - m[12] * m[2] * m[9];
	inv[2]  =  m[1] * m[6]  * m[15] - m[1] * m[7]  * m[14] - m[5] * m[2] * m[15] + m[5] * m[3] * m[14] + m[13] * m[2] * m[7]  - m[13] * m[3] * m[6];
	inv[6]  = -m[0] * m[6]  * m[15] + m[0] * m[7]  * m[14] + m[4] * m[2] * m[15] - m[4] * m[3] * m[14] - m[12] * m[2] * m[7]  + m[12] * m[3] * m[6];
	inv[10] =  m[0] * m[5]  * m[15] - m[0] * m[7]  * m[13] - m[4] * m[1] * m[15] + m[4] * m[3] * m[13] + m[12] * m[1] * m[7]  - m[12] * m[3] * m[5];
	inv[14] = -m[0] * m[5]  * m[14] + m[0] * m[6]  * m[13] + m[4] * m[1] * m[14] - m[4] * m[2] * m[13] - m[12] * m[1] * m[6]  + m[12] * m[2] * m[5];
	inv[3]  = -m[1] * m[6]  * m[11] + m[1] * m[7]  * m[10] + m[5] * m[2] * m[11] - m[5] * m[3] * m[10] - m[9]  * m[2] * m[7]  + m[9]  * m[3] * m[6];
	inv[7]  =  m[0] * m[6]  * m[11] - m[0] * m[7]  * m[10] - m[4] * m[2] * m[11] + m[4] * m[3] * m[10] + m[8]  * m[2] * m[7]  - m[8]  * m[3] * m[6];
	inv[11] = -m[0] * m[5]  * m[11] + m[0] * m[7]  * m[9]  + m[4] * m[1] * m[11] - m[4] * m[3] * m[9]  - m[8]  * m[1] * m[7]  + m[8]  * m[3] * m[5];
	inv[15] =  m[0] * m[5]  * m[10] - m[0] * m[6]  * m[9]  - m[4] * m[1] * m[10] + m[4] * m[2] * m[9]  + m[8]  * m[1] * m[6]  - m[8]  * m[2] * m[5];

	det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];
	if (det == 0.0f)
		return false;

	det = 1.0f / det;
	for (int i = 0; i < 16; ++i)
		out[i] = inv[i] * det;
	return true;
}

// ----------------------------------------------------------------------------
// Import transform (scale + orientation)
// ----------------------------------------------------------------------------

void KRetargeter::buildImportRotation() {
	float rx = m_importRotX * (float)M_PI / 180.0f;
	float ry = m_importRotY * (float)M_PI / 180.0f;
	float rz = m_importRotZ * (float)M_PI / 180.0f;
	float cx = cosf(rx), sx = sinf(rx);
	float cy = cosf(ry), sy = sinf(ry);
	float cz = cosf(rz), sz = sinf(rz);

	float Rx[3][3] = { { 1, 0, 0 }, { 0, cx, -sx }, { 0, sx, cx } };
	float Ry[3][3] = { { cy, 0, sy }, { 0, 1, 0 }, { -sy, 0, cy } };
	float Rz[3][3] = { { cz, -sz, 0 }, { sz, cz, 0 }, { 0, 0, 1 } };

	// R = Rz * Ry * Rx
	float tmp[3][3];
	mulMat3(Rz, Ry, tmp);
	mulMat3(tmp, Rx, m_importRotMat);
	transpose3(m_importRotMat, m_importRotMatT);
}

void KRetargeter::transformPoint(const float p[3], float out[3]) const {
	// model = R * (scale * p)
	float q[3] = { m_importScale * p[0], m_importScale * p[1], m_importScale * p[2] };
	out[0] = m_importRotMat[0][0] * q[0] + m_importRotMat[0][1] * q[1] + m_importRotMat[0][2] * q[2];
	out[1] = m_importRotMat[1][0] * q[0] + m_importRotMat[1][1] * q[1] + m_importRotMat[1][2] * q[2];
	out[2] = m_importRotMat[2][0] * q[0] + m_importRotMat[2][1] * q[1] + m_importRotMat[2][2] * q[2];
}

void KRetargeter::inverseTransform(const float p[3], float out[3]) const {
	// raw = (1/scale) * R^T * p
	out[0] = m_importRotMatT[0][0] * p[0] + m_importRotMatT[0][1] * p[1] + m_importRotMatT[0][2] * p[2];
	out[1] = m_importRotMatT[1][0] * p[0] + m_importRotMatT[1][1] * p[1] + m_importRotMatT[1][2] * p[2];
	out[2] = m_importRotMatT[2][0] * p[0] + m_importRotMatT[2][1] * p[1] + m_importRotMatT[2][2] * p[2];
	out[0] /= m_importScale;
	out[1] /= m_importScale;
	out[2] /= m_importScale;
}

void KRetargeter::setImportScale(float s) {
	if (!(s > 0.0f) || s > 1e6f)
		s = 1.0f;
	m_importScale = s;
	applyImportTransform();
}

void KRetargeter::setImportRotation(float rx, float ry, float rz) {
	m_importRotX = rx;
	m_importRotY = ry;
	m_importRotZ = rz;
	applyImportTransform();
}

void KRetargeter::applyImportTransform() {
	buildImportRotation();

	for (size_t i = 0; i < m_bones.size() && 3 * i + 2 < m_rawRestPos.size(); ++i) {
		float mod[3];
		transformPoint(&m_rawRestPos[3 * i], mod);
		m_bones[i].restPos[0] = mod[0];
		m_bones[i].restPos[1] = mod[1];
		m_bones[i].restPos[2] = mod[2];
	}
	finalizeBones();

	int nVerts = (int)(m_rawMeshPos.size() / 3);
	m_mesh.restPositions.resize(3 * nVerts);
	for (int i = 0; i < nVerts; ++i) {
		float mod[3];
		transformPoint(&m_rawMeshPos[3 * i], mod);
		m_mesh.restPositions[3 * i + 0] = mod[0];
		m_mesh.restPositions[3 * i + 1] = mod[1];
		m_mesh.restPositions[3 * i + 2] = mod[2];
	}

	m_pose = RetargetPose();
	m_scaleReady = false;
	m_meshVersion++;
}

// ----------------------------------------------------------------------------
// FBX loading helpers
// ----------------------------------------------------------------------------

static FbxAMatrix nodeGlobalMatrix(FbxNode *node, std::map<FbxNode *, FbxAMatrix> &cache) {
	std::map<FbxNode *, FbxAMatrix>::iterator it = cache.find(node);
	if (it != cache.end())
		return it->second;

	FbxVector4 t = node->LclTranslation.Get();
	FbxVector4 r = node->LclRotation.Get();
	FbxVector4 s = node->LclScaling.Get();

	// Full FBX static local transform: T * Pre * R * Post * S. Plain SetTRS
	// leaves out the static PreRotation, which put the whole rest pose off by
	// 8 deg on a rig that uses it - and every pose derived from that rest with
	// it. Pivots/offsets are zero on the rigs handled here. This is folded in
	// unconditionally (not only when RotationActive is set) because that is
	// what every third-party importer, Blender included, computes; the export
	// solves the keyed euler the same way round (Pre^-1 * L * Post^-1) so the
	// two stay consistent. Identity Pre/Post - the usual case - is a no-op.
	FbxAMatrix local;
	{
		FbxAMatrix mT, mPre, mR, mPost, mS;
		mT.SetT(t);
		mPre.SetR(node->PreRotation.Get());
		mR.SetR(r);
		mPost.SetR(node->PostRotation.Get());
		mS.SetS(s);
		local = mT * mPre * mR * mPost * mS;
	}

	FbxAMatrix gm = local;
	if (node->GetParent())
		gm = nodeGlobalMatrix(node->GetParent(), cache) * local;

	cache[node] = gm;
	return gm;
}

static void collectSkeletonNodes(FbxNode *node, std::vector<FbxNode *> &out) {
	for (int i = 0; i < node->GetChildCount(); ++i) {
		FbxNode *child = node->GetChild(i);
		FbxNodeAttribute *attr = child->GetNodeAttribute();
		if (attr && attr->GetAttributeType() == FbxNodeAttribute::eSkeleton)
			out.push_back(child);
		collectSkeletonNodes(child, out);
	}
}

static void collectMeshNodes(FbxNode *node, std::vector<FbxNode *> &out) {
	for (int i = 0; i < node->GetChildCount(); ++i) {
		FbxNode *child = node->GetChild(i);
		FbxNodeAttribute *attr = child->GetNodeAttribute();
		if (attr && attr->GetAttributeType() == FbxNodeAttribute::eMesh)
			out.push_back(child);
		collectMeshNodes(child, out);
	}
}

bool KRetargeter::buildSkeleton(FbxScene *scene) {
	std::vector<FbxNode *> nodes;
	collectSkeletonNodes(scene->GetRootNode(), nodes);
	if (nodes.empty())
		return false;

	std::map<FbxNode *, int> indexMap;
	for (size_t i = 0; i < nodes.size(); ++i)
		indexMap[nodes[i]] = (int)i;

	std::map<FbxNode *, FbxAMatrix> gcache;
	m_bones.resize(nodes.size());
	m_rawRestPos.assign(3 * nodes.size(), 0.0f);
	m_rawRestRot.assign(9 * nodes.size(), 0.0f);
	buildImportRotation();

	for (size_t i = 0; i < nodes.size(); ++i) {
		FbxNode *n = nodes[i];
		RetargetBone &b = m_bones[i];
		b.name = n->GetName();
		b.parent = -1;
		for (FbxNode *p = n->GetParent(); p; p = p->GetParent()) {
			std::map<FbxNode *, int>::iterator it = indexMap.find(p);
			if (it != indexMap.end()) { b.parent = it->second; break; }
		}
		FbxAMatrix gm = nodeGlobalMatrix(n, gcache);
		FbxVector4 t = gm.GetT();
		float raw[3] = { (float)t[0], (float)t[1], (float)t[2] };
		m_rawRestPos[3 * i + 0] = raw[0];
		m_rawRestPos[3 * i + 1] = raw[1];
		m_rawRestPos[3 * i + 2] = raw[2];
		// Raw global rest rotation (row-major), kept for the export rest pose.
		// The global matrix above carries the ancestors' scaling (e.g. a
		// Blender armature holder at 100x), which must NOT leak into this
		// rotation: the export solves every local rotation from it and
		// quatFromMatrix assumes a pure rotation. Strip the scale by
		// normalizing each column (exact for TRS compositions without
		// shear, which covers the rigs handled here). Positions
		// (m_rawRestPos) keep the scale - they need it.
		{
			double sc0 = sqrt((double)gm[0][0] * gm[0][0] + (double)gm[1][0] * gm[1][0] + (double)gm[2][0] * gm[2][0]);
			double sc1 = sqrt((double)gm[0][1] * gm[0][1] + (double)gm[1][1] * gm[1][1] + (double)gm[2][1] * gm[2][1]);
			double sc2 = sqrt((double)gm[0][2] * gm[0][2] + (double)gm[1][2] * gm[1][2] + (double)gm[2][2] * gm[2][2]);
			if (!(sc0 > 1e-12)) sc0 = 1.0;
			if (!(sc1 > 1e-12)) sc1 = 1.0;
			if (!(sc2 > 1e-12)) sc2 = 1.0;
			m_rawRestRot[9 * i + 0] = (float)(gm[0][0] / sc0);
			m_rawRestRot[9 * i + 1] = (float)(gm[0][1] / sc1);
			m_rawRestRot[9 * i + 2] = (float)(gm[0][2] / sc2);
			m_rawRestRot[9 * i + 3] = (float)(gm[1][0] / sc0);
			m_rawRestRot[9 * i + 4] = (float)(gm[1][1] / sc1);
			m_rawRestRot[9 * i + 5] = (float)(gm[1][2] / sc2);
			m_rawRestRot[9 * i + 6] = (float)(gm[2][0] / sc0);
			m_rawRestRot[9 * i + 7] = (float)(gm[2][1] / sc1);
			m_rawRestRot[9 * i + 8] = (float)(gm[2][2] / sc2);
		}
		transformPoint(raw, b.restPos);
		b.firstChild = -1;
		b.restLen = 0.0f;
		b.restAxis[0] = 0; b.restAxis[1] = 0; b.restAxis[2] = 0;
	}

	finalizeBones();
	return true;
}

void KRetargeter::finalizeBones() {
	for (size_t i = 0; i < m_bones.size(); ++i) {
		int firstChild = -1;
		for (size_t j = i + 1; j < m_bones.size(); ++j)
			if (m_bones[j].parent == (int)i) { firstChild = (int)j; break; }
		m_bones[i].firstChild = firstChild;

		float dx, dy, dz, len;
		if (firstChild >= 0) {
			dx = m_bones[firstChild].restPos[0] - m_bones[i].restPos[0];
			dy = m_bones[firstChild].restPos[1] - m_bones[i].restPos[1];
			dz = m_bones[firstChild].restPos[2] - m_bones[i].restPos[2];
		} else if (m_bones[i].parent >= 0) {
			dx = m_bones[i].restPos[0] - m_bones[m_bones[i].parent].restPos[0];
			dy = m_bones[i].restPos[1] - m_bones[m_bones[i].parent].restPos[1];
			dz = m_bones[i].restPos[2] - m_bones[m_bones[i].parent].restPos[2];
		} else {
			dx = 0; dy = 0; dz = 0;
		}
		len = sqrtf(dx * dx + dy * dy + dz * dz);
		m_bones[i].restLen = len;
		if (len > 1e-5f) {
			m_bones[i].restAxis[0] = dx / len;
			m_bones[i].restAxis[1] = dy / len;
			m_bones[i].restAxis[2] = dz / len;
		} else {
			m_bones[i].restAxis[1] = 1.0f;
		}
	}
}

bool KRetargeter::extractMesh(FbxScene *scene) {
	m_mesh.valid = false;
	m_mesh.restPositions.clear();
	m_mesh.indices.clear();
	m_mesh.weights.clear();
	m_rawMeshPos.clear();

	// Merge every mesh object of the scene (the character can be made of
	// several meshes, each with its own node transform and skin).
	std::vector<FbxNode *> meshNodes;
	collectMeshNodes(scene->GetRootNode(), meshNodes);
	if (meshNodes.empty())
		return false;

	std::map<std::string, int> boneIndex;
	for (size_t i = 0; i < m_bones.size(); ++i)
		boneIndex[m_bones[i].name] = (int)i;

	size_t totalVerts = 0;
	for (size_t m = 0; m < meshNodes.size(); ++m) {
		FbxMesh *mesh = meshNodes[m]->GetMesh();
		if (mesh)
			totalVerts += mesh->GetControlPointsCount();
	}
	if (totalVerts == 0)
		return false;
	m_mesh.weights.assign(totalVerts, std::vector<std::pair<int, float> >());

	std::map<FbxNode *, FbxAMatrix> gcache;
	int vertexBase = 0;

	for (size_t m = 0; m < meshNodes.size(); ++m) {
		FbxNode *meshNode = meshNodes[m];
		FbxMesh *mesh = meshNode->GetMesh();
		if (!mesh)
			continue;
		int nVerts = mesh->GetControlPointsCount();
		if (nVerts <= 0)
			continue;

		FbxAMatrix meshGlobal = nodeGlobalMatrix(meshNode, gcache);

		m_mesh.restPositions.resize(m_mesh.restPositions.size() + 3 * nVerts);
		for (int i = 0; i < nVerts; ++i) {
			FbxVector4 cp = meshGlobal.MultT(mesh->GetControlPointAt(i));
			float raw[3] = { (float)cp[0], (float)cp[1], (float)cp[2] };
			m_rawMeshPos.push_back(raw[0]);
			m_rawMeshPos.push_back(raw[1]);
			m_rawMeshPos.push_back(raw[2]);
			float mod[3];
			transformPoint(raw, mod);
			int vi = vertexBase + i;
			m_mesh.restPositions[3 * vi + 0] = mod[0];
			m_mesh.restPositions[3 * vi + 1] = mod[1];
			m_mesh.restPositions[3 * vi + 2] = mod[2];
		}

		int polyCount = mesh->GetPolygonCount();
		m_mesh.indices.reserve(m_mesh.indices.size() + polyCount * 3);
		for (int p = 0; p < polyCount; ++p) {
			int sz = mesh->GetPolygonSize(p);
			for (int v = 2; v < sz; ++v) {
				m_mesh.indices.push_back(vertexBase + mesh->GetPolygonVertex(p, 0));
				m_mesh.indices.push_back(vertexBase + mesh->GetPolygonVertex(p, v - 1));
				m_mesh.indices.push_back(vertexBase + mesh->GetPolygonVertex(p, v));
			}
		}

		int skinCount = mesh->GetDeformerCount(FbxDeformer::eSkin);
		for (int s = 0; s < skinCount; ++s) {
			FbxSkin *skin = (FbxSkin *)mesh->GetDeformer(s, FbxDeformer::eSkin);
			if (!skin)
				continue;
			int clusterCount = skin->GetClusterCount();
			for (int c = 0; c < clusterCount; ++c) {
				FbxCluster *cluster = skin->GetCluster(c);
				if (!cluster)
					continue;
				FbxNode *link = cluster->GetLink();
				if (!link)
					continue;
				std::map<std::string, int>::iterator it = boneIndex.find(link->GetName());
				if (it == boneIndex.end())
					continue;
				int boneIdx = it->second;

				int nIndices = cluster->GetControlPointIndicesCount();
				const int *idx = cluster->GetControlPointIndices();
				const double *w = cluster->GetControlPointWeights();
				for (int k = 0; k < nIndices; ++k) {
					int rawIdx = idx[k];
					if (rawIdx < 0 || rawIdx >= nVerts)
						continue;
					if (w[k] > 0.0001)
						m_mesh.weights[vertexBase + rawIdx].push_back(std::make_pair(boneIdx, (float)w[k]));
				}
			}
		}

		vertexBase += nVerts;
	}

	// Normalize weights
	for (size_t i = 0; i < m_mesh.weights.size(); ++i) {
		float sum = 0.0f;
		for (size_t k = 0; k < m_mesh.weights[i].size(); ++k)
			sum += m_mesh.weights[i][k].second;
		if (sum > 1e-5f)
			for (size_t k = 0; k < m_mesh.weights[i].size(); ++k)
				m_mesh.weights[i][k].second /= sum;
	}

	m_mesh.valid = !m_mesh.restPositions.empty() && !m_mesh.indices.empty();
	return m_mesh.valid;
}

bool KRetargeter::loadCharacter(const char *fbxPath) {
	clearCharacter();

	FbxManager *mgr = FbxManager::Create();
	if (!mgr)
		return false;

	FbxIOSettings *ios = FbxIOSettings::Create(mgr, IOSROOT);
	mgr->SetIOSettings(ios);

	FbxImporter *importer = FbxImporter::Create(mgr, "");
	bool ok = importer->Initialize(fbxPath, -1, mgr->GetIOSettings());
	if (ok) {
		FbxScene *scene = FbxScene::Create(mgr, "Character");
		ok = importer->Import(scene);
		if (ok) {
			buildSkeleton(scene);
			extractMesh(scene);
		}
		scene->Destroy();
	}
	importer->Destroy();
	mgr->Destroy();

	if (m_bones.empty())
		return false;

	m_sourcePath = fbxPath;
	const char *slash = strrchr(fbxPath, '\\');
	const char *fslash = strrchr(fbxPath, '/');
	if (fslash > slash) slash = fslash;
	m_characterName = (slash ? slash + 1 : fbxPath);
	m_scaleReady = false;
	m_rootBone = -1;
	m_nodeGraph.resetDefault();   // Fresh character, fresh default graph
	m_liveHistory.clear();
	m_footUpstream.clear();
	m_liveTick = 0;
	m_meshVersion++;
	return true;
}

void KRetargeter::clearCharacter() {
	m_bones.clear();
	m_mesh = RetargetMesh();
	m_pose = RetargetPose();
	memset(m_mapping, -1, sizeof(m_mapping));
	m_sourcePath.clear();
	m_characterName.clear();
	m_scaleReady = false;
	m_rootBone = -1;
	m_nodeGraph.resetDefault();
	m_liveHistory.clear();
	m_footUpstream.clear();
	m_liveTick = 0;
	m_meshVersion++;
}

// ----------------------------------------------------------------------------
// Bone mapping
// ----------------------------------------------------------------------------

static std::string lowerName(const std::string &s) {
	std::string out = s;
	std::transform(out.begin(), out.end(), out.begin(), ::tolower);
	return out;
}

// Side of a bone from its name: +1 left, -1 right, 0 unknown
static int nameSide(const std::string &lower) {
	if (lower.find("left") != std::string::npos)
		return 1;
	if (lower.find("right") != std::string::npos)
		return -1;
	// Token based: any token that is exactly "l" / "r"
	size_t start = 0;
	while (start < lower.size()) {
		size_t end = start;
		while (end < lower.size() && isalnum((unsigned char)lower[end])) end++;
		if (end - start == 1 && lower[start] == 'l')
			return 1;
		if (end - start == 1 && lower[start] == 'r')
			return -1;
		start = end + 1;
		while (start < lower.size() && !isalnum((unsigned char)lower[start])) start++;
	}
	return 0;
}

// Side of a bone, inferred from the name when possible, otherwise from its
// rest position (+X is the left side in scene space).
static int boneSide(const std::string &lower, float restX) {
	int s = nameSide(lower);
	if (s != 0)
		return s;
	return (restX >= 0.0f) ? 1 : -1;
}

// Canonical "base" name with the side marker removed. "Shoulder.L" and
// "Shoulder.R" both become "Shoulder", "bip_l_arm" / "bip_r_arm" become
// "bip_arm", "LeftArm" / "RightArm" become "Arm", "HandL" / "HandR" become
// "Hand". Used to detect left/right symmetric bone pairs.
static std::string sideKey(const std::string &lower) {
	std::string out;
	size_t start = 0;
	while (start < lower.size()) {
		size_t end = start;
		while (end < lower.size() && isalnum((unsigned char)lower[end])) end++;
		std::string tok = lower.substr(start, end - start);
		bool side = (tok == "left" || tok == "right")
			|| (tok.size() == 1 && (tok[0] == 'l' || tok[0] == 'r'));
		if (!side && tok.size() > 4) {
			size_t t = tok.size();
			if (tok.compare(t - 4, 4, "left") == 0 || tok.compare(t - 4, 4, "right") == 0) {
				side = true; tok = tok.substr(0, t - 4);
			} else if (tok.compare(0, 4, "left") == 0 || tok.compare(0, 4, "right") == 0) {
				side = true; tok = tok.substr(4);
			} else if (tok[t - 1] == 'l' || tok[t - 1] == 'r') {
				side = true; tok = tok.substr(0, t - 1);
			}
		}
		if (!side)
			out += tok;
		start = end;
		while (start < lower.size() && !isalnum((unsigned char)lower[start])) {
			out += lower[start];
			start++;
		}
	}
	return out;
}

// Marks every bone that is part of a left/right pair (two bones whose base
// name is identical and whose side markers are opposite, e.g. "Shoulder.L" /
// "Shoulder.R"). Such symmetric bones are limb bones and can never belong to
// the spine / center chain: a left clavicle must not be used as the neck.
static std::vector<char> findPairedBones(const std::vector<RetargetBone> &bones) {
	int n = (int)bones.size();
	std::vector<char> paired(n, 0);
	std::map<std::string, std::vector<std::pair<int, int> > > groups;
	for (int i = 0; i < n; ++i) {
		std::string lower = lowerName(bones[i].name);
		groups[sideKey(lower)].push_back(std::make_pair(i, nameSide(lower)));
	}
	for (std::map<std::string, std::vector<std::pair<int, int> > >::iterator it = groups.begin();
		it != groups.end(); ++it) {
		bool hasL = false, hasR = false;
		for (size_t k = 0; k < it->second.size(); ++k) {
			if (it->second[k].second == 1) hasL = true;
			else if (it->second[k].second == -1) hasR = true;
		}
		if (hasL && hasR)
			for (size_t k = 0; k < it->second.size(); ++k)
				if (it->second[k].second != 0)
					paired[it->second[k].first] = 1;
	}
	return paired;
}

static bool hasAny(const std::string &n, const char *const *toks, int count) {
	for (int i = 0; i < count; ++i)
		if (n.find(toks[i]) != std::string::npos)
			return true;
	return false;
}

// Precise bone-role detection from the name. Sets the Kinect joint and a
// priority score (higher = more confident), or returns false when the name is
// not a clear role. Small "helper" bones at the elbow / knee / wrist / ankle
// are rejected so a little "elbow" helper cannot steal the joint from the real
// upper arm / forearm / shin / foot bone.
static bool matchBoneRole(const std::string &n, int side, float restLen,
	float skelHeight, int &joint, int &score)
{
	static const char *pelvisToks[]   = { "pelvis", "hips", "hipbase" };
	static const char *spineMidToks[] = { "spinemid", "spine1", "abdomen", "waist", "belly", "lowerback" };
	static const char *spineTopToks[] = { "spineshoulder", "spine2", "spine3", "chest", "thorax", "sternum", "upperback" };
	static const char *neckToks[]     = { "neck" };
	static const char *headToks[]     = { "head", "skull" };

	if (hasAny(n, pelvisToks, _countof(pelvisToks))) {
		joint = JointType_SpineBase; score = 100; return true;
	}
	if (hasAny(n, spineMidToks, _countof(spineMidToks))) {
		joint = JointType_SpineMid; score = 100; return true;
	}
	if (hasAny(n, spineTopToks, _countof(spineTopToks))) {
		joint = JointType_SpineShoulder; score = 100; return true;
	}
	if (hasAny(n, neckToks, _countof(neckToks))) {
		joint = JointType_Neck; score = 100; return true;
	}
	if (hasAny(n, headToks, _countof(headToks))) {
		joint = JointType_Head; score = 100; return true;
	}

	if (side == 0)
		return false;

	static const char *shoulderToks[] = { "shoulder", "clavicle", "collarbone", "acromion" };
	static const char *upperArmToks[] = { "upperarm", "uparm", "upper_arm", "bicep", "humerus" };
	static const char *armToks[]      = { "arm" };
	static const char *elbowToks[]    = { "elbow" };
	static const char *wristToks[]    = { "wrist" };
	static const char *handToks[]     = { "hand", "palm" };

	static const char *thighToks[]    = { "thigh", "upleg", "upperleg", "upper_leg", "hip" };
	static const char *calfToks[]     = { "calf", "shin", "lowerleg", "lower_leg" };
	static const char *kneeToks[]     = { "knee" };
	static const char *ankleToks[]    = { "ankle" };
	static const char *footToks[]     = { "foot" };
	static const char *toeToks[]      = { "toe" };
	static const char *legToks[]      = { "leg" };

	int shL = JointType_ShoulderLeft, shR = JointType_ShoulderRight;
	int elL = JointType_ElbowLeft, elR = JointType_ElbowRight;
	int wrL = JointType_WristLeft, wrR = JointType_WristRight;
	int haL = JointType_HandLeft, haR = JointType_HandRight;
	int hiL = JointType_HipLeft, hiR = JointType_HipRight;
	int knL = JointType_KneeLeft, knR = JointType_KneeRight;
	int anL = JointType_AnkleLeft, anR = JointType_AnkleRight;
	int foL = JointType_FootLeft, foR = JointType_FootRight;
	bool L = side > 0;

	bool helper = true;
	if (hasAny(n, shoulderToks, _countof(shoulderToks))) {
		joint = L ? shL : shR; score = 100; helper = false;
	} else if (hasAny(n, upperArmToks, _countof(upperArmToks))) {
		joint = L ? shL : shR; score = 70; helper = false;
	} else if (hasAny(n, armToks, _countof(armToks))) {
		// generic "arm" (upper arm); must not catch "forearm" / "lowerarm" -
		// the chain propagation decides those from the skeleton hierarchy
		if (n.find("fore") == std::string::npos && n.find("lower") == std::string::npos) {
			joint = L ? shL : shR; score = 50; helper = false;
		} else {
			return false;
		}
	} else if (hasAny(n, elbowToks, _countof(elbowToks))) {
		joint = L ? elL : elR; score = 100;
	} else if (hasAny(n, wristToks, _countof(wristToks))) {
		joint = L ? wrL : wrR; score = 100;
	} else if (hasAny(n, handToks, _countof(handToks))) {
		joint = L ? haL : haR; score = 100; helper = false;
	} else if (hasAny(n, thighToks, _countof(thighToks))) {
		joint = L ? hiL : hiR; score = 100; helper = false;
	} else if (hasAny(n, calfToks, _countof(calfToks))) {
		joint = L ? knL : knR; score = 90; helper = false;
	} else if (hasAny(n, kneeToks, _countof(kneeToks))) {
		joint = L ? knL : knR; score = 100;
	} else if (hasAny(n, ankleToks, _countof(ankleToks))) {
		joint = L ? anL : anR; score = 100;
	} else if (hasAny(n, footToks, _countof(footToks))) {
		joint = L ? foL : foR; score = 100; helper = false;
	} else if (hasAny(n, toeToks, _countof(toeToks))) {
		joint = L ? foL : foR; score = 60; helper = false;
	} else if (hasAny(n, legToks, _countof(legToks)) &&
		n.find("fore") == std::string::npos) {
		// generic "leg" (lower leg); "upleg" / "upperleg" already matched above
		joint = L ? knL : knR; score = 40; helper = false;
	} else {
		return false;
	}

	// A helper joint bone (elbow / knee / wrist / ankle) must be long enough to
	// be the real limb bone; otherwise it's a little helper placed at the joint
	// and the actual limb bone (upper arm / forearm / shin / foot) keeps it.
	if (helper && restLen < 0.10f * skelHeight)
		return false;

	return true;
}

void KRetargeter::autoMap() {
	int nBones = (int)m_bones.size();
	memset(m_mapping, -1, sizeof(m_mapping));
	if (nBones == 0)
		return;

	// Skeleton height, used to reject tiny "helper" joint bones
	float minY = FLT_MAX, maxY = -FLT_MAX;
	for (int i = 0; i < nBones; ++i) {
		if (m_bones[i].restPos[1] < minY) minY = m_bones[i].restPos[1];
		if (m_bones[i].restPos[1] > maxY) maxY = m_bones[i].restPos[1];
	}
	float skelHeight = (maxY - minY > 1e-3f) ? (maxY - minY) : 1.0f;

	// ---- Pass 1: name-based matching, sorted by confidence ----
	// (a "clavicle" beats a "upperarm" for the shoulder joint, etc.)
	struct Candidate { int bone; int joint; int score; };
	std::vector<Candidate> cands;
	for (int i = 0; i < nBones; ++i) {
		std::string n = lowerName(m_bones[i].name);
		int side = boneSide(n, m_bones[i].restPos[0]);
		int joint = -1, score = 0;
		if (matchBoneRole(n, side, m_bones[i].restLen, skelHeight, joint, score)) {
			Candidate c;
			c.bone = i; c.joint = joint; c.score = score;
			cands.push_back(c);
		}
	}
	std::sort(cands.begin(), cands.end(), [](const Candidate &a, const Candidate &b) {
		if (a.score != b.score) return a.score > b.score;
		return a.bone < b.bone;
	});
	for (size_t c = 0; c < cands.size(); ++c) {
		if (m_mapping[cands[c].joint] < 0)
			m_mapping[cands[c].joint] = cands[c].bone;
	}

	// ---- Pass 2: parent / child propagation from the name anchors ----
	propagateFromAnchors();

	// ---- Pass 3: structure scan fills whatever the names could not find ----
	scanMapByPosition(true);
}

void KRetargeter::autoMapByPosition() {
	scanMapByPosition(false);
}

// Name anchors -> chain propagation. A bone named "shoulder" means the chain
// below it MUST be the upper arm (elbow), lower arm (wrist) and hand; a bone
// named "upper leg" means the chain below it MUST be the lower leg (knee),
// ankle and foot. The same anchors also work the other way: an "elbow" anchor
// gives its shoulder above, a "knee" anchor gives its hip above, etc. This is
// what keeps the auto-mapping precise when the rig renames or skips bones.
void KRetargeter::propagateFromAnchors() {
	int nBones = (int)m_bones.size();
	if (nBones == 0)
		return;

	// Left/right pairs (e.g. "Shoulder.L"/"Shoulder.R") are limb bones: they
	// must never be used to walk the spine chain.
	std::vector<char> paired = findPairedBones(m_bones);

	std::vector<std::vector<int> > children(nBones);
	for (int i = 0; i < nBones; ++i) {
		int p = m_bones[i].parent;
		if (p >= 0 && p < nBones)
			children[p].push_back(i);
	}

	// Subtree sizes (iterative post-order over the whole forest)
	std::vector<int> subtree(nBones, 1);
	{
		std::vector<int> stack;
		std::vector<char> entered(nBones, 0);
		std::vector<char> finished(nBones, 0);
		for (int r = 0; r < nBones; ++r) {
			int p = m_bones[r].parent;
			if (p >= 0 && p < nBones)
				continue;
			stack.push_back(r);
			while (!stack.empty()) {
				int cur = stack.back();
				if (!entered[cur]) {
					entered[cur] = 1;
					for (size_t c = 0; c < children[cur].size(); ++c)
						stack.push_back(children[cur][c]);
				} else if (!finished[cur]) {
					finished[cur] = 1;
					int s = 1;
					for (size_t c = 0; c < children[cur].size(); ++c)
						s += subtree[children[cur][c]];
					subtree[cur] = s;
					stack.pop_back();
				} else {
					stack.pop_back();
				}
			}
		}
	}

	auto used = [&](int bone) {
		for (int j = 0; j < 25; ++j)
			if (m_mapping[j] == bone)
				return true;
		return false;
	};
	auto assignFree = [&](int joint, int bone) {
		if (joint < 0 || joint >= 25 || bone < 0 || bone >= nBones)
			return;
		if (m_mapping[joint] >= 0 || used(bone))
			return;
		m_mapping[joint] = bone;
	};

	// Main child chain of a bone (follow the largest subtree, stop before the
	// fingers / toes). Includes root.
	auto mainChain = [&](int root) {
		std::vector<int> chain;
		chain.push_back(root);
		int b = root;
		while (true) {
			int next = -1, biggest = 0;
			for (size_t c = 0; c < children[b].size(); ++c) {
				int ch = children[b][c];
				if (subtree[ch] > biggest) { biggest = subtree[ch]; next = ch; }
			}
			if (next < 0)
				break;
			if (biggest <= 3 && (int)chain.size() >= 3)
				break;
			chain.push_back(next);
			b = next;
		}
		return chain;
	};

	// Children of root on the main chain (root excluded)
	auto downChain = [&](int root) {
		std::vector<int> chain = mainChain(root);
		std::vector<int> down;
		for (size_t i = 1; i < chain.size(); ++i)
			down.push_back(chain[i]);
		return down;
	};

	// Joints below a Shoulder / Hip anchor
	auto mapArmDown = [&](int anchor, bool isLeft) {
		std::vector<int> down = downChain(anchor);
		int m = (int)down.size();
		if (m >= 1) assignFree(isLeft ? JointType_ElbowLeft : JointType_ElbowRight, down[0]);
		if (m >= 2) assignFree(isLeft ? JointType_WristLeft : JointType_WristRight, down[1]);
		if (m >= 3) assignFree(isLeft ? JointType_HandLeft : JointType_HandRight, down[m - 1]);
	};
	auto mapLegDown = [&](int anchor, bool isLeft) {
		std::vector<int> down = downChain(anchor);
		int m = (int)down.size();
		if (m >= 1) assignFree(isLeft ? JointType_KneeLeft : JointType_KneeRight, down[0]);
		if (m >= 2) assignFree(isLeft ? JointType_AnkleLeft : JointType_AnkleRight, down[1]);
		if (m >= 3) assignFree(isLeft ? JointType_FootLeft : JointType_FootRight, down[m - 1]);
	};

	for (int j = 0; j < 25; ++j) {
		int b = m_mapping[j];
		if (b < 0)
			continue;
		bool L = (j == JointType_ShoulderLeft || j == JointType_ElbowLeft ||
			j == JointType_WristLeft || j == JointType_HandLeft ||
			j == JointType_HipLeft || j == JointType_KneeLeft ||
			j == JointType_AnkleLeft || j == JointType_FootLeft);
		switch (j) {
			case JointType_ShoulderLeft: case JointType_ShoulderRight:
				mapArmDown(b, L);
				break;
			case JointType_ElbowLeft: case JointType_ElbowRight:
				if (m_bones[b].parent >= 0)
					assignFree(L ? JointType_ShoulderLeft : JointType_ShoulderRight, m_bones[b].parent);
				mapArmDown(b, L);
				break;
			case JointType_WristLeft: case JointType_WristRight:
				if (m_bones[b].parent >= 0)
					assignFree(L ? JointType_ElbowLeft : JointType_ElbowRight, m_bones[b].parent);
				{
					int gp = (m_bones[b].parent >= 0) ? m_bones[m_bones[b].parent].parent : -1;
					if (gp >= 0)
						assignFree(L ? JointType_ShoulderLeft : JointType_ShoulderRight, gp);
				}
				break;
			case JointType_HandLeft: case JointType_HandRight:
				break;
			case JointType_HipLeft: case JointType_HipRight:
				mapLegDown(b, L);
				break;
			case JointType_KneeLeft: case JointType_KneeRight:
				if (m_bones[b].parent >= 0)
					assignFree(L ? JointType_HipLeft : JointType_HipRight, m_bones[b].parent);
				mapLegDown(b, L);
				break;
			case JointType_AnkleLeft: case JointType_AnkleRight:
				if (m_bones[b].parent >= 0)
					assignFree(L ? JointType_KneeLeft : JointType_KneeRight, m_bones[b].parent);
				{
					int gp = (m_bones[b].parent >= 0) ? m_bones[m_bones[b].parent].parent : -1;
					if (gp >= 0)
						assignFree(L ? JointType_HipLeft : JointType_HipRight, gp);
				}
				break;
			case JointType_FootLeft: case JointType_FootRight:
				break;
			case JointType_SpineBase:
				// walk up the spine: follow the non-paired child with the
				// biggest subtree each step (tie: highest-Y), so decorative
				// chains hanging off the torso never hijack the column.
				{
					int cur = b;
					const int nextJoints[4] = { JointType_SpineMid, JointType_SpineShoulder, JointType_Neck, JointType_Head };
					for (int k = 0; k < 4; ++k) {
						int next = -1; int bestSub = 0; float nextY = -FLT_MAX;
						for (size_t c = 0; c < children[cur].size(); ++c) {
							int ch = children[cur][c];
							if (paired[ch])
								continue;
							if (subtree[ch] > bestSub ||
								(subtree[ch] == bestSub && m_bones[ch].restPos[1] > nextY)) {
								bestSub = subtree[ch];
								nextY = m_bones[ch].restPos[1];
								next = ch;
							}
						}
						if (next < 0)
							break;
						assignFree(nextJoints[k], next);
						cur = next;
					}
				}
				break;
			case JointType_Head:
				// walk down the spine (toward the pelvis)
				{
					int cur = b;
					const int upJoints[4] = { JointType_Neck, JointType_SpineShoulder, JointType_SpineMid, JointType_SpineBase };
					for (int k = 0; k < 4; ++k) {
						int p = m_bones[cur].parent;
						if (p < 0)
							break;
						assignFree(upJoints[k], p);
						cur = p;
					}
				}
				break;
			case JointType_Neck:
				{
					int cur = b;
					const int upJoints[3] = { JointType_SpineShoulder, JointType_SpineMid, JointType_SpineBase };
					for (int k = 0; k < 3; ++k) {
						int p = m_bones[cur].parent;
						if (p < 0)
							break;
						assignFree(upJoints[k], p);
						cur = p;
					}
				}
				break;
			default:
				break;
		}
	}
}

// ----------------------------------------------------------------------------
// Structure-based bone mapping.
// Works from the skeleton hierarchy alone (plus the ghost T-pose for the limb
// proportions), so a character whose rest pose is NOT a perfect T (A-pose,
// relaxed arms, deep crouch, ...) is still mapped correctly and retargets into
// a T-pose. The character's own bone LENGTHS drive the elbow / knee / wrist /
// ankle positions, which is independent of how the limbs are currently bent.
// ----------------------------------------------------------------------------
void KRetargeter::scanMapByPosition(bool keepMapped)
{
	int nBones = (int)m_bones.size();
	if (nBones == 0)
		return;

	// Left/right pairs (e.g. "Shoulder.L"/"Shoulder.R") are limb bones: they
	// must never be picked for the spine / center chain.
	std::vector<char> paired = findPairedBones(m_bones);

	// Bones that are already mapped when we enter (by name / chain propagation)
	// are off-limits: the structure scan must not steal them from their joint.
	std::vector<char> preMapped(nBones, 0);
	if (keepMapped)
		for (int j = 0; j < 25; ++j)
			if (m_mapping[j] >= 0)
				preMapped[m_mapping[j]] = 1;

	// Joint assignment helper (keeps the "one bone per joint" invariant)
	auto assign = [&](int joint, int bone) {
		if (joint < 0 || joint >= 25 || bone < 0 || bone >= nBones)
			return;
		if (keepMapped && m_mapping[joint] >= 0)
			return;
		if (keepMapped && preMapped[bone])
			return;
		for (int j = 0; j < 25; ++j)
			if (m_mapping[j] == bone && j != joint)
				m_mapping[j] = -1;
		m_mapping[joint] = bone;
	};

	// ---- Skeleton tree ----
	std::vector<std::vector<int> > children(nBones);
	for (int i = 0; i < nBones; ++i) {
		int p = m_bones[i].parent;
		if (p >= 0 && p < nBones)
			children[p].push_back(i);
	}

	// Subtree sizes (iterative post-order over the whole forest)
	std::vector<int> subtree(nBones, 1);
	{
		std::vector<int> stack;
		std::vector<char> entered(nBones, 0);
		std::vector<char> finished(nBones, 0);
		for (int r = 0; r < nBones; ++r) {
			int p = m_bones[r].parent;
			if (p >= 0 && p < nBones)
				continue;
			stack.push_back(r);
			while (!stack.empty()) {
				int cur = stack.back();
				if (!entered[cur]) {
					entered[cur] = 1;
					for (size_t c = 0; c < children[cur].size(); ++c)
						stack.push_back(children[cur][c]);
				} else if (!finished[cur]) {
					finished[cur] = 1;
					int s = 1;
					for (size_t c = 0; c < children[cur].size(); ++c)
						s += subtree[children[cur][c]];
					subtree[cur] = s;
					stack.pop_back();
				} else {
					stack.pop_back();
				}
			}
		}
	}

	// ---- Pelvis: the centre bone with the biggest subtree, never a left/right
	// pair. Using the child count alone is fragile: a hand / palm with many
	// finger helpers often outranks the real pelvis, and a limb bone is never
	// a valid centre. Subtree size keeps the whole body together instead.
	int pelvis = -1;
	for (int i = 0; i < nBones; ++i) {
		if ((int)children[i].size() < 2 || paired[i])
			continue;
		if (pelvis < 0 || subtree[i] > subtree[pelvis])
			pelvis = i;
	}
	// Guard: a chest / spine bone can also have many helper children. The real
	// pelvis never lets a single child dominate its subtree, so step up in that
	// case. A pelvis must also be a unique center bone, never a left/right pair.
	while (pelvis >= 0) {
		int biggestChild = -1, biggest = 0;
		for (size_t c = 0; c < children[pelvis].size(); ++c) {
			int ch = children[pelvis][c];
			if (subtree[ch] > biggest) { biggest = subtree[ch]; biggestChild = ch; }
		}
		if (biggestChild >= 0 && biggest > (subtree[pelvis] - 1) * 70 / 100) {
			int p = m_bones[pelvis].parent;
			if (p >= 0 && p < nBones && (int)children[p].size() >= 2) {
				pelvis = p;
				continue;
			}
		}
		if (paired[pelvis]) {
			int p = m_bones[pelvis].parent;
			if (p >= 0 && p < nBones && !paired[p] && (int)children[p].size() >= 2) {
				pelvis = p;
				continue;
			}
		}
		break;
	}

	if (pelvis < 0) {
		// No branching node: not a humanoid hierarchy, plain position matching
		mapByPositionFallback(keepMapped);
		return;
	}

	// ---- Spine chain: pelvis -> ... -> head. Follow the non-paired child with
	// the biggest subtree (tie: the highest-Y one). Raw "highest Y" alone picks
	// decorative chains hanging off the torso (hoods, capes, bags) over the real
	// neck / head, so the subtree size keeps the walk on the main trunk.
	std::vector<int> spine;
	spine.push_back(pelvis);
	int cur = pelvis;
	while (true) {
		int next = -1;
		int bestSub = 0;
		float nextY = -FLT_MAX;
		for (size_t c = 0; c < children[cur].size(); ++c) {
			int ch = children[cur][c];
			if (paired[ch])
				continue;
			if (subtree[ch] > bestSub ||
				(subtree[ch] == bestSub && m_bones[ch].restPos[1] > nextY)) {
				bestSub = subtree[ch];
				nextY = m_bones[ch].restPos[1];
				next = ch;
			}
		}
		if (next < 0)
			break;
		spine.push_back(next);
		cur = next;
	}
	std::vector<char> inSpine(nBones, 0);
	for (size_t i = 0; i < spine.size(); ++i)
		inSpine[spine[i]] = 1;

	// Spine joints: the top of the chain is the head, its parent the neck, ...
	int L = (int)spine.size();
	assign(JointType_SpineBase, spine[0]);
	if (L >= 2) assign(JointType_SpineMid, spine[1]);
	if (L >= 3) assign(JointType_Neck, spine[L - 2]);
	if (L >= 4) assign(JointType_SpineShoulder, spine[L - 3]);
	assign(JointType_Head, spine[L - 1]);

	// ---- Limb chains (follow the child with the biggest subtree) ----
	struct ChainInfo { std::vector<int> bones; };
	auto buildChain = [&](int root) -> ChainInfo {
		ChainInfo ci;
		ci.bones.push_back(root);
		int b = root;
		while (true) {
			int next = -1, biggest = 0;
			for (size_t c = 0; c < children[b].size(); ++c) {
				int ch = children[b][c];
				if (subtree[ch] > biggest) { biggest = subtree[ch]; next = ch; }
			}
			if (next < 0)
				break;
			// Stop at the hand / foot: whatever is left is fingers / toes
			if (biggest <= 3 && (int)ci.bones.size() >= 3)
				break;
			ci.bones.push_back(next);
			b = next;
		}
		return ci;
	};

	// Legs: pelvis children that are not on the spine chain
	std::vector<ChainInfo> legs;
	for (size_t c = 0; c < children[pelvis].size(); ++c) {
		int ch = children[pelvis][c];
		if (!inSpine[ch])
			legs.push_back(buildChain(ch));
	}

	// Arms: side chains hanging off the spine, keep the two longest
	std::vector<ChainInfo> arms;
	for (size_t s = 1; s < spine.size(); ++s) {
		for (size_t c = 0; c < children[spine[s]].size(); ++c) {
			int ch = children[spine[s]][c];
			if (!inSpine[ch])
				arms.push_back(buildChain(ch));
		}
	}
	std::sort(arms.begin(), arms.end(), [](const ChainInfo &a, const ChainInfo &b) {
		return a.bones.size() > b.bones.size();
	});
	if (arms.size() > 2)
		arms.resize(2);

	// ---- Ghost T-pose limb proportions (fraction of the chain length) ----
	float gElbow = 0.45f, gWrist = 0.87f;
	float gKnee = 0.41f, gAnkle = 0.91f;
	{
		auto dist = [this](int a, int b) {
			float d = 0.0f;
			for (int k = 0; k < 3; ++k) {
				float e = m_ghostPos[a][k] - m_ghostPos[b][k];
				d += e * e;
			}
			return sqrtf(d);
		};
		float totA = dist(JointType_ShoulderLeft, JointType_ElbowLeft)
			+ dist(JointType_ElbowLeft, JointType_WristLeft)
			+ dist(JointType_WristLeft, JointType_HandLeft);
		if (totA > 1e-4f) {
			gElbow = dist(JointType_ShoulderLeft, JointType_ElbowLeft) / totA;
			gWrist = (dist(JointType_ShoulderLeft, JointType_ElbowLeft)
				+ dist(JointType_ElbowLeft, JointType_WristLeft)) / totA;
		}
		float totL = dist(JointType_HipLeft, JointType_KneeLeft)
			+ dist(JointType_KneeLeft, JointType_AnkleLeft)
			+ dist(JointType_AnkleLeft, JointType_FootLeft);
		if (totL > 1e-4f) {
			gKnee = dist(JointType_HipLeft, JointType_KneeLeft) / totL;
			gAnkle = (dist(JointType_HipLeft, JointType_KneeLeft)
				+ dist(JointType_KneeLeft, JointType_AnkleLeft)) / totL;
		}
	}

	// Map one limb chain onto its 4 joints using the chain-length fractions
	auto mapLimb = [&](const ChainInfo &ci, int j0, int jMid, float fMid,
		int jLow, float fLow, int jEnd) {
		int m = (int)ci.bones.size();
		if (m <= 0)
			return;
		assign(j0, ci.bones[0]);
		if (m == 1)
			return;   // single bone: the fill pass handles the rest of the limb
		// Cumulative length fractions along the chain (pose independent)
		std::vector<float> cf(m, 0.0f);
		float tot = 0.0f;
		for (int k = 0; k + 1 < m; ++k) {
			const float *a = m_bones[ci.bones[k]].restPos;
			const float *b = m_bones[ci.bones[k + 1]].restPos;
			float e0 = b[0] - a[0], e1 = b[1] - a[1], e2 = b[2] - a[2];
			tot += sqrtf(e0 * e0 + e1 * e1 + e2 * e2);
			cf[k + 1] = tot;
		}
		if (tot > 1e-5f)
			for (int k = 0; k < m; ++k) cf[k] /= tot;
		auto interior = [&](float target) {
			int best = 1;
			float bd = 1e30f;
			for (int k = 1; k + 1 < m; ++k) {
				float d = fabsf(cf[k] - target);
				if (d < bd) { bd = d; best = k; }
			}
			return best;
		};
		if (m == 2) {
			assign(jEnd, ci.bones[1]);
		} else {
			assign(jMid, ci.bones[interior(fMid)]);
			assign(jLow, ci.bones[interior(fLow)]);
			assign(jEnd, ci.bones[m - 1]);
		}
	};

	// Left/right: the side markers in the bone names win; otherwise the limb
	// whose root is further toward +X is treated as the left one
	auto leftOf = [&](const ChainInfo &a, const ChainInfo &b) {
		int sa = nameSide(lowerName(m_bones[a.bones[0]].name));
		int sb = nameSide(lowerName(m_bones[b.bones[0]].name));
		if (sa != 0 && sb != 0 && sa != sb)
			return sa > 0;
		return m_bones[a.bones[0]].restPos[0] >= m_bones[b.bones[0]].restPos[0];
	};

	if (legs.size() >= 2) {
		bool l = leftOf(legs[0], legs[1]);
		mapLimb(legs[l ? 0 : 1], JointType_HipLeft, JointType_KneeLeft, gKnee,
			JointType_AnkleLeft, gAnkle, JointType_FootLeft);
		mapLimb(legs[l ? 1 : 0], JointType_HipRight, JointType_KneeRight, gKnee,
			JointType_AnkleRight, gAnkle, JointType_FootRight);
	} else if (legs.size() == 1) {
		mapLimb(legs[0], JointType_HipLeft, JointType_KneeLeft, gKnee,
			JointType_AnkleLeft, gAnkle, JointType_FootLeft);
	}

	if (arms.size() >= 2) {
		bool l = leftOf(arms[0], arms[1]);
		mapLimb(arms[l ? 0 : 1], JointType_ShoulderLeft, JointType_ElbowLeft, gElbow,
			JointType_WristLeft, gWrist, JointType_HandLeft);
		mapLimb(arms[l ? 1 : 0], JointType_ShoulderRight, JointType_ElbowRight, gElbow,
			JointType_WristRight, gWrist, JointType_HandRight);
	} else if (arms.size() == 1) {
		mapLimb(arms[0], JointType_ShoulderLeft, JointType_ElbowLeft, gElbow,
			JointType_WristLeft, gWrist, JointType_HandLeft);
	}

	// ---- Fill whatever is left with plain position matching ----
	fillMapping(keepMapped);
}

// ----------------------------------------------------------------------------
// Plain position-based fallback (used when the hierarchy has no branching node).
// Both skeletons are normalized (height in [0,1], centered) so the matching
// also works before the user sets the import size / orientation.
// ----------------------------------------------------------------------------
void KRetargeter::mapByPositionFallback(bool keepMapped)
{
	if (!keepMapped)
		memset(m_mapping, -1, sizeof(m_mapping));
	fillMapping(keepMapped);
}

// Fills the still-unmapped joints with greedy nearest-position matching against
// the ghost, normalized the same way as the fallback above.
void KRetargeter::fillMapping(bool keepMapped)
{
	int nBones = (int)m_bones.size();
	if (nBones == 0)
		return;

	// Ghost reference: normalized joint positions
	float gMinY = FLT_MAX, gMaxY = -FLT_MAX;
	for (int j = 0; j < JointType_Count; ++j) {
		float y = m_ghostPos[j][1];
		if (y < gMinY) gMinY = y;
		if (y > gMaxY) gMaxY = y;
	}
	float gRef = gMaxY - gMinY;
	if (gRef < 1e-3f) gRef = 1.0f;
	float gcx = 0.0f, gcz = 0.0f;
	for (int j = 0; j < JointType_Count; ++j) {
		gcx += m_ghostPos[j][0];
		gcz += m_ghostPos[j][2];
	}
	gcx /= (float)JointType_Count;
	gcz /= (float)JointType_Count;

	struct GhostJoint { int joint; float x, y, z; };
	std::vector<GhostJoint> gj;
	for (int i = 0; i < g_retargetJointCount; ++i) {
		int j = g_retargetJoints[i].jointType;
		if (j < 0 || j >= 25 || m_mapping[j] >= 0)
			continue;
		GhostJoint gjj;
		gjj.joint = j;
		gjj.x = (m_ghostPos[j][0] - gcx) / gRef;
		gjj.y = (m_ghostPos[j][1] - gMinY) / gRef;
		gjj.z = (m_ghostPos[j][2] - gcz) / gRef;
		gj.push_back(gjj);
	}
	if (gj.empty())
		return;

	// Character: normalized bone positions
	float cMinY = FLT_MAX, cMaxY = -FLT_MAX;
	for (int i = 0; i < nBones; ++i) {
		float y = m_bones[i].restPos[1];
		if (y < cMinY) cMinY = y;
		if (y > cMaxY) cMaxY = y;
	}
	float cRef = cMaxY - cMinY;
	if (cRef < 1e-3f) cRef = 1.0f;
	float ccx = 0.0f, ccz = 0.0f;
	for (int i = 0; i < nBones; ++i) {
		ccx += m_bones[i].restPos[0];
		ccz += m_bones[i].restPos[2];
	}
	ccx /= (float)nBones;
	ccz /= (float)nBones;

	// Bones already claimed by the structural / name mapping stay untouched
	std::vector<char> usedBone(nBones, 0);
	for (int j = 0; j < 25; ++j)
		if (m_mapping[j] >= 0)
			usedBone[m_mapping[j]] = 1;

	const float maxDist = 0.30f;   // as a fraction of the body height
	const float maxDistSq = maxDist * maxDist;
	while (true) {
		float best = FLT_MAX;
		int bestJ = -1, bestB = -1;
		for (size_t j = 0; j < gj.size(); ++j) {
			for (int b = 0; b < nBones; ++b) {
				if (usedBone[b])
					continue;
				float nx = (m_bones[b].restPos[0] - ccx) / cRef;
				float ny = (m_bones[b].restPos[1] - cMinY) / cRef;
				float nz = (m_bones[b].restPos[2] - ccz) / cRef;
				float dx = nx - gj[j].x, dy = ny - gj[j].y, dz = nz - gj[j].z;
				float d = dx * dx + dy * dy + dz * dz;
				if (d < best) { best = d; bestJ = (int)j; bestB = b; }
			}
		}
		if (bestJ < 0)
			break;
		if (best > maxDistSq)
			break;
		m_mapping[gj[bestJ].joint] = bestB;
		usedBone[bestB] = 1;
		gj.erase(gj.begin() + bestJ);
	}
}

void KRetargeter::clearMapping() {
	memset(m_mapping, -1, sizeof(m_mapping));
}

void KRetargeter::setMapping(int kinectJoint, int boneIndex) {
	if (kinectJoint < 0 || kinectJoint >= 25)
		return;
	// The stored index is later used to index the bone arrays directly (the
	// auto-mappers fill preMapped[]/usedBone[] with it), so an out-of-range
	// value would corrupt memory rather than simply fail to map. Values come
	// from the UI, from the auto-mappers, and from a .kas file, where the
	// character may have been swapped or the file edited since it was saved.
	if (boneIndex >= (int)m_bones.size())
		boneIndex = -1;
	// Anything below -1 is garbage too (only -1 is the "unmapped" sentinel).
	if (boneIndex < -1)
		boneIndex = -1;
	// A bone can only be used once: clear it from other joints first
	if (boneIndex >= 0) {
		for (int j = 0; j < 25; ++j)
			if (m_mapping[j] == boneIndex && j != kinectJoint)
				m_mapping[j] = -1;
	}
	m_mapping[kinectJoint] = boneIndex;
}

int KRetargeter::getMapping(int kinectJoint) const {
	if (kinectJoint < 0 || kinectJoint >= 25)
		return -1;
	return m_mapping[kinectJoint];
}

int KRetargeter::mappingJointForBone(int boneIndex) const {
	for (int j = 0; j < 25; ++j)
		if (m_mapping[j] == boneIndex)
			return j;
	return -1;
}

// ----------------------------------------------------------------------------
// Pose evaluation
// ----------------------------------------------------------------------------

void KRetargeter::computeScale(const KSampleBody &body) {
	int bBase = m_mapping[JointType_SpineBase];
	int bMid  = m_mapping[JointType_SpineMid];
	int bHead = m_mapping[JointType_Head];

	float actorLen = 0.0f, charLen = 0.0f;

	if (bBase >= 0 && bMid >= 0) {
		const KSampleJoint &jb = body.joints[JointType_SpineBase];
		const KSampleJoint &jm = body.joints[JointType_SpineMid];
		if (jb.trackingState != TrackingState_NotTracked && jm.trackingState != TrackingState_NotTracked) {
			float dx = jm.position[0] - jb.position[0];
			float dy = jm.position[1] - jb.position[1];
			float dz = jm.position[2] - jb.position[2];
			actorLen = sqrtf(dx * dx + dy * dy + dz * dz);
			dx = m_bones[bMid].restPos[0] - m_bones[bBase].restPos[0];
			dy = m_bones[bMid].restPos[1] - m_bones[bBase].restPos[1];
			dz = m_bones[bMid].restPos[2] - m_bones[bBase].restPos[2];
			charLen = sqrtf(dx * dx + dy * dy + dz * dz);
		}
	}
	if (actorLen < 1e-3f && bBase >= 0 && bHead >= 0) {
		const KSampleJoint &jb = body.joints[JointType_SpineBase];
		const KSampleJoint &jh = body.joints[JointType_Head];
		if (jb.trackingState != TrackingState_NotTracked && jh.trackingState != TrackingState_NotTracked) {
			float dx = jh.position[0] - jb.position[0];
			float dy = jh.position[1] - jb.position[1];
			float dz = jh.position[2] - jb.position[2];
			actorLen = sqrtf(dx * dx + dy * dy + dz * dz);
			dx = m_bones[bHead].restPos[0] - m_bones[bBase].restPos[0];
			dy = m_bones[bHead].restPos[1] - m_bones[bBase].restPos[1];
			dz = m_bones[bHead].restPos[2] - m_bones[bBase].restPos[2];
			charLen = sqrtf(dx * dx + dy * dy + dz * dz);
		}
	}

	if (actorLen > 1e-3f && charLen > 1e-3f)
		m_scale = charLen / actorLen;
	else
		m_scale = 1.0f;

	// Guard against tracking glitches (a corrupt spine length would make the
	// root translation explode). The FK retarget ignores m_scale for the bone
	// lengths, it only scales the root translation.
	if (!(m_scale > 0.05f) || m_scale > 20.0f)
		m_scale = 1.0f;

	m_rootBone = bBase;
	m_scaleReady = true;
}

void KRetargeter::computePoseBody(const KSampleBody &body, bool withSkin) {
	int nBones = (int)m_bones.size();
	if ((int)m_pose.bonePos.size() != 3 * nBones)
		m_pose.bonePos.assign(3 * nBones, 0.0f);
	if ((int)m_pose.boneMat.size() != 16 * nBones)
		m_pose.boneMat.assign(16 * nBones, 0.0f);
	if ((int)m_pose.boneWorldMat.size() != 16 * nBones)
		m_pose.boneWorldMat.assign(16 * nBones, 0.0f);

	if (!m_scaleReady)
		computeScale(body);

	// Anchor = kinect spine base
	float base[3];
	const KSampleJoint &sjBase = body.joints[JointType_SpineBase];
	if (sjBase.trackingState != TrackingState_NotTracked) {
		base[0] = sjBase.position[0];
		base[1] = sjBase.position[1];
		base[2] = sjBase.position[2];
		m_anchor[0] = base[0]; m_anchor[1] = base[1]; m_anchor[2] = base[2];
	} else {
		base[0] = m_anchor[0]; base[1] = m_anchor[1]; base[2] = m_anchor[2];
	}

	// Motion anchor: the actor's root position when the take starts. The whole
	// motion is then added RELATIVE to the character's own root so the target
	// FBX never gets teleported to the mocap camera coordinates.
	if (!m_motionAnchorSet) {
		m_motionAnchor[0] = base[0];
		m_motionAnchor[1] = base[1];
		m_motionAnchor[2] = base[2];
		m_motionAnchorSet = true;
	}

	// ------------------------------------------------------------------
	// FK retargeting (model-space, space-consistent).
	//
	// The actor is captured in Kinect CAMERA space (Y-up, meters) while the
	// character lives in its own MODEL space (standing along its rest "up",
	// scaled by m_scale). Every actor direction is first mapped into model
	// space with ONE fixed rotation R_atm (camera up -> model up), so no
	// cross-space vector is ever compared directly and the character root
	// stays ~identity at rest. The mapped bones are then ORIENTED along the
	// actor's corresponding bone direction and positioned by forward
	// kinematics from the root using the character's OWN rest-pose bone
	// lengths. The motion (the joint angles) transfers faithfully while the
	// character keeps its own morphology - long torso / short legs / tiny
	// hands all stay as they are in the source FBX - and the whole take is
	// frame-consistent with the model's rest orientation.
	// ------------------------------------------------------------------

	// Keep the retarget strictly vertical (perpendicular to the floor).
	// The previous heuristic averaged the root's children or used the
	// spine's Y — on rigs whose rest spine leans 10-25° forward it
	// tilted R_atm by the same amount and pitched the whole mesh.
	float modelUp[3] = { 0.0f, 1.0f, 0.0f };

	// Fixed Kinect camera -> model-space mapping: R_atm sends the camera "up"
	// (+Y) onto the character's model "up", so an actor standing straight
	// maps to the character standing straight in its own frame.
	float yAxis[3] = { 0.0f, 1.0f, 0.0f };
	float R_atm[3][3];
	aimRotation(R_atm, yAxis, modelUp);

	// Character root reference (the bone mapped to SpineBase).
	float charRoot[3];
	bool rootOk = (m_rootBone >= 0 && m_rootBone < nBones);
	if (rootOk) {
		// Root stays at the character's own rest spot and only follows the
		// actor's translation relative to the saved motion anchor, mapped
		// into model space (R_atm), so the walking direction stays
		// consistent with the aimed limbs.
		// The translation is intentionally 1:1 (faithful root motion):
		// scaling it by the spine ratio (m_scale, measured 0.68 here)
		// shrinks walks and jumps and makes locomotion unfaithful to the
		// performed motion. m_scale is kept for unit comparisons elsewhere.
		float d[3] = {
			(base[0] - m_motionAnchor[0]),
			(base[1] - m_motionAnchor[1]),
			(base[2] - m_motionAnchor[2]) };
		charRoot[0] = m_bones[m_rootBone].restPos[0] + R_atm[0][0] * d[0] + R_atm[0][1] * d[1] + R_atm[0][2] * d[2];
		charRoot[1] = m_bones[m_rootBone].restPos[1] + R_atm[1][0] * d[0] + R_atm[1][1] * d[1] + R_atm[1][2] * d[2];
		charRoot[2] = m_bones[m_rootBone].restPos[2] + R_atm[2][0] * d[0] + R_atm[2][1] * d[1] + R_atm[2][2] * d[2];
	} else {
		// No SpineBase mapping: fall back to the old absolute placement
		charRoot[0] = base[0];
		charRoot[1] = base[1];
		charRoot[2] = base[2];
	}

	// Actor joint positions (camera space)
	float apos[JointType_Count][3];
	for (int j = 0; j < JointType_Count; ++j) {
		apos[j][0] = body.joints[j].position[0];
		apos[j][1] = body.joints[j].position[1];
		apos[j][2] = body.joints[j].position[2];
	}

	// Does each arm have a clavicle? When the bone mapped to the Kinect
	// Shoulder joint is much shorter than the one mapped to the Elbow, it is a
	// real clavicle and the whole arm chain must be driven by the actor
	// segments that END at each joint (clavicle = SpineShoulder->Shoulder,
	// upper arm = Shoulder->Elbow, forearm = Elbow->Wrist). Without this the
	// clavicle would be spun down along the humerus and the shoulder would
	// deform while the elbow / wrist angles shift one notch down the arm.
	auto boneForJoint = [&](int jointId) -> int {
		for (int k = 0; k < nBones; ++k)
			if (mappingJointForBone(k) == jointId)
				return k;
		return -1;
	};
	bool clavicleL = false, clavicleR = false;
	{
		int shL = boneForJoint(JointType_ShoulderLeft), elL = boneForJoint(JointType_ElbowLeft);
		if (shL >= 0) {
			if (elL >= 0 && m_bones[shL].restLen > 1e-4f)
				clavicleL = m_bones[shL].restLen < 0.75f * m_bones[elL].restLen;
			else if (m_scale > 1e-3f) {
				float hx = apos[JointType_ElbowLeft][0] - apos[JointType_ShoulderLeft][0];
				float hy = apos[JointType_ElbowLeft][1] - apos[JointType_ShoulderLeft][1];
				float hz = apos[JointType_ElbowLeft][2] - apos[JointType_ShoulderLeft][2];
				float hlen = sqrtf(hx * hx + hy * hy + hz * hz);
				if (hlen > 1e-3f)
					clavicleL = (m_bones[shL].restLen / m_scale) < 0.75f * hlen;
			}
		}
		int shR = boneForJoint(JointType_ShoulderRight), elR = boneForJoint(JointType_ElbowRight);
		if (shR >= 0) {
			if (elR >= 0 && m_bones[shR].restLen > 1e-4f)
				clavicleR = m_bones[shR].restLen < 0.75f * m_bones[elR].restLen;
			else if (m_scale > 1e-3f) {
				float hx = apos[JointType_ElbowRight][0] - apos[JointType_ShoulderRight][0];
				float hy = apos[JointType_ElbowRight][1] - apos[JointType_ShoulderRight][1];
				float hz = apos[JointType_ElbowRight][2] - apos[JointType_ShoulderRight][2];
				float hlen = sqrtf(hx * hx + hy * hy + hz * hz);
				if (hlen > 1e-3f)
					clavicleR = (m_bones[shR].restLen / m_scale) < 0.75f * hlen;
			}
		}
	}

	// FK state: world position and world rotation (row-major 3x3) per bone.
	// Parents are always processed before children (the FBX skeleton is read
	// in depth-first pre-order, so parent indices are always smaller).
	std::vector<float> G(3 * nBones, 0.0f);
	std::vector<float> R(9 * nBones, 0.0f);

	// Actor torso "up" (SpineBase -> SpineMid), mapped into model space. Used
	// to keep the clavicles lateral: a shoulder should stay roughly horizontal,
	// in the plane perpendicular to the torso, whatever the Kinect joint noise
	// says. Defaults to the model up so an untracked torso leaves the root at
	// its rest orientation.
	float torsoUp[3] = { modelUp[0], modelUp[1], modelUp[2] };
	if (body.joints[JointType_SpineBase].trackingState != TrackingState_NotTracked &&
		body.joints[JointType_SpineMid].trackingState != TrackingState_NotTracked)
	{
		float sx = apos[JointType_SpineMid][0] - apos[JointType_SpineBase][0];
		float sy = apos[JointType_SpineMid][1] - apos[JointType_SpineBase][1];
		float sz = apos[JointType_SpineMid][2] - apos[JointType_SpineBase][2];
		float l = sqrtf(sx * sx + sy * sy + sz * sz);
		if (l > 1e-4f) {
			sx /= l; sy /= l; sz /= l;
			torsoUp[0] = R_atm[0][0] * sx + R_atm[0][1] * sy + R_atm[0][2] * sz;
			torsoUp[1] = R_atm[1][0] * sx + R_atm[1][1] * sy + R_atm[1][2] * sz;
			torsoUp[2] = R_atm[2][0] * sx + R_atm[2][1] * sy + R_atm[2][2] * sz;
		}
	}

	for (int i = 0; i < nBones; ++i) {
		int j = mappingJointForBone(i);
		int p = m_bones[i].parent;
		bool isRoot = (j == JointType_SpineBase);

		float R3[3][3];
		memset(R3, 0, sizeof(R3));
		R3[0][0] = R3[1][1] = R3[2][2] = 1.0f;

		if (isRoot) {
			// Root: keep the character's standing axis aligned with the actor's
			// torso lean (identity at rest, since torsoUp is already model-space).
			aimRotation(R3, modelUp, torsoUp);
		} else {
			// Aim a mapped bone along the actor's bone direction when possible.
			// Legs and clavicle-less arms follow the segment that STARTS at the
			// mapped joint (thigh = Hip->Knee, upper arm = Shoulder->Elbow, ...).
			// Arms with a clavicle follow the segment that ENDS at the joint
			// instead (clavicle = SpineShoulder->Shoulder, upper arm =
			// Shoulder->Elbow, forearm = Elbow->Wrist) - one notch up the chain.
			bool ruleB = false;
			switch (j) {
				case JointType_ShoulderLeft:  case JointType_ElbowLeft:
				case JointType_WristLeft:     case JointType_HandLeft:
					ruleB = clavicleL; break;
				case JointType_ShoulderRight: case JointType_ElbowRight:
				case JointType_WristRight:    case JointType_HandRight:
					ruleB = clavicleR; break;
				default:
					break;
			}
			int q = -1;
			if (j >= 0)
				q = ruleB ? g_kinParent[j] : g_kinChild[j];
			if (j >= 0 && q >= 0 &&
				body.joints[j].trackingState != TrackingState_NotTracked &&
				body.joints[q].trackingState != TrackingState_NotTracked)
			{
			float dx, dy, dz;
			if (ruleB) {
				// parent -> joint
				dx = apos[j][0] - apos[q][0];
				dy = apos[j][1] - apos[q][1];
				dz = apos[j][2] - apos[q][2];
			} else {
				// joint -> child
				dx = apos[q][0] - apos[j][0];
				dy = apos[q][1] - apos[j][1];
				dz = apos[q][2] - apos[j][2];
			}

			// Map the actor segment from Kinect camera space into the character's
			// model space with the same fixed rotation used for the root, so all
			// aim vectors (and the torso projection below) live in one frame.
			{
				float mx = R_atm[0][0] * dx + R_atm[0][1] * dy + R_atm[0][2] * dz;
				float my = R_atm[1][0] * dx + R_atm[1][1] * dy + R_atm[1][2] * dz;
				float mz = R_atm[2][0] * dx + R_atm[2][1] * dy + R_atm[2][2] * dz;
				dx = mx; dy = my; dz = mz;
			}

			// The clavicle (the Rule B bone mapped to the Shoulder joint) must
			// stay lateral - perpendicular to the torso - and horizontal-ish. The
			// raw SpineShoulder->Shoulder vector is noisy and often tilts up, so
			// project it onto the torso plane. If Kinect gives a vertical /
			// degenerate direction, fall back to the character's own rest
			// shoulder axis, flattened the same way (a normal, horizontal
			// shoulder).
			bool isClavicle = ruleB &&
				(j == JointType_ShoulderLeft || j == JointType_ShoulderRight);
			if (isClavicle) {
				float dt = dx * torsoUp[0] + dy * torsoUp[1] + dz * torsoUp[2];
				dx -= dt * torsoUp[0];
				dy -= dt * torsoUp[1];
				dz -= dt * torsoUp[2];
				float len = sqrtf(dx * dx + dy * dy + dz * dz);
				if (len < 1e-4f) {
					dx = m_bones[i].restAxis[0];
					dy = m_bones[i].restAxis[1];
					dz = m_bones[i].restAxis[2];
					float dt2 = dx * torsoUp[0] + dy * torsoUp[1] + dz * torsoUp[2];
					dx -= dt2 * torsoUp[0];
					dy -= dt2 * torsoUp[1];
					dz -= dt2 * torsoUp[2];
				}
			}

			float len = sqrtf(dx * dx + dy * dy + dz * dz);
			if (len > 1e-4f) {
				float dir[3] = { dx / len, dy / len, dz / len };
				if (m_bones[i].restLen > 1e-4f) {
					aimRotation(R3, m_bones[i].restAxis, dir);
				} else {
					// Zero-length bone (some rigs use offset-only clavicles):
					// pick a stable axis perpendicular to both the target
					// direction and the torso so the aim is still coherent.
					float ax[3] = {
						torsoUp[1] * dir[2] - torsoUp[2] * dir[1],
						torsoUp[2] * dir[0] - torsoUp[0] * dir[2],
						torsoUp[0] * dir[1] - torsoUp[1] * dir[0],
					};
					float al = sqrtf(ax[0] * ax[0] + ax[1] * ax[1] + ax[2] * ax[2]);
					if (al > 1e-4f) {
						ax[0] /= al; ax[1] /= al; ax[2] /= al;
						aimRotation(R3, ax, dir);
					}
				}
			}
		} else if (p >= 0) {
			// Leaf / untracked / unmapped bone: keep the parent orientation
			memcpy(R3, &R[9 * p], sizeof(R3));
		}
	}

		memcpy(&R[9 * i], R3, sizeof(R3));

		// Position: forward kinematics from the parent using the character's
		// own rest bone lengths (rigid offset rotated by the parent world rot)
		if (isRoot) {
			G[3 * i + 0] = charRoot[0];
			G[3 * i + 1] = charRoot[1];
			G[3 * i + 2] = charRoot[2];
		} else if (p >= 0) {
			const float *pr = &R[9 * p];
			float rv[3] = {
				m_bones[i].restPos[0] - m_bones[p].restPos[0],
				m_bones[i].restPos[1] - m_bones[p].restPos[1],
				m_bones[i].restPos[2] - m_bones[p].restPos[2],
			};
			G[3 * i + 0] = G[3 * p + 0] + pr[0] * rv[0] + pr[1] * rv[1] + pr[2] * rv[2];
			G[3 * i + 1] = G[3 * p + 1] + pr[3] * rv[0] + pr[4] * rv[1] + pr[5] * rv[2];
			G[3 * i + 2] = G[3 * p + 2] + pr[6] * rv[0] + pr[7] * rv[1] + pr[8] * rv[2];
		} else {
			G[3 * i + 0] = m_bones[i].restPos[0];
			G[3 * i + 1] = m_bones[i].restPos[1];
			G[3 * i + 2] = m_bones[i].restPos[2];
		}
	}

	// Copy the FK results and build the world / skinning matrices
	finalizePoseFK(nBones > 0 ? &G[0] : NULL, nBones > 0 ? &R[0] : NULL, nBones, withSkin);
}

// Rebuilds m_pose (world / skinning matrices + skinned mesh) from FK results:
// world positions G and world rotations R9 (row-major 3x3). Shared by the
// retarget (computePoseBody) and the node graph (buildFinalPose).
void KRetargeter::finalizePoseFK(const float *G, const float *R9, int nBones, bool withSkin) {
	if (nBones <= 0 || !G || !R9) { m_pose.valid = false; return; }
	m_pose.bonePos.assign(G, G + 3 * nBones);
	for (int i = 0; i < nBones; ++i) {
		const float *Rr = &R9[9 * i];

		// World matrix W = T(pos) * Rr  (row-major, translation in column 3)
		float W[16];
		memset(W, 0, sizeof(W));
		W[0] = Rr[0]; W[1] = Rr[1]; W[2] = Rr[2]; W[3] = m_pose.bonePos[3 * i + 0];
		W[4] = Rr[3]; W[5] = Rr[4]; W[6] = Rr[5]; W[7] = m_pose.bonePos[3 * i + 1];
		W[8] = Rr[6]; W[9] = Rr[7]; W[10] = Rr[8]; W[11] = m_pose.bonePos[3 * i + 2];
		W[15] = 1.0f;
		memcpy(&m_pose.boneWorldMat[16 * i], W, sizeof(W));

		if (!withSkin)
			continue;
		// Bind inverse B^-1 = T(-restPos)  (row-major, translation in column 3)
		float Binv[16];
		memset(Binv, 0, sizeof(Binv));
		Binv[0] = Binv[5] = Binv[10] = Binv[15] = 1.0f;
		Binv[3] = -m_bones[i].restPos[0];
		Binv[7] = -m_bones[i].restPos[1];
		Binv[11] = -m_bones[i].restPos[2];

		mulMat4(W, Binv, &m_pose.boneMat[16 * i]);
	}

	if (!withSkin) { m_pose.valid = true; return; }  // World data valid; skin rebuilt by the final pose
	skinMesh();
	m_pose.valid = true;
}

void KRetargeter::skinMesh() {
	int nVerts = (int)(m_mesh.restPositions.size() / 3);
	m_pose.skinnedPos.assign(3 * nVerts, 0.0f);
	for (int v = 0; v < nVerts; ++v) {
		const std::vector<std::pair<int, float> > &wl = m_mesh.weights[v];
		if (wl.empty()) {
			// Unskinned vertex: keep it at its rest position
			m_pose.skinnedPos[3 * v + 0] = m_mesh.restPositions[3 * v + 0];
			m_pose.skinnedPos[3 * v + 1] = m_mesh.restPositions[3 * v + 1];
			m_pose.skinnedPos[3 * v + 2] = m_mesh.restPositions[3 * v + 2];
			continue;
		}
		float x = m_mesh.restPositions[3 * v + 0];
		float y = m_mesh.restPositions[3 * v + 1];
		float z = m_mesh.restPositions[3 * v + 2];
		float ox = 0, oy = 0, oz = 0;
		for (size_t k = 0; k < wl.size(); ++k) {
			const float *S = &m_pose.boneMat[16 * wl[k].first];
			float w = wl[k].second;
			// Row-major matrix application: out = S * v
			ox += w * (S[0] * x + S[1] * y + S[2] * z + S[3]);
			oy += w * (S[4] * x + S[5] * y + S[6] * z + S[7]);
			oz += w * (S[8] * x + S[9] * y + S[10] * z + S[11]);
		}
		m_pose.skinnedPos[3 * v + 0] = ox;
		m_pose.skinnedPos[3 * v + 1] = oy;
		m_pose.skinnedPos[3 * v + 2] = oz;
	}
}

void KRetargeter::poseRest() {
	int nBones = (int)m_bones.size();
	m_pose.bonePos.assign(3 * nBones, 0.0f);
	m_pose.boneMat.assign(16 * nBones, 0.0f);
	m_pose.boneWorldMat.assign(16 * nBones, 0.0f);
	for (int i = 0; i < nBones; ++i) {
		m_pose.bonePos[3 * i + 0] = m_bones[i].restPos[0];
		m_pose.bonePos[3 * i + 1] = m_bones[i].restPos[1];
		m_pose.bonePos[3 * i + 2] = m_bones[i].restPos[2];
		m_pose.boneMat[16 * i + 0] = 1.0f;
		m_pose.boneMat[16 * i + 5] = 1.0f;
		m_pose.boneMat[16 * i + 10] = 1.0f;
		m_pose.boneMat[16 * i + 15] = 1.0f;
		m_pose.boneWorldMat[16 * i + 0] = 1.0f;
		m_pose.boneWorldMat[16 * i + 5] = 1.0f;
		m_pose.boneWorldMat[16 * i + 10] = 1.0f;
		m_pose.boneWorldMat[16 * i + 15] = 1.0f;
	}
	m_pose.skinnedPos = m_mesh.restPositions;
	m_pose.valid = !m_bones.empty();
}

bool KRetargeter::poseFromFrame(const KSampleFrame &frame, int bodyIndex) {
	if (m_bones.empty() || frame.bodies.empty())
		return false;
	if (bodyIndex < 0 || bodyIndex >= (int)frame.bodies.size())
		bodyIndex = 0;
	computePoseBody(frame.bodies[bodyIndex]);
	return true;
}

// ----------------------------------------------------------------------------
// Node Motion Editor evaluation (non-destructive post-process graph)
// ----------------------------------------------------------------------------

// Base (unprocessed) pose of one body, converted to graph currency: local
// quats per bone + root world position. Non-root translations stay implicit
// (rest offsets, rebuilt by forward kinematics in buildFinalPose).
bool KRetargeter::basePoseForBody(const KSampleBody &body, MotionNodePose &out) {
	int nBones = (int)m_bones.size();
	out.valid = false;
	if (nBones <= 0)
		return false;
	computePoseBody(body, false);   // World data only; the final pose re-skins
	out.localQ.assign(4 * nBones, 0.0f);
	out.rootPos.assign(3, 0.0f);
	out.localT.assign(3 * nBones, 0.0f);
	int root = (m_rootBone >= 0 && m_rootBone < nBones) ? m_rootBone : 0;
	for (int i = 0; i < nBones; ++i) {
		const float *W = &m_pose.boneWorldMat[16 * i];
		float Mw[3][3] = { { W[0], W[1], W[2] }, { W[4], W[5], W[6] }, { W[8], W[9], W[10] } };
		float Ml[3][3];
		int p = m_bones[i].parent;
		if (i == root || p < 0 || p >= nBones) {
			memcpy(Ml, Mw, sizeof(Ml));
		} else {
			// Local = parent WORLD^T * world. The parent world rotation
			// comes straight from m_pose: it is NOT the local rotation
			// computed just above for the parent (using that local here
			// corrupts every bone below the first hierarchy level).
			const float *PW = &m_pose.boneWorldMat[16 * p];
			for (int r = 0; r < 3; ++r)
				for (int c = 0; c < 3; ++c)
					Ml[r][c] = PW[0 * 4 + r] * Mw[0][c]
						+ PW[1 * 4 + r] * Mw[1][c]
						+ PW[2 * 4 + r] * Mw[2][c];
		}
		float q[4];
		motionQuatFromMat3(Ml, q);
		out.localQ[4 * i + 0] = q[0]; out.localQ[4 * i + 1] = q[1];
		out.localQ[4 * i + 2] = q[2]; out.localQ[4 * i + 3] = q[3];
		if (i == root || p < 0 || p >= nBones) {
			out.localT[3 * i + 0] = 0.0f; out.localT[3 * i + 1] = 0.0f; out.localT[3 * i + 2] = 0.0f;
		} else {
			out.localT[3 * i + 0] = m_bones[i].restPos[0] - m_bones[p].restPos[0];
			out.localT[3 * i + 1] = m_bones[i].restPos[1] - m_bones[p].restPos[1];
			out.localT[3 * i + 2] = m_bones[i].restPos[2] - m_bones[p].restPos[2];
		}
	}
	out.rootPos[0] = m_pose.bonePos[3 * root + 0];
	out.rootPos[1] = m_pose.bonePos[3 * root + 1];
	out.rootPos[2] = m_pose.bonePos[3 * root + 2];
	out.valid = true;
	return true;
}

// Same, for a take frame. Neighbor evaluations (Smooth window) must not
// disturb the motion anchor / scale state: the center frame owns the anchor
// trajectory exactly as before the graph existed.
bool KRetargeter::basePoseForTakeFrame(const KRecording &rec, int frameIdx, int bodyIdx, MotionNodePose &out) {
	out.valid = false;
	if (frameIdx < 0 || frameIdx >= (int)rec.frames.size())
		return false;
	const KSampleFrame &fr = rec.frames[frameIdx];
	if (fr.bodies.empty() || bodyIdx < 0 || bodyIdx >= (int)fr.bodies.size())
		return false;
	float svMA[3] = { m_motionAnchor[0], m_motionAnchor[1], m_motionAnchor[2] };
	bool svMASet = m_motionAnchorSet;
	float svAn[3] = { m_anchor[0], m_anchor[1], m_anchor[2] };
	bool ok = basePoseForBody(fr.bodies[bodyIdx], out);
	m_motionAnchor[0] = svMA[0]; m_motionAnchor[1] = svMA[1]; m_motionAnchor[2] = svMA[2];
	m_motionAnchorSet = svMASet;
	m_anchor[0] = svAn[0]; m_anchor[1] = svAn[1]; m_anchor[2] = svAn[2];
	return ok;
}

// Blends the center pose toward the window mean (per targeted bone).
void KRetargeter::smoothBlendPoses(const MotionNode &nd, const MotionNodePose &center,
		const std::vector<MotionNodePose> &win, MotionNodePose &out) {
	int nBones = (int)m_bones.size();
	out = center;
	if (!center.valid || nBones <= 0 || win.empty() || (int)center.localQ.size() < 4 * nBones) {
		out.valid = false;
		return;
	}
	float s = MotionNodeParamGet(nd, 0);
	if (s < 0.0f) s = 0.0f;
	if (s > 1.0f) s = 1.0f;
	if (s <= 0.0f)
		return;   // out == center
	int root = (m_rootBone >= 0 && m_rootBone < nBones) ? m_rootBone : 0;
	std::vector<float> qs(win.size() * 4);
	for (int i = 0; i < nBones; ++i) {
		if (!motionNodeAffectsBone(nd, i, nBones))
			continue;
		for (size_t w = 0; w < win.size(); ++w) {
			const MotionNodePose &e = (win[w].valid && (int)win[w].localQ.size() >= 4 * nBones)
				? win[w] : center;
			qs[4 * w + 0] = e.localQ[4 * i + 0]; qs[4 * w + 1] = e.localQ[4 * i + 1];
			qs[4 * w + 2] = e.localQ[4 * i + 2]; qs[4 * w + 3] = e.localQ[4 * i + 3];
		}
		float avg[4];
		motionQuatAvg(win.empty() ? NULL : &qs[0], NULL, (int)win.size(), avg);
		float res[4];
		motionQuatSlerp(&center.localQ[4 * i], avg, s, res);
		out.localQ[4 * i + 0] = res[0]; out.localQ[4 * i + 1] = res[1];
		out.localQ[4 * i + 2] = res[2]; out.localQ[4 * i + 3] = res[3];
	}
	// The root translation follows the same blend when targeted.
	if (motionNodeAffectsBone(nd, root, nBones) && (int)center.rootPos.size() >= 3) {
		double ap[3] = { 0.0, 0.0, 0.0 };
		int cnt = 0;
		for (size_t w = 0; w < win.size(); ++w) {
			const MotionNodePose &e = (win[w].valid && (int)win[w].rootPos.size() >= 3) ? win[w] : center;
			ap[0] += e.rootPos[0]; ap[1] += e.rootPos[1]; ap[2] += e.rootPos[2];
			++cnt;
		}
		if (cnt > 0) {
			float mean[3] = { (float)(ap[0] / cnt), (float)(ap[1] / cnt), (float)(ap[2] / cnt) };
			motionLerp3(&center.rootPos[0], mean, s, &out.rootPos[0]);
		}
	}
	// Translation deltas follow the same blend when targeted.
	if ((int)center.localT.size() >= 3 * nBones) {
		for (int i = 0; i < nBones; ++i) {
			if (!motionNodeAffectsBone(nd, i, nBones)) continue;
			if ((int)out.localT.size() < 3 * nBones) break;
			double at[3] = { 0.0, 0.0, 0.0 };
			int cnt = 0;
			for (size_t w = 0; w < win.size(); ++w) {
				const MotionNodePose &e = (win[w].valid && (int)win[w].localT.size() >= 3 * nBones) ? win[w] : center;
				at[0] += e.localT[3 * i + 0]; at[1] += e.localT[3 * i + 1]; at[2] += e.localT[3 * i + 2];
				++cnt;
			}
			if (cnt > 0) {
				float mean[3] = { (float)(at[0] / cnt), (float)(at[1] / cnt), (float)(at[2] / cnt) };
				motionLerp3(&center.localT[3 * i], mean, s, &out.localT[3 * i]);
			}
		}
	}
}

// Smooth over the take window [f-W, f+W]. Missing frames reuse the center
// sample so edges and gaps exert no pull.
void KRetargeter::applySmoothNodeTake(const MotionNode &nd, const KRecording &rec, int frameIdx, int bodyIdx,
		const MotionNodePose &center, MotionNodePose &out) {
	int W = (int)floorf(MotionNodeParamGet(nd, 1) + 0.5f);
	if (W < 0) W = 0;
	if (W > 30) W = 30;
	std::vector<MotionNodePose> win;
	win.reserve(2 * (size_t)W + 1);
	int nF = (int)rec.frames.size();
	for (int k = frameIdx - W; k <= frameIdx + W; ++k) {
		if (k == frameIdx || k < 0 || k >= nF) {
			win.push_back(center);
			continue;
		}
		MotionNodePose b;
		if (basePoseForTakeFrame(rec, k, bodyIdx, b) && b.valid)
			win.push_back(b);
		else
			win.push_back(center);
	}
	smoothBlendPoses(nd, center, win, out);
}

// Live variant: the window runs over the recent live history (causal, the
// current pose is the newest entry).
void KRetargeter::applySmoothNodeLive(const MotionNode &nd, MotionNodePose &out) {
	int W = (int)floorf(MotionNodeParamGet(nd, 1) + 0.5f);
	if (W < 0) W = 0;
	if (W > 30) W = 30;
	if (m_liveHistory.empty()) { out.valid = false; return; }
	const MotionNodePose &center = m_liveHistory.back();
	size_t first = 0;
	if ((int)m_liveHistory.size() > W + 1)
		first = m_liveHistory.size() - (size_t)(W + 1);
	std::vector<MotionNodePose> win;
	win.reserve(m_liveHistory.size() - first);
	for (size_t k = first; k < m_liveHistory.size(); ++k)
		win.push_back(m_liveHistory[k]);
	smoothBlendPoses(nd, center, win, out);
}

// Scales local motion: rotation angle x factor around its own axis
// (1 = identity, 0 = rest pose, 2 = doubled), root translation relative
// to its rest spot. Only targeted bones move.
void KRetargeter::applyMultiplyNode(const MotionNode &nd, const MotionNodePose &in, MotionNodePose &out) {
	int nBones = (int)m_bones.size();
	out = in;
	if (!in.valid || nBones <= 0 || (int)in.localQ.size() < 4 * nBones) {
		out.valid = false;
		return;
	}
	float k = MotionNodeParamGet(nd, 0);
	int root = (m_rootBone >= 0 && m_rootBone < nBones) ? m_rootBone : 0;
	for (int i = 0; i < nBones; ++i) {
		if (!motionNodeAffectsBone(nd, i, nBones))
			continue;
		float r[4];
		motionQuatScaleAngle(&in.localQ[4 * i], k, r);
		out.localQ[4 * i + 0] = r[0]; out.localQ[4 * i + 1] = r[1];
		out.localQ[4 * i + 2] = r[2]; out.localQ[4 * i + 3] = r[3];
	}
	if (motionNodeAffectsBone(nd, root, nBones) && (int)in.rootPos.size() >= 3 && (int)out.rootPos.size() >= 3) {
		const float *rp = m_bones[root].restPos;
		out.rootPos[0] = rp[0] + (in.rootPos[0] - rp[0]) * k;
		out.rootPos[1] = rp[1] + (in.rootPos[1] - rp[1]) * k;
		out.rootPos[2] = rp[2] + (in.rootPos[2] - rp[2]) * k;
	}
}

// Constant rotation offset (p0/p1/p2 = degrees around X/Y/Z, applied in
// parent space so a limb can be re-aimed for another morphology). The
// animation flows underneath untouched: same motion, different posture.
void KRetargeter::applyAddNode(const MotionNode &nd, const MotionNodePose &in, MotionNodePose &out) {
	int nBones = (int)m_bones.size();
	out = in;
	if (!in.valid || nBones <= 0 || (int)in.localQ.size() < 4 * nBones) {
		out.valid = false;
		return;
	}
	// Offset matrix Roff = Rz * Ry * Rx (same XYZ convention as the import
	// rotation), built from degrees.
	float rx = MotionNodeParamGet(nd, 0) * (float)M_PI / 180.0f;
	float ry = MotionNodeParamGet(nd, 1) * (float)M_PI / 180.0f;
	float rz = MotionNodeParamGet(nd, 2) * (float)M_PI / 180.0f;
	float cx = cosf(rx), sx = sinf(rx);
	float cy = cosf(ry), sy = sinf(ry);
	float cz = cosf(rz), sz = sinf(rz);
	float Rx[3][3] = { { 1, 0, 0 }, { 0, cx, -sx }, { 0, sx, cx } };
	float Ry[3][3] = { { cy, 0, sy }, { 0, 1, 0 }, { -sy, 0, cy } };
	float Rz[3][3] = { { cz, -sz, 0 }, { sz, cz, 0 }, { 0, 0, 1 } };
	float tmp[3][3], Roff[3][3];
	for (int r = 0; r < 3; ++r)
		for (int c = 0; c < 3; ++c)
			tmp[r][c] = Rz[r][0] * Ry[0][c] + Rz[r][1] * Ry[1][c] + Rz[r][2] * Ry[2][c];
	for (int r = 0; r < 3; ++r)
		for (int c = 0; c < 3; ++c)
			Roff[r][c] = tmp[r][0] * Rx[0][c] + tmp[r][1] * Rx[1][c] + tmp[r][2] * Rx[2][c];
	float qoff[4];
	motionQuatFromMat3(Roff, qoff);
	for (int i = 0; i < nBones; ++i) {
		if (!motionNodeAffectsBone(nd, i, nBones))
			continue;
		float r[4];
		motionQuatMul(qoff, &in.localQ[4 * i], r);
		out.localQ[4 * i + 0] = r[0]; out.localQ[4 * i + 1] = r[1];
		out.localQ[4 * i + 2] = r[2]; out.localQ[4 * i + 3] = r[3];
	}
	// Rotation only: translations (including the root) stay untouched.
}

// Offset: constant translation added to targeted bones (parent space, m).
// Flows through localT into forward kinematics; the animation underneath
// is untouched (non-destructive posture adjust).
void KRetargeter::applyOffsetNode(const MotionNode &nd, const MotionNodePose &in, MotionNodePose &out) {
	int nBones = (int)m_bones.size();
	out = in;
	if (!in.valid || nBones <= 0) { out.valid = false; return; }
	if ((int)out.localT.size() < 3 * nBones)
		out.localT.assign((size_t)(3 * nBones), 0.0f);
	float dx = MotionNodeParamGet(nd, 0);
	float dy = MotionNodeParamGet(nd, 1);
	float dz = MotionNodeParamGet(nd, 2);
	if (dx == 0.0f && dy == 0.0f && dz == 0.0f) return;
	for (int i = 0; i < nBones; ++i) {
		if (!motionNodeAffectsBone(nd, i, nBones)) continue;
		out.localT[3 * i + 0] += dx;
		out.localT[3 * i + 1] += dy;
		out.localT[3 * i + 2] += dz;
	}
}

// ----------------------------------------------------------------------------
// Foot Lock node: pins feet to the ground on contact (two-bone IK legs).
// Contact = ankle low + slow (debounced over a few frames); the pin blends
// in/out over a few frames so engages and releases never pop. Everything is
// stateless per evaluated frame (scrub-safe): the contact walk only reads
// neighboring frames, never cached state.
// ----------------------------------------------------------------------------

bool KRetargeter::legChain(int side, int &thigh, int &knee, int &ankle, int &toe) const {
	int jHip = (side == 0) ? JointType_HipLeft : JointType_HipRight;
	int jKnee = (side == 0) ? JointType_KneeLeft : JointType_KneeRight;
	int jAnkle = (side == 0) ? JointType_AnkleLeft : JointType_AnkleRight;
	int jFoot = (side == 0) ? JointType_FootLeft : JointType_FootRight;
	thigh = knee = ankle = toe = -1;
	for (int k = 0; k < (int)m_bones.size(); ++k) {
		int j = mappingJointForBone(k);
		if (j == jHip) thigh = k;
		else if (j == jKnee) knee = k;
		else if (j == jAnkle) ankle = k;
		else if (j == jFoot) toe = k;
	}
	return thigh >= 0 && knee >= 0 && ankle >= 0;
}

// A leg is processed when the selection is empty (both legs) or holds any
// bone of that leg chain.
static bool footLegWanted(const MotionNode &nd, int thigh, int knee, int ankle, int toe, int nBones) {
	if (nd.bones.empty())
		return true;
	int legs[4] = { thigh, knee, ankle, toe };
	for (int i = 0; i < 4; ++i)
		if (legs[i] >= 0 && motionNodeAffectsBone(nd, legs[i], nBones))
			return true;
	return false;
}

// Analytic two-bone IK: places the knee between hip H and ankle target A
// given segment lengths, keeping the natural bend direction (pole).
// Outputs world rotation matrices for thigh and shin.
bool KRetargeter::solveFootLegIK(const float H[3], const float A[3], float L1, float L2,
		const float restAxisT[3], const float restAxisS[3], const float pole[3],
		float thighW[3][3], float shinW[3][3]) {
	float dv[3] = { A[0] - H[0], A[1] - H[1], A[2] - H[2] };
	float d = sqrtf(dv[0] * dv[0] + dv[1] * dv[1] + dv[2] * dv[2]);
	if (!(d > 1e-6f) || !(L1 > 1e-4f) || !(L2 > 1e-4f))
		return false;
	float axis[3] = { dv[0] / d, dv[1] / d, dv[2] / d };
	// Clamp reach (no stretching in v1: an overextended leg points at the
	// foot and simply cannot reach it).
	float dc = d;
	float maxR = L1 + L2 - 1e-4f, minR = fabsf(L1 - L2) + 1e-4f;
	if (dc > maxR) dc = maxR;
	if (dc < minR) dc = minR;
	float cosH = (L1 * L1 + dc * dc - L2 * L2) / (2.0f * L1 * dc);
	if (cosH > 1.0f) cosH = 1.0f;
	if (cosH < -1.0f) cosH = -1.0f;
	float angH = acosf(cosH);
	// Bend normal from the pole, with axis-aligned fallbacks.
	float bn[3] = {
		axis[1] * pole[2] - axis[2] * pole[1],
		axis[2] * pole[0] - axis[0] * pole[2],
		axis[0] * pole[1] - axis[1] * pole[0] };
	float bl = sqrtf(bn[0] * bn[0] + bn[1] * bn[1] + bn[2] * bn[2]);
	if (!(bl > 1e-4f)) {
		float fb[3] = { 0.0f, 0.0f, 1.0f };
		if (fabsf(axis[2]) > 0.9f) { fb[0] = 0.0f; fb[1] = 1.0f; fb[2] = 0.0f; }
		bn[0] = axis[1] * fb[2] - axis[2] * fb[1];
		bn[1] = axis[2] * fb[0] - axis[0] * fb[2];
		bn[2] = axis[0] * fb[1] - axis[1] * fb[0];
		bl = sqrtf(bn[0] * bn[0] + bn[1] * bn[1] + bn[2] * bn[2]);
		if (!(bl > 1e-4f)) return false;
	}
	bn[0] /= bl; bn[1] /= bl; bn[2] /= bl;
	float perp[3] = {
		bn[1] * axis[2] - bn[2] * axis[1],
		bn[2] * axis[0] - bn[0] * axis[2],
		bn[0] * axis[1] - bn[1] * axis[0] };
	float ch = cosf(angH), sh = sinf(angH);
	float thighDir[3] = {
		axis[0] * ch + perp[0] * sh,
		axis[1] * ch + perp[1] * sh,
		axis[2] * ch + perp[2] * sh };
	float knee[3] = { H[0] + thighDir[0] * L1, H[1] + thighDir[1] * L1, H[2] + thighDir[2] * L1 };
	float sd[3] = { A[0] - knee[0], A[1] - knee[1], A[2] - knee[2] };
	float sl = sqrtf(sd[0] * sd[0] + sd[1] * sd[1] + sd[2] * sd[2]);
	float shinDir[3];
	if (sl > 1e-6f) { shinDir[0] = sd[0] / sl; shinDir[1] = sd[1] / sl; shinDir[2] = sd[2] / sl; }
	else { shinDir[0] = axis[0]; shinDir[1] = axis[1]; shinDir[2] = axis[2]; }
	aimRotation(thighW, restAxisT, thighDir);
	aimRotation(shinW, restAxisS, shinDir);
	return true;
}

// Shared solver core: contact walk over the ankle window, pin + IK
// write-back into io (starts as a copy of the upstream center pose).
// G0/R90 hold the upstream FK worlds of the center frame.
void KRetargeter::solveFootLocks(const MotionNode &nd, const std::vector<FootLockSample> &win, int ci,
		const std::vector<float> &G0, const std::vector<float> &R90, MotionNodePose &io) {
	int nBones = (int)m_bones.size();
	if (nBones <= 0 || win.empty() || ci < 0 || ci >= (int)win.size())
		return;
	if ((int)G0.size() < 3 * nBones || (int)R90.size() < 9 * nBones)
		return;
	if ((int)io.localQ.size() < 4 * nBones)
		return;
	float Hh = MotionNodeParamGet(nd, 0);   // Contact height (m)
	float Vv = MotionNodeParamGet(nd, 1);   // Contact speed (m/s)
	int NF = (int)floorf(MotionNodeParamGet(nd, 2) + 0.5f);   // Debounce frames
	int BF = (int)floorf(MotionNodeParamGet(nd, 3) + 0.5f);   // Blend frames
	if (NF < 1) NF = 1; if (NF > 10) NF = 10;
	if (BF < 1) BF = 1; if (BF > 15) BF = 15;
	if (Hh < 0.005f) Hh = 0.005f;
	if (Vv < 0.05f) Vv = 0.05f;

	int W = (int)win.size();
	for (int side = 0; side < 2; ++side) {
		int thigh, knee, ankle, toe;
		if (!legChain(side, thigh, knee, ankle, toe))
			continue;
		if (!footLegWanted(nd, thigh, knee, ankle, toe, nBones))
			continue;
		// Segment lengths + hip spot (also feeds the reach gate below).
		float d1[3] = {
			m_bones[knee].restPos[0] - m_bones[thigh].restPos[0],
			m_bones[knee].restPos[1] - m_bones[thigh].restPos[1],
			m_bones[knee].restPos[2] - m_bones[thigh].restPos[2] };
		float d2[3] = {
			m_bones[ankle].restPos[0] - m_bones[knee].restPos[0],
			m_bones[ankle].restPos[1] - m_bones[knee].restPos[1],
			m_bones[ankle].restPos[2] - m_bones[knee].restPos[2] };
		float L1 = sqrtf(d1[0] * d1[0] + d1[1] * d1[1] + d1[2] * d1[2]);
		float L2 = sqrtf(d2[0] * d2[0] + d2[1] * d2[1] + d2[2] * d2[2]);
		if (!(L1 > 1e-4f) || !(L2 > 1e-4f)) continue;
		float Hpos[3] = { G0[3 * thigh + 0], G0[3 * thigh + 1], G0[3 * thigh + 2] };
		// Contact per frame: ankle low and slow.
		std::vector<char> contact((size_t)W, 0);
		for (int k = 0; k < W; ++k) {
			if (!win[k].ok) continue;
			const float *P = (side == 0) ? win[k].pl : win[k].pr;
			float v = 0.0f;
			if (k > 0 && win[k - 1].ok && win[k].dt > 1e-4f) {
				const float *Q = (side == 0) ? win[k - 1].pl : win[k - 1].pr;
				float dx = P[0] - Q[0], dy = P[1] - Q[1], dz = P[2] - Q[2];
				v = sqrtf(dx * dx + dy * dy + dz * dz) / win[k].dt;
			}
			if (P[1] < Hh && v < Vv) contact[(size_t)k] = 1;
		}
		// Engaged when the last NF frames all hold contact.
		bool engaged = true;
		for (int k = ci - NF + 1; k <= ci; ++k)
			if (k < 0 || !contact[(size_t)k]) { engaged = false; break; }
		float lockP[3] = { 0, 0, 0 };
		float lockQ[4] = { 0, 0, 0, 1 };
		float w = 0.0f;
		if (engaged) {
			int start = ci;
			while (start - 1 >= 0 && contact[(size_t)(start - 1)]) --start;
			const FootLockSample &s0 = win[(size_t)start];
			const float *P0 = (side == 0) ? s0.pl : s0.pr;
			const float *Q0 = (side == 0) ? s0.ql : s0.qr;
			lockP[0] = P0[0]; lockP[1] = P0[1]; lockP[2] = P0[2];
			lockQ[0] = Q0[0]; lockQ[1] = Q0[1]; lockQ[2] = Q0[2]; lockQ[3] = Q0[3];
			int runLen = ci - start + 1;
			w = (float)runLen / (float)BF;
			if (w > 1.0f) w = 1.0f;
		} else {
			// Recently released: decay the pin over BF frames.
			int lastC = -1;
			for (int k = ci - 1; k >= 0 && k > ci - BF; --k)
				if (contact[(size_t)k]) { lastC = k; break; }
			if (lastC < 0) continue;   // w = 0: keep the upstream pose
			int start = lastC;
			while (start - 1 >= 0 && contact[(size_t)(start - 1)]) --start;
			const FootLockSample &s0 = win[(size_t)start];
			const float *P0 = (side == 0) ? s0.pl : s0.pr;
			const float *Q0 = (side == 0) ? s0.ql : s0.qr;
			lockP[0] = P0[0]; lockP[1] = P0[1]; lockP[2] = P0[2];
			lockQ[0] = Q0[0]; lockQ[1] = Q0[1]; lockQ[2] = Q0[2]; lockQ[3] = Q0[3];
			w = 1.0f - (float)(ci - lastC) / (float)BF;
		if (w <= 0.0f) continue;
		// Overextended leg: no lock effect (keep the upstream pose), with a
		// short fade so the release never pops.
		{
			float dhp[3] = { lockP[0] - Hpos[0], lockP[1] - Hpos[1], lockP[2] - Hpos[2] };
			float dh = sqrtf(dhp[0] * dhp[0] + dhp[1] * dhp[1] + dhp[2] * dhp[2]);
			float reach = L1 + L2;
			const float fade = 0.05f;
			if (dh >= reach) continue;
			if (dh > reach - fade) {
				w *= (reach - dh) / fade;
				if (w <= 0.0f) continue;
			}
		}
		}
		if (w <= 0.0f) continue;
		// Upstream ankle (center FK) blended toward the pin.
		float Aup[3] = { G0[3 * ankle + 0], G0[3 * ankle + 1], G0[3 * ankle + 2] };
		float upM[3][3] = {
			{ R90[9 * ankle + 0], R90[9 * ankle + 1], R90[9 * ankle + 2] },
			{ R90[9 * ankle + 3], R90[9 * ankle + 4], R90[9 * ankle + 5] },
			{ R90[9 * ankle + 6], R90[9 * ankle + 7], R90[9 * ankle + 8] } };
		float AupQ[4];
		motionQuatFromMat3(upM, AupQ);
		float At[3];
		motionLerp3(Aup, lockP, w, At);
		float AQt[4];
		motionQuatSlerp(AupQ, lockQ, w, AQt);
		float ankleW[3][3];
		motionMat3FromQuat(AQt, ankleW);
		// (d1/d2/L1/L2/Hpos computed above for the reach gate.)
		// Pole: current knee direction minus its axial part (rest fallback).
		float ax0[3] = { At[0] - Hpos[0], At[1] - Hpos[1], At[2] - Hpos[2] };
		float al = sqrtf(ax0[0] * ax0[0] + ax0[1] * ax0[1] + ax0[2] * ax0[2]);
		if (!(al > 1e-6f)) continue;
		ax0[0] /= al; ax0[1] /= al; ax0[2] /= al;
		float pole[3];
		{
			float ku[3] = { G0[3 * knee + 0] - Hpos[0], G0[3 * knee + 1] - Hpos[1], G0[3 * knee + 2] - Hpos[2] };
			float dp = ku[0] * ax0[0] + ku[1] * ax0[1] + ku[2] * ax0[2];
			pole[0] = ku[0] - ax0[0] * dp; pole[1] = ku[1] - ax0[1] * dp; pole[2] = ku[2] - ax0[2] * dp;
			float pl = sqrtf(pole[0] * pole[0] + pole[1] * pole[1] + pole[2] * pole[2]);
			if (!(pl > 1e-3f)) {
				float rk[3] = { d1[0], d1[1], d1[2] };
				dp = rk[0] * ax0[0] + rk[1] * ax0[1] + rk[2] * ax0[2];
				pole[0] = rk[0] - ax0[0] * dp; pole[1] = rk[1] - ax0[1] * dp; pole[2] = rk[2] - ax0[2] * dp;
				pl = sqrtf(pole[0] * pole[0] + pole[1] * pole[1] + pole[2] * pole[2]);
				if (!(pl > 1e-3f)) continue;
			}
			pole[0] /= pl; pole[1] /= pl; pole[2] /= pl;
		}
		float thighW[3][3], shinW[3][3];
		if (!solveFootLegIK(Hpos, At, L1, L2,
				m_bones[thigh].restAxis, m_bones[knee].restAxis, pole, thighW, shinW))
			continue;
		// Write back locals: thigh vs its actual upstream parent, then chain.
		float pW[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
		int pth = m_bones[thigh].parent;
		if (pth >= 0 && pth < nBones) {
			pW[0][0] = R90[9 * pth + 0]; pW[0][1] = R90[9 * pth + 1]; pW[0][2] = R90[9 * pth + 2];
			pW[1][0] = R90[9 * pth + 3]; pW[1][1] = R90[9 * pth + 4]; pW[1][2] = R90[9 * pth + 5];
			pW[2][0] = R90[9 * pth + 6]; pW[2][1] = R90[9 * pth + 7]; pW[2][2] = R90[9 * pth + 8];
		}
		float lt[3][3], ls[3][3], la[3][3];
		for (int r = 0; r < 3; ++r)
			for (int c = 0; c < 3; ++c) {
				lt[r][c] = pW[0][r] * thighW[0][c] + pW[1][r] * thighW[1][c] + pW[2][r] * thighW[2][c];
				ls[r][c] = thighW[0][r] * shinW[0][c] + thighW[1][r] * shinW[1][c] + thighW[2][r] * shinW[2][c];
				la[r][c] = shinW[0][r] * ankleW[0][c] + shinW[1][r] * ankleW[1][c] + shinW[2][r] * ankleW[2][c];
			}
		float qt[4], qs[4], qa[4];
		motionQuatFromMat3(lt, qt);
		motionQuatFromMat3(ls, qs);
		motionQuatFromMat3(la, qa);
		io.localQ[4 * thigh + 0] = qt[0]; io.localQ[4 * thigh + 1] = qt[1];
		io.localQ[4 * thigh + 2] = qt[2]; io.localQ[4 * thigh + 3] = qt[3];
		io.localQ[4 * knee + 0] = qs[0]; io.localQ[4 * knee + 1] = qs[1];
		io.localQ[4 * knee + 2] = qs[2]; io.localQ[4 * knee + 3] = qs[3];
		io.localQ[4 * ankle + 0] = qa[0]; io.localQ[4 * ankle + 1] = qa[1];
		io.localQ[4 * ankle + 2] = qa[2]; io.localQ[4 * ankle + 3] = qa[3];
	}
}

void KRetargeter::applyFootLockNodeTake(const MotionNode &nd, const KRecording &rec, int frameIdx, int bodyIdx,
		const std::vector<int> &order, const MotionNodePose &center, MotionNodePose &out) {
	int nBones = (int)m_bones.size();
	out = center;
	if (!center.valid || nBones <= 0 || (int)center.localQ.size() < 4 * nBones) {
		out.valid = false;
		return;
	}
	// Upstream FK of the center frame (pin target base + hip/knee refs).
	std::vector<float> G0, R90;
	fkPose(center, G0, R90);
	if ((int)G0.size() < 3 * nBones || (int)R90.size() < 9 * nBones) {
		out.valid = false;
		return;
	}
	// Ankle trajectory: recent span with full upstream, older span base-only
	// (anchor-preserving and skinless: cheap). Missing frames carry forward.
	const int RECENT = 45;
	const int OLD = 200;
	int nF = (int)rec.frames.size();
	int lo = frameIdx - OLD;
	if (lo < 0) lo = 0;
	std::vector<FootLockSample> win;
	win.reserve((size_t)(frameIdx - lo + 1));
	FootLockSample last;
	last.ok = false;
	last.dt = 1.0f / 30.0f;
	bool haveLast = false;
	int legA[2] = { -1, -1 };
	{
		int th, kn, an, to;
		if (legChain(0, th, kn, an, to)) legA[0] = an;
		if (legChain(1, th, kn, an, to)) legA[1] = an;
	}
	for (int k = lo; k <= frameIdx; ++k) {
		FootLockSample s;
		s.ok = false;
		s.dt = 1.0f / 30.0f;
		if (k > lo && k < nF) {
			INT64 dtm = rec.frames[k].timeMS - rec.frames[k - 1].timeMS;
			if (dtm > 0 && dtm < 1000) s.dt = (float)dtm / 1000.0f;
		}
		bool got = false;
		MotionNodePose up;
		if (k == frameIdx) {
			up = center;
			got = true;
		} else if (k > frameIdx - RECENT) {
			got = evalUpstream(rec, k, bodyIdx, order, nd.id, false, up) && up.valid;
		} else {
			MotionNodePose bp;
			got = basePoseForTakeFrame(rec, k, bodyIdx, bp) && bp.valid;
			if (got) up = bp;
		}
		if (got) {
			std::vector<float> Gk, R9k;
			fkPose(up, Gk, R9k);
			if ((int)Gk.size() >= 3 * nBones && (int)R9k.size() >= 9 * nBones) {
				for (int sd = 0; sd < 2; ++sd) {
					int an = legA[sd];
					if (an < 0) continue;
					float *P = (sd == 0) ? s.pl : s.pr;
					float *Q = (sd == 0) ? s.ql : s.qr;
					P[0] = Gk[3 * an + 0]; P[1] = Gk[3 * an + 1]; P[2] = Gk[3 * an + 2];
					float M[3][3] = {
						{ R9k[9 * an + 0], R9k[9 * an + 1], R9k[9 * an + 2] },
						{ R9k[9 * an + 3], R9k[9 * an + 4], R9k[9 * an + 5] },
						{ R9k[9 * an + 6], R9k[9 * an + 7], R9k[9 * an + 8] } };
					motionQuatFromMat3(M, Q);
				}
				s.ok = true;
			}
		}
		if (!s.ok && haveLast) {
			float savedDt = s.dt;
			s = last;
			s.dt = savedDt;
		}
		win.push_back(s);
		if (s.ok) { last = s; haveLast = true; }
	}
	solveFootLocks(nd, win, (int)win.size() - 1, G0, R90, out);
}

void KRetargeter::applyFootLockNodeLive(const MotionNode &nd, const MotionNodePose &in, MotionNodePose &out) {
	int nBones = (int)m_bones.size();
	out = in;
	if (!in.valid || nBones <= 0 || (int)in.localQ.size() < 4 * nBones) {
		out.valid = false;
		return;
	}
	m_footUpstream.push_back(in);
	while (m_footUpstream.size() > 96)
		m_footUpstream.erase(m_footUpstream.begin());
	int legA[2] = { -1, -1 };
	{
		int th, kn, an, to;
		if (legChain(0, th, kn, an, to)) legA[0] = an;
		if (legChain(1, th, kn, an, to)) legA[1] = an;
	}
	std::vector<float> G0, R90;
	fkPose(in, G0, R90);
	if ((int)G0.size() < 3 * nBones || (int)R90.size() < 9 * nBones) {
		out.valid = false;
		return;
	}
	std::vector<FootLockSample> win;
	win.reserve(m_footUpstream.size());
	FootLockSample last;
	last.ok = false;
	last.dt = 1.0f / 30.0f;
	bool haveLast = false;
	for (size_t k = 0; k < m_footUpstream.size(); ++k) {
		FootLockSample s;
		s.ok = false;
		s.dt = 1.0f / 30.0f;
		std::vector<float> Gk, R9k;
		fkPose(m_footUpstream[k], Gk, R9k);
		if ((int)Gk.size() >= 3 * nBones && (int)R9k.size() >= 9 * nBones) {
			for (int sd = 0; sd < 2; ++sd) {
				int an = legA[sd];
				if (an < 0) continue;
				float *P = (sd == 0) ? s.pl : s.pr;
				float *Q = (sd == 0) ? s.ql : s.qr;
				P[0] = Gk[3 * an + 0]; P[1] = Gk[3 * an + 1]; P[2] = Gk[3 * an + 2];
				float M[3][3] = {
					{ R9k[9 * an + 0], R9k[9 * an + 1], R9k[9 * an + 2] },
					{ R9k[9 * an + 3], R9k[9 * an + 4], R9k[9 * an + 5] },
					{ R9k[9 * an + 6], R9k[9 * an + 7], R9k[9 * an + 8] } };
				motionQuatFromMat3(M, Q);
			}
			s.ok = true;
		}
		if (!s.ok && haveLast) {
			float savedDt = s.dt;
			s = last;
			s.dt = savedDt;
		}
		win.push_back(s);
		if (s.ok) { last = s; haveLast = true; }
	}
	solveFootLocks(nd, win, (int)win.size() - 1, G0, R90, out);
}

// ----------------------------------------------------------------------------
// Remaining processing nodes (Clamp/Mirror/Offset/TimeScale/Reverse/
// FloorSnap/Noise/Spring/Aim/Mask).
// ----------------------------------------------------------------------------

// Per-bone pick/blend from a source pose (TimeScale replace w=1, Mask mix).
// Empty selection = every bone.
static void pickBlendBones(const MotionNode &nd, const MotionNodePose &src, float w,
		MotionNodePose &dst, int nBones, int rootBone) {
	if (!src.valid || !dst.valid || nBones <= 0) return;
	if (w <= 0.0f) return;
	if (w > 1.0f) w = 1.0f;
	if ((int)src.localQ.size() < 4 * nBones || (int)dst.localQ.size() < 4 * nBones) return;
	for (int i = 0; i < nBones; ++i) {
		if (!motionNodeAffectsBone(nd, i, nBones)) continue;
		float r[4];
		motionQuatSlerp(&dst.localQ[4 * i], &src.localQ[4 * i], w, r);
		dst.localQ[4 * i + 0] = r[0]; dst.localQ[4 * i + 1] = r[1];
		dst.localQ[4 * i + 2] = r[2]; dst.localQ[4 * i + 3] = r[3];
		if ((int)src.localT.size() >= 3 * nBones && (int)dst.localT.size() >= 3 * nBones)
			motionLerp3(&dst.localT[3 * i], &src.localT[3 * i], w, &dst.localT[3 * i]);
	}
	if (rootBone >= 0 && rootBone < nBones && motionNodeAffectsBone(nd, rootBone, nBones)
		&& (int)src.rootPos.size() >= 3 && (int)dst.rootPos.size() >= 3)
		motionLerp3(&dst.rootPos[0], &src.rootPos[0], w, &dst.rootPos[0]);
}

// Time remap of one TimeScale node (rate about take start, clamped).
static float nodeRemapTime(const MotionNode &nd, float f, int nF) {
	float rate = MotionNodeParamGet(nd, 0);
	if (!(rate > 0.01f)) rate = 1.0f;
	float t = f * rate;
	if (t < 0.0f) t = 0.0f;
	if (nF > 0 && t > (float)(nF - 1)) t = (float)(nF - 1);
	return t;
}

// Clamp: per-axis euler limits (degrees) on the local rotation.
void KRetargeter::applyClampNode(const MotionNode &nd, const MotionNodePose &in, MotionNodePose &out) {
	int nBones = (int)m_bones.size();
	out = in;
	if (!in.valid || nBones <= 0 || (int)in.localQ.size() < 4 * nBones) {
		out.valid = false;
		return;
	}
	float mnx = MotionNodeParamGet(nd, 0), mxx = MotionNodeParamGet(nd, 1);
	float mny = MotionNodeParamGet(nd, 2), mxy = MotionNodeParamGet(nd, 3);
	float mnz = MotionNodeParamGet(nd, 4), mxz = MotionNodeParamGet(nd, 5);
	for (int i = 0; i < nBones; ++i) {
		if (!motionNodeAffectsBone(nd, i, nBones)) continue;
		float e[3];
		motionQuatToEulerXYZ(&in.localQ[4 * i], e);
		bool ch = false;
		if (e[0] < mnx) { e[0] = mnx; ch = true; } else if (e[0] > mxx) { e[0] = mxx; ch = true; }
		if (e[1] < mny) { e[1] = mny; ch = true; } else if (e[1] > mxy) { e[1] = mxy; ch = true; }
		if (e[2] < mnz) { e[2] = mnz; ch = true; } else if (e[2] > mxz) { e[2] = mxz; ch = true; }
		if (!ch) continue;
		float q[4];
		motionEulerXYZToQuat(e, q);
		out.localQ[4 * i + 0] = q[0]; out.localQ[4 * i + 1] = q[1];
		out.localQ[4 * i + 2] = q[2]; out.localQ[4 * i + 3] = q[3];
	}
}

// Mirror: mirror across the body plane (x -> -x), with optional L/R swap
// (without the swap, mirrored limbs would stay crossed).
void KRetargeter::applyMirrorNode(const MotionNode &nd, const MotionNodePose &in, MotionNodePose &out) {
	int nBones = (int)m_bones.size();
	out = in;
	if (!in.valid || nBones <= 0 || (int)in.localQ.size() < 4 * nBones) {
		out.valid = false;
		return;
	}
	bool swap = MotionNodeParamGet(nd, 0) >= 0.5f;
	// Left/right pairs from the mapping; unpaired bones mirror in place.
	static const int JL[8] = { JointType_HipLeft, JointType_KneeLeft, JointType_AnkleLeft, JointType_FootLeft,
		JointType_ShoulderLeft, JointType_ElbowLeft, JointType_WristLeft, JointType_HandLeft };
	static const int JR[8] = { JointType_HipRight, JointType_KneeRight, JointType_AnkleRight, JointType_FootRight,
		JointType_ShoulderRight, JointType_ElbowRight, JointType_WristRight, JointType_HandRight };
	std::vector<int> mate((size_t)nBones, -1);
	for (int k = 0; k < 8; ++k) {
		int bL = -1, bR = -1;
		for (int b = 0; b < nBones; ++b) {
			int j = mappingJointForBone(b);
			if (j == JL[k]) bL = b;
			else if (j == JR[k]) bR = b;
		}
		if (bL >= 0 && bR >= 0) { mate[(size_t)bL] = bR; mate[(size_t)bR] = bL; }
	}
	for (int i = 0; i < nBones; ++i)
		if (mate[(size_t)i] < 0) mate[(size_t)i] = i;
	bool hasT = (int)in.localT.size() >= 3 * nBones && (int)out.localT.size() >= 3 * nBones;
	for (int i = 0; i < nBones; ++i) {
		if (!motionNodeAffectsBone(nd, i, nBones)) continue;
		int s = swap ? mate[(size_t)i] : i;
		float M[3][3];
		motionMat3FromQuat(&in.localQ[4 * s], M);
		float Mm[3][3];
		for (int r = 0; r < 3; ++r)
			for (int c = 0; c < 3; ++c)
				Mm[r][c] = M[r][c] * ((r == 0) ? -1.0f : 1.0f) * ((c == 0) ? -1.0f : 1.0f);
		float q[4];
		motionQuatFromMat3(Mm, q);
		out.localQ[4 * i + 0] = q[0]; out.localQ[4 * i + 1] = q[1];
		out.localQ[4 * i + 2] = q[2]; out.localQ[4 * i + 3] = q[3];
		if (hasT) {
			out.localT[3 * i + 0] = -in.localT[3 * s + 0];
			out.localT[3 * i + 1] = in.localT[3 * s + 1];
			out.localT[3 * i + 2] = in.localT[3 * s + 2];
		}
	}
	int root = (m_rootBone >= 0 && m_rootBone < nBones) ? m_rootBone : 0;
	if (motionNodeAffectsBone(nd, root, nBones)
		&& (int)in.rootPos.size() >= 3 && (int)out.rootPos.size() >= 3)
		out.rootPos[0] = -in.rootPos[0];
}

// FloorSnap: lifts the character so the lowest foot touches y=0.
// One-sided (never pulls down): jumps and stairs keep working.
void KRetargeter::applyFloorSnapNode(const MotionNode &nd, const MotionNodePose &in, MotionNodePose &out) {
	int nBones = (int)m_bones.size();
	out = in;
	if (!in.valid || nBones <= 0) { out.valid = false; return; }
	float w = MotionNodeParamGet(nd, 0);
	if (w < 0.0f) w = 0.0f;
	if (w > 1.0f) w = 1.0f;
	if (w <= 0.0f) return;
	std::vector<float> G, R9;
	fkPose(in, G, R9);
	if ((int)G.size() < 3 * nBones) { out.valid = false; return; }
	float minY = 1e9f;
	bool any = false;
	for (int side = 0; side < 2; ++side) {
		int thigh, knee, ankle, toe;
		if (!legChain(side, thigh, knee, ankle, toe)) continue;
		if (!footLegWanted(nd, thigh, knee, ankle, toe, nBones)) continue;
		if (G[3 * ankle + 1] < minY) { minY = G[3 * ankle + 1]; any = true; }
		if (toe >= 0 && G[3 * toe + 1] < minY) { minY = G[3 * toe + 1]; any = true; }
	}
	if (!any) return;   // No lockable leg: passthrough
	float lift = -minY;
	if (lift < 0.0f) lift = 0.0f;   // One-sided: only fix sinking
	if (lift <= 0.0f) return;
	if ((int)out.rootPos.size() < 3) return;
	out.rootPos[1] += lift * w;
}

// Spring: underdamped follow over a causal window (secondary motion).
// Simulates per scalar component (quat xyzw normalized after), scrub-safe:
// the window is re-simulated from scratch on every evaluation.
void KRetargeter::springSimWindow(const MotionNode &nd, const std::vector<MotionNodePose> &win,
		const std::vector<float> &dts, const MotionNodePose &center, MotionNodePose &out) {
	int nBones = (int)m_bones.size();
	out = center;
	if (!center.valid || nBones <= 0 || win.empty()) { out.valid = false; return; }
	float stiff = MotionNodeParamGet(nd, 0);   // Responsiveness
	float zeta = MotionNodeParamGet(nd, 1);    // Damping ratio
	if (!(stiff > 0.01f)) stiff = 0.01f;
	if (zeta < 0.0f) zeta = 0.0f;
	if (zeta > 1.0f) zeta = 1.0f;
	float k = stiff * 30.0f;
	float c = zeta * 2.0f * sqrtf(k);
	int root = (m_rootBone >= 0 && m_rootBone < nBones) ? m_rootBone : 0;
	size_t W = win.size();
	// Per-bone state, simulated from the window start.
	for (int i = 0; i < nBones; ++i) {
		if (!motionNodeAffectsBone(nd, i, nBones)) continue;
		if ((int)center.localQ.size() < 4 * nBones) { out.valid = false; return; }
		float y[4] = { 0, 0, 0, 1 }, v[4] = { 0, 0, 0, 0 };
		bool haveY = false;
		// Seed from the first valid sample.
		for (size_t s0 = 0; s0 < W; ++s0) {
			if (win[s0].valid && (int)win[s0].localQ.size() >= 4 * nBones) {
				y[0] = win[s0].localQ[4 * i + 0]; y[1] = win[s0].localQ[4 * i + 1];
				y[2] = win[s0].localQ[4 * i + 2]; y[3] = win[s0].localQ[4 * i + 3];
				haveY = true;
				break;
			}
		}
		if (!haveY) continue;
		for (size_t s = 0; s < W; ++s) {
			if (!win[s].valid || (int)win[s].localQ.size() < 4 * nBones) continue;
			float dt = (s < dts.size() && dts[s] > 0.001f && dts[s] < 0.1f) ? dts[s] : (1.0f / 30.0f);
			float x[4] = { win[s].localQ[4 * i + 0], win[s].localQ[4 * i + 1],
				win[s].localQ[4 * i + 2], win[s].localQ[4 * i + 3] };
			// Hemisphere-align the target to the state.
			if (x[0] * y[0] + x[1] * y[1] + x[2] * y[2] + x[3] * y[3] < 0.0f) {
				x[0] = -x[0]; x[1] = -x[1]; x[2] = -x[2]; x[3] = -x[3];
			}
			for (int q = 0; q < 4; ++q) {
				float a = k * (x[q] - y[q]) - c * v[q];
				v[q] += a * dt;
				y[q] += v[q] * dt;
			}
		}
		float yl = sqrtf(y[0] * y[0] + y[1] * y[1] + y[2] * y[2] + y[3] * y[3]);
		if (yl > 1e-9f) { y[0] /= yl; y[1] /= yl; y[2] /= yl; y[3] /= yl; }
		else { y[0] = 0; y[1] = 0; y[2] = 0; y[3] = 1; }
		out.localQ[4 * i + 0] = y[0]; out.localQ[4 * i + 1] = y[1];
		out.localQ[4 * i + 2] = y[2]; out.localQ[4 * i + 3] = y[3];
	}
	// Root translation + local translations follow the same spring.
	if (motionNodeAffectsBone(nd, root, nBones)
		&& (int)center.rootPos.size() >= 3 && (int)out.rootPos.size() >= 3) {
		float y[3] = { 0, 0, 0 }, v[3] = { 0, 0, 0 };
		bool haveY = false;
		for (size_t s0 = 0; s0 < W; ++s0) {
			if (win[s0].valid && (int)win[s0].rootPos.size() >= 3) {
				y[0] = win[s0].rootPos[0]; y[1] = win[s0].rootPos[1]; y[2] = win[s0].rootPos[2];
				haveY = true;
				break;
			}
		}
		if (haveY) {
			for (size_t s = 0; s < W; ++s) {
				if (!win[s].valid || (int)win[s].rootPos.size() < 3) continue;
				float dt = (s < dts.size() && dts[s] > 0.001f && dts[s] < 0.1f) ? dts[s] : (1.0f / 30.0f);
				for (int a = 0; a < 3; ++a) {
					float acc = k * (win[s].rootPos[a] - y[a]) - c * v[a];
					v[a] += acc * dt;
					y[a] += v[a] * dt;
				}
			}
			out.rootPos[0] = y[0]; out.rootPos[1] = y[1]; out.rootPos[2] = y[2];
		}
	}
	if ((int)center.localT.size() >= 3 * nBones && (int)out.localT.size() >= 3 * nBones) {
		for (int i = 0; i < nBones; ++i) {
			if (!motionNodeAffectsBone(nd, i, nBones)) continue;
			float y[3] = { 0, 0, 0 }, v[3] = { 0, 0, 0 };
			bool haveY = false;
			for (size_t s0 = 0; s0 < W; ++s0) {
				if (win[s0].valid && (int)win[s0].localT.size() >= 3 * nBones) {
					y[0] = win[s0].localT[3 * i + 0]; y[1] = win[s0].localT[3 * i + 1]; y[2] = win[s0].localT[3 * i + 2];
					haveY = true;
					break;
				}
			}
			if (!haveY) continue;
			for (size_t s = 0; s < W; ++s) {
				if (!win[s].valid || (int)win[s].localT.size() < 3 * nBones) continue;
				float dt = (s < dts.size() && dts[s] > 0.001f && dts[s] < 0.1f) ? dts[s] : (1.0f / 30.0f);
				for (int a = 0; a < 3; ++a) {
					float acc = k * (win[s].localT[3 * i + a] - y[a]) - c * v[a];
					v[a] += acc * dt;
					y[a] += v[a] * dt;
				}
			}
			out.localT[3 * i + 0] = y[0]; out.localT[3 * i + 1] = y[1]; out.localT[3 * i + 2] = y[2];
		}
	}
}

void KRetargeter::applySpringNodeTake(const MotionNode &nd, const KRecording &rec, int frameIdx, int bodyIdx,
		const std::vector<int> &order, const MotionNodePose &center, MotionNodePose &out) {
	out = center;
	if (!center.valid) { out.valid = false; return; }
	int W = 30;
	{
		int wpar = (int)floorf(MotionNodeParamGet(nd, 2) + 0.5f);
		if (wpar >= 10 && wpar <= 60) W = wpar;
	}
	int nF = (int)rec.frames.size();
	int lo = frameIdx - W;
	if (lo < 0) lo = 0;
	std::vector<MotionNodePose> win;
	std::vector<float> dts;
	win.reserve((size_t)(frameIdx - lo + 1));
	dts.reserve((size_t)(frameIdx - lo + 1));
	for (int k = lo; k <= frameIdx; ++k) {
		float dt = 1.0f / 30.0f;
		if (k > lo && k < nF) {
			INT64 dtm = rec.frames[k].timeMS - rec.frames[k - 1].timeMS;
			if (dtm > 0 && dtm < 1000) dt = (float)dtm / 1000.0f;
		}
		dts.push_back(dt);
		if (k == frameIdx) {
			win.push_back(center);
			continue;
		}
		MotionNodePose p;
		if (evalUpstream(rec, k, bodyIdx, order, nd.id, false, p) && p.valid)
			win.push_back(p);
		else if (!win.empty())
			win.push_back(win.back());   // carry forward
		else {
			win.push_back(center);
		}
	}
	springSimWindow(nd, win, dts, center, out);
}

void KRetargeter::applySpringNodeLive(const MotionNode &nd, MotionNodePose &out) {
	// Live approximation: simulate over the recent base history (upstream
	// would need its own ring; base is exact when Spring sits near Output).
	out.valid = false;
	if (m_liveHistory.empty()) return;
	int W = 30;
	{
		int wpar = (int)floorf(MotionNodeParamGet(nd, 2) + 0.5f);
		if (wpar >= 10 && wpar <= 60) W = wpar;
	}
	size_t first = 0;
	if ((int)m_liveHistory.size() > W + 1)
		first = m_liveHistory.size() - (size_t)(W + 1);
	std::vector<MotionNodePose> win;
	std::vector<float> dts;
	for (size_t k = first; k < m_liveHistory.size(); ++k) {
		win.push_back(m_liveHistory[k]);
		dts.push_back(1.0f / 30.0f);
	}
	const MotionNodePose &center = m_liveHistory.back();
	springSimWindow(nd, win, dts, center, out);
}

// Noise: deterministic sine wobble layered on top (scrub-safe: pure
// function of take time and bone index; live clock for live mode).
void KRetargeter::applyNoiseOnPose(const MotionNode &nd, float tSec, const MotionNodePose &in, MotionNodePose &out) {
	int nBones = (int)m_bones.size();
	out = in;
	if (!in.valid || nBones <= 0 || (int)in.localQ.size() < 4 * nBones) {
		out.valid = false;
		return;
	}
	float ampD = MotionNodeParamGet(nd, 0);
	float spd = MotionNodeParamGet(nd, 1);
	if (!(ampD > 0.0f) || !(spd > 0.0f)) return;   // out == in
	const double TAU = 6.283185307179586;
	for (int i = 0; i < nBones; ++i) {
		if (!motionNodeAffectsBone(nd, i, nBones)) continue;
		double e[3] = {
			(double)ampD * sin(TAU * ((double)spd * tSec + (double)i * 0.13)),
			(double)ampD * sin(TAU * ((double)spd * tSec * 1.31 + (double)i * 0.29 + 1.0)),
			(double)ampD * sin(TAU * ((double)spd * tSec * 0.77 + (double)i * 0.53 + 2.0)) };
		float ed[3] = { (float)e[0], (float)e[1], (float)e[2] };
		float qoff[4];
		motionEulerXYZToQuat(ed, qoff);
		float r[4];
		motionQuatMul(qoff, &in.localQ[4 * i], r);
		out.localQ[4 * i + 0] = r[0]; out.localQ[4 * i + 1] = r[1];
		out.localQ[4 * i + 2] = r[2]; out.localQ[4 * i + 3] = r[3];
	}
}

void KRetargeter::applyNoiseNodeTake(const MotionNode &nd, const KRecording &rec, int frameIdx,
		const MotionNodePose &in, MotionNodePose &out) {
	float t = 0.0f;
	int N = (int)rec.frames.size();
	if (N > 0) {
		int fi = frameIdx;
		if (fi < 0) fi = 0;
		if (fi > N - 1) fi = N - 1;
		t = (float)(rec.frames[fi].timeMS - rec.frames[0].timeMS) / 1000.0f;
	}
	applyNoiseOnPose(nd, t, in, out);
}

void KRetargeter::applyNoiseNodeLive(const MotionNode &nd, const MotionNodePose &in, MotionNodePose &out) {
	applyNoiseOnPose(nd, (float)m_liveTick / 30.0f, in, out);
}

// Mask: mixes the chain pose back toward the raw take pose on masked bones.
void KRetargeter::applyMaskNodeTake(const MotionNode &nd, const KRecording &rec, int frameIdx, int bodyIdx,
		const MotionNodePose &in, MotionNodePose &out) {
	out = in;
	if (!in.valid) { out.valid = false; return; }
	float w = MotionNodeParamGet(nd, 0);
	if (w < 0.0f) w = 0.0f;
	if (w > 1.0f) w = 1.0f;
	if (w <= 0.0f) return;   // out == in
	MotionNodePose b;
	if (!basePoseForTakeFrame(rec, frameIdx, bodyIdx, b) || !b.valid) {
		out.valid = false;
		return;
	}
	int nBones = (int)m_bones.size();
	int root = (m_rootBone >= 0 && m_rootBone < nBones) ? m_rootBone : 0;
	pickBlendBones(nd, b, w, out, nBones, root);
	if (!out.valid) return;
}

void KRetargeter::applyMaskNodeLive(const MotionNode &nd, const MotionNodePose &in, MotionNodePose &out) {
	out = in;
	if (!in.valid) { out.valid = false; return; }
	float w = MotionNodeParamGet(nd, 0);
	if (w < 0.0f) w = 0.0f;
	if (w > 1.0f) w = 1.0f;
	if (w <= 0.0f) return;
	if (m_liveHistory.empty()) { out.valid = false; return; }
	const MotionNodePose &b = m_liveHistory.back();
	int nBones = (int)m_bones.size();
	int root = (m_rootBone >= 0 && m_rootBone < nBones) ? m_rootBone : 0;
	pickBlendBones(nd, b, w, out, nBones, root);
	if (!out.valid) return;
}

// TimeScale: the upstream prefix runs at the remapped (possibly fractional)
// frame; selected bones take that pose (empty = every bone).
void KRetargeter::applyTimeScaleNode(const MotionNode &nd, const KRecording &rec, float selfFrame, int bodyIdx,
		const std::vector<int> &order, const MotionNodePose &cur, MotionNodePose &out) {
	out = cur;
	float F = nodeRemapTime(nd, selfFrame, (int)rec.frames.size());
	if (fabsf(F - selfFrame) < 1e-6f) return;   // Rate 1: transparent
	MotionNodePose h;
	if (!evalUpstreamFloat(rec, F, bodyIdx, order, nd.id, h) || !h.valid) {
		out.valid = false;
		return;
	}
	int nBones = (int)m_bones.size();
	int root = (m_rootBone >= 0 && m_rootBone < nBones) ? m_rootBone : 0;
	pickBlendBones(nd, h, 1.0f, out, nBones, root);
	if (!out.valid) return;
}

// Reverse: the whole upstream runs backwards (take only, global).
void KRetargeter::applyReverseNodeTake(const MotionNode &nd, const KRecording &rec, float selfFrame, int bodyIdx,
		const std::vector<int> &order, const MotionNodePose &cur, MotionNodePose &out) {
	out = cur;
	int N = (int)rec.frames.size();
	if (N <= 0) { out.valid = false; return; }
	float F = (float)(N - 1) - selfFrame;
	MotionNodePose h;
	if (!evalUpstreamFloat(rec, F, bodyIdx, order, nd.id, h) || !h.valid) {
		out.valid = false;
		return;
	}
	out = h;
}

// Aim: points selected bones toward a yaw/pitch world direction.
void KRetargeter::applyAimNode(const MotionNode &nd, const MotionNodePose &in, MotionNodePose &out) {
	int nBones = (int)m_bones.size();
	out = in;
	if (!in.valid || nBones <= 0 || (int)in.localQ.size() < 4 * nBones) {
		out.valid = false;
		return;
	}
	float yaw = MotionNodeParamGet(nd, 0) * (float)M_PI / 180.0f;
	float pitch = MotionNodeParamGet(nd, 1) * (float)M_PI / 180.0f;
	float w = MotionNodeParamGet(nd, 2);
	if (w < 0.0f) w = 0.0f;
	if (w > 1.0f) w = 1.0f;
	if (w <= 0.0f) return;
	float cp = cosf(pitch);
	float dir[3] = { sinf(yaw) * cp, sinf(pitch), cosf(yaw) * cp };
	std::vector<float> G, R9;
	fkPose(in, G, R9);
	if ((int)G.size() < 3 * nBones || (int)R9.size() < 9 * nBones) {
		out.valid = false;
		return;
	}
	int root = (m_rootBone >= 0 && m_rootBone < nBones) ? m_rootBone : 0;
	for (int i = 0; i < nBones; ++i) {
		if (i == root) continue;   // Never turns the whole body
		if (!motionNodeAffectsBone(nd, i, nBones)) continue;
		int p = m_bones[i].parent;
		float Rp[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
		if (p >= 0 && p < nBones) {
			Rp[0][0] = R9[9 * p + 0]; Rp[0][1] = R9[9 * p + 1]; Rp[0][2] = R9[9 * p + 2];
			Rp[1][0] = R9[9 * p + 3]; Rp[1][1] = R9[9 * p + 4]; Rp[1][2] = R9[9 * p + 5];
			Rp[2][0] = R9[9 * p + 6]; Rp[2][1] = R9[9 * p + 7]; Rp[2][2] = R9[9 * p + 8];
		}
		// Direction in parent space, aim the rest axis at it.
		float dp[3] = {
			Rp[0][0] * dir[0] + Rp[1][0] * dir[1] + Rp[2][0] * dir[2],
			Rp[0][1] * dir[0] + Rp[1][1] * dir[1] + Rp[2][1] * dir[2],
			Rp[0][2] * dir[0] + Rp[1][2] * dir[1] + Rp[2][2] * dir[2] };
		float dl = sqrtf(dp[0] * dp[0] + dp[1] * dp[1] + dp[2] * dp[2]);
		if (!(dl > 1e-6f)) continue;
		dp[0] /= dl; dp[1] /= dl; dp[2] /= dl;
		float Ma[3][3];
		aimRotation(Ma, m_bones[i].restAxis, dp);
		float qa[4];
		motionQuatFromMat3(Ma, qa);
		float r[4];
		motionQuatSlerp(&in.localQ[4 * i], qa, w, r);
		out.localQ[4 * i + 0] = r[0]; out.localQ[4 * i + 1] = r[1];
		out.localQ[4 * i + 2] = r[2]; out.localQ[4 * i + 3] = r[3];
	}
}

// Forward kinematics shared by buildFinalPose and the Foot Lock solver:
// graph-local pose -> world positions G + world rotations R9 (row-major).
// Empty outputs when the input is unusable.
void KRetargeter::fkPose(const MotionNodePose &np, std::vector<float> &G, std::vector<float> &R9) {
	int nBones = (int)m_bones.size();
	G.clear(); R9.clear();
	if (nBones <= 0 || !np.valid || (int)np.localQ.size() < 4 * nBones || (int)np.rootPos.size() < 3
		|| (int)np.localT.size() < 3 * nBones)
		return;
	int root = (m_rootBone >= 0 && m_rootBone < nBones) ? m_rootBone : 0;
	G.assign(3 * nBones, 0.0f); R9.assign(9 * nBones, 0.0f);
	for (int i = 0; i < nBones; ++i) {
		float Li[3][3];
		motionMat3FromQuat(&np.localQ[4 * i], Li);
		int p = m_bones[i].parent;
		if (i == root || p < 0 || p >= nBones) {
			memcpy(&R9[9 * i], Li, sizeof(Li));
			if (i == root) {
				G[3 * i + 0] = np.rootPos[0];
				G[3 * i + 1] = np.rootPos[1];
				G[3 * i + 2] = np.rootPos[2];
			} else {
				// Parentless non-root bone: keep its rest spot (as computePoseBody does).
				G[3 * i + 0] = m_bones[i].restPos[0];
				G[3 * i + 1] = m_bones[i].restPos[1];
				G[3 * i + 2] = m_bones[i].restPos[2];
			}
		} else {
			const float *Rp = &R9[9 * p];
			// Local translation comes from the pose (rest offset + node deltas).
			float rv[3] = { np.localT[3 * i + 0], np.localT[3 * i + 1], np.localT[3 * i + 2] };
			float *Ri = &R9[9 * i];
			for (int r = 0; r < 3; ++r)
				for (int c = 0; c < 3; ++c)
					Ri[r * 3 + c] = Rp[r * 3 + 0] * Li[0][c] + Rp[r * 3 + 1] * Li[1][c] + Rp[r * 3 + 2] * Li[2][c];
			G[3 * i + 0] = G[3 * p + 0] + Rp[0] * rv[0] + Rp[1] * rv[1] + Rp[2] * rv[2];
			G[3 * i + 1] = G[3 * p + 1] + Rp[3] * rv[0] + Rp[4] * rv[1] + Rp[5] * rv[2];
			G[3 * i + 2] = G[3 * p + 2] + Rp[6] * rv[0] + Rp[7] * rv[1] + Rp[8] * rv[2];
		}
	}
}

// Forward kinematics: graph-local pose -> m_pose (viewports + export).
void KRetargeter::buildFinalPose(const MotionNodePose &np) {
	int nBones = (int)m_bones.size();
	std::vector<float> G, R9;
	fkPose(np, G, R9);
	if (nBones <= 0 || (int)G.size() < 3 * nBones || (int)R9.size() < 9 * nBones) {
		m_pose.valid = false;
		return;
	}
	finalizePoseFK(&G[0], &R9[0], nBones, true);
}

// Full-pose blend (slerp locals, lerp rootPos/localT). Invalid input falls
// back to the valid side.
void KRetargeter::blendNodePoses(const MotionNodePose &a, const MotionNodePose &b, float t, MotionNodePose &out) {
	int nBones = (int)m_bones.size();
	if (!a.valid && !b.valid) { out.valid = false; return; }
	if (!a.valid) { out = b; return; }
	if (!b.valid) { out = a; return; }
	if (t <= 0.0f) { out = a; return; }
	if (t >= 1.0f) { out = b; return; }
	if (nBones <= 0 || (int)a.localQ.size() < 4 * nBones || (int)b.localQ.size() < 4 * nBones) {
		out = a;
		return;
	}
	out = a;
	for (int i = 0; i < nBones; ++i) {
		float r[4];
		motionQuatSlerp(&a.localQ[4 * i], &b.localQ[4 * i], t, r);
		out.localQ[4 * i + 0] = r[0]; out.localQ[4 * i + 1] = r[1];
		out.localQ[4 * i + 2] = r[2]; out.localQ[4 * i + 3] = r[3];
	}
	if ((int)a.rootPos.size() >= 3 && (int)b.rootPos.size() >= 3 && (int)out.rootPos.size() >= 3)
		motionLerp3(&a.rootPos[0], &b.rootPos[0], t, &out.rootPos[0]);
	if ((int)a.localT.size() >= 3 * nBones && (int)b.localT.size() >= 3 * nBones
		&& (int)out.localT.size() >= 3 * nBones) {
		for (int i = 0; i < nBones; ++i)
			motionLerp3(&a.localT[3 * i], &b.localT[3 * i], t, &out.localT[3 * i]);
	}
	out.valid = true;
}

// Base pose at a fractional take frame: blends the two surrounding
// integer base poses (anchor-preserving).
bool KRetargeter::evalBaseFloat(const KRecording &rec, float frameF, int bodyIdx, MotionNodePose &outPose) {
	int N = (int)rec.frames.size();
	outPose.valid = false;
	if (N <= 0) return false;
	float fc = frameF;
	if (!(fc >= 0.0f)) fc = 0.0f;
	if (fc > (float)(N - 1)) fc = (float)(N - 1);
	int f0 = (int)floorf(fc);
	if (f0 < 0) f0 = 0;
	if (f0 > N - 1) f0 = N - 1;
	int f1 = f0 + 1;
	if (f1 > N - 1) f1 = N - 1;
	MotionNodePose p0;
	if (!basePoseForTakeFrame(rec, f0, bodyIdx, p0) || !p0.valid) return false;
	if (f1 == f0) { outPose = p0; return true; }
	MotionNodePose p1;
	if (!basePoseForTakeFrame(rec, f1, bodyIdx, p1) || !p1.valid) {
		outPose = p0;
		return true;
	}
	blendNodePoses(p0, p1, fc - (float)f0, outPose);
	return outPose.valid;
}

// Fractional-frame prefix evaluation (TimeScale rate): blends the two
// surrounding integer evaluations. ownAnchor is always false here.
bool KRetargeter::evalUpstreamFloat(const KRecording &rec, float frameF, int bodyIdx,
		const std::vector<int> &order, int stopId, MotionNodePose &outPose) {
	int N = (int)rec.frames.size();
	outPose.valid = false;
	if (N <= 0) return false;
	float fc = frameF;
	if (!(fc >= 0.0f)) fc = 0.0f;
	if (fc > (float)(N - 1)) fc = (float)(N - 1);
	int f0 = (int)floorf(fc);
	if (f0 < 0) f0 = 0;
	if (f0 > N - 1) f0 = N - 1;
	int f1 = f0 + 1;
	if (f1 > N - 1) f1 = N - 1;
	MotionNodePose p0;
	if (!evalUpstream(rec, f0, bodyIdx, order, stopId, false, p0) || !p0.valid) return false;
	if (f1 == f0) { outPose = p0; return true; }
	MotionNodePose p1;
	if (!evalUpstream(rec, f1, bodyIdx, order, stopId, false, p1) || !p1.valid) {
		outPose = p0;
		return true;
	}
	blendNodePoses(p0, p1, fc - (float)f0, outPose);
	return outPose.valid;
}

bool KRetargeter::evalUpstream(const KRecording &rec, int frameIdx, int bodyIdx,
		const std::vector<int> &order, int stopId, bool ownAnchor, MotionNodePose &outPose) {
	int nF = (int)rec.frames.size();
	size_t M = order.size();
	// Pass 1: effective frame per node (TimeScale remaps its upstream time).
	std::vector<float> feff(M, (float)frameIdx);
	{
		float fcur = (float)frameIdx;
		for (int i = (int)M - 1; i >= 0; --i) {
			feff[(size_t)i] = fcur;
			const MotionNode *tnd = m_nodeGraph.find(order[(size_t)i]);
			if (tnd && tnd->type == MotionNodeType::TimeScale)
				fcur = nodeRemapTime(*tnd, fcur, nF);
		}
	}
	// Anchor advances from the true frame when it owns it; remapped motion
	// below always preserves the anchor state.
	bool remapped = false;
	for (size_t i = 0; i < M; ++i)
		if (fabsf(feff[i] - (float)frameIdx) > 1e-6f) { remapped = true; break; }
	if (ownAnchor && remapped) {
			if (frameIdx < 0 || frameIdx >= nF) return false;
			const KSampleFrame &fr0 = rec.frames[frameIdx];
			if (fr0.bodies.empty() || bodyIdx < 0 || bodyIdx >= (int)fr0.bodies.size()) return false;
		MotionNodePose adv;
		if (!basePoseForBody(fr0.bodies[bodyIdx], adv)) return false;
	}
	MotionNodePose cur;
	cur.valid = false;
	for (size_t oi = 0; oi < M; ++oi) {
		int nid = order[oi];
		if (nid == stopId) break;
		const MotionNode *nd = m_nodeGraph.find(nid);
		if (!nd) return false;
		int feffI = (int)floorf(feff[oi] + 0.5f);   // node's own frame (rounded)
		switch (nd->type) {
		case MotionNodeType::Output: {
			if (ownAnchor && !remapped) {
				if (frameIdx < 0 || frameIdx >= nF) return false;
				const KSampleFrame &fr = rec.frames[frameIdx];
				if (fr.bodies.empty() || bodyIdx < 0 || bodyIdx >= (int)fr.bodies.size()) return false;
				if (!basePoseForBody(fr.bodies[bodyIdx], cur)) return false;
			} else {
				if (!evalBaseFloat(rec, feff[oi], bodyIdx, cur)) return false;
			}
			break;
		}
		case MotionNodeType::Smooth: {
			MotionNodePose out2;
			applySmoothNodeTake(*nd, rec, feffI, bodyIdx, cur, out2);
			cur = out2;
			break;
		}
		case MotionNodeType::Multiply: {
			MotionNodePose out2;
			applyMultiplyNode(*nd, cur, out2);
			cur = out2;
			break;
		}
		case MotionNodeType::Add: {
			MotionNodePose out2;
			applyAddNode(*nd, cur, out2);
			cur = out2;
			break;
		}
		case MotionNodeType::FootLock: {
			MotionNodePose out2;
			applyFootLockNodeTake(*nd, rec, feffI, bodyIdx, order, cur, out2);
			cur = out2;
			break;
		}
		case MotionNodeType::Clamp: {
			MotionNodePose out2;
			applyClampNode(*nd, cur, out2);
			cur = out2;
			break;
		}
		case MotionNodeType::Mirror: {
			MotionNodePose out2;
			applyMirrorNode(*nd, cur, out2);
			cur = out2;
			break;
		}
		case MotionNodeType::Offset: {
			MotionNodePose out2;
			applyOffsetNode(*nd, cur, out2);
			cur = out2;
			break;
		}
		case MotionNodeType::TimeScale: {
			MotionNodePose out2;
			applyTimeScaleNode(*nd, rec, feff[oi], bodyIdx, order, cur, out2);
			cur = out2;
			break;
		}
		case MotionNodeType::Reverse: {
			MotionNodePose out2;
			applyReverseNodeTake(*nd, rec, feff[oi], bodyIdx, order, cur, out2);
			cur = out2;
			break;
		}
		case MotionNodeType::FloorSnap: {
			MotionNodePose out2;
			applyFloorSnapNode(*nd, cur, out2);
			cur = out2;
			break;
		}
		case MotionNodeType::Noise: {
			MotionNodePose out2;
			applyNoiseNodeTake(*nd, rec, feffI, cur, out2);
			cur = out2;
			break;
		}
		case MotionNodeType::Spring: {
			MotionNodePose out2;
			applySpringNodeTake(*nd, rec, feffI, bodyIdx, order, cur, out2);
			cur = out2;
			break;
		}
		case MotionNodeType::Aim: {
			MotionNodePose out2;
			applyAimNode(*nd, cur, out2);
			cur = out2;
			break;
		}
		case MotionNodeType::Mask: {
			MotionNodePose out2;
			applyMaskNodeTake(*nd, rec, feffI, bodyIdx, cur, out2);
			cur = out2;
			break;
		}
		case MotionNodeType::Input:
			return false;   // Must be passed as stopId, never evaluated here
		default:
			return false;
		}
		if (!cur.valid) return false;
	}
	outPose = cur;
	return cur.valid;
}

// Evaluates the graph for a recorded take frame. The final (reworked) pose
// lands in m_pose; a broken chain or an unusable frame yields the T-pose.
bool KRetargeter::poseFinal(const KRecording &rec, int frameIdx, int bodyIdx) {
	int nBones = (int)m_bones.size();
	if (nBones <= 0) { m_pose.valid = false; return false; }
	std::vector<int> order;
	if (!m_nodeGraph.chainFromInput(order) || order.empty()) { poseRest(); return true; }
	MotionNodePose cur;
	if (!evalUpstream(rec, frameIdx, bodyIdx, order, order.back(), true, cur) || !cur.valid) {
		poseRest();
		return true;
	}
	buildFinalPose(cur);
	return true;
}

// Live variant (no take context): Smooth draws on the recent live history.
bool KRetargeter::poseFinalLive(const KSampleBody &body) {
	int nBones = (int)m_bones.size();
	if (nBones <= 0) { m_pose.valid = false; return false; }
	std::vector<int> order;
	if (!m_nodeGraph.chainFromInput(order)) { m_liveHistory.clear(); m_footUpstream.clear(); poseRest(); return true; }
	MotionNodePose base;
	if (!basePoseForBody(body, base)) { poseRest(); return true; }
	m_liveHistory.push_back(base);
	while (m_liveHistory.size() > 64)
		m_liveHistory.erase(m_liveHistory.begin());
	++m_liveTick;
	MotionNodePose cur;
	cur.valid = false;
	for (size_t oi = 0; oi < order.size(); ++oi) {
		const MotionNode *nd = m_nodeGraph.find(order[oi]);
		if (!nd) { poseRest(); return true; }
		switch (nd->type) {
		case MotionNodeType::Output:
			cur = base;
			break;
		case MotionNodeType::Smooth: {
			MotionNodePose out;
			applySmoothNodeLive(*nd, out);
			cur = out;
			break;
		}
		case MotionNodeType::Multiply: {
			MotionNodePose out;
			applyMultiplyNode(*nd, cur, out);
			cur = out;
			break;
		}
		case MotionNodeType::Add: {
			MotionNodePose out;
			applyAddNode(*nd, cur, out);
			cur = out;
			break;
		}
		case MotionNodeType::FootLock: {
			MotionNodePose out;
			applyFootLockNodeLive(*nd, cur, out);
			cur = out;
			break;
		}
		case MotionNodeType::Clamp: {
			MotionNodePose out;
			applyClampNode(*nd, cur, out);
			cur = out;
			break;
		}
		case MotionNodeType::Mirror: {
			MotionNodePose out;
			applyMirrorNode(*nd, cur, out);
			cur = out;
			break;
		}
		case MotionNodeType::Offset: {
			MotionNodePose out;
			applyOffsetNode(*nd, cur, out);
			cur = out;
			break;
		}
		case MotionNodeType::TimeScale:
			// No take in live mode: transparent passthrough.
			break;
		case MotionNodeType::Reverse:
			// No take in live mode: transparent passthrough.
			break;
		case MotionNodeType::FloorSnap: {
			MotionNodePose out;
			applyFloorSnapNode(*nd, cur, out);
			cur = out;
			break;
		}
		case MotionNodeType::Noise: {
			MotionNodePose out;
			applyNoiseNodeLive(*nd, cur, out);
			cur = out;
			break;
		}
		case MotionNodeType::Spring: {
			MotionNodePose out;
			applySpringNodeLive(*nd, out);
			cur = out;
			break;
		}
		case MotionNodeType::Aim: {
			MotionNodePose out;
			applyAimNode(*nd, cur, out);
			cur = out;
			break;
		}
		case MotionNodeType::Mask: {
			MotionNodePose out;
			applyMaskNodeLive(*nd, cur, out);
			cur = out;
			break;
		}
		case MotionNodeType::Input:
			if (!cur.valid) { poseRest(); return true; }
			buildFinalPose(cur);
			return true;
		default:
			poseRest();
			return true;
		}
		if (!cur.valid) { poseRest(); return true; }
	}
	poseRest();
	return true;
}

void KRetargeter::resetMotionAnchor() {
	m_motionAnchorSet = false;
}

// ----------------------------------------------------------------------------
// Export
// ----------------------------------------------------------------------------

extern KBodyExporter *g_pExporter;

static void quatFromMatrix(const float R[3][3], double q[4]) {
	// The extraction below assumes a pure rotation matrix. Purify the
	// input first: a scaled matrix slipping through (e.g. from a scaled
	// armature holder upstream) keeps its axis but inflates every angle,
	// and normalizing the quaternion afterwards would NOT recover the
	// angle - the input itself must be fixed. Per-column normalization is
	// exact for TRS compositions without shear.
	float N[3][3];
	{
		double n0 = sqrt((double)R[0][0] * R[0][0] + (double)R[1][0] * R[1][0] + (double)R[2][0] * R[2][0]);
		double n1 = sqrt((double)R[0][1] * R[0][1] + (double)R[1][1] * R[1][1] + (double)R[2][1] * R[2][1]);
		double n2 = sqrt((double)R[0][2] * R[0][2] + (double)R[1][2] * R[1][2] + (double)R[2][2] * R[2][2]);
		if (!(n0 > 1e-12)) n0 = 1.0;
		if (!(n1 > 1e-12)) n1 = 1.0;
		if (!(n2 > 1e-12)) n2 = 1.0;
		for (int r = 0; r < 3; ++r) {
			N[r][0] = (float)(R[r][0] / n0);
			N[r][1] = (float)(R[r][1] / n1);
			N[r][2] = (float)(R[r][2] / n2);
		}
	}
	double tr = (double)N[0][0] + (double)N[1][1] + (double)N[2][2];
	if (tr > 0.0) {
		double s = sqrt(tr + 1.0) * 2.0;
		q[0] = (N[2][1] - N[1][2]) / s;
		q[1] = (N[0][2] - N[2][0]) / s;
		q[2] = (N[1][0] - N[0][1]) / s;
		q[3] = 0.25 * s;
	} else if (N[0][0] > N[1][1] && N[0][0] > N[2][2]) {
		double s = sqrt(1.0 + N[0][0] - N[1][1] - N[2][2]) * 2.0;
		q[0] = 0.25 * s;
		q[1] = (N[0][1] + N[1][0]) / s;
		q[2] = (N[0][2] + N[2][0]) / s;
		q[3] = (N[2][1] - N[1][2]) / s;
	} else if (N[1][1] > N[2][2]) {
		double s = sqrt(1.0 + N[1][1] - N[0][0] - N[2][2]) * 2.0;
		q[0] = (N[0][1] + N[1][0]) / s;
		q[1] = 0.25 * s;
		q[2] = (N[1][2] + N[2][1]) / s;
		q[3] = (N[0][2] - N[2][0]) / s;
	} else {
		double s = sqrt(1.0 + N[2][2] - N[0][0] - N[1][1]) * 2.0;
		q[0] = (N[0][2] + N[2][0]) / s;
		q[1] = (N[1][2] + N[2][1]) / s;
		q[2] = 0.25 * s;
		q[3] = (N[1][0] - N[0][1]) / s;
	}
	// Guarantee a unit quaternion out.
	{
		double n = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
		if (n > 1e-12) {
			q[0] /= n; q[1] /= n; q[2] /= n; q[3] /= n;
		} else {
			q[0] = 0.0; q[1] = 0.0; q[2] = 0.0; q[3] = 1.0;
		}
	}
}

// Removes every mesh (and its skin deformers) from a loaded scene so the
// exported file only carries the armature + the retargeted take.
static void removeMeshes(FbxScene *scene) {
	std::vector<FbxNode *> meshes;
	std::vector<FbxNode *> walk;
	walk.push_back(scene->GetRootNode());
	while (!walk.empty()) {
		FbxNode *n = walk.back();
		walk.pop_back();
		for (int i = 0; i < n->GetChildCount(); ++i)
			walk.push_back(n->GetChild(i));
		FbxNodeAttribute *attr = n->GetNodeAttribute();
		if (attr && attr->GetAttributeType() == FbxNodeAttribute::eMesh)
			meshes.push_back(n);
	}
	for (size_t i = 0; i < meshes.size(); ++i) {
		FbxNode *n = meshes[i];
		FbxNodeAttribute *attr = n->GetNodeAttribute();
		if (attr) {
			n->SetNodeAttribute(NULL);
			attr->Destroy();
		}
		n->Destroy();
	}
}

// Removes every animation (stacks, layers and curves) from a loaded scene so
// the original model can be re-exported with only the retargeted take on it.
static void removeSceneAnimation(FbxScene *scene) {
	std::vector<FbxNode *> nodes;
	std::vector<FbxNode *> walk;
	walk.push_back(scene->GetRootNode());
	while (!walk.empty()) {
		FbxNode *n = walk.back();
		walk.pop_back();
		nodes.push_back(n);
		for (int i = 0; i < n->GetChildCount(); ++i)
			walk.push_back(n->GetChild(i));
	}

	for (size_t k = 0; k < nodes.size(); ++k) {
		FbxNode *n = nodes[k];
		FbxProperty props[3] = { n->LclTranslation, n->LclRotation, n->LclScaling };
		for (int p = 0; p < 3; ++p) {
			while (props[p].GetSrcObjectCount(
				       FbxCriteria::ObjectType(FbxAnimCurveNode::ClassId)) > 0) {
				FbxAnimCurveNode *cn = (FbxAnimCurveNode *)props[p].GetSrcObject(
					FbxCriteria::ObjectType(FbxAnimCurveNode::ClassId), 0);
				while (cn->GetDstObjectCount(
					       FbxCriteria::ObjectType(FbxAnimCurve::ClassId)) > 0) {
					FbxAnimCurve *curve = (FbxAnimCurve *)cn->GetDstObject(
						FbxCriteria::ObjectType(FbxAnimCurve::ClassId), 0);
					curve->Destroy();
				}
				props[p].DisconnectSrcObject(cn);
				cn->Destroy();
			}
		}
	}

	// Remove and destroy every animation stack (which takes the layers with it).
	while (scene->GetSrcObjectCount(FbxCriteria::ObjectType(FbxAnimStack::ClassId)) > 0) {
		FbxAnimStack *animStack = (FbxAnimStack *)scene->GetSrcObject(
			FbxCriteria::ObjectType(FbxAnimStack::ClassId), 0);
		scene->RemoveMember(animStack);
		animStack->Destroy();
	}
}

// Finds the skeleton node whose name matches `name` (exact, or without the FBX
// namespace prefix "prefix:name").
static FbxNode *findSkeletonNode(FbxNode *root, const std::string &name) {
	std::vector<FbxNode *> walk;
	walk.push_back(root);
	while (!walk.empty()) {
		FbxNode *n = walk.back();
		walk.pop_back();
		FbxNodeAttribute *attr = n->GetNodeAttribute();
		if (attr && attr->GetAttributeType() == FbxNodeAttribute::eSkeleton) {
			const char *nName = n->GetName();
			if (name == nName)
				return n;
			const char *col = strrchr(nName, ':');
			if (col && name == std::string(col + 1))
				return n;
		}
		for (int i = 0; i < n->GetChildCount(); ++i)
			walk.push_back(n->GetChild(i));
	}
	return NULL;
}

bool KRetargeter::exportRetargetToFile(const char *outPath) {
	if (m_sourcePath.empty() || m_bones.empty())
		return false;
	if (!g_pExporter || g_pExporter->recording().frames.empty()) {
		UI_Printf("Nothing recorded yet. Press Record before exporting.");
		return false;
	}

	FbxManager *mgr = FbxManager::Create();
	if (!mgr)
		return false;

	FbxIOSettings *ios = FbxIOSettings::Create(mgr, IOSROOT);
	mgr->SetIOSettings(ios);

	bool ok = false;

	// Load the original character scene as-is. The mesh, skin, rest pose,
	// hierarchy, materials, axes and units are kept exactly as they were
	// imported - only the animation is removed and replaced by the retargeted
	// take, so the exported file stays identical to the source model.
	FbxScene *scene = FbxScene::Create(mgr, "ImportScene");
	if (!LoadScene(mgr, scene, m_sourcePath.c_str())) {
		UI_Printf("Failed to load the source scene %s.", m_sourcePath.c_str());
		scene->Destroy();
		mgr->Destroy();
		return false;
	}

	removeSceneAnimation(scene);
	removeMeshes(scene);

	int nBones = (int)m_bones.size();
	if (nBones > 0) {
		// Match the retarget skeleton to the nodes of the loaded scene. The
		// original nodes keep their own transforms, rotation order and pivots,
		// so the static model stays byte-identical to the source.
		std::vector<FbxNode *> boneNodes(nBones, (FbxNode *)NULL);
		bool allFound = true;
		for (int i = 0; i < nBones; ++i) {
			boneNodes[i] = findSkeletonNode(scene->GetRootNode(), m_bones[i].name);
			if (!boneNodes[i]) {
				UI_Printf("Bone \"%s\" not found in %s. Aborting export.",
					m_bones[i].name.c_str(), m_sourcePath.c_str());
				allFound = false;
			}
		}
		if (!allFound) {
			scene->Destroy();
			mgr->Destroy();
			return false;
		}

		// Every parentless bone keeps its original placement: its animated
		// local values are written relative to ITS OWN static parent world
		// transform. Evaluated per root (not once for the first root) so a
		// rig with several top-level bones and different parents works too.
		std::vector<FbxAMatrix> rootParentWorlds(nBones);
		std::vector<FbxAMatrix> rootParentInvs(nBones);
		for (int i = 0; i < nBones; ++i) {
			rootParentWorlds[i].SetIdentity();
			rootParentInvs[i].SetIdentity();
			if (m_bones[i].parent >= 0)
				continue;
			FbxNode *parent = boneNodes[i]->GetParent();
			if (parent) {
				rootParentWorlds[i] = parent->EvaluateGlobalTransform(FbxTime(), FbxNode::eSourcePivot);
				rootParentInvs[i] = rootParentWorlds[i].Inverse();
			}
		}

		FbxAnimStack *animStack = FbxAnimStack::Create(scene, "Retarget Animation");
		FbxAnimLayer *animLayer = FbxAnimLayer::Create(scene, "Retarget Layer");
		animStack->AddMember(animLayer);
		scene->SetCurrentAnimationStack(animStack);

		scene->GetGlobalSettings().SetTimeMode(FbxTime::eFrames30);

		const KRecording &rec = g_pExporter->recording();
		INT64 baseTime = rec.startTimeMS();

		// Keep euler rotation continuous across frames
		std::vector<float> prevRot(3 * nBones, 0.0f);
		std::vector<bool>  hasPrev(nBones, false);

		// Anchor the root on the first frame of the take
		resetMotionAnchor();

		int frameCount = (int)rec.frames.size();
		for (int f = 0; f < frameCount; ++f) {
			const KSampleFrame &frame = rec.frames[f];
			if (frame.bodies.empty())
				continue;

			int bodyIdx = 0;
			int bestTracked = -1;
			for (size_t b = 0; b < frame.bodies.size(); ++b) {
				int tracked = 0;
				for (int j = 0; j < JointType_Count; ++j)
					if (frame.bodies[b].joints[j].trackingState != TrackingState_NotTracked)
						tracked++;
				if (tracked > bestTracked) { bestTracked = tracked; bodyIdx = (int)b; }
			}
			poseFinal(rec, f, bodyIdx);

			// The poses are computed in model space (after the import scale +
			// rotation). Unwind them back to the original FBX space so the
			// exported model keeps its original size and orientation.
			buildImportRotation();
			std::vector<float> origPos(3 * nBones, 0.0f);
			std::vector<float> origMat(16 * nBones, 0.0f);
			for (int i = 0; i < nBones; ++i) {
				const float *pos = &m_pose.bonePos[3 * i];
				const float *W = &m_pose.boneWorldMat[16 * i];
				float po[3];
				inverseTransform(pos, po);
				origPos[3 * i + 0] = po[0];
				origPos[3 * i + 1] = po[1];
				origPos[3 * i + 2] = po[2];

				// R_orig = R^T * R_model * R. A world rotation computed in the
				// model frame must be CONJUGATED into the source FBX frame
				// (model = R_import * raw), so left-multiplying by R^T alone is
				// not enough: it would bake a R_import^-1 roll into every bone.
				float Ro[3][3];
				for (int r = 0; r < 3; ++r)
					for (int c = 0; c < 3; ++c) {
						double acc = 0.0;
						for (int a = 0; a < 3; ++a) {
							double wc = m_importRotMatT[r][0] * W[0 + a] +
										m_importRotMatT[r][1] * W[4 + a] +
										m_importRotMatT[r][2] * W[8 + a];
							acc += wc * m_importRotMat[a][c];
						}
						Ro[r][c] = (float)acc;
					}

				// The FBX replays a bone's world position as parentRot * (child
				// offset). The retarget FK positions come from the source's
				// STATIC bone orientations (restPos = R * scale * rawGlobalT),
				// so the exported world rotation must carry the static rest
				// orientation as well - else the offsets recompose a different
				// rest pose and every rolled bone twists when the file is
				// imported by a third-party tool (Blender). The static world
				// rotation is applied TRANSPOSED because the FBX SDK applies
				// the parent rotation to the child offset in column-vector
				// sense while Ro is stored row-major (verified by round-trip).
				const float *SR = &m_rawRestRot[9 * i];
				float S[3][3] = { { SR[0], SR[1], SR[2] },
								  { SR[3], SR[4], SR[5] },
								  { SR[6], SR[7], SR[8] } };
				float Sfull[3][3];
				for (int r = 0; r < 3; ++r)
					for (int c = 0; c < 3; ++c)
						Sfull[r][c] = Ro[r][0] * S[c][0] +
									  Ro[r][1] * S[c][1] +
									  Ro[r][2] * S[c][2];
				memcpy(Ro, Sfull, sizeof(Ro));
				origMat[16 * i + 0] = Ro[0][0]; origMat[16 * i + 1] = Ro[0][1]; origMat[16 * i + 2] = Ro[0][2];
			origMat[16 * i + 4] = Ro[1][0]; origMat[16 * i + 5] = Ro[1][1]; origMat[16 * i + 6] = Ro[1][2];
			origMat[16 * i + 8] = Ro[2][0]; origMat[16 * i + 9] = Ro[2][1]; origMat[16 * i + 10] = Ro[2][2];
			origMat[16 * i + 15] = 1.0f;
			}

			INT64 frameTime = frame.timeMS - baseTime + 1;
			FbxTime time;
			time.SetMilliSeconds(frameTime);

			for (int i = 0; i < nBones; ++i) {
				FbxNode *node = boneNodes[i];
				const float *pos = &origPos[3 * i];
				const float *W = &origMat[16 * i];

				float R[3][3] = { { W[0], W[1], W[2] },
								  { W[4], W[5], W[6] },
								  { W[8], W[9], W[10] } };
				int p = m_bones[i].parent;

				// Local rotation = R_parent^T * R_bone, computed in MATRIX form
				// (row-major, column-vector convention) so no quaternion
				// half-turn / multiplication convention can inject a small
				// screw along the arm axis. The quaternion round-trip used
				// to add up to ~1 deg on fully swung limbs; the matrix path
				// reconstructs the world pose to <0.05 deg in the SDK.
				float L[3][3];
				if (p >= 0) {
					const float *PW = &origMat[16 * p];
					float PR[3][3] = { { PW[0], PW[1], PW[2] },
									   { PW[4], PW[5], PW[6] },
									   { PW[8], PW[9], PW[10] } };
					// L = PR^T * R
					for (int r = 0; r < 3; ++r)
						for (int c = 0; c < 3; ++c)
							L[r][c] = PR[0][r] * R[0][c] + PR[1][r] * R[1][c] + PR[2][r] * R[2][c];
				} else {
					// Root: its parent is the static armature holder. Local =
					// parent^-1 * world, exactly like the PR^T * R branch
					// above (for a rotation-only parent the inverse is the
					// transpose). The holder often carries a scale (e.g. a
					// Blender armature node at 100x): strip it explicitly,
					// otherwise it leaks into L and corrupts quatFromMatrix
					// below. The FbxAMatrix is only storage here; the math is
					// written out explicitly so no SDK decomposition
					// convention can interfere. Translation is handled
					// separately through rootParentInvs and stays untouched.
					FbxAMatrix rw = rootParentWorlds[i];
					double pw[3][3];
					for (int r = 0; r < 3; ++r)
						for (int c = 0; c < 3; ++c)
							pw[r][c] = rw[r][c];
					// Column norms = holder scale (uniform on the rigs
					// handled here); guard against a degenerate parent.
					double sc[3];
					for (int c = 0; c < 3; ++c) {
						sc[c] = sqrt(pw[0][c] * pw[0][c] + pw[1][c] * pw[1][c] + pw[2][c] * pw[2][c]);
						if (!(sc[c] > 1e-12)) sc[c] = 1.0;
					}
					for (int r = 0; r < 3; ++r)
						for (int c = 0; c < 3; ++c) {
							double acc = 0.0;
							for (int a = 0; a < 3; ++a)
								acc += (pw[a][r] / sc[r]) * R[a][c];
							L[r][c] = (float)acc;
						}
				}
				double ql[4];
				quatFromMatrix(L, ql);

				// A bone that carries a static PreRotation/PostRotation has
				// them multiplied around LclRotation again by every importer
				// (FBX transform = T * ... * Pre * R * Post * ... * S), so the
				// keyed euler has to be solved for the other way round:
				// euler = Pre^-1 * L * Post^-1. Identity pre/post (the usual
				// case) leaves L untouched. Verified round-trip: 0.00 deg on
				// a rig with PreRotation, unchanged on the three without.
				{
					FbxAMatrix lm, pre, post, solved;
					lm.SetQ(FbxQuaternion(ql[0], ql[1], ql[2], ql[3]));
					pre.SetR(node->PreRotation.Get());
					post.SetR(node->PostRotation.Get());
					solved = pre.Inverse() * lm * post.Inverse();
					FbxQuaternion qo = solved.GetQ();
					ql[0] = qo[0]; ql[1] = qo[1]; ql[2] = qo[2]; ql[3] = qo[3];
				}
				FbxQuaternion qLocal(ql[0], ql[1], ql[2], ql[3]);

				// Express the rotation as XYZ euler. The FBX SDK ALWAYS
				// interprets a node's LclRotation with the eEulerXYZ order,
				// whatever rotation order the node carries (verified: the SDK's
				// EvaluateGlobalTransform returns 0 deg error for XYZ and 14-30
				// deg for every other order). Re-encoding with the node's
				// stored order would corrupt every non-XYZ bone (arms twisted
				// ~75 deg). Force the stored order to XYZ as well so third-party
				// importers (Blender honors the stored order) agree with the SDK.
				FbxEuler::EOrder order = FbxEuler::eEulerXYZ;
				node->SetRotationOrder(FbxNode::eSourcePivot, order);

				// The pre/post inversion above is applied to the rotation only;
				// the file's own static values (LclRotation, PreRotation,
				// PostRotation) are never overwritten.
				FbxAMatrix comp;
				comp.SetQ(qLocal);
				FbxVector4 euler = comp.GetR((int)order);

				// Keep euler rotation continuous across frames
				float ex = (float)euler[0], ey = (float)euler[1], ez = (float)euler[2];
				if (hasPrev[i]) {
					const float *pe = &prevRot[3 * i];
					while (ex - pe[0] > 180.0f) ex -= 360.0f;
					while (ex - pe[0] < -180.0f) ex += 360.0f;
					while (ey - pe[1] > 180.0f) ey -= 360.0f;
					while (ey - pe[1] < -180.0f) ey += 360.0f;
					while (ez - pe[2] > 180.0f) ez -= 360.0f;
					while (ez - pe[2] < -180.0f) ez += 360.0f;
				}
				prevRot[3 * i + 0] = ex;
				prevRot[3 * i + 1] = ey;
				prevRot[3 * i + 2] = ez;
				hasPrev[i] = true;

				// Every bone animates rotation. Only the root translates:
				// non-root bones keep their constant rest offsets (the FK
				// preserves bone lengths), which keeps the file small.
				FbxDouble3 rot(ex, ey, ez);
				applyTransformation(animLayer, node, rot, time, false);
				if (p < 0) {
					FbxVector4 localPos = rootParentInvs[i].MultT(FbxVector4(pos[0], pos[1], pos[2]));
					FbxDouble3 trans(localPos[0], localPos[1], localPos[2]);
					applyTransformation(animLayer, node, trans, time, true);
				}
			}
		}

		int lFileFormat = -1;
		int wFormat = mgr->GetIOPluginRegistry()->FindWriterIDByDescription("FBX binary (*.fbx)");
		if (wFormat >= 0)
			lFileFormat = wFormat;

		ok = SaveScene(mgr, scene, outPath, lFileFormat, false);
	}

	scene->Destroy();
	mgr->Destroy();

	if (ok)
		UI_Printf("Retargeted take exported to %s.", outPath);
	else
		UI_Printf("Failed to export retargeted take to %s.", outPath);
	return ok;
}

// Decomposes a row-major rotation matrix into ZXY euler angles (BVH channel
// order "Zrotation Xrotation Yrotation"): R = Rz(z) * Rx(x) * Ry(y).
// Returns the angles in degrees.
static void eulerZXY(const float R[3][3], float &z, float &x, float &y) {
	float sy = R[2][1];
	if (sy > 1.0f) sy = 1.0f;
	if (sy < -1.0f) sy = -1.0f;
	x = asinf(sy);
	float cz = sqrtf(1.0f - sy * sy);
	// Guard against a degenerate case (gimbal lock)
	if (cz < 1e-6f) {
		z = 0.0f;
		y = atan2f(R[1][0], R[0][0]);
	} else {
		z = atan2f(-R[0][1], R[1][1]);
		y = atan2f(-R[2][0], R[2][2]);
	}
	z *= 180.0f / 3.14159265f;
	x *= 180.0f / 3.14159265f;
	y *= 180.0f / 3.14159265f;
}

bool KRetargeter::exportRetargetBVH(const char *outPath) {
	if (m_bones.empty())
		return false;
	if (!g_pExporter || g_pExporter->recording().frames.empty()) {
		UI_Printf("Nothing recorded yet. Press Record before exporting.");
		return false;
	}

	FILE *fp = NULL;
	fopen_s(&fp, outPath, "w");
	if (!fp) {
		UI_Printf("Failed to open %s for writing.", outPath);
		return false;
	}

	int nBones = (int)m_bones.size();

	// Parent indices come from the imported hierarchy. A malformed model can
	// carry an out-of-range value, and these are used as array indexes below,
	// so normalise them into a local table without touching the model.
	std::vector<int> parentOf(nBones, -1);
	for (int i = 0; i < nBones; ++i)
		if (m_bones[i].parent >= 0 && m_bones[i].parent < nBones)
			parentOf[i] = m_bones[i].parent;

	int root = -1;
	for (int i = 0; i < nBones; ++i)
		if (parentOf[i] < 0) { root = i; break; }
	if (root < 0)
		root = 0;

	// Children lists + pre-order traversal (BVH data is written in this order)
	std::vector<std::vector<int> > children(nBones);
	for (int i = 0; i < nBones; ++i)
		if (parentOf[i] >= 0)
			children[parentOf[i]].push_back(i);

	std::vector<int> order;
	std::vector<int> stack;
	stack.push_back(root);
	while (!stack.empty()) {
		int idx = stack.back();
		stack.pop_back();
		order.push_back(idx);
		// Push children in reverse so they come out in order
		for (size_t c = children[idx].size(); c > 0; --c)
			stack.push_back(children[idx][c - 1]);
	}

	// Collect frames that contain at least one tracked body
	std::vector<int> frameIdxs;
	const KRecording &rec = g_pExporter->recording();
	for (size_t f = 0; f < rec.frames.size(); ++f)
		if (!rec.frames[f].bodies.empty())
			frameIdxs.push_back((int)f);
	if (frameIdxs.empty()) {
		fclose(fp);
		UI_Printf("No tracked frames in the recording.");
		return false;
	}

	// ---- HIERARCHY ----
	fprintf(fp, "HIERARCHY\n");
	std::vector<int> depth(nBones, 0);
	for (size_t oi = 0; oi < order.size(); ++oi) {
		int idx = order[oi];
		int d = (idx == root) ? 0 : (depth[parentOf[idx]] + 1);
		depth[idx] = d;
		std::string ind(d, '\t');

		float ox = m_bones[idx].restPos[0];
		float oy = m_bones[idx].restPos[1];
		float oz = m_bones[idx].restPos[2];
		if (parentOf[idx] >= 0) {
			ox -= m_bones[parentOf[idx]].restPos[0];
			oy -= m_bones[parentOf[idx]].restPos[1];
			oz -= m_bones[parentOf[idx]].restPos[2];
		}

		fprintf(fp, "%s%s %s\n", ind.c_str(),
			(idx == root) ? "ROOT" : "JOINT", m_bones[idx].name.c_str());
		fprintf(fp, "%s{\n", ind.c_str());
		fprintf(fp, "%s\tOFFSET %.6f %.6f %.6f\n", ind.c_str(), ox, oy, oz);
		fprintf(fp, "%s\tCHANNELS %d", ind.c_str(), (idx == root) ? 6 : 3);
		if (idx == root)
			fprintf(fp, " Xposition Yposition Zposition");
		fprintf(fp, " Zrotation Xrotation Yrotation\n");

		if (children[idx].empty()) {
			fprintf(fp, "%s\tEnd Site\n", ind.c_str());
			fprintf(fp, "%s\t{\n", ind.c_str());
			fprintf(fp, "%s\t\tOFFSET 0 0 0\n", ind.c_str());
			fprintf(fp, "%s\t}\n", ind.c_str());
		}
	}
	for (size_t oi = order.size(); oi > 0; --oi) {
		int idx = order[oi - 1];
		std::string ind(depth[idx], '\t');
		fprintf(fp, "%s}\n", ind.c_str());
	}

	// ---- MOTION ----
	double fps = 30.0;
	if (frameIdxs.size() > 1) {
		double dt = (double)(rec.frames[frameIdxs[1]].timeMS -
			rec.frames[frameIdxs[0]].timeMS) / 1000.0;
		if (dt > 1e-6)
			fps = 1.0 / dt;
	}
	fprintf(fp, "MOTION\n");
	fprintf(fp, "Frames: %d\n", (int)frameIdxs.size());
	fprintf(fp, "Frame Time: %.6f\n", 1.0 / fps);

	// Per-channel euler continuity (unwind > 180 deg jumps)
	std::vector<float> prevE(3 * nBones, 0.0f);
	std::vector<bool> hasPrev(nBones, false);

	buildImportRotation();
	resetMotionAnchor();

	for (size_t fi = 0; fi < frameIdxs.size(); ++fi) {
		const KSampleFrame &frame = rec.frames[frameIdxs[fi]];

		int bodyIdx = 0;
		int bestTracked = -1;
		for (size_t b = 0; b < frame.bodies.size(); ++b) {
			int tracked = 0;
			for (int j = 0; j < JointType_Count; ++j)
				if (frame.bodies[b].joints[j].trackingState != TrackingState_NotTracked)
					tracked++;
			if (tracked > bestTracked) { bestTracked = tracked; bodyIdx = (int)b; }
		}
			poseFinal(rec, (int)frameIdxs[fi], bodyIdx);

			// Root translation (model space, same frame as the hierarchy offsets)
		const float *rpos = &m_pose.bonePos[3 * root];

		// Pre-order euler angles (local, ZXY)
		std::vector<float> eul(3 * nBones, 0.0f);
		for (size_t oi = 0; oi < order.size(); ++oi) {
			int i = order[oi];
			const float *W = &m_pose.boneWorldMat[16 * i];
			float R[3][3] = {
				{ W[0], W[1], W[2] },
				{ W[4], W[5], W[6] },
				{ W[8], W[9], W[10] },
			};
			float L[3][3];
			int p = parentOf[i];
			if (p >= 0) {
				const float *PW = &m_pose.boneWorldMat[16 * p];
				float PR[3][3] = {
					{ PW[0], PW[1], PW[2] },
					{ PW[4], PW[5], PW[6] },
					{ PW[8], PW[9], PW[10] },
				};
				// L = PR^T * R
				for (int r = 0; r < 3; ++r)
					for (int c = 0; c < 3; ++c)
						L[r][c] = PR[0][r] * R[0][c] +
								  PR[1][r] * R[1][c] +
								  PR[2][r] * R[2][c];
			} else {
				for (int r = 0; r < 3; ++r)
					for (int c = 0; c < 3; ++c)
						L[r][c] = R[r][c];
			}

			float z, x, y;
			eulerZXY(L, z, x, y);
			if (hasPrev[i]) {
				float *pe = &prevE[3 * i];
				while (z - pe[0] > 180.0f) z -= 360.0f;
				while (z - pe[0] < -180.0f) z += 360.0f;
				while (x - pe[1] > 180.0f) x -= 360.0f;
				while (x - pe[1] < -180.0f) x += 360.0f;
				while (y - pe[2] > 180.0f) y -= 360.0f;
				while (y - pe[2] < -180.0f) y += 360.0f;
			}
			prevE[3 * i + 0] = z;
			prevE[3 * i + 1] = x;
			prevE[3 * i + 2] = y;
			eul[3 * i + 0] = z;
			eul[3 * i + 1] = x;
			eul[3 * i + 2] = y;
		}

		fprintf(fp, "%.6f %.6f %.6f", rpos[0], rpos[1], rpos[2]);
		for (size_t oi = 0; oi < order.size(); ++oi) {
			int i = order[oi];
			fprintf(fp, " %.6f %.6f %.6f",
				eul[3 * i + 0], eul[3 * i + 1], eul[3 * i + 2]);
		}
		fprintf(fp, "\n");
	}

	fclose(fp);
	UI_Printf("Retargeted take exported to %s.", outPath);
	return true;
}

bool KRetargeter::exportRetarget(const char *outPath) {
	return exportRetargetToFile(outPath);
}
