#include "KNodeGraph.h"

#include <cmath>

#include <cmath>

// ----------------------------------------------------------------------------
// Node metadata (left details panel)
// ----------------------------------------------------------------------------

const char* MotionNodeTypeName(MotionNodeType t)
{
	switch (t)
	{
	case MotionNodeType::Output:   return "Output";
	case MotionNodeType::Input:    return "Input";
	case MotionNodeType::Smooth:   return "Smooth";
	case MotionNodeType::Multiply: return "Multiply";
	case MotionNodeType::Add:      return "Add";
	case MotionNodeType::FootLock: return "Foot Lock";
	case MotionNodeType::Clamp:    return "Clamp";
	case MotionNodeType::Mirror:   return "Mirror";
	case MotionNodeType::Offset:   return "Offset";
	case MotionNodeType::TimeScale: return "TimeScale";
	case MotionNodeType::Reverse:  return "Reverse";
	case MotionNodeType::FloorSnap: return "FloorSnap";
	case MotionNodeType::Noise:    return "Noise";
	case MotionNodeType::Spring:   return "Spring";
	case MotionNodeType::Aim:      return "Aim";
	case MotionNodeType::Mask:     return "Mask";
	default:                        return "?";
	}
}

bool MotionNodeHasInputPin(MotionNodeType t)
{
	return t != MotionNodeType::Output;
}

bool MotionNodeHasOutputPin(MotionNodeType t)
{
	return t != MotionNodeType::Input;
}

bool MotionNodeHasBoneArray(MotionNodeType t)
{
	// Every processing node targets bones, except Reverse (whole take).
	return t != MotionNodeType::Output && t != MotionNodeType::Input
		&& t != MotionNodeType::Reverse;
}

int MotionNodeParamCount(MotionNodeType t)
{
	switch (t)
	{
	case MotionNodeType::Smooth:   return 2;
	case MotionNodeType::Multiply: return 1;
	case MotionNodeType::Add:      return 3;
	case MotionNodeType::FootLock: return 4;
	case MotionNodeType::Clamp:    return 6;
	case MotionNodeType::Mirror:   return 1;
	case MotionNodeType::Offset:   return 3;
	case MotionNodeType::TimeScale: return 1;
	case MotionNodeType::FloorSnap: return 1;
	case MotionNodeType::Noise:    return 2;
	case MotionNodeType::Spring:   return 3;
	case MotionNodeType::Aim:      return 3;
	case MotionNodeType::Mask:     return 1;
	default:                        return 0;
	}
}

MotionParamDef MotionNodeParamDef(MotionNodeType t, int idx)
{
	// Smooth: strength 0..1 (def 0.5), window 1..30 frames (def 8)
	// Multiply: factor 0..3 (def 1.0)
	if (t == MotionNodeType::Smooth && idx == 0)
	{
		MotionParamDef d = { "Strength", 0.0f, 1.0f, 0.5f, false };
		return d;
	}
	if (t == MotionNodeType::Smooth && idx == 1)
	{
		MotionParamDef d = { "Window (frames)", 1.0f, 30.0f, 8.0f, true };
		return d;
	}
	if (t == MotionNodeType::Multiply && idx == 0)
	{
		MotionParamDef d = { "Factor", 0.0f, 3.0f, 1.0f, false };
		return d;
	}
	// Add: constant rotation offset in degrees, one slider per axis.
	if (t == MotionNodeType::Add && idx >= 0 && idx <= 2)
	{
		MotionParamDef d = { (idx == 0) ? "Rot X (deg)" : ((idx == 1) ? "Rot Y (deg)" : "Rot Z (deg)"),
			0.0f, 360.0f, 0.0f, false };
		return d;
	}
	// Foot Lock: contact height (m), contact speed (m/s), debounce frames,
	// engage/release blend frames.
	if (t == MotionNodeType::FootLock && idx == 0)
	{
		MotionParamDef d = { "Height (m)", 0.02f, 0.30f, 0.10f, false };
		return d;
	}
	if (t == MotionNodeType::FootLock && idx == 1)
	{
		MotionParamDef d = { "Speed (m/s)", 0.10f, 3.0f, 0.80f, false };
		return d;
	}
	if (t == MotionNodeType::FootLock && idx == 2)
	{
		MotionParamDef d = { "Min frames", 1.0f, 10.0f, 3.0f, true };
		return d;
	}
	if (t == MotionNodeType::FootLock && idx == 3)
	{
		MotionParamDef d = { "Blend (frames)", 1.0f, 15.0f, 5.0f, true };
		return d;
	}
	// Clamp: per-axis euler limits in degrees (defaults = no-op).
	if (t == MotionNodeType::Clamp && idx >= 0 && idx <= 5)
	{
		static const char *labels[6] = { "Min X", "Max X", "Min Y", "Max Y", "Min Z", "Max Z" };
		static const float defs[6] = { -180.0f, 180.0f, -180.0f, 180.0f, -180.0f, 180.0f };
		MotionParamDef d = { labels[idx], -180.0f, 180.0f, defs[idx], false };
		return d;
	}
	// Mirror: cross-swap left/right bones (else mirrored limbs stay crossed).
	if (t == MotionNodeType::Mirror && idx == 0)
	{
		MotionParamDef d = { "Swap sides", 0.0f, 1.0f, 1.0f, true };
		return d;
	}
	// Offset: constant translation added to targeted bones (m).
	if (t == MotionNodeType::Offset && idx >= 0 && idx <= 2)
	{
		MotionParamDef d = { (idx == 0) ? "dX (m)" : ((idx == 1) ? "dY (m)" : "dZ (m)"),
			-1.0f, 1.0f, 0.0f, false };
		return d;
	}
	// TimeScale: playback rate of the upstream motion (>1 = faster).
	if (t == MotionNodeType::TimeScale && idx == 0)
	{
		MotionParamDef d = { "Rate", 0.10f, 3.0f, 1.0f, false };
		return d;
	}
	// FloorSnap: how strongly the lowest foot is pulled to y=0.
	if (t == MotionNodeType::FloorSnap && idx == 0)
	{
		MotionParamDef d = { "Amount", 0.0f, 1.0f, 1.0f, false };
		return d;
	}
	// Noise: procedural wobble (scrub-safe sines).
	if (t == MotionNodeType::Noise && idx == 0)
	{
		MotionParamDef d = { "Amp (deg)", 0.0f, 10.0f, 1.5f, false };
		return d;
	}
	if (t == MotionNodeType::Noise && idx == 1)
	{
		MotionParamDef d = { "Speed", 0.10f, 3.0f, 1.0f, false };
		return d;
	}
	// Spring: underdamped follow (stiffness, damping ratio, sim window).
	if (t == MotionNodeType::Spring && idx == 0)
	{
		MotionParamDef d = { "Stiffness", 0.5f, 8.0f, 3.0f, false };
		return d;
	}
	if (t == MotionNodeType::Spring && idx == 1)
	{
		MotionParamDef d = { "Damping", 0.05f, 1.0f, 0.35f, false };
		return d;
	}
	if (t == MotionNodeType::Spring && idx == 2)
	{
		MotionParamDef d = { "Window", 10.0f, 60.0f, 30.0f, true };
		return d;
	}
	// Aim: point bones toward a yaw/pitch world direction.
	if (t == MotionNodeType::Aim && idx == 0)
	{
		MotionParamDef d = { "Yaw (deg)", -180.0f, 180.0f, 0.0f, false };
		return d;
	}
	if (t == MotionNodeType::Aim && idx == 1)
	{
		MotionParamDef d = { "Pitch (deg)", -90.0f, 90.0f, 0.0f, false };
		return d;
	}
	if (t == MotionNodeType::Aim && idx == 2)
	{
		MotionParamDef d = { "Blend", 0.0f, 1.0f, 1.0f, false };
		return d;
	}
	// Mask: mixes the chain pose with the raw take pose on masked bones.
	if (t == MotionNodeType::Mask && idx == 0)
	{
		MotionParamDef d = { "Raw weight", 0.0f, 1.0f, 0.5f, false };
		return d;
	}
	MotionParamDef d = { "?", 0.0f, 1.0f, 0.0f, false };
	return d;
}

float MotionNodeParamGet(const MotionNode &n, int idx)
{
	MotionParamDef d = MotionNodeParamDef(n.type, idx);
	if (idx < 0 || idx >= (int)n.params.size())
		return d.defVal;   // Missing entry (older graph): definition default
	return n.params[idx];
}

void MotionNodeParamSet(MotionNode &n, int idx, float v)
{
	if (idx < 0)
		return;
	MotionParamDef d = MotionNodeParamDef(n.type, idx);
	if (v < d.minVal) v = d.minVal;
	if (v > d.maxVal) v = d.maxVal;
	if (d.isInt) v = floorf(v + 0.5f);
	if ((int)n.params.size() <= idx)
		n.params.resize((size_t)idx + 1, 0.0f);
	n.params[idx] = v;
}

static void MotionNodeSetDefaults(MotionNode &n)
{
	int c = MotionNodeParamCount(n.type);
	n.params.assign((size_t)(c > 0 ? c : 0), 0.0f);
	for (int i = 0; i < c; ++i)
		n.params[i] = MotionNodeParamDef(n.type, i).defVal;
}

// ----------------------------------------------------------------------------
// Quaternion helpers (row-major matrices, quats as x,y,z,w)
// ----------------------------------------------------------------------------

void motionQuatFromMat3(const float m[3][3], float q[4])
{
	double tr = (double)m[0][0] + (double)m[1][1] + (double)m[2][2];
	double s, n;
	if (tr > 0.0)
	{
		s = sqrt(tr + 1.0) * 2.0;
		q[0] = (float)((m[2][1] - m[1][2]) / s);
		q[1] = (float)((m[0][2] - m[2][0]) / s);
		q[2] = (float)((m[1][0] - m[0][1]) / s);
		q[3] = (float)(0.25 * s);
	}
	else if (m[0][0] > m[1][1] && m[0][0] > m[2][2])
	{
		s = sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0;
		q[0] = (float)(0.25 * s);
		q[1] = (float)((m[0][1] + m[1][0]) / s);
		q[2] = (float)((m[0][2] + m[2][0]) / s);
		q[3] = (float)((m[2][1] - m[1][2]) / s);
	}
	else if (m[1][1] > m[2][2])
	{
		s = sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0;
		q[0] = (float)((m[0][1] + m[1][0]) / s);
		q[1] = (float)(0.25 * s);
		q[2] = (float)((m[1][2] + m[2][1]) / s);
		q[3] = (float)((m[0][2] - m[2][0]) / s);
	}
	else
	{
		s = sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0;
		q[0] = (float)((m[0][2] + m[2][0]) / s);
		q[1] = (float)((m[1][2] + m[2][1]) / s);
		q[2] = (float)(0.25 * s);
		q[3] = (float)((m[1][0] - m[0][1]) / s);
	}
	n = sqrt((double)q[0] * q[0] + (double)q[1] * q[1] + (double)q[2] * q[2] + (double)q[3] * q[3]);
	if (n > 1e-12)
	{
		q[0] = (float)(q[0] / n); q[1] = (float)(q[1] / n);
		q[2] = (float)(q[2] / n); q[3] = (float)(q[3] / n);
	}
	else
	{
		q[0] = 0.0f; q[1] = 0.0f; q[2] = 0.0f; q[3] = 1.0f;
	}
}

void motionMat3FromQuat(const float q[4], float m[3][3])
{
	double x = q[0], y = q[1], z = q[2], w = q[3];
	double n = sqrt(x * x + y * y + z * z + w * w);
	if (!(n > 1e-12)) { m[0][0] = 1; m[0][1] = 0; m[0][2] = 0; m[1][0] = 0; m[1][1] = 1; m[1][2] = 0; m[2][0] = 0; m[2][1] = 0; m[2][2] = 1; return; }
	x /= n; y /= n; z /= n; w /= n;
	m[0][0] = (float)(1 - 2 * (y * y + z * z));
	m[0][1] = (float)(2 * (x * y - z * w));
	m[0][2] = (float)(2 * (x * z + y * w));
	m[1][0] = (float)(2 * (x * y + z * w));
	m[1][1] = (float)(1 - 2 * (x * x + z * z));
	m[1][2] = (float)(2 * (y * z - x * w));
	m[2][0] = (float)(2 * (x * z - y * w));
	m[2][1] = (float)(2 * (y * z + x * w));
	m[2][2] = (float)(1 - 2 * (x * x + y * y));
}

// Scales a rotation around its own axis: out = q scaled by `factor`
// (factor 1 = identity, 0 = rest orientation, 2 = doubled angle).
void motionQuatScaleAngle(const float q[4], float factor, float out[4])
{
	double x = q[0], y = q[1], z = q[2], w = q[3];
	double n = sqrt(x * x + y * y + z * z + w * w);
	if (!(n > 1e-12)) { out[0] = 0; out[1] = 0; out[2] = 0; out[3] = 1; return; }
	x /= n; y /= n; z /= n; w /= n;
	if (w > 1.0) w = 1.0;
	if (w < -1.0) w = -1.0;
	double ang = 2.0 * acos(w);
	double s = sqrt(x * x + y * y + z * z);
	double ax = 0, ay = 0, az = 0;
	if (s > 1e-9) { ax = x / s; ay = y / s; az = z / s; }
	double a2 = ang * (double)factor;
	double h = sin(a2 * 0.5);
	out[0] = (float)(ax * h); out[1] = (float)(ay * h);
	out[2] = (float)(az * h); out[3] = (float)cos(a2 * 0.5);
}

// Hamilton product: out = a * b (b applied first, column-vector convention).
void motionQuatMul(const float a[4], const float b[4], float out[4])
{
	double ax = a[0], ay = a[1], az = a[2], aw = a[3];
	double bx = b[0], by = b[1], bz = b[2], bw = b[3];
	out[0] = (float)(aw * bx + ax * bw + ay * bz - az * by);
	out[1] = (float)(aw * by - ax * bz + ay * bw + az * bx);
	out[2] = (float)(aw * bz + ax * by - ay * bx + az * bw);
	out[3] = (float)(aw * bw - ax * bx - ay * by - az * bz);
}

// Spherical interpolation (hemisphere-aligned), normalized out.
void motionQuatSlerp(const float a[4], const float b[4], float t, float out[4])
{
	double ax = a[0], ay = a[1], az = a[2], aw = a[3];
	double bx = b[0], by = b[1], bz = b[2], bw = b[3];
	double dot = ax * bx + ay * by + az * bz + aw * bw;
	if (dot < 0.0) { bx = -bx; by = -by; bz = -bz; bw = -bw; dot = -dot; }
	if (dot > 0.999999) dot = 0.999999;
	if (dot < -0.999999) dot = -0.999999;
	if (t <= 0.0) { out[0] = (float)ax; out[1] = (float)ay; out[2] = (float)az; out[3] = (float)aw; return; }
	if (t >= 1.0) { out[0] = (float)bx; out[1] = (float)by; out[2] = (float)bz; out[3] = (float)bw; return; }
	double th = acos(dot);
	double s = sin(th);
	if (!(s > 1e-9))
	{
		out[0] = (float)ax; out[1] = (float)ay; out[2] = (float)az; out[3] = (float)aw;
		return;
	}
	double wa = sin((1.0 - t) * th) / s;
	double wb = sin(t * th) / s;
	out[0] = (float)(ax * wa + bx * wb); out[1] = (float)(ay * wa + by * wb);
	out[2] = (float)(az * wa + bz * wb); out[3] = (float)(aw * wa + bw * wb);
}

// XYZ euler (degrees, R = Rz*Ry*Rx, same convention as the FBX layer).
void motionQuatToEulerXYZ(const float q[4], float e[3])
{
	float m[3][3];
	motionMat3FromQuat(q, m);
	double sy = -(double)m[2][0];
	if (sy > 1.0) sy = 1.0;
	if (sy < -1.0) sy = -1.0;
	const double RAD2DEG = 180.0 / 3.14159265358979323846;
	e[1] = (float)(asin(sy) * RAD2DEG);
	if (fabs(sy) < 0.999999) {
		e[0] = (float)(atan2((double)m[2][1], (double)m[2][2]) * RAD2DEG);
		e[2] = (float)(atan2((double)m[1][0], (double)m[0][0]) * RAD2DEG);
	} else {
		// Gimbal lock: fold the twist into Z, keep X at zero.
		e[0] = 0.0f;
		e[2] = (float)(atan2(-(double)m[1][2], (double)m[1][1]) * RAD2DEG);
	}
}

void motionEulerXYZToQuat(const float e[3], float q[4])
{
	const double DEG2RAD = 3.14159265358979323846 / 180.0;
	double x = (double)e[0] * DEG2RAD, y = (double)e[1] * DEG2RAD, z = (double)e[2] * DEG2RAD;
	double cx = cos(x * 0.5), sx = sin(x * 0.5);
	double cy = cos(y * 0.5), sy = sin(y * 0.5);
	double cz = cos(z * 0.5), sz = sin(z * 0.5);
	// q = qz * qy * qx (column-vector convention, R = Rz*Ry*Rx).
	float qx[4] = { (float)sx, 0.0f, 0.0f, (float)cx };
	float qy[4] = { 0.0f, (float)sy, 0.0f, (float)cy };
	float qz[4] = { 0.0f, 0.0f, (float)sz, (float)cz };
	float t[4], o[4];
	motionQuatMul(qy, qx, t);
	motionQuatMul(qz, t, o);
	q[0] = o[0]; q[1] = o[1]; q[2] = o[2]; q[3] = o[3];
}

// Hemisphere-aligned weighted mean of quats, normalized out.
void motionQuatAvg(const float *qs, const float *ws, int n, float out[4])
{
	double x = 0, y = 0, z = 0, w = 0, tw = 0;
	for (int i = 0; i < n; ++i)
	{
		double qx = qs[4 * i + 0], qy = qs[4 * i + 1], qz = qs[4 * i + 2], qw = qs[4 * i + 3];
		double wt = ws ? (double)ws[i] : 1.0;
		if (i > 0)
		{
			double dot = qx * x + qy * y + qz * z + qw * w;
			if (dot < 0) { qx = -qx; qy = -qy; qz = -qz; qw = -qw; }
		}
		x += qx * wt; y += qy * wt; z += qz * wt; w += qw * wt;
		tw += wt;
	}
	double l = sqrt(x * x + y * y + z * z + w * w);
	if (l > 1e-12 && tw > 1e-12)
	{
		out[0] = (float)(x / l); out[1] = (float)(y / l);
		out[2] = (float)(z / l); out[3] = (float)(w / l);
	}
	else
	{
		out[0] = 0; out[1] = 0; out[2] = 0; out[3] = 1;
	}
}

void motionLerp3(const float a[3], const float b[3], float t, float out[3])
{
	out[0] = a[0] + (b[0] - a[0]) * t;
	out[1] = a[1] + (b[1] - a[1]) * t;
	out[2] = a[2] + (b[2] - a[2]) * t;
}

bool motionNodeAffectsBone(const MotionNode &n, int bone, int boneCount)
{
	if (bone < 0 || bone >= boneCount)
		return false;
	if (n.bones.empty())
		return true;   // Empty selection = every bone
	for (size_t i = 0; i < n.bones.size(); ++i)
		if (n.bones[i] == bone) return true;
	return false;
}

// ----------------------------------------------------------------------------
// Graph
// ----------------------------------------------------------------------------

KNodeGraph::KNodeGraph()
	: m_nextId(1)
{
	resetDefault();
}

void KNodeGraph::clear()
{
	m_nodes.clear();
	m_links.clear();
	m_nextId = 1;
}

void KNodeGraph::resetDefault()
{
	clear();
	MotionNode out;
	out.id = allocId(); out.type = MotionNodeType::Output;
	out.x = 70.0f; out.y = 220.0f; out.locked = true;
	MotionNodeSetDefaults(out);
	MotionNode in;
	in.id = allocId(); in.type = MotionNodeType::Input;
	in.x = 620.0f; in.y = 220.0f; in.locked = true;
	MotionNodeSetDefaults(in);
	m_nodes.push_back(out);
	m_nodes.push_back(in);
	MotionNodeLink l; l.from = out.id; l.to = in.id;
	m_links.push_back(l);
}

MotionNode* KNodeGraph::find(int id)
{
	for (size_t i = 0; i < m_nodes.size(); ++i)
		if (m_nodes[i].id == id) return &m_nodes[i];
	return NULL;
}

const MotionNode* KNodeGraph::find(int id) const
{
	for (size_t i = 0; i < m_nodes.size(); ++i)
		if (m_nodes[i].id == id) return &m_nodes[i];
	return NULL;
}

MotionNode* KNodeGraph::findByType(MotionNodeType t)
{
	for (size_t i = 0; i < m_nodes.size(); ++i)
		if (m_nodes[i].type == t) return &m_nodes[i];
	return NULL;
}

const MotionNode* KNodeGraph::findByType(MotionNodeType t) const
{
	for (size_t i = 0; i < m_nodes.size(); ++i)
		if (m_nodes[i].type == t) return &m_nodes[i];
	return NULL;
}

int KNodeGraph::addNode(MotionNodeType t, float x, float y)
{
	if (t == MotionNodeType::Output || t == MotionNodeType::Input)
		return -1;   // Singletons, created with the default graph
	if ((int)m_nodes.size() >= 256)
		return -1;
	MotionNode n;
	n.id = allocId(); n.type = t; n.x = x; n.y = y; n.locked = false;
	MotionNodeSetDefaults(n);
	m_nodes.push_back(n);
	return n.id;
}

bool KNodeGraph::removeNode(int id)
{
	MotionNode *n = find(id);
	if (!n || n->locked)
		return false;
	for (size_t i = 0; i < m_nodes.size(); ++i)
		if (m_nodes[i].id == id) { m_nodes.erase(m_nodes.begin() + i); break; }
	for (size_t i = 0; i < m_links.size();)
	{
		if (m_links[i].from == id || m_links[i].to == id)
			m_links.erase(m_links.begin() + i);
		else
			++i;
	}
	return true;
}

bool KNodeGraph::setNodePos(int id, float x, float y)
{
	MotionNode *n = find(id);
	if (!n) return false;
	n->x = x; n->y = y;
	return true;
}

bool KNodeGraph::link(int fromId, int toId)
{
	if (fromId == toId)
		return false;
	MotionNode *from = find(fromId);
	MotionNode *to = find(toId);
	if (!from || !to)
		return false;
	if (!MotionNodeHasOutputPin(from->type) || !MotionNodeHasInputPin(to->type))
		return false;
	// No cycles: `to` must not already reach `from` upstream.
	if (reaches(toId, fromId))
		return false;
	// Single in-pin: replace the existing feeder.
	unlinkTo(toId);
	MotionNodeLink l; l.from = fromId; l.to = toId;
	m_links.push_back(l);
	return true;
}

bool KNodeGraph::unlinkTo(int toId)
{
	bool removed = false;
	for (size_t i = 0; i < m_links.size();)
	{
		if (m_links[i].to == toId) { m_links.erase(m_links.begin() + i); removed = true; }
		else ++i;
	}
	return removed;
}

bool KNodeGraph::removeLink(int fromId, int toId)
{
	for (size_t i = 0; i < m_links.size(); ++i)
		if (m_links[i].from == fromId && m_links[i].to == toId)
		{
			m_links.erase(m_links.begin() + i);
			return true;
		}
	return false;
}

bool KNodeGraph::hasLinkTo(int toId) const
{
	for (size_t i = 0; i < m_links.size(); ++i)
		if (m_links[i].to == toId) return true;
	return false;
}

bool KNodeGraph::linked(int fromId, int toId) const
{
	for (size_t i = 0; i < m_links.size(); ++i)
		if (m_links[i].from == fromId && m_links[i].to == toId) return true;
	return false;
}

// Upstream walk: can evaluation starting at `fromId` (following in-pins)
// reach `targetId`? Used for cycle rejection.
bool KNodeGraph::reaches(int fromId, int targetId) const
{
	int cur = fromId;
	for (int guard = 0; guard < 512; ++guard)
	{
		if (cur == targetId) return true;
		bool stepped = false;
		for (size_t i = 0; i < m_links.size(); ++i)
			if (m_links[i].to == cur) { cur = m_links[i].from; stepped = true; break; }
		if (!stepped) return false;
	}
	return false;
}

bool KNodeGraph::chainFromInput(std::vector<int> &orderOut) const
{
	orderOut.clear();
	const MotionNode *in = findByType(MotionNodeType::Input);
	const MotionNode *out = findByType(MotionNodeType::Output);
	if (!in || !out)
		return false;
	// Walk back from Input to Output.
	std::vector<int> rev;
	int cur = in->id;
	for (int guard = 0; guard < 512; ++guard)
	{
		rev.push_back(cur);
		if (cur == out->id) break;
		bool stepped = false;
		for (size_t i = 0; i < m_links.size(); ++i)
			if (m_links[i].to == cur) { cur = m_links[i].from; stepped = true; break; }
		if (!stepped) return false;   // Broken chain -> T-pose
	}
	if (rev.empty() || rev.back() != out->id)
		return false;
	for (int i = (int)rev.size() - 1; i >= 0; --i)
		orderOut.push_back(rev[i]);
	return true;
}

void KNodeGraph::addLoadedNode(const MotionNode &n)
{
	if ((int)m_nodes.size() >= 512)
		return;
	m_nodes.push_back(n);
	if (n.id >= m_nextId) m_nextId = n.id + 1;
}

void KNodeGraph::addLoadedLink(int fromId, int toId)
{
	if ((int)m_links.size() >= 512)
		return;
	if (!find(fromId) || !find(toId))
		return;
	MotionNodeLink l; l.from = fromId; l.to = toId;
	m_links.push_back(l);
}
