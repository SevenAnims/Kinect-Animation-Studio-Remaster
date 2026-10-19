#pragma once

#include <vector>

// ============================================================================
// Node Motion Editor : non-destructive animation post-process graph.
//
// Data flow: Output node (raw recorded take, invisible) -> processing nodes
// (Smooth, Multiply, ...) -> Input node (final reworked animation, shown in
// the 3D viewports and baked by the FBX/BVH exporters).
// When the Input node has no incoming link, the character falls back to the
// T-pose (poseRest) to show that nothing feeds the output.
// A node acts on its selected bones; an empty selection means every bone.
// ============================================================================

// Node kinds. Output/Input are created with the default graph, locked
// (cannot be deleted) and always present. Explicit values: stored in .kas.
enum class MotionNodeType
{
	Output   = 0,   // Source: raw recorded take (invisible). Out-pin only.
	Input    = 1,   // Sink: final reworked pose (viewports + export). In-pin only.
	Smooth   = 2,   // Temporal smoothing over a frame window.
	Multiply = 3,   // Scales local motion (rotation angle, root translation).
	Add      = 4,   // Constant rotation offset in degrees (X/Y/Z separately).
	FootLock = 5,   // Pins feet to the ground on contact (two-bone IK legs).
	Clamp    = 6,   // Joint limits: clamps local euler angles (deg).
	Mirror   = 7,   // Mirrors the pose across the sagittal plane (+L/R swap).
	Offset   = 8,   // Constant translation offset (m) on targeted bones.
	TimeScale = 9,  // Retimes the upstream motion (rate, interpolated).
	Reverse  = 10,  // Plays the upstream motion backwards (take only).
	FloorSnap = 11, // Shifts the character so the lowest foot touches y=0.
	Noise    = 12,  // Procedural wobble (scrub-safe sines).
	Spring   = 13,  // Underdamped follow spring (secondary motion).
	Aim      = 14,  // Points bones toward a yaw/pitch direction.
	Mask     = 15,  // Mixes the chain pose with the raw take pose per bone.
};

// A single node instance.
struct MotionNode
{
	int            id;       // Unique inside the graph
	MotionNodeType type;
	float          x, y;     // Canvas position (top-left, pixels)
	bool           locked;   // True for Output/Input: cannot be deleted
	std::vector<int> bones;  // Target bone indices, empty = all bones
	// Parameters, indexed by MotionNodeParamDef(type, idx). Sized on creation;
	// missing entries read back as their definition default (forward/backward
	// compatible across graph section versions).
	std::vector<float> params;
};

// A directed link: pose flows from node `from` (out-pin) to node `to` (in-pin).
// v1: every node has a single in-pin, so at most one link targets a node.
struct MotionNodeLink
{
	int from;
	int to;
};

// Pose currency of the graph: local rotations + local translations + root.
// Non-root translations default to rest offsets (rebuilt by FK).
struct MotionNodePose
{
	std::vector<float> localQ;  // 4 * boneCount local quats (x, y, z, w)
	std::vector<float> localT;  // 3 * boneCount local translations (offsets)
	std::vector<float> rootPos; // 3 floats: world position of the root bone
	bool valid;

	MotionNodePose() : valid(false) {}
};

// One ankle-trajectory sample for the Foot Lock contact walk (both legs).
struct FootLockSample
{
	float pl[3], pr[3];   // Ankle world positions, left / right
	float ql[4], qr[4];   // Ankle world orientations
	float dt;             // Seconds since the previous sample
	bool  ok;
};

// Parameter metadata for the left details panel (Unreal-style).
struct MotionParamDef
{
	const char *label;
	float       minVal;
	float       maxVal;
	float       defVal;
	bool        isInt;   // Round to whole numbers (e.g. frame window)
};

const char* MotionNodeTypeName(MotionNodeType t);
bool MotionNodeHasInputPin(MotionNodeType t);    // False for Output
bool MotionNodeHasOutputPin(MotionNodeType t);   // False for Input
bool MotionNodeHasBoneArray(MotionNodeType t);   // False for Output/Input
int  MotionNodeParamCount(MotionNodeType t);     // 0 for Output/Input
MotionParamDef MotionNodeParamDef(MotionNodeType t, int idx);
float MotionNodeParamGet(const MotionNode &n, int idx);
void  MotionNodeParamSet(MotionNode &n, int idx, float v);

// Small quaternion helpers shared by the graph evaluation (KRetargeter).
// Matrices are row-major 3x3, quats are (x, y, z, w).
void motionQuatFromMat3(const float m[3][3], float q[4]);   // Normalized out
void motionMat3FromQuat(const float q[4], float m[3][3]);
void motionQuatScaleAngle(const float q[4], float factor, float out[4]); // q^factor around its axis
void motionQuatMul(const float a[4], const float b[4], float out[4]); // out = a * b (b applied first)
void motionQuatSlerp(const float a[4], const float b[4], float t, float out[4]);
// XYZ euler (degrees, R = Rz*Ry*Rx, same convention as the FBX layer).
void motionQuatToEulerXYZ(const float q[4], float e[3]);
void motionEulerXYZToQuat(const float e[3], float q[4]);
void motionQuatAvg(const float *qs, const float *ws, int n, float out[4]); // Weighted mean, normalized
void motionLerp3(const float a[3], const float b[3], float t, float out[3]);

// True when `bone` is targeted by the node (empty list = every bone).
// Out-of-range indices never match.
bool motionNodeAffectsBone(const MotionNode &n, int bone, int boneCount);

class KNodeGraph
{
public:
	KNodeGraph();

	// Builds the default graph: Output -> Input linked, both locked.
	void resetDefault();
	void clear();

	bool empty() const { return m_nodes.empty(); }

	const std::vector<MotionNode>& nodes() const { return m_nodes; }
	const std::vector<MotionNodeLink>& links() const { return m_links; }

	MotionNode* find(int id);
	const MotionNode* find(int id) const;
	MotionNode* findByType(MotionNodeType t);
	const MotionNode* findByType(MotionNodeType t) const;

	int allocId() { return m_nextId++; }
	int addNode(MotionNodeType t, float x, float y);   // Returns the new id, -1 on failure
	bool removeNode(int id);                            // False for locked/missing nodes
	bool setNodePos(int id, float x, float y);

	// Creates/replaces the single in-pin link of `to`. Rejects self-links,
	// links into Output / out of Input, and links that would close a cycle.
	bool link(int fromId, int toId);
	bool unlinkTo(int toId);                            // Removes the link feeding `to`
	bool removeLink(int fromId, int toId);
	bool hasLinkTo(int toId) const;
	bool linked(int fromId, int toId) const;

	// Walks back from the Input node to the Output node. On success fills
	// `orderOut` with node ids from Output to Input (evaluation order) and
	// returns true. Returns false when the chain is broken (then the caller
	// must fall back to the T-pose).
	bool chainFromInput(std::vector<int> &orderOut) const;

	// Loading support (KASProject): raw insert + id counter restore.
	void addLoadedNode(const MotionNode &n);
	void addLoadedLink(int fromId, int toId);
	void setNextId(int v) { if (v > m_nextId) m_nextId = v; }
	int  nextId() const { return m_nextId; }

private:
	bool reaches(int fromId, int targetId) const;   // Upstream walk: can `fromId` reach `targetId`?

	std::vector<MotionNode>     m_nodes;
	std::vector<MotionNodeLink> m_links;
	int m_nextId;
};
