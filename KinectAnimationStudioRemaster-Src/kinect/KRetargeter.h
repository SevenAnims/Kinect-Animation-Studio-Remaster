#pragma once

#include "..\common\stdafx.h"
#include "KRecording.h"
#include "KNodeGraph.h"
#include <vector>
#include <string>

// A single bone of the imported character skeleton
struct RetargetBone
{
	std::string name;
	int         parent;          // Index of the parent bone, -1 for the root
	float       restPos[3];      // Rest (bind) global position, scene space
	float       restAxis[3];     // Normalized rest direction toward the first child
	float       restLen;         // Distance toward the first child (rest pose)
	int         firstChild;      // Index of the first child bone, -1 when leaf
};

// Skinned mesh extracted from the FBX. Control points are in scene space.
struct RetargetMesh
{
	std::vector<float>                            restPositions; // 3 * vertexCount
	std::vector<unsigned int>                     indices;       // Triangle list
	std::vector<std::vector<std::pair<int, float> > > weights;   // Per vertex: (bone, weight)
	bool valid;

	RetargetMesh() : valid(false) {}
};

// Pose evaluation result for a single frame
struct RetargetPose
{
	std::vector<float> bonePos;      // 3 * boneCount, world positions
	std::vector<float> boneMat;      // 16 * boneCount, skinning matrices (row-major, W*B^-1)
	std::vector<float> boneWorldMat; // 16 * boneCount, world matrices (row-major, T(pos)*R_aim)
	std::vector<float> skinnedPos;   // 3 * vertexCount, skinned vertices
	bool valid;

	RetargetPose() : valid(false) {}
};

// Kinect joints exposed by the mapping UI
struct RetargetJointOption
{
	int   jointType;   // Kinect JointType
	const char *label;
};

// Mappable Kinect joints, in the order shown by the mapping window
extern const RetargetJointOption g_retargetJoints[];
extern const int g_retargetJointCount;

// Reference skeleton (the "ghost") joint positions, in scene space (metres).
// Used as the position reference for the position-based bone auto-mapping.
extern const float g_ghostJointPositions[JointType_Count][3];

/*
  Imported character (FBX skeleton + skinned mesh) + retarget engine.
  Maps recorded Kinect frames onto the imported skeleton and renders / exports it.
*/
class KRetargeter
{
public:
	KRetargeter();
	~KRetargeter();

	/// <summary>
	/// Imports a character FBX (skeleton + skinned mesh). Returns true on success.
	/// </summary>
	bool loadCharacter(const char *fbxPath);

	/// <summary>
	/// Frees the imported character
	/// </summary>
	void clearCharacter();

	/// <summary>
	/// Returns true when a character is loaded
	/// </summary>
	bool hasCharacter() const { return !m_bones.empty(); }

	const char* characterName() const { return m_characterName.c_str(); }
	const char* sourcePath() const { return m_sourcePath.c_str(); }

	int  boneCount() const { return (int)m_bones.size(); }
	const RetargetBone& bone(int i) const { return m_bones[i]; }
	const char* boneName(int i) const { return m_bones[i].name.c_str(); }
	const RetargetMesh& mesh() const { return m_mesh; }

	/// <summary>
	/// Version counter bumped every time the imported mesh data changes
	/// (import, size / rotation edit). Lets GPU-side caches know when to rebuild.
	/// </summary>
	int  meshVersion() const { return m_meshVersion; }

	// ---- Bone mapping ----
	void autoMap();
	void autoMapByPosition();
	void clearMapping();
	void setMapping(int kinectJoint, int boneIndex);
	int  getMapping(int kinectJoint) const;
	int  mappingJointForBone(int boneIndex) const;   // -1 when unmapped

	// ---- Ghost reference pose (editable to fine-tune the position mapping) ----
	/// <summary>
	/// Returns 25x3 joint positions of the ghost reference skeleton
	/// (the pose drawn when the ghost is enabled, used by the auto-mapping).
	/// You can bend it with rotateGhost() to adapt it to the imported skeleton.
	/// </summary>
	const float *ghostPositions() const { return &m_ghostPos[0][0]; }
	const float *ghostPosition(int joint) const {
		return (joint >= 0 && joint < JointType_Count) ? m_ghostPos[joint] : NULL;
	}
	/// <summary>
	/// Restores the default upright T-pose.
	/// </summary>
	void resetGhost();
	/// <summary>
	/// Index of the canonical parent of a Kinect joint (-1 for the root).
	/// This is the pivot used by the ghost rotation gizmo.
	/// </summary>
	int ghostParent(int joint) const;
	/// <summary>
	/// Rotates the subtree rooted at `joint` around the current position of its
	/// parent joint (itself when it is the root). Used by the viewport rotation
	/// gizmo to bend the ghost into the character's proportions.
	/// </summary>
	void rotateGhost(int joint, const float axis[3], float angleRad);

	// ---- Import transform ----
	void setImportScale(float s);
	float importScale() const { return m_importScale; }
	void setImportRotation(float rx, float ry, float rz);
	float importRotX() const { return m_importRotX; }
	float importRotY() const { return m_importRotY; }
	float importRotZ() const { return m_importRotZ; }
	/// <summary>
	/// Re-applies the current scale + rotation to the raw skeleton / mesh data
	/// (used after the user edits the size / orientation values).
	/// </summary>
	void applyImportTransform();

	// ---- Pose evaluation ----
	bool poseFromFrame(const KSampleFrame &frame, int bodyIndex);
	void poseRest();
	const RetargetPose& pose() const { return m_pose; }

	// ---- Node Motion Editor (non-destructive post-process graph) ----
	// The graph reworks the retargeted pose: Output (raw take, invisible) ->
	// processing nodes -> Input (final pose, shown + exported). When the
	// Input node is unlinked, poseFinal*() falls back to the T-pose.
	KNodeGraph& nodeGraph() { return m_nodeGraph; }
	const KNodeGraph& nodeGraph() const { return m_nodeGraph; }
	// Evaluates the graph for a recorded take frame. Leaves the final pose
	// in m_pose (or the T-pose when the chain is broken / frame unusable).
	bool poseFinal(const KRecording &rec, int frameIdx, int bodyIdx);
	// Live variant (no take context): Smooth works over a short history of
	// the live poses instead of the take window.
	bool poseFinalLive(const KSampleBody &body);

	/// <summary>
	/// Forgets the saved motion anchor. The next pose will re-capture the actor's
	/// root position and use it as the reference for the relative retarget.
	/// Call this when a new take starts playing.
	/// </summary>
	void resetMotionAnchor();

	// ---- Export ----
	/// <summary>
	/// Bakes the recorded take onto the imported character and writes a new FBX.
	/// </summary>
	bool exportRetarget(const char *outPath);

	/// <summary>
	/// Bakes the recorded take onto the imported character and writes a BVH
	/// motion file (hierarchy + channels Zrotation Xrotation Yrotation).
	/// </summary>
	bool exportRetargetBVH(const char *outPath);

private:
	std::vector<RetargetBone> m_bones;
	RetargetMesh              m_mesh;
	int                       m_mapping[25];
	float                     m_ghostPos[JointType_Count][3]; // Editable ghost reference pose
	RetargetPose              m_pose;
	KNodeGraph                m_nodeGraph;    // Motion post-process graph (Output -> ... -> Input)
	std::vector<MotionNodePose> m_liveHistory; // Recent live base poses (Smooth window, live mode)
	std::vector<MotionNodePose> m_footUpstream; // Recent live upstream poses (Foot Lock window, live mode)
	long long               m_liveTick;     // Live evaluation clock (Noise node)
	std::string               m_characterName;
	std::string               m_sourcePath;
	float                     m_scale;
	float                     m_anchor[3];   // Last known Kinect spine base (fallback)
	float                     m_motionAnchor[3]; // Spine base of the first posed frame (root reference)
	bool                      m_motionAnchorSet;
	int                       m_rootBone;    // Bone mapped to SpineBase
	bool                      m_scaleReady;
	int                       m_meshVersion; // Bumped when the mesh data changes

	// Import transform (applied on top of the raw FBX data)
	float m_importScale;          // Size multiplier
	float m_importRotX, m_importRotY, m_importRotZ;  // Degrees around X/Y/Z
	float m_importRotMat[3][3];   // R = Rz * Ry * Rx (built from the euler angles)
	float m_importRotMatT[3][3];  // R transposed (inverse rotation)

	// Raw (untransformed) data, kept so the import transform can be re-applied
	// without re-reading the FBX file.
	std::vector<float> m_rawRestPos;  // 3 * boneCount, raw global positions
	std::vector<float> m_rawRestRot;  // 9 * boneCount, raw global rest rotations, ancestor scale stripped (row-major 3x3)
	std::vector<float> m_rawMeshPos;  // 3 * vertexCount, raw positions

	bool  buildSkeleton(FbxScene *scene);
	bool  extractMesh(FbxScene *scene);
	void  computePoseBody(const KSampleBody &body, bool withSkin = true);
	void  computeScale(const KSampleBody &body);
	// Rebuilds m_pose (world / skinning matrices + skinned mesh) from FK
	// results: world positions G and world rotations R9 (row-major 3x3).
	// withSkin=false skips skinning matrices + mesh (world data only, used
	// by intermediate node evaluations for speed).
	void  finalizePoseFK(const float *G, const float *R9, int nBones, bool withSkin);
	// Forward kinematics shared by buildFinalPose and the Foot Lock solver:
	// graph-local pose -> world positions G + world rotations R9.
	void  fkPose(const MotionNodePose &np, std::vector<float> &G, std::vector<float> &R9);
	// Base (unprocessed) pose of one body, as graph currency (local quats).
	bool  basePoseForBody(const KSampleBody &body, MotionNodePose &out);
	// Same, for a take frame, preserving the motion anchor/scale state.
	bool  basePoseForTakeFrame(const KRecording &rec, int frameIdx, int bodyIdx, MotionNodePose &out);
	// Evaluates the node chain up to (excluding) stopId. ownAnchor=true only
	// for the center frame of a take evaluation (the anchor advances there);
	// neighbor/nested evaluations preserve it. TimeScale nodes remap the
	// frame of their upstream prefix (per-node effective frames).
	bool  evalUpstream(const KRecording &rec, int frameIdx, int bodyIdx,
		const std::vector<int> &order, int stopId, bool ownAnchor, MotionNodePose &outPose);
	// Base pose at a fractional take frame (TimeScale rate): blends the two
	// surrounding integer base poses. Anchor-preserving.
	bool  evalBaseFloat(const KRecording &rec, float frameF, int bodyIdx, MotionNodePose &outPose);
	// Fractional-frame variant (TimeScale rate): blends the two surrounding
	// integer evaluations. ownAnchor is always false here.
	bool  evalUpstreamFloat(const KRecording &rec, float frameF, int bodyIdx,
		const std::vector<int> &order, int stopId, MotionNodePose &outPose);
	// Node operators (out = processed copy of center).
	void  smoothBlendPoses(const MotionNode &nd, const MotionNodePose &center,
		const std::vector<MotionNodePose> &win, MotionNodePose &out);
	void  applySmoothNodeTake(const MotionNode &nd, const KRecording &rec, int frameIdx, int bodyIdx,
		const MotionNodePose &center, MotionNodePose &out);
	void  applySmoothNodeLive(const MotionNode &nd, MotionNodePose &out);
	void  applyMultiplyNode(const MotionNode &nd, const MotionNodePose &in, MotionNodePose &out);
	// Constant rotation offset (degrees, X/Y/Z separately) added in parent
	// space: lets limbs be re-aimed for fat/thin morphologies without
	// touching the animation itself (non-destructive).
	void  applyAddNode(const MotionNode &nd, const MotionNodePose &in, MotionNodePose &out);
	// Resolves the thigh/knee/ankle/toe bone chain of one leg (side 0 =
	// left, 1 = right) from the current mapping. False when incomplete.
	bool  legChain(int side, int &thigh, int &knee, int &ankle, int &toe) const;
	// Analytic two-bone IK: places the knee between hip H and ankle target A
	// (lengths L1/L2, natural bend kept via pole). Outputs world rotations.
	static bool solveFootLegIK(const float H[3], const float A[3], float L1, float L2,
		const float restAxisT[3], const float restAxisS[3], const float pole[3],
		float thighW[3][3], float shinW[3][3]);
	// Shared Foot Lock solver: contact walk over the ankle window, pin + IK
	// write-back into io (starts as a copy of the upstream center pose).
	// G0/R90 hold the upstream FK worlds of the center frame.
	void  solveFootLocks(const MotionNode &nd, const std::vector<FootLockSample> &win, int ci,
		const std::vector<float> &G0, const std::vector<float> &R90, MotionNodePose &io);
	// Foot Lock: pins feet on ground contact (two-bone IK legs). Take and
	// live variants; `order` is the evaluation order, `center` the upstream
	// pose of the current frame.
	void  applyFootLockNodeTake(const MotionNode &nd, const KRecording &rec, int frameIdx, int bodyIdx,
		const std::vector<int> &order, const MotionNodePose &center, MotionNodePose &out);
	void  applyFootLockNodeLive(const MotionNode &nd, const MotionNodePose &in, MotionNodePose &out);
	// Full-pose blend (slerp locals, lerp rootPos/localT). Invalid input
	// falls back to the valid side.
	void  blendNodePoses(const MotionNodePose &a, const MotionNodePose &b, float t, MotionNodePose &out);
	// Remaining processing nodes (take + live variants share the helpers).
	void  applyClampNode(const MotionNode &nd, const MotionNodePose &in, MotionNodePose &out);
	void  applyMirrorNode(const MotionNode &nd, const MotionNodePose &in, MotionNodePose &out);
	void  applyOffsetNode(const MotionNode &nd, const MotionNodePose &in, MotionNodePose &out);
	void  applyTimeScaleNode(const MotionNode &nd, const KRecording &rec, float selfFrame, int bodyIdx,
		const std::vector<int> &order, const MotionNodePose &cur, MotionNodePose &out);
	void  applyReverseNodeTake(const MotionNode &nd, const KRecording &rec, float selfFrame, int bodyIdx,
		const std::vector<int> &order, const MotionNodePose &cur, MotionNodePose &out);
	void  applyAimNode(const MotionNode &nd, const MotionNodePose &in, MotionNodePose &out);
	void  applyFloorSnapNode(const MotionNode &nd, const MotionNodePose &in, MotionNodePose &out);
	void  applySpringNodeTake(const MotionNode &nd, const KRecording &rec, int frameIdx, int bodyIdx,
		const std::vector<int> &order, const MotionNodePose &center, MotionNodePose &out);
	void  applySpringNodeLive(const MotionNode &nd, MotionNodePose &out);
	void  applyNoiseNodeTake(const MotionNode &nd, const KRecording &rec, int frameIdx,
		const MotionNodePose &in, MotionNodePose &out);
	void  applyNoiseNodeLive(const MotionNode &nd, const MotionNodePose &in, MotionNodePose &out);
	void  applyMaskNodeTake(const MotionNode &nd, const KRecording &rec, int frameIdx, int bodyIdx,
		const MotionNodePose &in, MotionNodePose &out);
	void  applyMaskNodeLive(const MotionNode &nd, const MotionNodePose &in, MotionNodePose &out);
	// Damped-spring simulation over an upstream window (shared take/live).
	void  springSimWindow(const MotionNode &nd, const std::vector<MotionNodePose> &win,
		const std::vector<float> &dts, const MotionNodePose &center, MotionNodePose &out);
	// Deterministic sine wobble at clock tSec (shared take/live).
	void  applyNoiseOnPose(const MotionNode &nd, float tSec, const MotionNodePose &in, MotionNodePose &out);
	// Forward kinematics: graph-local pose -> m_pose (viewports + export).
	void  buildFinalPose(const MotionNodePose &np);
	void  skinMesh();
	bool  exportRetargetToFile(const char *outPath);

	// Structure-based auto-mapping (pose independent)
	void scanMapByPosition(bool keepMapped);
	void mapByPositionFallback(bool keepMapped);
	void fillMapping(bool keepMapped);
	// Name anchors -> parent/child chain propagation (shoulder -> arm/forearm,
	// upper leg -> lower leg/foot, and so on up and down each limb)
	void propagateFromAnchors();

	// Import transform helpers
	void  buildImportRotation();
	void  transformPoint(const float p[3], float out[3]) const;     // model = R * (scale * p)
	void  inverseTransform(const float p[3], float out[3]) const;   // raw = (1/scale) * R^T * p
	void  finalizeBones();   // Recomputes restLen / restAxis from restPos

	// Matrices
	static void  aimRotation(float R[3][3], const float from[3], const float to[3]);
	static void  transpose3(const float in[3][3], float out[3][3]);
	static void  mulMat3(const float a[3][3], const float b[3][3], float out[3][3]);
	static void  mulMat4(const float a[16], const float b[16], float out[16]);
	static bool  invert4(const float m[16], float out[16]);
};
