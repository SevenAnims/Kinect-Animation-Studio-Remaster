#include "KASProject.h"
#include "KRetargeter.h"
#include "KBodyExporter.h"
#include "KBodyVisualizer.h"
#include "KRecording.h"
#include <cstdio>
#include <cstring>

// Fixed Kinect joint count (Kinect v2). JointType_Count from <Kinect.h>.
static const int kJointCount = 25;

// Kinect v2 never reports more than BODY_COUNT bodies per frame.
static const unsigned int kMaxBodiesPerFrame = 6;

// Smallest possible on-disk size of one recorded frame: the timestamp plus
// the body count (a frame with zero bodies has nothing else).
static const unsigned long long kMinFrameBytes = sizeof(INT64) + sizeof(unsigned int);

// ----------------------------------------------------------------------------
// Small binary helpers
// ----------------------------------------------------------------------------

static bool writeBytes(FILE *f, const void *data, size_t size)
{
	return fwrite(data, 1, size, f) == size;
}

static bool readBytes(FILE *f, void *data, size_t size)
{
	return fread(data, 1, size, f) == size;
}

static bool writeRecording(FILE *f, const KRecording &rec)
{
	unsigned int frameCount = (unsigned int)rec.frames.size();
	if (!writeBytes(f, &frameCount, sizeof(frameCount)))
		return false;

	for (size_t fi = 0; fi < rec.frames.size(); ++fi) {
		const KSampleFrame &frame = rec.frames[fi];
		if (!writeBytes(f, &frame.timeMS, sizeof(frame.timeMS)))
			return false;

		unsigned int bodyCount = (unsigned int)frame.bodies.size();
		if (!writeBytes(f, &bodyCount, sizeof(bodyCount)))
			return false;

		for (size_t b = 0; b < frame.bodies.size(); ++b) {
			const KSampleBody &body = frame.bodies[b];
			if (!writeBytes(f, &body.trackingId, sizeof(body.trackingId)))
				return false;
			int isTracked = body.isTracked ? 1 : 0;
			if (!writeBytes(f, &isTracked, sizeof(isTracked)))
				return false;

			for (int j = 0; j < kJointCount; ++j) {
				if (!writeBytes(f, body.joints[j].position, sizeof(float) * 3))
					return false;
				if (!writeBytes(f, body.joints[j].orientation, sizeof(float) * 4))
					return false;
				int ts = body.joints[j].trackingState;
				if (!writeBytes(f, &ts, sizeof(ts)))
					return false;
			}
		}
	}
	return true;
}

static bool readRecording(FILE *f, KRecording &out)
{
	out.clear();

	// The counts come straight from the file, so a truncated or hand-edited
	// .kas could otherwise make us reserve gigabytes before the first failed
	// read ever reports an error. Sanity-check them against what is actually
	// left in the file first.
	long long fileSize = -1;
	long long startPos = _ftelli64(f);
	if (startPos >= 0)
	{
		if (_fseeki64(f, 0, SEEK_END) == 0)
			fileSize = _ftelli64(f);
		if (_fseeki64(f, startPos, SEEK_SET) != 0)
			return false;
	}

	unsigned int frameCount = 0;
	if (!readBytes(f, &frameCount, sizeof(frameCount)))
		return false;

	if (fileSize >= startPos)
	{
		unsigned long long remaining =
			(unsigned long long)(fileSize - startPos) - sizeof(frameCount);
		if ((unsigned long long)frameCount > remaining / kMinFrameBytes)
			return false;
	}

	out.frames.reserve(frameCount);
	for (unsigned int fi = 0; fi < frameCount; ++fi) {
		KSampleFrame frame;
		if (!readBytes(f, &frame.timeMS, sizeof(frame.timeMS)))
			return false;

		unsigned int bodyCount = 0;
		if (!readBytes(f, &bodyCount, sizeof(bodyCount)))
			return false;
		if (bodyCount > kMaxBodiesPerFrame)
			return false;
		frame.bodies.resize(bodyCount);

		for (unsigned int b = 0; b < bodyCount; ++b) {
			KSampleBody &body = frame.bodies[b];
			if (!readBytes(f, &body.trackingId, sizeof(body.trackingId)))
				return false;
			int isTracked = 0;
			if (!readBytes(f, &isTracked, sizeof(isTracked)))
				return false;
			body.isTracked = isTracked != 0;

			for (int j = 0; j < kJointCount; ++j) {
				if (!readBytes(f, body.joints[j].position, sizeof(float) * 3))
					return false;
				if (!readBytes(f, body.joints[j].orientation, sizeof(float) * 4))
					return false;
				int ts = 0;
				if (!readBytes(f, &ts, sizeof(ts)))
					return false;
				body.joints[j].trackingState = ts;
			}
		}
		out.frames.push_back(frame);
	}
	return true;
}

// ----------------------------------------------------------------------------
// Node motion graph section (v4+): nodes then links. Malformed content never
// fails the project load - the caller falls back to the default graph.
// ----------------------------------------------------------------------------

static bool writeNodeGraph(FILE *f, const KNodeGraph &g)
{
	// Graph section v2: versioned header, then per-node paramCount +
	// params (extensible when new node kinds arrive).
	int graphVersion = 2;
	if (!writeBytes(f, &graphVersion, sizeof(int)))
		return false;
	const std::vector<MotionNode> &nodes = g.nodes();
	const std::vector<MotionNodeLink> &links = g.links();
	int nodeCount = (int)nodes.size();
	if (!writeBytes(f, &nodeCount, sizeof(int)))
		return false;
	for (int i = 0; i < nodeCount; ++i)
	{
		const MotionNode &n = nodes[i];
		int t = (int)n.type;
		if (!writeBytes(f, &t, sizeof(int))) return false;
		if (!writeBytes(f, &n.id, sizeof(int))) return false;
		if (!writeBytes(f, &n.x, sizeof(float))) return false;
		if (!writeBytes(f, &n.y, sizeof(float))) return false;
		int pc = MotionNodeParamCount(n.type);
		if (!writeBytes(f, &pc, sizeof(int))) return false;
		for (int pi = 0; pi < pc; ++pi)
		{
			float v = MotionNodeParamGet(n, pi);
			if (!writeBytes(f, &v, sizeof(float))) return false;
		}
		int bc = (int)n.bones.size();
		if (!writeBytes(f, &bc, sizeof(int))) return false;
		for (int b = 0; b < bc; ++b)
		{
			int bi = n.bones[b];
			if (!writeBytes(f, &bi, sizeof(int))) return false;
		}
	}
	int linkCount = (int)links.size();
	if (!writeBytes(f, &linkCount, sizeof(int)))
		return false;
	for (int i = 0; i < linkCount; ++i)
	{
		if (!writeBytes(f, &links[i].from, sizeof(int))) return false;
		if (!writeBytes(f, &links[i].to, sizeof(int))) return false;
	}
	int nextId = g.nextId();
	if (!writeBytes(f, &nextId, sizeof(int)))
		return false;
	return true;
}

static float saneFloat(float v, float fallback)
{
	if (!(v == v)) return fallback;             // NaN
	if (v > 1e6f || v < -1e6f) return fallback; // Wild
	return v;
}

static bool readNodeGraphV1(FILE *f, KNodeGraph &g)
{
	// Legacy fixed layout (project v4 files): p0/p1 per node, no version header.
	g.clear();
	int nodeCount = 0;
	if (!readBytes(f, &nodeCount, sizeof(int))) return false;
	if (nodeCount < 0 || nodeCount > 512) return false;
	std::vector<int> seenIds;
	for (int i = 0; i < nodeCount; ++i)
	{
		MotionNode n;
		int t = -1;
		if (!readBytes(f, &t, sizeof(int))) return false;
		if (t < 0 || t > 15) return false;
		n.type = (MotionNodeType)t;
		if (!readBytes(f, &n.id, sizeof(int))) return false;
		if (n.id <= 0 || n.id > 100000) return false;
		bool dup = false;
		for (size_t k = 0; k < seenIds.size(); ++k)
			if (seenIds[k] == n.id) { dup = true; break; }
		if (dup) return false;
		seenIds.push_back(n.id);
		if (!readBytes(f, &n.x, sizeof(float))) return false;
		if (!readBytes(f, &n.y, sizeof(float))) return false;
		float rv0 = 0.0f, rv1 = 0.0f;
		if (!readBytes(f, &rv0, sizeof(float))) return false;
		if (!readBytes(f, &rv1, sizeof(float))) return false;
		n.params.assign(2, 0.0f);   // V1 files predate extensible params
		n.params[0] = rv0; n.params[1] = rv1;
		n.x = saneFloat(n.x, 0.0f); n.y = saneFloat(n.y, 0.0f);
		n.params[0] = saneFloat(n.params[0], 0.0f); n.params[1] = saneFloat(n.params[1], 0.0f);
		// Clamp params into their definition range.
		for (int pi = 0; pi < MotionNodeParamCount(n.type); ++pi)
			MotionNodeParamSet(n, pi, MotionNodeParamGet(n, pi));
		n.locked = (n.type == MotionNodeType::Output || n.type == MotionNodeType::Input);
		int bc = 0;
		if (!readBytes(f, &bc, sizeof(int))) return false;
		if (bc < 0 || bc > 4096) return false;
		n.bones.clear();
		for (int b = 0; b < bc; ++b)
		{
			int bi = -1;
			if (!readBytes(f, &bi, sizeof(int))) return false;
			if (bi >= 0 && bi < 100000)
				n.bones.push_back(bi);
		}
		g.addLoadedNode(n);
	}
	int linkCount = 0;
	if (!readBytes(f, &linkCount, sizeof(int))) return false;
	if (linkCount < 0 || linkCount > 1024) return false;
	for (int i = 0; i < linkCount; ++i)
	{
		int from = 0, to = 0;
		if (!readBytes(f, &from, sizeof(int))) return false;
		if (!readBytes(f, &to, sizeof(int))) return false;
		g.addLoadedLink(from, to);
	}
	int nextId = 1;
	if (!readBytes(f, &nextId, sizeof(int))) return false;
	g.setNextId(nextId);
	return true;
}

static bool readNodeGraphV2(FILE *f, KNodeGraph &g)
{
	// Extensible layout (project v5+): version header, then per-node
	// paramCount + params.
	int graphVersion = 0;
	if (!readBytes(f, &graphVersion, sizeof(int))) return false;
	if (graphVersion != 2) return false;
	g.clear();
	int nodeCount = 0;
	if (!readBytes(f, &nodeCount, sizeof(int))) return false;
	if (nodeCount < 0 || nodeCount > 512) return false;
	std::vector<int> seenIds;
	for (int i = 0; i < nodeCount; ++i)
	{
		MotionNode n;
		int t = -1;
		if (!readBytes(f, &t, sizeof(int))) return false;
		if (t < 0 || t > 15) return false;
		n.type = (MotionNodeType)t;
		if (!readBytes(f, &n.id, sizeof(int))) return false;
		if (n.id <= 0 || n.id > 100000) return false;
		bool dup = false;
		for (size_t k = 0; k < seenIds.size(); ++k)
			if (seenIds[k] == n.id) { dup = true; break; }
		if (dup) return false;
		seenIds.push_back(n.id);
		if (!readBytes(f, &n.x, sizeof(float))) return false;
		if (!readBytes(f, &n.y, sizeof(float))) return false;
		int pc = 0;
		if (!readBytes(f, &pc, sizeof(int))) return false;
		if (pc < 0 || pc > 16) return false;
		float pv[16] = { 0 };
		for (int pi = 0; pi < pc; ++pi)
			if (!readBytes(f, &pv[pi], sizeof(float))) return false;
		n.params.assign((size_t)pc, 0.0f);
		for (int pi = 0; pi < pc; ++pi)
			n.params[pi] = saneFloat(pv[pi], 0.0f);
		n.x = saneFloat(n.x, 0.0f); n.y = saneFloat(n.y, 0.0f);
		// Clamp params into their definition range.
		for (int pi = 0; pi < MotionNodeParamCount(n.type); ++pi)
			MotionNodeParamSet(n, pi, MotionNodeParamGet(n, pi));
		n.locked = (n.type == MotionNodeType::Output || n.type == MotionNodeType::Input);
		int bc = 0;
		if (!readBytes(f, &bc, sizeof(int))) return false;
		if (bc < 0 || bc > 4096) return false;
		n.bones.clear();
		for (int b = 0; b < bc; ++b)
		{
			int bi = -1;
			if (!readBytes(f, &bi, sizeof(int))) return false;
			if (bi >= 0 && bi < 100000)
				n.bones.push_back(bi);
		}
		g.addLoadedNode(n);
	}
	int linkCount = 0;
	if (!readBytes(f, &linkCount, sizeof(int))) return false;
	if (linkCount < 0 || linkCount > 1024) return false;
	for (int i = 0; i < linkCount; ++i)
	{
		int from = 0, to = 0;
		if (!readBytes(f, &from, sizeof(int))) return false;
		if (!readBytes(f, &to, sizeof(int))) return false;
		g.addLoadedLink(from, to);
	}
	int nextId = 1;
	if (!readBytes(f, &nextId, sizeof(int))) return false;
	g.setNextId(nextId);
	return true;
}

// ----------------------------------------------------------------------------
// KASProject
// ----------------------------------------------------------------------------

bool KASProject::save(const char *path, const KRetargeter *rt, const KBodyExporter *exporter)
{
	return save(path, rt, exporter, nullptr);
}

bool KASProject::save(const char *path, const KRetargeter *rt, const KBodyExporter *exporter, const KBodyVisualizer *visualizer)
{
	FILE *f = NULL;
	fopen_s(&f, path, "wb");
	if (!f)
		return false;

	bool ok = false;
	do {
		char magic[4] = { 'K', 'A', 'S', '1' };
		if (!writeBytes(f, magic, 4))
			break;
		unsigned int version = 5u;
		if (!writeBytes(f, &version, sizeof(version)))
			break;

		char charPath[_MAX_PATH] = { 0 };
		if (rt && rt->sourcePath())
			strcpy_s(charPath, rt->sourcePath());
		if (!writeBytes(f, charPath, _MAX_PATH))
			break;

		float importScale = rt ? rt->importScale() : 1.0f;
		float rx = rt ? rt->importRotX() : 0.0f;
		float ry = rt ? rt->importRotY() : 0.0f;
		float rz = rt ? rt->importRotZ() : 0.0f;
		if (!writeBytes(f, &importScale, sizeof(float))) break;
		if (!writeBytes(f, &rx, sizeof(float))) break;
		if (!writeBytes(f, &ry, sizeof(float))) break;
		if (!writeBytes(f, &rz, sizeof(float))) break;

		int mapping[25];
		for (int j = 0; j < 25; ++j)
			mapping[j] = rt ? rt->getMapping(j) : -1;
		if (!writeBytes(f, mapping, sizeof(int) * 25))
			break;

		KRecording emptyRec;
		const KRecording &rec = exporter ? exporter->recording() : emptyRec;
		if (!writeRecording(f, rec))
			break;

		// Camera (v2+) — 6 floats
		{
			float yaw = 0.f, pitch = -0.15f, dist = 3.8f, panX = 0.f, panY = 0.f, fov = 0.95993f;
			if (visualizer) visualizer->getCamera(yaw, pitch, dist, panX, panY, fov);
			if (!writeBytes(f, &yaw, sizeof(float))) break;
			if (!writeBytes(f, &pitch, sizeof(float))) break;
			if (!writeBytes(f, &dist, sizeof(float))) break;
			if (!writeBytes(f, &panX, sizeof(float))) break;
			if (!writeBytes(f, &panY, sizeof(float))) break;
			if (!writeBytes(f, &fov, sizeof(float))) break;
		}

		// Floor reference (v3) — origin for baked animation
		{
			int hasFloorInt = rec.hasFloor ? 1 : 0;
			float fp[4] = { rec.floorPlane[0], rec.floorPlane[1], rec.floorPlane[2], rec.floorPlane[3] };
			if (!writeBytes(f, &hasFloorInt, sizeof(int))) break;
			if (!writeBytes(f, fp, sizeof(float)*4)) break;
		}

		// Node motion graph (v4) — post-process animation (Output -> ... -> Input)
		{
			static KNodeGraph emptyGraph;
			const KNodeGraph &g = rt ? rt->nodeGraph() : emptyGraph;
			if (!writeNodeGraph(f, g))
				break;
		}

		ok = true;
	} while (0);

	fclose(f);
	return ok;
}

bool KASProject::load(const char *path, KRetargeter *rt, KBodyExporter *exporter)
{
	return load(path, rt, exporter, nullptr);
}

bool KASProject::load(const char *path, KRetargeter *rt, KBodyExporter *exporter, KBodyVisualizer *visualizer)
{
	FILE *f = NULL;
	fopen_s(&f, path, "rb");
	if (!f)
		return false;

	bool ok = false;
	do {
		char magic[4] = { 0 };
		if (!readBytes(f, magic, 4))
			break;
		if (magic[0] != 'K' || magic[1] != 'A' || magic[2] != 'S')
			break;
		unsigned int version = 0;
		if (!readBytes(f, &version, sizeof(version)))
			break;
		if (version < 1 || version > 5)
			break;

		char charPath[_MAX_PATH] = { 0 };
		if (!readBytes(f, charPath, _MAX_PATH))
			break;
		// A corrupted file may not contain a terminator at all.
		charPath[_MAX_PATH - 1] = 0;

		float importScale = 1.0f, rx = 0.0f, ry = 0.0f, rz = 0.0f;
		if (!readBytes(f, &importScale, sizeof(float))) break;
		if (!readBytes(f, &rx, sizeof(float))) break;
		if (!readBytes(f, &ry, sizeof(float))) break;
		if (!readBytes(f, &rz, sizeof(float))) break;

		int mapping[25];
		if (!readBytes(f, mapping, sizeof(int) * 25))
			break;

		KRecording rec;
		if (!readRecording(f, rec))
			break;

		float yaw = 0.f, pitch = -0.15f, dist = 3.8f, panX = 0.f, panY = 0.f, fov = 0.95993f;
		if (version >= 2)
		{
			if (!readBytes(f, &yaw, sizeof(float))) break;
			if (!readBytes(f, &pitch, sizeof(float))) break;
			if (!readBytes(f, &dist, sizeof(float))) break;
			if (!readBytes(f, &panX, sizeof(float))) break;
			if (!readBytes(f, &panY, sizeof(float))) break;
			if (!readBytes(f, &fov, sizeof(float))) break;
		}

		if (version >= 3)
		{
			int hasFloorInt = 0; float fp[4]={0,1,0,0};
			if (!readBytes(f, &hasFloorInt, sizeof(int))) break;
			if (!readBytes(f, fp, sizeof(float)*4)) break;
			if (fp[1] < 0) { fp[0]=-fp[0]; fp[1]=-fp[1]; fp[2]=-fp[2]; fp[3]=-fp[3]; }
			rec.hasFloor = hasFloorInt != 0;
			rec.floorPlane[0]=fp[0]; rec.floorPlane[1]=fp[1]; rec.floorPlane[2]=fp[2]; rec.floorPlane[3]=fp[3];
		}
		else
		{
			rec.hasFloor = false; rec.floorPlane[0]=0; rec.floorPlane[1]=1; rec.floorPlane[2]=0; rec.floorPlane[3]=0;
		}

		// Node motion graph (v4). Read now (file order), applied to the
		// retargeter after the character is (re)loaded below. Malformed
		// content falls back to the default graph without failing the load.
		KNodeGraph loadedGraph;
		bool graphOk = false;
		if (version == 4)
			graphOk = readNodeGraphV1(f, loadedGraph);
		else if (version >= 5)
			graphOk = readNodeGraphV2(f, loadedGraph);

		// Re-import the character (fresh, default transform) then re-apply
		// the saved transform and mapping.
		if (!rt || !rt->loadCharacter(charPath))
			break;
		rt->setImportScale(importScale);
		rt->setImportRotation(rx, ry, rz);
		for (int j = 0; j < 25; ++j)
			rt->setMapping(j, mapping[j]);
		if (graphOk)
			rt->nodeGraph() = loadedGraph;

		if (exporter)
			exporter->setRecording(rec);

		if (visualizer && version >= 2)
			visualizer->setCamera(yaw, pitch, dist, panX, panY, fov);

		ok = true;
	} while (0);

	fclose(f);
	return ok;
}
