// ============================================================================
// Node Motion Editor : separate top-level window (same pattern as the 2D
// mocap T-pose editor). Canvas on the right (nodes show their name only),
// Unreal-style details panel on the left (sliders + bone array with a +
// button opening a dropdown, like UE array variables).
//
// Every mutation repaints the 3D viewports, which re-evaluate the graph:
// the animation updates in real time.
// ============================================================================

#include "NodeEditor.h"
#include "Theme.h"
#include "resource.h"
#include "..\kinect\KRetargeter.h"
#include "..\..\CommonKinect\helpers\WindowIDS.h"
#include <windowsx.h>
#include <strsafe.h>
#include <string>
#include <vector>
#include <cmath>

// Global retargeter (defined in UI.cpp)
extern KRetargeter *g_retargeter;

static HWND g_hNodeEditorWnd = NULL;

#define NODEED_PANEL_W   320
#define NODEED_STATUS_H  26
#define NODE_W           150
#define NODE_H           54
#define NODE_PIN_R       6
#define NODEED_MENU_SMOOTH   9001
#define NODEED_MENU_MULTIPLY 9002
#define NODEED_MENU_ADD      9003
#define NODEED_MENU_FOOTLOCK 9004
#define NODEED_MENU_CLAMP    9005
#define NODEED_MENU_MIRROR   9006
#define NODEED_MENU_OFFSET   9007
#define NODEED_MENU_TIMESCALE 9008
#define NODEED_MENU_REVERSE  9009
#define NODEED_MENU_FLOORSNAP 9010
#define NODEED_MENU_NOISE    9011
#define NODEED_MENU_SPRING   9012
#define NODEED_MENU_AIM      9013
#define NODEED_MENU_MASK     9014
#define NODEED_MENU_RESETVIEW 9015

struct NodeEdState
{
	HWND hWnd;
	HWND hMain;
	int  selNode;                 // Selected node id, -1 = none
	int  selLinkFrom, selLinkTo;  // Selected link, -1/-1 = none
	int  dragMode;                // 0 none, 1 move node, 2 link drag, 3 slider, 4 pan
	int  dragNode;
	int  linkFrom;
	POINT linkCur;
	POINT dragOff;
	int  dragSlider;              // Param index while dragging a slider
	POINT panStart;               // Pan drag anchor (screen px)
	float panOX, panOY;           // Pan offset at drag start
	float panX, panY;             // Canvas offset (screen px)
	float zoom;                   // Canvas zoom (0.3..2.5)
	int  panelScroll;             // Left panel vertical scroll (px)
	HFONT hFont, hFontBold, hFontSmall;
	HBRUSH hBg, hPanel, hNode, hNodeSel;
	HPEN hPenBorder, hPenAccent, hPenHot, hPenLink, hPenLinkSel;
	HPEN hPenDim, hPenOrange;
};

#define NODEED_ORANGE RGB(255, 150, 40)

// T-pose joint layout, same normalized map as the 2D mocap editor
// (x 0..1 left->right, y 0..1 top->bottom, joints 0..24).
struct NodeEdJointPos { int j; float x, y; };
static const NodeEdJointPos kNodeEdTPose[] = {
	{ 3,0.50f,0.075f },{ 2,0.50f,0.155f },{ 20,0.50f,0.205f },
	{ 4,0.375f,0.215f },{ 5,0.245f,0.215f },{ 6,0.125f,0.215f },{ 7,0.045f,0.215f },
	{ 21,0.045f,0.165f },{ 22,0.075f,0.180f },
	{ 8,0.625f,0.215f },{ 9,0.755f,0.215f },{ 10,0.875f,0.215f },{ 11,0.955f,0.215f },
	{ 23,0.955f,0.165f },{ 24,0.925f,0.180f },
	{ 1,0.50f,0.315f },{ 0,0.50f,0.440f },
	{ 12,0.435f,0.455f },{ 13,0.435f,0.640f },{ 14,0.435f,0.815f },{ 15,0.435f,0.875f },
	{ 16,0.565f,0.455f },{ 17,0.565f,0.640f },{ 18,0.565f,0.815f },{ 19,0.565f,0.875f },
};
// Canonical parent of each joint, in the same index space (for the links).
static int NodeEdJointParent(int j)
{
	switch (j)
	{
	case 1: return 0;
	case 2: return 20;
	case 3: return 2;
	case 4: case 8: return 20;
	case 5: return 4;
	case 6: return 5;
	case 7: return 6;
	case 21: return 7;
	case 22: return 6;
	case 9: return 8;
	case 10: return 9;
	case 11: return 10;
	case 23: return 11;
	case 24: return 10;
	case 12: case 16: return 0;
	case 13: return 12;
	case 14: return 13;
	case 15: return 14;
	case 17: return 16;
	case 18: return 17;
	case 19: return 18;
	case 20: return 1;
	default: return -1;   // 0 (root), tips without links
	}
}

static bool NodeEdJointXY(int j, float &x, float &y)
{
	for (size_t i = 0; i < sizeof(kNodeEdTPose) / sizeof(kNodeEdTPose[0]); ++i)
		if (kNodeEdTPose[i].j == j) { x = kNodeEdTPose[i].x; y = kNodeEdTPose[i].y; return true; }
	return false;
}

// Skeleton area inside the left panel (below the bone header at y0).
#define NODEED_SK_SIZE 264
static RECT NodeEdSkelRect(int y0)
{
	RECT r;
	r.left = (NODEED_PANEL_W - NODEED_SK_SIZE) / 2;
	r.top = y0 + 24;
	r.right = r.left + NODEED_SK_SIZE;
	r.bottom = r.top + NODEED_SK_SIZE;
	return r;
}

static POINT NodeEdSkelPt(int j, const RECT &sk)
{
	float x = 0.5f, y = 0.5f;
	NodeEdJointXY(j, x, y);
	POINT p;
	p.x = sk.left + 8 + (int)(x * (sk.right - sk.left - 16));
	p.y = sk.top + 8 + (int)(y * (sk.bottom - sk.top - 16));
	return p;
}

// Bone currently driven by joint `j` (-1 when the joint is unmapped).
static int NodeEdJointBone(int j)
{
	if (!g_retargeter || j < 0 || j >= JointType_Count)
		return -1;
	int b = g_retargeter->getMapping(j);
	if (b < 0 || b >= g_retargeter->boneCount())
		return -1;
	return b;
}

// True when the node's bone list already drives joint `j`.
static bool NodeEdJointSelected(const MotionNode &n, int j)
{
	int b = NodeEdJointBone(j);
	if (b < 0) return false;
	for (size_t k = 0; k < n.bones.size(); ++k)
		if (n.bones[k] == b) return true;
	return false;
}

static NodeEdState* NodeEdGet(HWND hWnd)
{
	return (NodeEdState*)GetWindowLongPtr(hWnd, GWLP_USERDATA);
}

// ----------------------------------------------------------------------------
// Small helpers
// ----------------------------------------------------------------------------

static void NodeEdToWide(const char *utf8, WCHAR *out, int outLen)
{
	if (!utf8 || !utf8[0]) { out[0] = L'?'; out[1] = 0; return; }
	MultiByteToWideChar(CP_UTF8, 0, utf8, -1, out, outLen);
	out[outLen - 1] = 0;
}

// Repaints the editor and the main 3D view (which re-evaluates the graph:
// edits apply in real time).
static void NodeEdRefresh(NodeEdState *st)
{
	if (!st) return;
	InvalidateRect(st->hWnd, NULL, FALSE);
	if (st->hMain)
	{
		HWND hV = GetDlgItem(st->hMain, VIEW_STATUS);
		if (hV) InvalidateRect(hV, NULL, FALSE);
	}
}

// Canvas <-> screen transforms (navigation: pan + zoom).
static void NodeEdToScreen(NodeEdState *st, float cx, float cy, int &sx, int &sy)
{
	sx = NODEED_PANEL_W + (int)(st->panX + cx * st->zoom);
	sy = (int)(st->panY + cy * st->zoom);
}

static void NodeEdToCanvas(NodeEdState *st, int sx, int sy, float &cx, float &cy)
{
	float z = (st->zoom < 0.05f) ? 1.0f : st->zoom;
	cx = ((float)sx - NODEED_PANEL_W - st->panX) / z;
	cy = ((float)sy - st->panY) / z;
}

// Canvas-space rect of a node (client coords, navigation applied).
static RECT NodeEdNodeRect(NodeEdState *st, const MotionNode &n)
{
	int x, y, x2, y2;
	NodeEdToScreen(st, n.x, n.y, x, y);
	NodeEdToScreen(st, n.x + NODE_W, n.y + NODE_H, x2, y2);
	RECT r;
	r.left = x; r.top = y; r.right = x2; r.bottom = y2;
	return r;
}

static POINT NodeEdInPin(NodeEdState *st, const MotionNode &n)
{
	RECT r = NodeEdNodeRect(st, n);
	POINT p; p.x = r.left; p.y = (r.top + r.bottom) / 2;
	return p;
}

static POINT NodeEdOutPin(NodeEdState *st, const MotionNode &n)
{
	RECT r = NodeEdNodeRect(st, n);
	POINT p; p.x = r.right; p.y = (r.top + r.bottom) / 2;
	return p;
}

static bool NodeEdPtInRect(int x, int y, const RECT &r)
{
	return x >= r.left && x <= r.right && y >= r.top && y <= r.bottom;
}

// Panel details target. Returns false when the panel shows no node details.
static bool NodeEdPanelNode(NodeEdState *st, const MotionNode* &out)
{
	out = NULL;
	if (!st || !g_retargeter || !g_retargeter->hasCharacter())
		return false;
	if (st->selNode < 0)
		return false;
	out = g_retargeter->nodeGraph().find(st->selNode);
	return out != NULL;
}

// Y of param block `idx` (label + slider).
static int NodeEdParamY(int idx) { return 64 + idx * 56; }

// Y where the bone section starts (after `paramCount` param blocks).
static int NodeEdBonesY0(int paramCount) { return 64 + paramCount * 56 + 10; }

static RECT NodeEdSliderTrack(int idx)
{
	RECT r;
	r.left = 16; r.right = NODEED_PANEL_W - 16;
	r.top = NodeEdParamY(idx) + 26;
	r.bottom = r.top + 6;
	return r;
}

// ----------------------------------------------------------------------------
// Window setup
// ----------------------------------------------------------------------------

ATOM NodeEditorRegisterClass(HINSTANCE hInst)
{
	WNDCLASSEX wcex = { 0 };
	wcex.cbSize = sizeof(wcex);
	wcex.style = CS_HREDRAW | CS_VREDRAW;
	wcex.lpfnWndProc = NodeEditorProc;
	wcex.hInstance = hInst;
	wcex.hCursor = LoadCursor(NULL, IDC_ARROW);
	wcex.hbrBackground = NULL;   // we paint everything
	wcex.lpszClassName = L"KAS_NodeEditorClass";
	wcex.hIcon = LoadIcon(hInst, MAKEINTRESOURCE(IDI_UI));
	return RegisterClassEx(&wcex);
}

HWND CreateNodeEditorWindow(HWND hParent)
{
	if (g_hNodeEditorWnd && IsWindow(g_hNodeEditorWnd))
	{
		ShowWindow(g_hNodeEditorWnd, SW_RESTORE);
		SetForegroundWindow(g_hNodeEditorWnd);
		return g_hNodeEditorWnd;
	}
	HINSTANCE hInst = (HINSTANCE)GetWindowLongPtr(hParent, GWLP_HINSTANCE);
	g_hNodeEditorWnd = CreateWindowEx(0, L"KAS_NodeEditorClass", L"Node Motion Editor",
		WS_OVERLAPPEDWINDOW | WS_CLIPCHILDREN,
		140, 60, 1120, 700, hParent, NULL, hInst, NULL);
	if (g_hNodeEditorWnd) ShowWindow(g_hNodeEditorWnd, SW_SHOW);
	return g_hNodeEditorWnd;
}

// ----------------------------------------------------------------------------
// Painting
// ----------------------------------------------------------------------------

static void NodeEdPaintNode(HDC hdc, NodeEdState *st, const MotionNode &n, bool selected)
{
	RECT r = NodeEdNodeRect(st, n);
	HBRUSH body = selected ? st->hNodeSel : st->hNode;
	HPEN border = selected ? st->hPenHot
		: ((n.type == MotionNodeType::Output || n.type == MotionNodeType::Input) ? st->hPenAccent : st->hPenBorder);
	HGDIOBJ oldB = SelectObject(hdc, body);
	HGDIOBJ oldP = SelectObject(hdc, border);
	RoundRect(hdc, r.left, r.top, r.right, r.bottom, 12, 12);
	SelectObject(hdc, oldB);
	SelectObject(hdc, oldP);
	// Name only (locked nodes are told apart by the accent border).
	WCHAR wname[64];
	NodeEdToWide(MotionNodeTypeName(n.type), wname, 64);
	HGDIOBJ oldF = SelectObject(hdc, selected ? st->hFontBold : st->hFont);
	SetBkMode(hdc, TRANSPARENT);
	SetTextColor(hdc, Theme::TEXT);
	RECT tr = r;
	tr.top += 12;
	DrawText(hdc, wname, -1, &tr, DT_CENTER | DT_TOP | DT_SINGLELINE);
	SelectObject(hdc, oldF);
	// Pins
	HGDIOBJ oldPB = SelectObject(hdc, GetStockObject(DC_BRUSH));
	SetDCBrushColor(hdc, Theme::ACCENT);
	HGDIOBJ oldPP = SelectObject(hdc, GetStockObject(DC_PEN));
	SetDCPenColor(hdc, Theme::ACCENT);
	if (MotionNodeHasInputPin(n.type))
	{
		POINT p = NodeEdInPin(st, n);
		Ellipse(hdc, p.x - NODE_PIN_R, p.y - NODE_PIN_R, p.x + NODE_PIN_R, p.y + NODE_PIN_R);
	}
	if (MotionNodeHasOutputPin(n.type))
	{
		POINT p = NodeEdOutPin(st, n);
		Ellipse(hdc, p.x - NODE_PIN_R, p.y - NODE_PIN_R, p.x + NODE_PIN_R, p.y + NODE_PIN_R);
	}
	SelectObject(hdc, oldPB);
	SelectObject(hdc, oldPP);
}

static void NodeEdPaintLink(HDC hdc, NodeEdState *st, POINT a, POINT b, bool selected)
{
	int dx = abs(b.x - a.x);
	if (dx < 24) dx = 24;
	if (dx > 140) dx = 140;
	POINT pts[4] = { a, { a.x + dx, a.y }, { b.x - dx, b.y }, b };
	HPEN pen = selected ? st->hPenLinkSel : st->hPenLink;
	HGDIOBJ old = SelectObject(hdc, pen);
	PolyBezier(hdc, pts, 4);
	SelectObject(hdc, old);
}

static double NodeEdDistPtSeg(int px, int py, int ax, int ay, int bx, int by)
{
	double dx = (double)(bx - ax), dy = (double)(by - ay);
	double L2 = dx * dx + dy * dy;
	double t = (L2 > 1e-9) ? ((px - ax) * dx + (py - ay) * dy) / L2 : 0.0;
	if (t < 0.0) t = 0.0;
	if (t > 1.0) t = 1.0;
	double cx = ax + t * dx, cy = ay + t * dy;
	double ex = px - cx, ey = py - cy;
	return sqrt(ex * ex + ey * ey);
}

// Distance from a point to a painted link curve (sampled).
static double NodeEdLinkDist(int px, int py, POINT a, POINT b)
{
	int dx = abs(b.x - a.x);
	if (dx < 24) dx = 24;
	if (dx > 140) dx = 140;
	double best = 1e9;
	int prevx = a.x, prevy = a.y;
	for (int i = 1; i <= 24; ++i)
	{
		double t = (double)i / 24.0;
		double mt = 1.0 - t;
		double x = mt*mt*mt*a.x + 3*mt*mt*t*(a.x+dx) + 3*mt*t*t*(b.x-dx) + t*t*t*b.x;
		double y = mt*mt*mt*a.y + 3*mt*mt*t*a.y + 3*mt*t*t*b.y + t*t*t*b.y;
		double d = NodeEdDistPtSeg(px, py, prevx, prevy, (int)x, (int)y);
		if (d < best) best = d;
		prevx = (int)x; prevy = (int)y;
	}
	return best;
}

static void NodeEdPaintSlider(HDC hdc, NodeEdState *st, const MotionNode &n, int idx)
{
	MotionParamDef d = MotionNodeParamDef(n.type, idx);
	float v = MotionNodeParamGet(n, idx);
	int y = NodeEdParamY(idx);
	// Label + value
	WCHAR wlabel[64], wval[32];
	NodeEdToWide(d.label, wlabel, 64);
	if (d.isInt) StringCchPrintf(wval, 32, L"%d", (int)v);
	else StringCchPrintf(wval, 32, L"%.2f", v);
	HGDIOBJ oldF = SelectObject(hdc, st->hFontSmall);
	SetBkMode(hdc, TRANSPARENT);
	SetTextColor(hdc, Theme::TEXT_DIM);
	RECT lr = { 16, y, NODEED_PANEL_W - 70, y + 18 };
	DrawText(hdc, wlabel, -1, &lr, DT_LEFT | DT_TOP | DT_SINGLELINE);
	SetTextColor(hdc, Theme::TEXT);
	RECT vr = { NODEED_PANEL_W - 70, y, NODEED_PANEL_W - 16, y + 18 };
	DrawText(hdc, wval, -1, &vr, DT_RIGHT | DT_TOP | DT_SINGLELINE);
	SelectObject(hdc, oldF);
	// Track
	RECT tr = NodeEdSliderTrack(idx);
	HGDIOBJ oldB = SelectObject(hdc, st->hPanel);
	HGDIOBJ oldP = SelectObject(hdc, st->hPenBorder);
	RoundRect(hdc, tr.left, tr.top, tr.right, tr.bottom, 3, 3);
	SelectObject(hdc, oldB);
	SelectObject(hdc, oldP);
	// Thumb
	double t = (d.maxVal > d.minVal) ? (double)(v - d.minVal) / (double)(d.maxVal - d.minVal) : 0.0;
	if (t < 0.0) t = 0.0;
	if (t > 1.0) t = 1.0;
	int tx = tr.left + (int)(t * (tr.right - tr.left));
	int ty = (tr.top + tr.bottom) / 2;
	HGDIOBJ oldTB = SelectObject(hdc, GetStockObject(DC_BRUSH));
	SetDCBrushColor(hdc, (st->dragMode == 3 && st->dragSlider == idx) ? Theme::ACCENT_HOT : Theme::ACCENT);
	HGDIOBJ oldTP = SelectObject(hdc, GetStockObject(DC_PEN));
	SetDCPenColor(hdc, Theme::ACCENT);
	Ellipse(hdc, tx - 7, ty - 7, tx + 7, ty + 7);
	SelectObject(hdc, oldTB);
	SelectObject(hdc, oldTP);
}

static void NodeEdPaintPanelBody(HDC hdc, NodeEdState *st, int H)
{
	// Panel background + separator
	HGDIOBJ oldB = SelectObject(hdc, st->hPanel);
	HGDIOBJ oldP = SelectObject(hdc, GetStockObject(NULL_PEN));
	Rectangle(hdc, 0, 0, NODEED_PANEL_W, H - NODEED_STATUS_H);
	SelectObject(hdc, oldB);
	SelectObject(hdc, oldP);
	HGDIOBJ oldLP = SelectObject(hdc, st->hPenBorder);
	MoveToEx(hdc, NODEED_PANEL_W, 0, NULL);
	LineTo(hdc, NODEED_PANEL_W, H - NODEED_STATUS_H);
	SelectObject(hdc, oldLP);

	const MotionNode *n = NULL;
	if (!NodeEdPanelNode(st, n))
	{
		HGDIOBJ oldF = SelectObject(hdc, st->hFont);
		SetBkMode(hdc, TRANSPARENT);
		SetTextColor(hdc, Theme::TEXT_DIM);
		RECT r = { 16, 20, NODEED_PANEL_W - 16, 140 };
		DrawText(hdc, L"No selection.\n\nClick a node on the canvas to edit its parameters.", -1, &r, DT_LEFT | DT_TOP | DT_WORDBREAK);
		SelectObject(hdc, oldF);
		return;
	}

	// Header: node name
	WCHAR wname[64];
	NodeEdToWide(MotionNodeTypeName(n->type), wname, 64);
	HGDIOBJ oldF = SelectObject(hdc, st->hFontBold);
	SetBkMode(hdc, TRANSPARENT);
	SetTextColor(hdc, Theme::TEXT);
	RECT hr = { 16, 10, NODEED_PANEL_W - 16, 34 };
	DrawText(hdc, wname, -1, &hr, DT_LEFT | DT_TOP | DT_SINGLELINE);
	SelectObject(hdc, oldF);
	HGDIOBJ oldFS = SelectObject(hdc, st->hFontSmall);
	SetTextColor(hdc, Theme::TEXT_DIM);
	RECT sr = { 16, 34, NODEED_PANEL_W - 16, 54 };
	const WCHAR *sub = L"";
	if (n->type == MotionNodeType::Output) sub = L"Source: recorded take (hidden)";
	else if (n->type == MotionNodeType::Input) sub = L"Final pose: viewports + export";
	DrawText(hdc, sub, -1, &sr, DT_LEFT | DT_TOP | DT_SINGLELINE);

	int pc = MotionNodeParamCount(n->type);
	for (int i = 0; i < pc; ++i)
		NodeEdPaintSlider(hdc, st, *n, i);

	if (MotionNodeHasBoneArray(n->type))
	{
		int y0 = NodeEdBonesY0(pc);
		WCHAR whead[96];
		StringCchPrintf(whead, 96, L"BONES - click joints, empty = all");
		RECT bhr = { 16, y0, NODEED_PANEL_W - 16, y0 + 20 };
		DrawText(hdc, whead, -1, &bhr, DT_LEFT | DT_TOP | DT_SINGLELINE);
		RECT sk = NodeEdSkelRect(y0);
		// Skeleton links first.
		for (int j = 0; j < JointType_Count; ++j)
		{
			int p = NodeEdJointParent(j);
			if (p < 0) continue;
			POINT a = NodeEdSkelPt(j, sk), b = NodeEdSkelPt(p, sk);
			HGDIOBJ oldL = SelectObject(hdc, st->hPenDim);
			MoveToEx(hdc, a.x, a.y, NULL);
			LineTo(hdc, b.x, b.y);
			SelectObject(hdc, oldL);
		}
		// Joints as bool buttons: orange when their bone is targeted.
		HGDIOBJ oldDB = SelectObject(hdc, GetStockObject(DC_BRUSH));
		HGDIOBJ oldDP = SelectObject(hdc, GetStockObject(DC_PEN));
		for (int j = 0; j < JointType_Count; ++j)
		{
			float jx, jy;
			if (!NodeEdJointXY(j, jx, jy)) continue;
			POINT c = NodeEdSkelPt(j, sk);
			int b = NodeEdJointBone(j);
			bool sel = (b >= 0) && NodeEdJointSelected(*n, j);
			if (sel)
			{
				SetDCBrushColor(hdc, NODEED_ORANGE);
				SetDCPenColor(hdc, NODEED_ORANGE);
			}
			else if (b >= 0)
			{
				SetDCBrushColor(hdc, Theme::PANEL);
				SetDCPenColor(hdc, Theme::TEXT_DIM);
			}
			else
			{
				SetDCBrushColor(hdc, Theme::PANEL);
				SetDCPenColor(hdc, RGB(90, 94, 108));
			}
			Ellipse(hdc, c.x - 9, c.y - 9, c.x + 9, c.y + 9);
		}
		SelectObject(hdc, oldDB);
		SelectObject(hdc, oldDP);
		// Targeted bone names (dim block below the skeleton).
		RECT nr = { 16, sk.bottom + 6, NODEED_PANEL_W - 16, sk.bottom + 58 };
		SetTextColor(hdc, Theme::TEXT_DIM);
		if (n->bones.empty())
		{
			DrawText(hdc, L"All bones", -1, &nr, DT_LEFT | DT_TOP | DT_SINGLELINE);
		}
		else
		{
			std::string names;
			for (size_t bi = 0; bi < n->bones.size() && names.size() < 200; ++bi)
			{
				int b = n->bones[bi];
				const char *bn = (g_retargeter && b >= 0 && b < g_retargeter->boneCount())
					? g_retargeter->boneName(b) : "?";
				if (bi > 0) names += ", ";
				names += bn;
			}
			WCHAR wnames[256];
			NodeEdToWide(names.c_str(), wnames, 256);
			DrawText(hdc, wnames, -1, &nr, DT_LEFT | DT_TOP | DT_WORDBREAK | DT_END_ELLIPSIS);
		}
	}
	else
	{
		RECT ir = { 16, 70, NODEED_PANEL_W - 16, 220 };
		SetTextColor(hdc, Theme::TEXT_DIM);
		const WCHAR *info = (n->type == MotionNodeType::Output)
			? L"Provides the raw recorded animation to the graph. It is never displayed: only the Input result is visible in the 3D views."
			: ((n->type == MotionNodeType::Reverse)
				? L"Plays the whole upstream backwards (take only; transparent in live)."
				: L"Holds the reworked animation shown in the 3D views and baked by the export.\n\nUnlink its input to drop the model back to the T-pose.");
		DrawText(hdc, info, -1, &ir, DT_LEFT | DT_TOP | DT_WORDBREAK);
	}
	SelectObject(hdc, oldFS);
}

// Panel content height (for scrolling tall node editors, e.g. Clamp).
static int NodeEdPanelContentH(NodeEdState *st)
{
	const MotionNode *n = NULL;
	if (!NodeEdPanelNode(st, n)) return 140;
	int h = 54 + MotionNodeParamCount(n->type) * 56;
	if (MotionNodeHasBoneArray(n->type))
		h += 32 + NODEED_SK_SIZE + 76;
	else
		h += 170;
	return h + 8;
}

static void NodeEdPaintPanel(HDC hdc, NodeEdState *st, int H)
{
	int visH = H - NODEED_STATUS_H;
	int maxS = NodeEdPanelContentH(st) - visH + 8;
	if (maxS < 0) maxS = 0;
	if (st->panelScroll > maxS) st->panelScroll = maxS;
	if (st->panelScroll < 0) st->panelScroll = 0;
	HRGN clip = CreateRectRgn(0, 0, NODEED_PANEL_W, visH);
	SelectClipRgn(hdc, clip);
	SetViewportOrgEx(hdc, 0, -st->panelScroll, NULL);
	NodeEdPaintPanelBody(hdc, st, H);
	SetViewportOrgEx(hdc, 0, 0, NULL);
	SelectClipRgn(hdc, NULL);
	DeleteObject(clip);
}

static void NodeEdPaintDC(HDC hdc, NodeEdState *st, int W, int H)
{
	if (!hdc) return;

	// Background
	HGDIOBJ oldB = SelectObject(hdc, st ? st->hBg : (HBRUSH)GetStockObject(BLACK_BRUSH));
	HGDIOBJ oldP = SelectObject(hdc, GetStockObject(NULL_PEN));
	Rectangle(hdc, 0, 0, W, H);
	SelectObject(hdc, oldB);
	SelectObject(hdc, oldP);

	if (!st || !g_retargeter || !g_retargeter->hasCharacter())
	{
		HGDIOBJ oldF = SelectObject(hdc, st ? st->hFont : (HFONT)GetStockObject(DEFAULT_GUI_FONT));
		SetBkMode(hdc, TRANSPARENT);
		SetTextColor(hdc, Theme::TEXT_DIM);
		RECT r = { 20, 20, W - 20, 120 };
		DrawText(hdc, L"Load a character to edit the motion graph.", -1, &r, DT_LEFT | DT_TOP | DT_WORDBREAK);
		SelectObject(hdc, oldF);
		return;
	}

	const KNodeGraph &g = g_retargeter->nodeGraph();

	// Links under the nodes
	for (size_t i = 0; i < g.links().size(); ++i)
	{
		const MotionNode *a = g.find(g.links()[i].from);
		const MotionNode *b = g.find(g.links()[i].to);
		if (!a || !b) continue;
		bool sel = (st->selLinkFrom == a->id && st->selLinkTo == b->id);
		NodeEdPaintLink(hdc, st, NodeEdOutPin(st, *a), NodeEdInPin(st, *b), sel);
	}
	// Temp link while dragging
	if (st->dragMode == 2)
	{
		const MotionNode *a = g.find(st->linkFrom);
		if (a) NodeEdPaintLink(hdc, st, NodeEdOutPin(st, *a), st->linkCur, false);
	}
	// Nodes
	for (size_t i = 0; i < g.nodes().size(); ++i)
		NodeEdPaintNode(hdc, st, g.nodes()[i], g.nodes()[i].id == st->selNode);

	NodeEdPaintPanel(hdc, st, H);

	// Status bar
	HGDIOBJ oldSB = SelectObject(hdc, st->hPanel);
	HGDIOBJ oldSP = SelectObject(hdc, GetStockObject(NULL_PEN));
	Rectangle(hdc, 0, H - NODEED_STATUS_H, W, H);
	SelectObject(hdc, oldSB);
	SelectObject(hdc, oldSP);
	std::vector<int> order;
	bool chained = g.chainFromInput(order);
	HGDIOBJ oldSF = SelectObject(hdc, st->hFontSmall);
	SetBkMode(hdc, TRANSPARENT);
	WCHAR wstatus[256];
	if (chained)
	{
		SetTextColor(hdc, Theme::GREEN);
		StringCchPrintf(wstatus, 256, L"Connected: Output -> Input (%d nodes)   |   Right-click canvas: add node   |   Del: remove   |   Drag pins to link",
			(int)order.size());
	}
	else
	{
		SetTextColor(hdc, Theme::DANGER);
		StringCchPrintf(wstatus, 256, L"NOT CONNECTED - T-POSE (link the Output chain into Input)   |   Right-click canvas: add node");
	}
	WCHAR wzoom[80];
	StringCchPrintf(wzoom, 80, L"   |   Zoom %d%% (wheel), pan (MMB)", (int)(st->zoom * 100.0f));
	StringCchCat(wstatus, 256, wzoom);
	RECT str = { 10, H - NODEED_STATUS_H, W - 10, H };
	DrawText(hdc, wstatus, -1, &str, DT_LEFT | DT_VCENTER | DT_SINGLELINE);
	SelectObject(hdc, oldSF);
}

// Double-buffered entry point: everything renders offscreen, then blits in
// one go, so slider drags and live updates never flicker.
static void NodeEdPaint(HWND hWnd)
{
	NodeEdState *st = NodeEdGet(hWnd);
	PAINTSTRUCT ps;
	HDC hdc = BeginPaint(hWnd, &ps);
	RECT rc;
	GetClientRect(hWnd, &rc);
	int W = rc.right - rc.left, H = rc.bottom - rc.top;
	if (W > 0 && H > 0)
	{
		HDC mem = CreateCompatibleDC(hdc);
		HBITMAP bmp = CreateCompatibleBitmap(hdc, W, H);
		HGDIOBJ old = SelectObject(mem, bmp);
		NodeEdPaintDC(mem, st, W, H);
		BitBlt(hdc, 0, 0, W, H, mem, 0, 0, SRCCOPY);
		SelectObject(mem, old);
		DeleteObject(bmp);
		DeleteDC(mem);
	}
	EndPaint(hWnd, &ps);
}

// ----------------------------------------------------------------------------
// Interaction
// ----------------------------------------------------------------------------

static void NodeEdSliderSet(NodeEdState *st, MotionNode *n, int idx, int x)
{
	RECT tr = NodeEdSliderTrack(idx);	MotionParamDef d = MotionNodeParamDef(n->type, idx);
	double t = (tr.right > tr.left) ? (double)(x - tr.left) / (double)(tr.right - tr.left) : 0.0;
	if (t < 0.0) t = 0.0;
	if (t > 1.0) t = 1.0;
	MotionNodeParamSet(*n, idx, (float)(d.minVal + t * (d.maxVal - d.minVal)));
}

// Panel hit test. Returns true when the click lands on an interactive row.
static bool NodeEdPanelHit(NodeEdState *st, int x, int y, bool down)
{
	const MotionNode *cn = NULL;
	if (!NodeEdPanelNode(st, cn)) return false;
	y += st->panelScroll;   // Content coords (panel may be scrolled)
	MotionNode *n = g_retargeter->nodeGraph().find(cn->id);
	if (!n) return false;
	int pc = MotionNodeParamCount(n->type);
	for (int i = 0; i < pc; ++i)
	{
		RECT tr = NodeEdSliderTrack(i);
		RECT hit = { 8, tr.top - 16, NODEED_PANEL_W - 8, tr.bottom + 12 };
		if (NodeEdPtInRect(x, y, hit))
		{
			if (down)
			{
				st->dragMode = 3; st->dragSlider = i;
				SetCapture(st->hWnd);
				NodeEdSliderSet(st, n, i, x);
				NodeEdRefresh(st);
			}
			return true;
		}
	}
	if (MotionNodeHasBoneArray(n->type))
	{
		// T-pose skeleton picker: every joint is a bool button toggling its
		// mapped bone (orange when targeted). Unmapped joints stay dim.
		int y0 = NodeEdBonesY0(pc);
		RECT sk = NodeEdSkelRect(y0);
		for (int j = 0; j < JointType_Count; ++j)
		{
			float jx, jy;
			if (!NodeEdJointXY(j, jx, jy)) continue;
			POINT c = NodeEdSkelPt(j, sk);
			double dx = (double)(x - c.x), dy = (double)(y - c.y);
			if (dx * dx + dy * dy <= 13.0 * 13.0)
			{
				if (down)
				{
					int b = NodeEdJointBone(j);
					if (b >= 0)
					{
						bool has = false;
						for (size_t k = 0; k < n->bones.size(); ++k)
							if (n->bones[k] == b) { has = true; break; }
						if (has)
						{
							for (size_t k = 0; k < n->bones.size(); ++k)
								if (n->bones[k] == b) { n->bones.erase(n->bones.begin() + k); break; }
						}
						else
							n->bones.push_back(b);
						NodeEdRefresh(st);
					}
				}
				return true;
			}
		}
	}
	return false;
}

static double NodeEdDistPt(int x1, int y1, int x2, int y2)
{
	double dx = (double)(x1 - x2), dy = (double)(y1 - y2);
	return sqrt(dx * dx + dy * dy);
}

static void NodeEdCanvasDown(NodeEdState *st, int x, int y)
{
	if (!g_retargeter || !g_retargeter->hasCharacter()) return;
	KNodeGraph &g = g_retargeter->nodeGraph();
	// Out-pin: start a link drag.
	for (size_t i = 0; i < g.nodes().size(); ++i)
	{
		const MotionNode &n = g.nodes()[i];
		if (!MotionNodeHasOutputPin(n.type)) continue;
		POINT p = NodeEdOutPin(st, n);
		if (NodeEdDistPt(x, y, p.x, p.y) <= 9.0)
		{
			st->selNode = n.id; st->selLinkFrom = -1; st->selLinkTo = -1;
			st->dragMode = 2; st->linkFrom = n.id;
			st->linkCur.x = x; st->linkCur.y = y;
			SetCapture(st->hWnd);
			NodeEdRefresh(st);
			return;
		}
	}
	// In-pin: select the feeding link (Del removes it).
	for (size_t i = 0; i < g.nodes().size(); ++i)
	{
		const MotionNode &n = g.nodes()[i];
		if (!MotionNodeHasInputPin(n.type)) continue;
		POINT p = NodeEdInPin(st, n);
		if (NodeEdDistPt(x, y, p.x, p.y) <= 9.0)
		{
			st->selNode = n.id;
			st->selLinkFrom = -1; st->selLinkTo = -1;
			for (size_t k = 0; k < g.links().size(); ++k)
				if (g.links()[k].to == n.id)
				{
					st->selLinkFrom = g.links()[k].from;
					st->selLinkTo = g.links()[k].to;
					break;
				}
			NodeEdRefresh(st);
			return;
		}
	}
	// Node body (topmost first): select + move (drag offset in canvas units).
	for (size_t k = 0; k < g.nodes().size(); ++k)
	{
		const MotionNode &n = g.nodes()[g.nodes().size() - 1 - k];
		RECT r = NodeEdNodeRect(st, n);
		if (NodeEdPtInRect(x, y, r))
		{
			st->selNode = n.id; st->selLinkFrom = -1; st->selLinkTo = -1;
			st->dragMode = 1; st->dragNode = n.id;
			float mcx, mcy;
			NodeEdToCanvas(st, x, y, mcx, mcy);
			st->dragOff.x = (int)(mcx - n.x); st->dragOff.y = (int)(mcy - n.y);
			SetCapture(st->hWnd);
			NodeEdRefresh(st);
			return;
		}
	}
	// Link curve: select it.
	double best = 1e9;
	int bf = -1, bt = -1;
	for (size_t i = 0; i < g.links().size(); ++i)
	{
		const MotionNode *a = g.find(g.links()[i].from);
		const MotionNode *b = g.find(g.links()[i].to);
		if (!a || !b) continue;
		double d = NodeEdLinkDist(x, y, NodeEdOutPin(st, *a), NodeEdInPin(st, *b));
		if (d < 7.0 && d < best) { best = d; bf = a->id; bt = b->id; }
	}
	st->selNode = -1;
	st->selLinkFrom = bf; st->selLinkTo = bt;
	NodeEdRefresh(st);
}

static void NodeEdCanvasMove(NodeEdState *st, int x, int y)
{
	if (!g_retargeter) return;
	KNodeGraph &g = g_retargeter->nodeGraph();
	if (st->dragMode == 1)
	{
		MotionNode *n = g.find(st->dragNode);
		if (n)
		{
			float mcx, mcy;
			NodeEdToCanvas(st, x, y, mcx, mcy);
			float nx = mcx - (float)st->dragOff.x;
			float ny = mcy - (float)st->dragOff.y;
			if (nx < -4000.0f) nx = -4000.0f;
			if (ny < -4000.0f) ny = -4000.0f;
			g.setNodePos(n->id, nx, ny);
			NodeEdRefresh(st);
		}
	}
	else if (st->dragMode == 2)
	{
		st->linkCur.x = x; st->linkCur.y = y;
		NodeEdRefresh(st);
	}
	else if (st->dragMode == 4)
	{
		st->panX = st->panOX + (float)(x - st->panStart.x);
		st->panY = st->panOY + (float)(y - st->panStart.y);
		NodeEdRefresh(st);
	}
	else if (st->dragMode == 3)
	{
		MotionNode *n = g.find(st->selNode);
		if (n) { NodeEdSliderSet(st, n, st->dragSlider, x); NodeEdRefresh(st); }
	}
}

static void NodeEdCanvasUp(NodeEdState *st, int x, int y)
{
	if (st->dragMode == 2 && g_retargeter && g_retargeter->hasCharacter())
	{
		KNodeGraph &g = g_retargeter->nodeGraph();
		for (size_t i = 0; i < g.nodes().size(); ++i)
		{
			const MotionNode &n = g.nodes()[i];
			if (n.id == st->linkFrom || !MotionNodeHasInputPin(n.type)) continue;
			POINT p = NodeEdInPin(st, n);
			if (NodeEdDistPt(x, y, p.x, p.y) <= 10.0)
			{
				if (g.link(st->linkFrom, n.id))
				{
					st->selNode = n.id;
					st->selLinkFrom = -1; st->selLinkTo = -1;
				}
				break;
			}
		}
	}
	st->dragMode = 0;
	ReleaseCapture();
	NodeEdRefresh(st);
}

static void NodeEdCanvasMenu(NodeEdState *st, int x, int y)
{
	if (!g_retargeter || !g_retargeter->hasCharacter()) return;
	POINT pt = { x, y };
	ClientToScreen(st->hWnd, &pt);
	HMENU m = CreatePopupMenu();
	AppendMenu(m, MF_STRING, NODEED_MENU_SMOOTH, L"Add Smooth here");
	AppendMenu(m, MF_STRING, NODEED_MENU_MULTIPLY, L"Add Multiply here");
	AppendMenu(m, MF_STRING, NODEED_MENU_ADD, L"Add rotation offset (Add) here");
	AppendMenu(m, MF_STRING, NODEED_MENU_FOOTLOCK, L"Add Foot Lock here");
	AppendMenu(m, MF_SEPARATOR, 0, NULL);
	AppendMenu(m, MF_STRING, NODEED_MENU_CLAMP, L"Add Clamp (joint limits) here");
	AppendMenu(m, MF_STRING, NODEED_MENU_MIRROR, L"Add Mirror here");
	AppendMenu(m, MF_STRING, NODEED_MENU_OFFSET, L"Add Offset (translation) here");
	AppendMenu(m, MF_STRING, NODEED_MENU_TIMESCALE, L"Add TimeScale here");
	AppendMenu(m, MF_STRING, NODEED_MENU_REVERSE, L"Add Reverse here");
	AppendMenu(m, MF_STRING, NODEED_MENU_FLOORSNAP, L"Add FloorSnap here");
	AppendMenu(m, MF_STRING, NODEED_MENU_NOISE, L"Add Noise here");
	AppendMenu(m, MF_STRING, NODEED_MENU_SPRING, L"Add Spring here");
	AppendMenu(m, MF_STRING, NODEED_MENU_AIM, L"Add Aim here");
	AppendMenu(m, MF_STRING, NODEED_MENU_MASK, L"Add Mask (raw mix) here");
	AppendMenu(m, MF_SEPARATOR, 0, NULL);
	AppendMenu(m, MF_STRING, NODEED_MENU_RESETVIEW, L"Reset view");
	int cmd = TrackPopupMenu(m, TPM_RIGHTBUTTON | TPM_RETURNCMD, pt.x, pt.y, 0, st->hWnd, NULL);
	DestroyMenu(m);
	if (cmd == NODEED_MENU_RESETVIEW) {
		st->panX = 0.0f; st->panY = 0.0f; st->zoom = 1.0f;
		NodeEdRefresh(st);
		return;
	}
	if (cmd != NODEED_MENU_SMOOTH && cmd != NODEED_MENU_MULTIPLY && cmd != NODEED_MENU_ADD
		&& cmd != NODEED_MENU_FOOTLOCK && cmd != NODEED_MENU_CLAMP && cmd != NODEED_MENU_MIRROR
		&& cmd != NODEED_MENU_OFFSET && cmd != NODEED_MENU_TIMESCALE && cmd != NODEED_MENU_REVERSE
		&& cmd != NODEED_MENU_FLOORSNAP && cmd != NODEED_MENU_NOISE && cmd != NODEED_MENU_SPRING
		&& cmd != NODEED_MENU_AIM && cmd != NODEED_MENU_MASK) return;
		float cx, cy;
		NodeEdToCanvas(st, x, y, cx, cy);
		float nx = cx - NODE_W / 2.0f;
		float ny = cy - NODE_H / 2.0f;
		if (nx < -4000.0f) nx = -4000.0f;
		if (ny < -4000.0f) ny = -4000.0f;
		MotionNodeType nt = MotionNodeType::Smooth;
	if (cmd == NODEED_MENU_MULTIPLY) nt = MotionNodeType::Multiply;
	else if (cmd == NODEED_MENU_ADD) nt = MotionNodeType::Add;
	else if (cmd == NODEED_MENU_FOOTLOCK) nt = MotionNodeType::FootLock;
	else if (cmd == NODEED_MENU_CLAMP) nt = MotionNodeType::Clamp;
	else if (cmd == NODEED_MENU_MIRROR) nt = MotionNodeType::Mirror;
	else if (cmd == NODEED_MENU_OFFSET) nt = MotionNodeType::Offset;
	else if (cmd == NODEED_MENU_TIMESCALE) nt = MotionNodeType::TimeScale;
	else if (cmd == NODEED_MENU_REVERSE) nt = MotionNodeType::Reverse;
	else if (cmd == NODEED_MENU_FLOORSNAP) nt = MotionNodeType::FloorSnap;
	else if (cmd == NODEED_MENU_NOISE) nt = MotionNodeType::Noise;
	else if (cmd == NODEED_MENU_SPRING) nt = MotionNodeType::Spring;
	else if (cmd == NODEED_MENU_AIM) nt = MotionNodeType::Aim;
	else if (cmd == NODEED_MENU_MASK) nt = MotionNodeType::Mask;
	int id = g_retargeter->nodeGraph().addNode(nt, cx, cy);
	if (id > 0)
	{
		st->selNode = id;
		st->selLinkFrom = -1; st->selLinkTo = -1;
		NodeEdRefresh(st);
	}
}

static void NodeEdDeleteSel(NodeEdState *st)
{
	if (!g_retargeter) return;
	KNodeGraph &g = g_retargeter->nodeGraph();
	if (st->selNode > 0)
	{
		if (g.removeNode(st->selNode))
		{
			st->selNode = -1;
			NodeEdRefresh(st);
			return;
		}
	}
	if (st->selLinkFrom >= 0 && st->selLinkTo >= 0)
	{
		if (g.removeLink(st->selLinkFrom, st->selLinkTo))
		{
			st->selLinkFrom = -1; st->selLinkTo = -1;
			NodeEdRefresh(st);
		}
	}
}

// ----------------------------------------------------------------------------
// Window procedure
// ----------------------------------------------------------------------------

LRESULT CALLBACK NodeEditorProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	NodeEdState *st = NodeEdGet(hWnd);
	switch (msg)
	{
	case WM_CREATE: {
		NodeEdState *s = new NodeEdState();
		memset(s, 0, sizeof(NodeEdState));
		s->hWnd = hWnd;
		s->hMain = GetParent(hWnd);
		s->selNode = -1; s->selLinkFrom = -1; s->selLinkTo = -1;
		s->dragSlider = -1;
		s->zoom = 1.0f;
		s->hFont = CreateFont(16, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
			DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
			CLEARTYPE_QUALITY, DEFAULT_PITCH, L"Segoe UI");
		s->hFontBold = CreateFont(15, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE,
			DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
			CLEARTYPE_QUALITY, DEFAULT_PITCH, L"Segoe UI");
		s->hFontSmall = CreateFont(13, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
			DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
			CLEARTYPE_QUALITY, DEFAULT_PITCH, L"Segoe UI");
		s->hBg = CreateSolidBrush(Theme::BG);
		s->hPanel = CreateSolidBrush(Theme::PANEL);
		s->hNode = CreateSolidBrush(RGB(45, 49, 62));
		s->hNodeSel = CreateSolidBrush(RGB(52, 66, 96));
		s->hPenBorder = CreatePen(PS_SOLID, 1, Theme::BORDER);
		s->hPenAccent = CreatePen(PS_SOLID, 2, Theme::ACCENT);
		s->hPenHot = CreatePen(PS_SOLID, 2, Theme::ACCENT_HOT);
		s->hPenLink = CreatePen(PS_SOLID, 2, Theme::ACCENT);
		s->hPenLinkSel = CreatePen(PS_SOLID, 3, Theme::ACCENT_HOT);
		s->hPenDim = CreatePen(PS_SOLID, 1, RGB(90, 94, 108));
		s->hPenOrange = CreatePen(PS_SOLID, 2, NODEED_ORANGE);
		SetWindowLongPtr(hWnd, GWLP_USERDATA, (LONG_PTR)s);
		return 0;
	}
	case WM_DESTROY: {
		if (st)
		{
			DeleteObject(st->hFont); DeleteObject(st->hFontBold); DeleteObject(st->hFontSmall);
			DeleteObject(st->hBg); DeleteObject(st->hPanel);
			DeleteObject(st->hNode); DeleteObject(st->hNodeSel);
			DeleteObject(st->hPenBorder); DeleteObject(st->hPenAccent); DeleteObject(st->hPenHot);
			DeleteObject(st->hPenLink); DeleteObject(st->hPenLinkSel);
			DeleteObject(st->hPenDim); DeleteObject(st->hPenOrange);
			delete st;
			SetWindowLongPtr(hWnd, GWLP_USERDATA, 0);
		}
		g_hNodeEditorWnd = NULL;
		return 0;
	}
	case WM_SIZE: {
		InvalidateRect(hWnd, NULL, FALSE);
		return 0;
	}
	case WM_ERASEBKGND:
		return 1;
	case WM_PAINT:
		NodeEdPaint(hWnd);
		return 0;
	case WM_LBUTTONDOWN: {
		if (!st) break;
		SetFocus(hWnd);
		int x = GET_X_LPARAM(lParam), y = GET_Y_LPARAM(lParam);
		if (x < NODEED_PANEL_W)
			NodeEdPanelHit(st, x, y, true);
		else
			NodeEdCanvasDown(st, x, y);
		return 0;
	}
	case WM_MOUSEMOVE: {
		if (!st) break;
		int x = GET_X_LPARAM(lParam), y = GET_Y_LPARAM(lParam);
		if (st->dragMode != 0)
		{
			if (st->dragMode == 3)
			{
				MotionNode *n = (g_retargeter) ? g_retargeter->nodeGraph().find(st->selNode) : NULL;
				if (n) { NodeEdSliderSet(st, n, st->dragSlider, x); NodeEdRefresh(st); }
			}
			else
				NodeEdCanvasMove(st, x, y);
		}
		return 0;
	}
	case WM_LBUTTONUP: {
		if (!st) break;
		if (st->dragMode == 1 || st->dragMode == 2)
			NodeEdCanvasUp(st, GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam));
		else if (st->dragMode == 3)
		{
			st->dragMode = 0;
			ReleaseCapture();
			NodeEdRefresh(st);
		}
		return 0;
	}
	case WM_RBUTTONDOWN: {
		if (!st) break;
		int x = GET_X_LPARAM(lParam), y = GET_Y_LPARAM(lParam);
		if (x >= NODEED_PANEL_W)
			NodeEdCanvasMenu(st, x, y);
		return 0;
	}
	case WM_MBUTTONDOWN: {
		if (!st) break;
		int x = GET_X_LPARAM(lParam), y = GET_Y_LPARAM(lParam);
		if (x >= NODEED_PANEL_W)
		{
			SetFocus(hWnd);
			st->dragMode = 4;
			st->panStart.x = x; st->panStart.y = y;
			st->panOX = st->panX; st->panOY = st->panY;
			SetCapture(hWnd);
		}
		return 0;
	}
	case WM_MBUTTONUP: {
		if (!st) break;
		if (st->dragMode == 4)
		{
			st->dragMode = 0;
			ReleaseCapture();
			NodeEdRefresh(st);
		}
		return 0;
	}
	case WM_MOUSEWHEEL: {
		if (!st) break;
		POINT pt = { GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam) };
		ScreenToClient(hWnd, &pt);
		int dz = GET_WHEEL_DELTA_WPARAM(wParam);
		if (pt.x < NODEED_PANEL_W)
		{
			// Panel scroll.
			st->panelScroll -= (dz / 120) * 30;
			if (st->panelScroll < 0) st->panelScroll = 0;
			NodeEdRefresh(st);
		}
		else
		{
			// Canvas zoom anchored at the cursor.
			float nz = st->zoom * ((dz > 0) ? 1.12f : (1.0f / 1.12f));
			if (nz < 0.30f) nz = 0.30f;
			if (nz > 2.50f) nz = 2.50f;
			float cx, cy;
			NodeEdToCanvas(st, pt.x, pt.y, cx, cy);
			st->zoom = nz;
			st->panX = (float)pt.x - NODEED_PANEL_W - cx * nz;
			st->panY = (float)pt.y - cy * nz;
			NodeEdRefresh(st);
		}
		return 0;
	}
	case WM_KEYDOWN: {
		if (!st) break;
		if (wParam == VK_DELETE) { NodeEdDeleteSel(st); return 0; }
		if (wParam == VK_ESCAPE)
		{
			if (st->dragMode == 2) { st->dragMode = 0; ReleaseCapture(); NodeEdRefresh(st); }
			return 0;
		}
		break;
	}
	}
	return DefWindowProc(hWnd, msg, wParam, lParam);
}
