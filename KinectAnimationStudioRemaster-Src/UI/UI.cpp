
// UI.CPP : Defines the entry point for the application.
#include "UI.h"
#include "NodeEditor.h"
#include "..\kinect\KRecording.h"
#include "..\kinect\KDepthReader.h"
#include "..\kinect\KRetargeter.h"
#include "..\kinect\KASProject.h"

#include <commctrl.h>
#include <windowsx.h>
#include <strsafe.h>
#include <string>
#include <vector>
#include <set>

#pragma comment(lib, "comctl32.lib")

#define MAX_LOADSTRING 100
#define TIMER_UI 1

/********************************
Begin of Global Variables:
*****************************/
HINSTANCE hInst;                        // current instance
HWND      ghWnd = NULL;                 // main window

TCHAR szTitle[MAX_LOADSTRING];          // The title bar text
TCHAR szWindowClass[MAX_LOADSTRING];    // the main window class name

TCHAR viewWindowClass[MAX_LOADSTRING];  // the view window class name
TCHAR timelineWindowClass[MAX_LOADSTRING]; // the timeline window class name


char gszOutputFile[_MAX_PATH];          // File name to write to

// Global FBX SDK manager
FbxManager *gSdkManager;

// Global Kinect Sensor
IKinectSensor *gKinectSensor;

// Reponsible for processing incoming frames
KinectFrameProcessor kFrameProcessor;

// Use Smart pointers for our Body readers
KVisualizer_ptr kVisualizer = std::make_shared<KBodyVisualizer>();
KExporter_ptr kExporter = std::make_shared<KBodyExporter>();
std::shared_ptr<KRetargeter> kRetargeter = std::make_shared<KRetargeter>();

// Global session state
AppSession g_session = {
	false,     // livePaused
	false,     // playing
	true,      // loop
	true,      // ghost
	true,      // ghostSymmetry
	false,     // recording
	35.0f,     // smoothStrength
	0,         // playheadMS
	0,         // takeDurationMS
	0,         // playInMS
	0,         // playOutMS
	false,     // hasRange
	true,      // depthView
	true,      // cameraView
	false,     // floorAlign
	true,      // gestureRecord
	0,         // workspace
	false,     // exportBVH
	false,     // showRetargetLive
	false,     // pipetteActive
	-1,        // pipetteJoint
	-1         // pipetteBone (target)
};

// Drag & drop state for bone mapping (imported -> mocap, mocap is always parent)
static bool g_dragActive = false;
static int g_dragJoint = -1;
static int g_dragBone = -1;
static HWND g_dragSource = NULL;
static POINT g_dragStartPt = {0,0};
static HWND g_dragWnd = NULL; // small popup showing bone name attached to cursor
static WCHAR g_dragText[128] = L"";

// Persistent memory of expanded/collapsed bone groups (survives every
// tree rebuild: mapping, import, refresh). Key = bone index.
static std::set<int> s_expandedMain; static int s_expandedMainBones = -1;
static std::set<int> s_expandedEd;   static int s_expandedEdBones = -1;

// Global depth stream reader (feeds the viewport point cloud)
KDepthReader kDepthReader;

// Global pointer used by the timeline & viewport
KBodyExporter *g_pExporter = NULL;

// Global pointer used by the retarget workspace
KRetargeter *g_retargeter = NULL;

// Layout metrics
static const int M = 8;
static const int HEADER_H = 44;
static const int STATUS_H = 26;
static const int TIMELINE_H = 60;
static const int LOG_H = 120;
static const int PANEL_W = 216;

// Draggable splitters (Blender-like): left tree / viewport / right panel / timeline
static int g_splitLeftW = 260;      // left bone-tree width (retarget)
static int g_splitPanelW = PANEL_W; // right panel width
static int g_splitTimelineH = TIMELINE_H; // timeline strip height
static int g_splitDrag = 0;         // 0 none, 1 left, 2 panel, 3 timeline
static int g_splitHover = 0;
static POINT g_splitStartPt = {0,0};
static int g_splitStartL = 0, g_splitStartP = 0, g_splitStartT = 0;

// Shared geometry with LayoutUI (mappingH=0, log hidden)
static void MainSplitGeom(HWND hWnd, int &contentTop, int &midH, int &panelX, int &tlineY, int &cw)
{
	RECT rc; GetClientRect(hWnd, &rc);
	cw = rc.right;
	int ch = rc.bottom;
	contentTop = HEADER_H + M;
	int baseMidH = ch - HEADER_H - STATUS_H - g_splitTimelineH - M * 5;
	midH = baseMidH;
	if (midH < 340) midH = 340;
	panelX = cw - g_splitPanelW - M;
	tlineY = contentTop + midH + M;
}
// Hit test splitters: 1 left tree/viewport, 2 viewport/panel, 3 viewport/timeline
static int MainSplitHit(HWND hWnd, int x, int y)
{
	bool isRetarget = (g_session.workspace == 1);
	int contentTop, midH, panelX, tlineY, cw;
	MainSplitGeom(hWnd, contentTop, midH, panelX, tlineY, cw);
	int y0 = contentTop, y1 = contentTop + midH;
	if (y >= y0 && y <= y1) {
		if (isRetarget) {
			int cx = M + g_splitLeftW + M / 2;
			if (x >= cx - 4 && x <= cx + 4) return 1;
		}
		int px = panelX - M / 2;
		if (x >= px - 4 && x <= px + 4) return 2;
	}
	if (y >= tlineY - M / 2 - 4 && y <= tlineY - M / 2 + 4 && x >= M && x <= cw - M) return 3;
	return 0;
}

// Theme resources
static HBRUSH g_hbrPanel = NULL;
static HBRUSH g_hbrHeader = NULL;
static HBRUSH g_hbrStatus = NULL;
static HBRUSH g_hbrEdit = NULL;
static HFONT  g_fontTitle = NULL;
static HFONT  g_fontSub = NULL;
static HFONT  g_fontSmall = NULL;
static HFONT  g_fontSection = NULL; // 12 semibold: panel section titles
static HFONT  g_fontBtn = NULL;     // 15 semibold: owner-drawn buttons
static HFONT  g_fontCheck = NULL;   // 13 normal: owner-drawn checkboxes

// Status bar strings (updated by the UI timer)
static bool  g_sensorOk = false;
static WCHAR g_statusLeft[160] = L"";
static WCHAR g_statusRight[96] = L"";
/********************************
End of Global Variables:
*****************************/

// Button visual styles
enum class BtnStyle { Record, Stop, Export, Clear, Play, Neutral };

static BtnStyle StyleForId(int id)
{
	switch (id)
	{
	case RECORD_BUTTON: return BtnStyle::Record;
	case STOP_RECORD_BUTTON: return BtnStyle::Stop;
	case EXPORT_TO_BUTTON: return BtnStyle::Export;
	case CLEAR_BUTTON: return BtnStyle::Clear;
	case PLAY_BUTTON: return BtnStyle::Play;
	default: return BtnStyle::Neutral;
	}
}

// Forward declarations
ATOM                UIRegisterClass(HINSTANCE hInstance, WNDPROC WndProc, LPCWSTR lpszClassName, bool isSub = false);
BOOL                InitInstance(HINSTANCE, int);
LRESULT CALLBACK    WndProc(HWND, UINT, WPARAM, LPARAM);
LRESULT CALLBACK    ViewProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK    About(HWND, UINT, WPARAM, LPARAM);

void CreateUIControls(HWND hWndParent);
void ApplyControlFonts(HWND hWnd);
void LayoutUI(HWND hWnd);
void UpdateControlStates(HWND hWnd);
void UpdateLabels(HWND hWnd);
void DrawHeader(HWND hWnd, HDC hdc, RECT &rc);
void DrawStatusBar(HWND hWnd, HDC hdc, RECT &rc);
const WCHAR* formatTimeStatic(INT64 ms);

static HWND CreateFlatButton(HWND parent, int id, LPCWSTR text, int x, int y, int w, int h);
static HWND CreateFlatCheck(HWND parent, int id, LPCWSTR text, int x, int y, int w, int h);
static void DrawFlatButton(DRAWITEMSTRUCT *dis);
static void DrawFlatCheck(DRAWITEMSTRUCT *dis);

bool GetOpenFBXFileName(HWND hWndParent, char *outPath);
bool GetOpenKASFileName(HWND hWndParent, char *outPath);
bool GetSaveKASFileName(HWND hWndParent, char *outPath);
void RefreshRetargetLists(HWND hWnd);
void UpdateWorkspaceVisibility(HWND hWnd);
void SetWorkspace(HWND hWnd, int workspace);
std::string MakeEnglish(const std::string &utf8);

// T-pose window (external mocap hierarchy)
static HWND g_hTPoseWnd = NULL;
static int g_tposeSel = -1;
static POINT g_tposeDragStart = {0,0};
static bool g_tposeDragging = false;
ATOM TPoseRegisterClass(HINSTANCE hInst);
HWND CreateTPoseWindow(HWND hParent);
LRESULT CALLBACK TPoseProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

// Retarget import transform helpers
static float ReadRetargetEdit(HWND hWnd, int id, float def);
static void ApplyImportTransformFromEdits(HWND hWnd);
static void SyncImportTransformEdits(HWND hWnd);

// entry point for the application
int APIENTRY _tWinMain(
	HINSTANCE hInstance,
	HINSTANCE hPrevInstance,
	LPTSTR    lpCmdLine,
	int       nCmdShow
	)
{
	UNREFERENCED_PARAMETER(hPrevInstance);

	INITCOMMONCONTROLSEX icc;
	icc.dwSize = sizeof(icc);
	icc.dwICC = ICC_BAR_CLASSES | ICC_STANDARD_CLASSES;
	InitCommonControlsEx(&icc);

	MSG msg;
	HACCEL hAccelTable;

	// Initialize global strings
	LoadString(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
	LoadString(hInstance, IDC_UI, szWindowClass, MAX_LOADSTRING);
	if (UIRegisterClass(hInstance, WndProc, szWindowClass) == 0)
		return FALSE;

	// Initialize view window class
	LoadString(hInstance, IDC_VIEW, viewWindowClass, MAX_LOADSTRING);
	if (UIRegisterClass(hInstance, ViewProc, viewWindowClass, true) == 0)
		return FALSE;

	// Initialize timeline window class
	StringCchCopy(timelineWindowClass, MAX_LOADSTRING, L"KAS_TimelineClass");
	if (TimelineRegisterClass(hInstance) == 0)
		return FALSE;

	// T-pose window for mocap hierarchy
	if (TPoseRegisterClass(hInstance) == 0)
		return FALSE;

	// Node motion editor window
	if (NodeEditorRegisterClass(hInstance) == 0)
		return FALSE;

	INITCOMMONCONTROLSEX iccTree = { sizeof(iccTree), ICC_TREEVIEW_CLASSES };
	InitCommonControlsEx(&iccTree);

	// empty our global file name buffer
	ZeroMemory(gszOutputFile, sizeof(gszOutputFile));

	// Perform application initialization:
	if (!InitInstance(hInstance, nCmdShow))
	{
		return FALSE;
	}

	hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_UI));

	// Main message loop:
	while (GetMessage(&msg, NULL, 0, 0))
	{
		if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
	}

	return (int)msg.wParam;
}

//
//  FUNCTION: UIRegisterClass()
//
//  PURPOSE: Registers the window class.
//
ATOM UIRegisterClass(
	HINSTANCE hInstance, WNDPROC Proc, LPCWSTR lpszClassName, bool isSub
	)
{
	WNDCLASSEX wcex;

	wcex.cbSize = sizeof(WNDCLASSEX);
	wcex.style = CS_HREDRAW | CS_VREDRAW;
	wcex.lpfnWndProc = Proc;
	wcex.cbClsExtra = 0;
	wcex.cbWndExtra = 0;
	wcex.hInstance = hInstance;
	wcex.hIcon = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_UI));
	wcex.hCursor = LoadCursor(NULL, IDC_ARROW);
	wcex.hbrBackground = NULL;   // we paint everything ourselves
	wcex.lpszMenuName = isSub ? NULL : MAKEINTRESOURCE(IDC_UI);
	wcex.lpszClassName = lpszClassName;
	wcex.hIconSm = LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_UI));

	return RegisterClassEx(&wcex);
}

//
//   FUNCTION: InitInstance(HINSTANCE, int)
//
//   PURPOSE: Saves instance handle and creates main window
//
BOOL InitInstance(
	HINSTANCE hInstance,
	int nCmdShow
	)
{
	hInst = hInstance; // Store instance handle in our global variable

	ghWnd = CreateWindow(
		szWindowClass,                        // LPCTSTR lpClassName
		L"Kinect Animation Studio Remaster",  // LPCTSTR caption
		WS_OVERLAPPEDWINDOW | WS_CLIPCHILDREN,// DWORD dwStyle
		CW_USEDEFAULT,                        // int x
		CW_USEDEFAULT,                        // int Y
		1400,                                 // int nWidth
		860,                                  // int nHeight
		NULL,                                 // HWND hWndParent
		NULL,                                 // HMENU hMenu
		hInstance,                            // HINSTANCE hInstance
		NULL                                  // LPVOID lpParam
		);

	if (!ghWnd)
	{
		return FALSE;
	}

	ShowWindow(ghWnd, nCmdShow);
	UpdateWindow(ghWnd);

	return TRUE;
}

// Resolves the active in/out trim range. Returns false when trim is disabled
// or covers the whole take (no effective trimming).
static bool GetTrimRange(INT64 &inMS, INT64 &outMS)
{
	INT64 dur = kExporter ? kExporter->takeDurationMS() : 0;
	inMS = 0; outMS = dur;
	if (!g_session.hasRange)
		return false;
	inMS = g_session.playInMS; if (inMS < 0) inMS = 0; if (inMS > dur) inMS = dur;
	outMS = g_session.playOutMS; if (outMS < 0) outMS = 0; if (outMS > dur) outMS = dur;
	if (inMS > outMS) { INT64 t = inMS; inMS = outMS; outMS = t; }
	if (inMS <= 0 && outMS >= dur)
		return false;   // full-range = no effective trim
	return true;
}

// Steps the playhead by one recorded frame (dir = -1 prev, +1 next)
static void StepFrame(int dir)
{
	if (!kExporter || !kExporter->hasTake())
		return;

	const KRecording &rec = kExporter->recording();
	size_t n = rec.size();
	INT64 dur = kExporter->takeDurationMS();

	size_t idx = 0;
	const KSampleFrame *cur = rec.frameAt(g_session.playheadMS);
	if (cur)
		idx = (size_t)(cur - &rec.frames[0]);

	if (dir < 0)
		g_session.playheadMS = (idx > 0) ? rec.frames[idx - 1].timeMS : 0;
	else
		g_session.playheadMS = (idx + 1 < n) ? rec.frames[idx + 1].timeMS : dur;

	g_session.playing = false;
}

// Repaints everything that depends on the playhead / trim state
static void RefreshPlaybackUI(HWND hWnd)
{
	UpdateControlStates(hWnd);
	UpdateLabels(hWnd);

	HWND hTl = GetDlgItem(hWnd, TIMELINE_WINDOW);
	if (hTl) InvalidateRect(hTl, NULL, FALSE);
	HWND hV = GetDlgItem(hWnd, VIEW_STATUS);
	if (hV) InvalidateRect(hV, NULL, FALSE);

	RECT rc; GetClientRect(hWnd, &rc);
	RECT headR = { 0, 0, rc.right, HEADER_H };
	RECT statusR = { 0, rc.bottom - STATUS_H, rc.right, rc.bottom };
	InvalidateRect(hWnd, &headR, FALSE);
	InvalidateRect(hWnd, &statusR, FALSE);
}

//
//  FUNCTION: WndProc(HWND, UINT, WPARAM, LPARAM)
//
//  PURPOSE:  Processes messages for the main window.
//
static void UpdateDragWindow(POINT ptScreen) {
	if (!g_dragWnd) return;
	RECT rc; GetWindowRect(g_dragWnd, &rc);
	int w = rc.right - rc.left, h = rc.bottom - rc.top;
	MoveWindow(g_dragWnd, ptScreen.x + 14, ptScreen.y + 10, w, h, TRUE);
}
static HFONT g_dragFont = NULL; // kept alive: sent to the drag tooltip via WM_SETFONT
static void CreateDragWindow(HWND hParent, const WCHAR *text, POINT ptScreen) {
	if (g_dragWnd) { DestroyWindow(g_dragWnd); g_dragWnd = NULL; }
	wcsncpy_s(g_dragText, text, _TRUNCATE);
	if (!g_dragFont)
		g_dragFont = CreateFont(12,0,0,0,FW_SEMIBOLD,0,0,0,DEFAULT_CHARSET,OUT_DEFAULT_PRECIS,CLIP_DEFAULT_PRECIS,CLEARTYPE_QUALITY,DEFAULT_PITCH,L"Segoe UI");
	HDC hdc = GetDC(hParent);
	HFONT old = NULL;
	if (hdc && g_dragFont) old = (HFONT)SelectObject(hdc, g_dragFont);
	RECT rc={0,0,0,0};
	if (hdc) DrawText(hdc, text, -1, &rc, DT_CALCRECT|DT_SINGLELINE);
	if (hdc && g_dragFont && old) SelectObject(hdc, old);
	if (hdc) ReleaseDC(hParent, hdc);
	int w = (rc.right - rc.left) + 16, h = (rc.bottom - rc.top) + 8;
	if (w < 40) w = 40; if (h < 16) h = 16;
	g_dragWnd = CreateWindowEx(WS_EX_TOPMOST|WS_EX_TOOLWINDOW, L"STATIC", text, WS_POPUP|WS_BORDER|SS_CENTER,
		ptScreen.x+14, ptScreen.y+10, w, h, hParent, NULL, hInst, NULL);
	if (g_dragWnd) {
		if (g_dragFont) SendMessage(g_dragWnd, WM_SETFONT, (WPARAM)g_dragFont, FALSE);
		ShowWindow(g_dragWnd, SW_SHOW); UpdateWindow(g_dragWnd);
	}
}
static void DestroyDragWindow() { if (g_dragWnd) { DestroyWindow(g_dragWnd); g_dragWnd=NULL; } }

LRESULT CALLBACK WndProc(
	HWND hWnd,
	UINT message,
	WPARAM wParam,
	LPARAM lParam
	)
{
	// Drag & drop: imported bone (TreeView) -> mocap bone (TreeView/List) with name attached to cursor
	if (message == WM_NOTIFY) {
		LPNMHDR hdr = (LPNMHDR)lParam;
		if (hdr && hdr->idFrom == RETARGET_BONE_TREE && hdr->code == TVN_BEGINDRAG) {
			LPNMTREEVIEW tv = (LPNMTREEVIEW)lParam;
			TVITEM item = {0}; item.hItem = tv->itemNew.hItem; item.mask = TVIF_PARAM|TVIF_TEXT; WCHAR buf[128]; item.pszText=buf; item.cchTextMax=128;
			TreeView_GetItem(hdr->hwndFrom, &item);
			g_dragActive = true; g_dragBone = (int)item.lParam; g_dragJoint = -1; g_dragSource = hdr->hwndFrom;
			g_dragStartPt = tv->ptDrag;
			POINT ptScreen = tv->ptDrag; ClientToScreen(hdr->hwndFrom, &ptScreen);
			CreateDragWindow(hWnd, buf, ptScreen);
			SetCapture(hWnd); SetCursor(LoadCursor(NULL, IDC_HAND));
			return 0;
		}
		if (hdr && hdr->idFrom == RETARGET_JOINT_LIST && hdr->code == TVN_BEGINDRAG) {
			LPNMTREEVIEW tv = (LPNMTREEVIEW)lParam;
			TVITEM item = {0}; item.hItem = tv->itemNew.hItem; item.mask = TVIF_PARAM|TVIF_TEXT; WCHAR buf[128]; item.pszText=buf; item.cchTextMax=128;
			TreeView_GetItem(hdr->hwndFrom, &item);
			g_dragActive = true; g_dragJoint = (int)item.lParam; g_dragBone = -1; g_dragSource = hdr->hwndFrom;
			g_dragStartPt = tv->ptDrag;
			POINT ptScreen = tv->ptDrag; ClientToScreen(hdr->hwndFrom, &ptScreen);
			CreateDragWindow(hWnd, buf, ptScreen);
			SetCapture(hWnd); SetCursor(LoadCursor(NULL, IDC_HAND));
			return 0;
		}
		// Remember expanded/collapsed groups of the imported-bones tree
		if (hdr && hdr->idFrom == RETARGET_BONE_TREE && hdr->code == TVN_ITEMEXPANDED) {
			LPNMTREEVIEW tv = (LPNMTREEVIEW)lParam;
			int bone = (int)tv->itemNew.lParam;
			if (tv->action & TVE_EXPAND) s_expandedMain.insert(bone);
			else s_expandedMain.erase(bone);
			if (g_retargeter) s_expandedMainBones = g_retargeter->boneCount();
		}
	}
	// Blender-like draggable splitters (cursor + press before bone drag)
	if (message == WM_SETCURSOR && (HWND)wParam == hWnd) {
		if (GetCapture() == hWnd && g_splitDrag) {
			SetCursor(LoadCursor(NULL, g_splitDrag == 3 ? IDC_SIZENS : IDC_SIZEWE));
			return TRUE;
		}
		POINT pt; GetCursorPos(&pt); ScreenToClient(hWnd, &pt);
		int z = (!g_dragActive) ? MainSplitHit(hWnd, pt.x, pt.y) : 0;
		if (z != g_splitHover) { g_splitHover = z; InvalidateRect(hWnd, NULL, FALSE); }
		if (z) { SetCursor(LoadCursor(NULL, z == 3 ? IDC_SIZENS : IDC_SIZEWE)); return TRUE; }
		if (g_splitHover) { g_splitHover = 0; InvalidateRect(hWnd, NULL, FALSE); }
	}
	if (message == WM_LBUTTONDOWN && !g_dragActive && !g_splitDrag) {
		POINT pt = { GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam) };
		int z = MainSplitHit(hWnd, pt.x, pt.y);
		if (z) {
			g_splitDrag = z; g_splitHover = z;
			g_splitStartPt = pt;
			g_splitStartL = g_splitLeftW; g_splitStartP = g_splitPanelW; g_splitStartT = g_splitTimelineH;
			SetCapture(hWnd);
			SetCursor(LoadCursor(NULL, z == 3 ? IDC_SIZENS : IDC_SIZEWE));
			return 0;
		}
	}
	if (message == WM_MOUSEMOVE && g_splitDrag && !g_dragActive) {
		POINT pt; GetCursorPos(&pt); ScreenToClient(hWnd, &pt);
		int dx = pt.x - g_splitStartPt.x, dy = pt.y - g_splitStartPt.y;
		RECT rc; GetClientRect(hWnd, &rc);
		if (g_splitDrag == 1) {
			int v = g_splitStartL + dx;
			int maxL = (rc.right - g_splitPanelW - M) - 2 * M - 220;
			if (v < 140) v = 140;
			if (v > maxL && maxL > 140) v = maxL;
			g_splitLeftW = v;
		} else if (g_splitDrag == 2) {
			int v = g_splitStartP - dx;
			bool isRetarget = (g_session.workspace == 1);
			int viewX = isRetarget ? (M + g_splitLeftW + M) : M;
			int maxP = (rc.right - M) - viewX - 220;
			if (v < 150) v = 150;
			if (v > maxP && maxP > 150) v = maxP;
			if (v > 420) v = 420;
			g_splitPanelW = v;
		} else if (g_splitDrag == 3) {
			int v = g_splitStartT - dy;
			if (v < 34) v = 34;
			if (v > 260) v = 260;
			g_splitTimelineH = v;
		}
		LayoutUI(hWnd);
		InvalidateRect(hWnd, NULL, FALSE);
		return 0;
	}
	if (message == WM_LBUTTONUP && g_splitDrag) {
		g_splitDrag = 0;
		if (GetCapture() == hWnd) ReleaseCapture();
		SetCursor(LoadCursor(NULL, IDC_ARROW));
		InvalidateRect(hWnd, NULL, FALSE);
		return 0;
	}
	if (message == WM_LBUTTONDOWN) {
		POINT pt = { GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam) };
		HWND hJoint = GetDlgItem(hWnd, RETARGET_JOINT_LIST);
		POINT ptScreen = pt; ClientToScreen(hWnd, &ptScreen);
		HWND hOver = WindowFromPoint(ptScreen);
		bool isJointTree = hJoint && (hOver == hJoint || IsChild(hJoint, hOver));
		// Check if over joint TreeView (mocap hierarchy) to start drag
		if (isJointTree) {
			POINT ptc = ptScreen; ScreenToClient(hJoint, &ptc);
			TVHITTESTINFO ht={0}; ht.pt=ptc; TreeView_HitTest(hJoint, &ht);
			if (ht.hItem && (ht.flags & TVHT_ONITEM)) {
				TVITEM tvi={0}; tvi.mask=TVIF_PARAM; tvi.hItem=ht.hItem; TreeView_GetItem(hJoint,&tvi);
				int jt = (int)tvi.lParam;
				g_dragActive = true; g_dragJoint = jt; g_dragBone = -1; g_dragSource = hJoint;
				g_dragStartPt = pt; SetCapture(hWnd); SetCursor(LoadCursor(NULL, IDC_HAND));
				return 0;
			}
		}
	}
	if (message == WM_MOUSEMOVE && g_dragActive) {
		POINT pt; GetCursorPos(&pt);
		UpdateDragWindow(pt);
		// Encadre la cible survolée (bone qui va recevoir le drop)
		HWND hJoint = GetDlgItem(hWnd, RETARGET_JOINT_LIST);
		HWND hTree = GetDlgItem(hWnd, RETARGET_BONE_TREE);
		POINT ptScreen = pt;
		HWND hOver = WindowFromPoint(ptScreen);
		// Clear previous drop highlight
		if (hJoint) TreeView_SelectDropTarget(hJoint, NULL);
		if (hTree) TreeView_SelectDropTarget(hTree, NULL);
		if (hJoint && (hOver == hJoint || IsChild(hJoint, hOver))) {
			POINT ptc = ptScreen; ScreenToClient(hJoint, &ptc);
			TVHITTESTINFO ht={0}; ht.pt=ptc; TreeView_HitTest(hJoint, &ht);
			if (ht.hItem) TreeView_SelectDropTarget(hJoint, ht.hItem);
		} else if (hTree && (hOver == hTree || IsChild(hTree, hOver))) {
			POINT ptc = ptScreen; ScreenToClient(hTree, &ptc);
			TVHITTESTINFO ht={0}; ht.pt=ptc; TreeView_HitTest(hTree, &ht);
			if (ht.hItem) TreeView_SelectDropTarget(hTree, ht.hItem);
		} else {
			HWND hView = GetDlgItem(hWnd, VIEW_STATUS);
			if (hView && (hOver == hView || IsChild(hView, hOver)) && kVisualizer && g_dragBone >= 0) {
				POINT ptView = pt; ScreenToClient(hView, &ptView);
				int joint = kVisualizer->pickGhostJointPublic((float)ptView.x, (float)ptView.y);
				// ghost highlight handled in viewport draw, just invalidate
				InvalidateRect(hView, NULL, FALSE);
			}
		}
		SetCursor(LoadCursor(NULL, IDC_HAND));
		return 0;
	}
	if (message == WM_LBUTTONUP && g_dragActive) {
		POINT pt = { GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam) };
		HWND hJoint = GetDlgItem(hWnd, RETARGET_JOINT_LIST);
		HWND hTree = GetDlgItem(hWnd, RETARGET_BONE_TREE);
		POINT ptScreen = pt; ClientToScreen(hWnd, &ptScreen);
		HWND hOver = WindowFromPoint(ptScreen);
		if (hJoint) TreeView_SelectDropTarget(hJoint, NULL);
		if (hTree) TreeView_SelectDropTarget(hTree, NULL);
		// Joint (mocap parent) -> Bone (imported child) : drop on TreeView
		if (g_dragJoint >= 0 && hTree && (hOver == hTree || IsChild(hTree, hOver))) {
			TVHITTESTINFO ht = {0}; ht.pt = pt; ScreenToClient(hTree, &ht.pt);
			TreeView_HitTest(hTree, &ht);
			if (ht.hItem) {
				TVITEM tvi={0}; tvi.mask=TVIF_PARAM; tvi.hItem=ht.hItem; TreeView_GetItem(hTree, &tvi);
				int bone = (int)tvi.lParam;
				if (bone >=0 && g_retargeter) {
					g_retargeter->setMapping(g_dragJoint, bone);
					RefreshRetargetLists(hWnd);
					for(int i=0;i<g_retargetJointCount;++i) if(g_retargetJoints[i].jointType==g_dragJoint){
						UI_Printf("Drag: %s -> %s.", g_retargetJoints[i].label, MakeEnglish(g_retargeter->boneName(bone)).c_str()); break;
					}
				}
			}
		}
		// Imported bone (enfant) -> Mocap joint (parent) : drop bone onto joint TreeView — encadré
		else if (g_dragBone >= 0 && hJoint) {
			POINT ptc = ptScreen; ScreenToClient(hJoint, &ptc);
			TVHITTESTINFO ht={0}; ht.pt=ptc;
			HTREEITEM hItem = TreeView_HitTest(hJoint, &ht);
			if (hItem && (ht.flags & TVHT_ONITEM)) {
				TVITEM tvi={0}; tvi.mask=TVIF_PARAM; tvi.hItem=hItem; TreeView_GetItem(hJoint,&tvi);
				int joint = (int)tvi.lParam;
				if (g_retargeter) {
					g_retargeter->setMapping(joint, g_dragBone);
					RefreshRetargetLists(hWnd);
					for(int i=0;i<g_retargetJointCount;++i) if(g_retargetJoints[i].jointType==joint){
						UI_Printf("Drag: %s -> %s.", g_retargetJoints[i].label, MakeEnglish(g_retargeter->boneName(g_dragBone)).c_str()); break;
					}
				}
			}
		}
		else {
			HWND hView = GetDlgItem(hWnd, VIEW_STATUS);
			if (hView && (hOver == hView || IsChild(hView, hOver)) && kVisualizer) {
				if (g_dragJoint >= 0) {
					POINT ptView = pt; ScreenToClient(hView, &ptView);
					int bone = kVisualizer->pickRetargetBonePublic((float)ptView.x, (float)ptView.y);
					if (bone >= 0 && g_retargeter) {
						g_retargeter->setMapping(g_dragJoint, bone);
						RefreshRetargetLists(hWnd);
						for(int i=0;i<g_retargetJointCount;++i) if(g_retargetJoints[i].jointType==g_dragJoint){
							UI_Printf("Drag: %s -> %s (3D).", g_retargetJoints[i].label, MakeEnglish(g_retargeter->boneName(bone)).c_str()); break;
						}
					}
				} else if (g_dragBone >= 0) {
					POINT ptView2 = pt; ScreenToClient(hView, &ptView2);
					int joint = kVisualizer->pickGhostJointPublic((float)ptView2.x, (float)ptView2.y);
					if (joint >=0 && g_retargeter) {
						g_retargeter->setMapping(joint, g_dragBone);
						RefreshRetargetLists(hWnd);
						for(int i=0;i<g_retargetJointCount;++i) if(g_retargetJoints[i].jointType==joint){
							UI_Printf("Drag: %s -> %s (3D).", g_retargetJoints[i].label, MakeEnglish(g_retargeter->boneName(g_dragBone)).c_str()); break;
						}
					}
				}
			}
		}
		DestroyDragWindow();
		g_dragActive=false; g_dragJoint=-1; g_dragBone=-1; g_dragSource=NULL;
		ReleaseCapture(); SetCursor(LoadCursor(NULL, IDC_ARROW));
		InvalidateRect(hWnd, NULL, FALSE);
		return 0;
	}
	switch (message)
	{

	case WM_CREATE:
	{
		// Needed right away: UI_Printf does nothing while ghWnd is NULL, which
		// swallowed every launch-time message (notably "No ready Kinect
		// found!") emitted further down in this same handler.
		ghWnd = hWnd;

		// Create the UI
		CreateUIControls(hWnd);

		// Theme resources
		g_hbrPanel = CreateSolidBrush(Theme::PANEL);
		g_hbrHeader = CreateSolidBrush(Theme::PANEL);
		g_hbrStatus = CreateSolidBrush(Theme::PANEL);
		g_hbrEdit = CreateSolidBrush(Theme::EDIT_BG);

		g_fontTitle = CreateFont(20, 0, 0, 0, FW_SEMIBOLD, 0, 0, 0, DEFAULT_CHARSET,
			OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY,
			DEFAULT_PITCH | FF_DONTCARE, L"Segoe UI");
		g_fontSub = CreateFont(12, 0, 0, 0, FW_NORMAL, 0, 0, 0, DEFAULT_CHARSET,
			OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY,
			DEFAULT_PITCH | FF_DONTCARE, L"Segoe UI");
		g_fontSmall = CreateFont(11, 0, 0, 0, FW_NORMAL, 0, 0, 0, DEFAULT_CHARSET,
			OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY,
			DEFAULT_PITCH | FF_DONTCARE, L"Segoe UI");
		g_fontSection = CreateFont(12, 0, 0, 0, FW_SEMIBOLD, 0, 0, 0, DEFAULT_CHARSET,
			OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY,
			DEFAULT_PITCH | FF_DONTCARE, L"Segoe UI");
		g_fontBtn = CreateFont(15, 0, 0, 0, FW_SEMIBOLD, 0, 0, 0, DEFAULT_CHARSET,
			OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY,
			DEFAULT_PITCH | FF_DONTCARE, L"Segoe UI");
		g_fontCheck = CreateFont(13, 0, 0, 0, FW_NORMAL, 0, 0, 0, DEFAULT_CHARSET,
			OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY,
			DEFAULT_PITCH | FF_DONTCARE, L"Segoe UI");

		ApplyControlFonts(hWnd);

		// Initialize FBX SDK Manager
		InitializeSdkManager();

		// Initialize Kinect Sensor
		g_sensorOk = SUCCEEDED(InitializeDefaultSensor(&gKinectSensor));

		// Associate the Kinect Exporter with the FBX SDK manager
		kExporter->initFBXSDKManager(gSdkManager);

		// Associate frame reader Kinect frame processor
		kFrameProcessor.init(gKinectSensor);

		// Start the depth stream reader for the viewport point cloud
		kDepthReader.init(gKinectSensor);

		// Wire the visualizer
		g_pExporter = kExporter.get();
		g_retargeter = kRetargeter.get();
		kVisualizer->setExporter(kExporter.get());
		kVisualizer->setRetargeter(g_retargeter);
		kVisualizer->setDepthReader(&kDepthReader);
		kVisualizer->setSmoothing(g_session.smoothStrength);
		kExporter->setSmoothing(g_session.smoothStrength);

		// Attach visualizer to window VIEW_STATUS
		HWND hWndView = GetDlgItem(hWnd, VIEW_STATUS);
		if (hWndView && !kVisualizer->is_attached())
		{
			HRESULT hr = kVisualizer->attach(hWndView);
			if (FAILED(hr))
			{
				UI_Printf("Failed to attach to Kinect Viewing Window.");
			}
		}

		// kExporter and kVisualizer need to subscribe to the frame processor
		kFrameProcessor.subscribe(std::dynamic_pointer_cast<KBodyReader>(kExporter));
		kFrameProcessor.subscribe(std::dynamic_pointer_cast<KBodyReader>(kVisualizer));

		// Refresh the UI at ~30 Hz
		SetTimer(hWnd, TIMER_UI, 33, NULL);

		LayoutUI(hWnd);
		UpdateControlStates(hWnd);
		UpdateLabels(hWnd);
	}
	break;

	case WM_GETMINMAXINFO:
	{
		MINMAXINFO *mmi = (MINMAXINFO*)lParam;
		mmi->ptMinTrackSize.x = 1100;
		mmi->ptMinTrackSize.y = 800;
	}
	return 0;

	case WM_SIZE:
		if (wParam != SIZE_MINIMIZED)
			LayoutUI(hWnd);
		return 0;

	case WM_GESTURE_RECORD:
		// Fist-gesture countdown finished (wParam: 1 = start, 0 = stop)
		if (kExporter)
		{
			if (wParam)
			{
				if (!kExporter->recordingStatus())
				{
					g_session.playing = false;
					g_session.playheadMS = 0;
					kExporter->startRecording();
					g_session.recording = true;
				}
			}
			else
			{
				if (kExporter->recordingStatus())
				{
					kExporter->stopRecording();
					// Drop the frames polluted by the countdown stop pose
					// (fists held closed for 5 s before the gesture fires)
					kExporter->trimRecordingEnd(6000);
					g_session.recording = false;
				}
			}
		}
		return 0;

	case WM_TIMER:
		if (wParam == TIMER_UI)
		{
			// Pull the frames the recording thread staged into the take, before
			// anything below (and the upcoming paint) reads the take.
			if (kExporter) kExporter->flushPendingFrames();

			// Refresh take duration while recording
			if (g_session.recording && kExporter)
				g_session.takeDurationMS = kExporter->takeDurationMS();

			// Advance playback (respecting the in/out trim range)
			INT64 dur = g_session.takeDurationMS;
			if (dur <= 0 && kExporter)
				dur = kExporter->takeDurationMS();

			INT64 inMS = 0, outMS = dur;
			GetTrimRange(inMS, outMS);

			if (g_session.playing)
			{
				g_session.playheadMS += 33;

				if (outMS > 0 && g_session.playheadMS >= outMS)
				{
					if (g_session.loop)
						g_session.playheadMS = inMS;
					else
					{
						g_session.playheadMS = outMS;
						g_session.playing = false;
					}
				}
			}

			HWND hTl = GetDlgItem(hWnd, TIMELINE_WINDOW);
			if (hTl) InvalidateRect(hTl, NULL, FALSE);

			HWND hV = GetDlgItem(hWnd, VIEW_STATUS);
			if (hV) InvalidateRect(hV, NULL, FALSE);

			RECT rc; GetClientRect(hWnd, &rc);
			RECT headR = { 0, 0, rc.right, HEADER_H };
			RECT statusR = { 0, rc.bottom - STATUS_H, rc.right, rc.bottom };
			InvalidateRect(hWnd, &headR, FALSE);
			InvalidateRect(hWnd, &statusR, FALSE);

			UpdateControlStates(hWnd);
			UpdateLabels(hWnd);
		}
		return 0;

	case WM_COMMAND:
	{
		int wmId = LOWORD(wParam);
		int wmEvent = HIWORD(wParam);

		// Menu selections
		switch (wmId)
		{
		case IDM_ABOUT:
			DialogBox(hInst, MAKEINTRESOURCE(IDD_ABOUTBOX), hWnd, About);
			break;

		case IDM_TPOSE_EDITOR:
			CreateTPoseWindow(hWnd);
			break;

		case IDM_NODE_EDITOR:
			CreateNodeEditorWindow(hWnd);
			break;

		case IDM_SHOW_LOG:
			{
				HWND hLog = GetDlgItem(hWnd, EXECUTE_STATUS);
				if (hLog) {
					ShowWindow(hLog, IsWindowVisible(hLog) ? SW_HIDE : SW_SHOW);
					LayoutUI(hWnd);
					UI_Printf("Log %s.", IsWindowVisible(hLog) ? "shown" : "hidden");
				}
			}
			break;

		case IDM_EXIT:
			DestroyWindow(hWnd);
			break;

		case IDM_EXPORT_FBX:
		case EXPORT_TO_BUTTON:
			if (wmEvent == BN_CLICKED || wmId == IDM_EXPORT_FBX)
			{
				ZeroMemory(gszOutputFile, sizeof(gszOutputFile));
				GetOutputFileName(hWnd, gszOutputFile);
				if (gszOutputFile[0] != '\0')
				{
					if (g_session.hasRange)
						kExporter->exportTake(gszOutputFile, g_session.playInMS, g_session.playOutMS);
					else
						kExporter->exportTake(gszOutputFile);
				}
			}
			break;

		case IDM_CLEAR_TAKE:
		case CLEAR_BUTTON:
		case IDM_DELETE_TIMELINE:
			if (wmEvent == BN_CLICKED || wmId == IDM_CLEAR_TAKE || wmId == IDM_DELETE_TIMELINE)
			{
				if (kExporter && kExporter->recordingStatus())
				{
					kExporter->stopRecording();
					g_session.recording = false;
				}
				if (kExporter) kExporter->clearTake();
				g_session.playheadMS = 0;
				g_session.takeDurationMS = 0;
				g_session.playing = false;
				g_session.hasRange = false;
				UpdateControlStates(hWnd);
				UpdateLabels(hWnd);
				HWND hTl = GetDlgItem(hWnd, TIMELINE_WINDOW);
				if (hTl) InvalidateRect(hTl, NULL, FALSE);
				HWND hView = GetDlgItem(hWnd, VIEW_STATUS);
				if (hView) InvalidateRect(hView, NULL, FALSE);
				UI_Printf("Timeline deleted.");
			}
			break;

		case IDM_NEW_SESSION:
			if (wmEvent == BN_CLICKED || wmId == IDM_NEW_SESSION)
			{
				if (kExporter && kExporter->recordingStatus())
				{
					kExporter->stopRecording();
					g_session.recording = false;
				}
				if (kExporter) kExporter->clearTake();
				g_session.playheadMS = 0;
				g_session.takeDurationMS = 0;
				g_session.playing = false;
				g_session.hasRange = false;
				g_session.pipetteActive = false;
				g_session.pipetteJoint = -1;
				if (kVisualizer) kVisualizer->resetGhostPose();
				UpdateControlStates(hWnd);
				UpdateLabels(hWnd);
				HWND hTl2 = GetDlgItem(hWnd, TIMELINE_WINDOW);
				if (hTl2) InvalidateRect(hTl2, NULL, FALSE);
				HWND hView2 = GetDlgItem(hWnd, VIEW_STATUS);
				if (hView2) InvalidateRect(hView2, NULL, FALSE);
				UI_Printf("New session ready.");
			}
			break;

		case IDM_SAVE_PROJECT:
		{
			if (!kExporter || !g_retargeter || !g_retargeter->hasCharacter())
			{
				UI_Printf("Nothing to save: import a character first.");
				break;
			}
			char path[_MAX_PATH] = "";
			if (GetSaveKASFileName(hWnd, path) && path[0] != '\0')
			{
				if (KASProject::save(path, g_retargeter, kExporter.get(), kVisualizer.get()))
					UI_Printf("Project saved: %s", path);
				else
					UI_Printf("Failed to save project: %s", path);
			}
		}
		break;

		case IDM_OPEN_PROJECT:
		{
			char path[_MAX_PATH] = "";
			if (GetOpenKASFileName(hWnd, path) && path[0] != '\0')
			{
				if (kExporter && g_retargeter && KASProject::load(path, g_retargeter, kExporter.get(), kVisualizer.get()))
				{
					g_session.playheadMS = 0;
					g_session.playing = false;
					g_session.recording = false;
					SyncImportTransformEdits(hWnd);
					RefreshRetargetLists(hWnd);
					SetWorkspace(hWnd, 1);
					UpdateControlStates(hWnd);
					UpdateLabels(hWnd);
					HWND hView = GetDlgItem(hWnd, VIEW_STATUS);
					if (hView) InvalidateRect(hView, NULL, FALSE);
					HWND hTl = GetDlgItem(hWnd, TIMELINE_WINDOW);
					if (hTl) InvalidateRect(hTl, NULL, FALSE);
					UI_Printf("Project loaded: %s", path);
				}
				else
				{
					UI_Printf("Failed to load project: %s", path);
				}
			}
		}
		break;

		case RECORD_BUTTON:
			if (kExporter->recordingStatus())
			{
				kExporter->stopRecording();
				g_session.recording = false;
			}
			else
			{
				g_session.playing = false;
				g_session.playheadMS = 0;
				kExporter->startRecording();
				g_session.recording = true;
			}
			break;

		case STOP_RECORD_BUTTON:
			if (kExporter->recordingStatus())
			{
				kExporter->stopRecording();
				g_session.recording = false;
			}
			break;

		case PLAY_BUTTON:
		{
			if (g_session.playing)
			{
				g_session.playing = false;
			}
			else
			{
				if (kExporter->hasTake())
				{
					INT64 dur = kExporter->takeDurationMS();
					INT64 inMS = 0, outMS = dur;
					if (GetTrimRange(inMS, outMS))
					{
						if (g_session.playheadMS < inMS || g_session.playheadMS > outMS)
							g_session.playheadMS = inMS;
					}
					else if (g_session.playheadMS >= dur)
						g_session.playheadMS = 0;
					g_session.playing = true;
				}
			}
		}
		break;

		case FIRST_FRAME_BUTTON:
		case IDM_GO_START:
			g_session.playing = false;
			g_session.playheadMS = 0;
			RefreshPlaybackUI(hWnd);
			break;

		case PREV_FRAME_BUTTON:
		case IDM_PREV_FRAME:
			StepFrame(-1);
			RefreshPlaybackUI(hWnd);
			break;

		case NEXT_FRAME_BUTTON:
		case IDM_NEXT_FRAME:
			StepFrame(+1);
			RefreshPlaybackUI(hWnd);
			break;

		case LAST_FRAME_BUTTON:
		case IDM_GO_END:
			g_session.playing = false;
			if (kExporter)
				g_session.playheadMS = kExporter->takeDurationMS();
			RefreshPlaybackUI(hWnd);
			break;

		case IDM_SET_IN:
			g_session.playInMS = g_session.playheadMS;
			g_session.hasRange = true;
			RefreshPlaybackUI(hWnd);
			UI_Printf("In point set at %s.", formatTimeStatic(g_session.playInMS));
			break;

		case IDM_SET_OUT:
			g_session.playOutMS = g_session.playheadMS;
			g_session.hasRange = true;
			RefreshPlaybackUI(hWnd);
			UI_Printf("Out point set at %s.", formatTimeStatic(g_session.playOutMS));
			break;

		case IDM_CLEAR_RANGE:
			g_session.hasRange = false;
			g_session.playInMS = 0;
			g_session.playOutMS = 0;
			RefreshPlaybackUI(hWnd);
			UI_Printf("In/out points cleared.");
			break;

		case IDM_TOGGLE_TRIM:
			g_session.hasRange = !g_session.hasRange;
			RefreshPlaybackUI(hWnd);
			break;

		case IDM_RESET_CAM:
			kVisualizer->resetCamera();
			break;

		case IDM_LOOP:
			g_session.loop = !g_session.loop;
			CheckMenuItem(GetMenu(hWnd), IDM_LOOP,
				MF_BYCOMMAND | (g_session.loop ? MF_CHECKED : MF_UNCHECKED));
			break;

		case IDM_GHOST:
			g_session.ghost = !g_session.ghost;
			CheckMenuItem(GetMenu(hWnd), IDM_GHOST,
				MF_BYCOMMAND | (g_session.ghost ? MF_CHECKED : MF_UNCHECKED));
			break;

		case IDM_PAUSE_LIVE:
			g_session.livePaused = !g_session.livePaused;
			CheckMenuItem(GetMenu(hWnd), IDM_PAUSE_LIVE,
				MF_BYCOMMAND | (g_session.livePaused ? MF_CHECKED : MF_UNCHECKED));
			break;

		case LOOP_CHECK:
			if (wmEvent == BN_CLICKED)
			{
				g_session.loop = !g_session.loop;
				InvalidateRect((HWND)lParam, NULL, TRUE);
				CheckMenuItem(GetMenu(hWnd), IDM_LOOP,
					MF_BYCOMMAND | (g_session.loop ? MF_CHECKED : MF_UNCHECKED));
			}
			break;

		case GHOST_CHECK:
			if (wmEvent == BN_CLICKED)
			{
				g_session.ghost = !g_session.ghost;
				InvalidateRect((HWND)lParam, NULL, TRUE);
				CheckMenuItem(GetMenu(hWnd), IDM_GHOST,
					MF_BYCOMMAND | (g_session.ghost ? MF_CHECKED : MF_UNCHECKED));
			}
			break;

		case PAUSE_CHECK:
			if (wmEvent == BN_CLICKED)
			{
				g_session.livePaused = !g_session.livePaused;
				InvalidateRect((HWND)lParam, NULL, TRUE);
				CheckMenuItem(GetMenu(hWnd), IDM_PAUSE_LIVE,
					MF_BYCOMMAND | (g_session.livePaused ? MF_CHECKED : MF_UNCHECKED));
			}
			break;

		case DEPTH_CHECK:
			if (wmEvent == BN_CLICKED)
			{
				g_session.depthView = !g_session.depthView;
				InvalidateRect((HWND)lParam, NULL, TRUE);
			}
			break;

		case CAMERA_CHECK:
			if (wmEvent == BN_CLICKED)
			{
				g_session.cameraView = !g_session.cameraView;
				InvalidateRect((HWND)lParam, NULL, TRUE);
			}
			break;

		case FLOOR_CHECK:
			if (wmEvent == BN_CLICKED)
			{
				g_session.floorAlign = !g_session.floorAlign;
				InvalidateRect((HWND)lParam, NULL, TRUE);
			}
			break;

		case GESTURE_CHECK:
			if (wmEvent == BN_CLICKED)
			{
				g_session.gestureRecord = !g_session.gestureRecord;
				InvalidateRect((HWND)lParam, NULL, TRUE);
			}
			break;

		case SHOW_RETARGET_LIVE_CHECK:
			if (wmEvent == BN_CLICKED)
			{
				g_session.showRetargetLive = !g_session.showRetargetLive;
				InvalidateRect((HWND)lParam, NULL, TRUE);
				HWND hView = GetDlgItem(hWnd, VIEW_STATUS);
				if (hView) InvalidateRect(hView, NULL, FALSE);
				UI_Printf("Live retarget %s.", g_session.showRetargetLive ? "shown" : "hidden");
			}
			break;

		case WORKSPACE_MOCAP_TAB:
			if (wmEvent == BN_CLICKED)
				SetWorkspace(hWnd, 0);
			break;

		case WORKSPACE_RETARGET_TAB:
			if (wmEvent == BN_CLICKED)
				SetWorkspace(hWnd, 1);
			break;

		case IMPORT_FBX_BUTTON:
			if (wmEvent == BN_CLICKED)
			{
				char path[_MAX_PATH] = "";
				if (GetOpenFBXFileName(hWnd, path) && path[0] != '\0')
				{
					if (g_retargeter && g_retargeter->loadCharacter(path))
					{
						g_retargeter->autoMap();
						SyncImportTransformEdits(hWnd);
						RefreshRetargetLists(hWnd);
						HWND hView = GetDlgItem(hWnd, VIEW_STATUS);
						if (hView) { InvalidateRect(hView, NULL, FALSE); UpdateWindow(hView); }
						UI_Printf("Character loaded: %s (%d bones%s).",
							MakeEnglish(g_retargeter->characterName()).c_str(),
							g_retargeter->boneCount(),
							g_retargeter->mesh().valid ? ", mesh" : ", no mesh");
					}
					else
					{
						UI_Printf("Failed to import character from %s.", path);
					}
				}
			}
			break;

		case AUTOMAP_BUTTON:
			if (wmEvent == BN_CLICKED && g_retargeter)
			{
				g_retargeter->autoMap();
				RefreshRetargetLists(hWnd);
				UI_Printf("Auto-mapping done. Open the mapping window to refine.");
			}
			break;

		case AUTOMAP_POS_BUTTON:
			if (wmEvent == BN_CLICKED && g_retargeter)
			{
				g_retargeter->autoMapByPosition();
				RefreshRetargetLists(hWnd);
				UI_Printf("Position auto-mapping done (ghost reference). Open the mapping window to refine.");
			}
			break;

		case SIZE_EDIT:
		case ROT_X_EDIT:
		case ROT_Y_EDIT:
		case ROT_Z_EDIT:
			if (wmEvent == EN_KILLFOCUS && g_retargeter && g_retargeter->hasCharacter())
				ApplyImportTransformFromEdits(hWnd);
			break;

		case APPLY_TRANSFORM_BUTTON:
			if (wmEvent == BN_CLICKED && g_retargeter && g_retargeter->hasCharacter())
			{
				ApplyImportTransformFromEdits(hWnd);
				UI_Printf("Import transform applied: size %.3f, rotation X %.1f Y %.1f Z %.1f.",
					ReadRetargetEdit(hWnd, SIZE_EDIT, 1.0f),
					ReadRetargetEdit(hWnd, ROT_X_EDIT, 0.0f),
					ReadRetargetEdit(hWnd, ROT_Y_EDIT, 0.0f),
					ReadRetargetEdit(hWnd, ROT_Z_EDIT, 0.0f));
			}
			break;

		case RETARGET_PLAY_BUTTON:
			if (wmEvent == BN_CLICKED)
			{
				if (g_session.playing)
					g_session.playing = false;
				else if (kExporter->hasTake())
				{
					INT64 dur = kExporter->takeDurationMS();
					INT64 inMS = 0, outMS = dur;
					if (GetTrimRange(inMS, outMS))
					{
						if (g_session.playheadMS < inMS || g_session.playheadMS > outMS)
							g_session.playheadMS = inMS;
					}
					else if (g_session.playheadMS >= dur)
						g_session.playheadMS = 0;
					g_session.playing = true;
				}
			}
			break;

		case EXPORT_RETARGET_BUTTON:
			if (wmEvent == BN_CLICKED && g_retargeter)
			{
				char path[_MAX_PATH] = "";
				ZeroMemory(gszOutputFile, sizeof(gszOutputFile));
				GetOutputFileName(hWnd, gszOutputFile);
				if (gszOutputFile[0] != '\0')
				{
					if (g_session.exportBVH)
					{
						// Force the .bvh extension
						char *dot = strrchr(gszOutputFile, '.');
						if (dot)
							*dot = '\0';
						strcat_s(gszOutputFile, ".bvh");
						g_retargeter->exportRetargetBVH(gszOutputFile);
					}
					else
					{
						g_retargeter->exportRetarget(gszOutputFile);
					}
				}
			}
			break;

		case MAPPING_ASSIGN_BUTTON:
			if (wmEvent == BN_CLICKED && g_retargeter)
			{
				int jSel = (int)SendMessage(GetDlgItem(hWnd, RETARGET_JOINT_LIST), LB_GETCURSEL, 0, 0);
				int bSel = (int)SendMessage(GetDlgItem(hWnd, RETARGET_BONE_LIST), LB_GETCURSEL, 0, 0);
				if (jSel >= 0 && jSel < g_retargetJointCount && bSel >= 0 && bSel < g_retargeter->boneCount())
				{
					g_retargeter->setMapping(g_retargetJoints[jSel].jointType, bSel);
					RefreshRetargetLists(hWnd);
					UI_Printf("Mapped %s -> %s.", g_retargetJoints[jSel].label,
						MakeEnglish(g_retargeter->boneName(bSel)).c_str());
				}
			}
			break;

		case MAPPING_UNASSIGN_BUTTON:
			if (wmEvent == BN_CLICKED && g_retargeter)
			{
				int jSel = (int)SendMessage(GetDlgItem(hWnd, RETARGET_JOINT_LIST), LB_GETCURSEL, 0, 0);
				if (jSel >= 0 && jSel < g_retargetJointCount)
				{
					g_retargeter->setMapping(g_retargetJoints[jSel].jointType, -1);
					RefreshRetargetLists(hWnd);
				}
			}
			break;

		case RESET_GHOST_BUTTON:
			if (wmEvent == BN_CLICKED && kVisualizer)
			{
				kVisualizer->resetGhostPose();
				InvalidateRect(GetDlgItem(hWnd, VIEW_STATUS), NULL, TRUE);
				UI_Printf("Ghost reference pose reset to the default T-pose.");
			}
			break;

		case SYMMETRY_CHECK:
			if (wmEvent == BN_CLICKED)
			{
				g_session.ghostSymmetry = !g_session.ghostSymmetry;
				InvalidateRect((HWND)lParam, NULL, TRUE);
			}
			break;

		case PIPETTE_BUTTON:
			// Deprecated — drag & drop replaces pipette. Kept hidden for compat.
			break;

		case RETARGET_CLEAR_BUTTON:
			if (wmEvent == BN_CLICKED)
			{
				if (kExporter) {
					kExporter->clearTake();
					g_session.playheadMS = 0;
					g_session.playing = false;
					g_session.hasRange = false;
					UpdateControlStates(hWnd);
					UpdateLabels(hWnd);
					HWND hTl = GetDlgItem(hWnd, TIMELINE_WINDOW);
					if (hTl) InvalidateRect(hTl, NULL, FALSE);
					HWND hView = GetDlgItem(hWnd, VIEW_STATUS);
					if (hView) InvalidateRect(hView, NULL, FALSE);
					UI_Printf("Timeline deleted.");
				}
			}
			break;

		case NEW_SESSION_BUTTON:
			if (wmEvent == BN_CLICKED)
			{
				if (kExporter) {
					if (g_session.recording) { UI_Printf("Stop recording first."); break; }
					kExporter->clearTake();
					g_session.playheadMS = 0;
					g_session.playing = false;
					g_session.hasRange = false;
					g_session.pipetteActive = false;
					g_session.pipetteJoint = -1;
					if (kVisualizer) kVisualizer->resetGhostPose();
					UpdateControlStates(hWnd);
					UpdateLabels(hWnd);
					HWND hTl = GetDlgItem(hWnd, TIMELINE_WINDOW);
					if (hTl) InvalidateRect(hTl, NULL, FALSE);
					HWND hView = GetDlgItem(hWnd, VIEW_STATUS);
					if (hView) InvalidateRect(hView, NULL, FALSE);
					UI_Printf("New session ready — take cleared, ghost reset.");
				}
			}
			break;

		case BVH_EXPORT_CHECK:
			if (wmEvent == BN_CLICKED)
			{
				g_session.exportBVH = !g_session.exportBVH;
				InvalidateRect((HWND)lParam, NULL, TRUE);
			}
			break;

		case RETARGET_JOINT_LIST:
			// Drag & drop: start drag on left button down, pipette on double-click kept for compat
			if (wmEvent == LBN_DBLCLK && g_retargeter)
			{
				int jSel = (int)SendMessage((HWND)lParam, LB_GETCURSEL, 0, 0);
				if (jSel >= 0 && jSel < g_retargetJointCount) {
					g_session.pipetteActive = true;
					g_session.pipetteJoint = g_retargetJoints[jSel].jointType;
					UI_Printf("Pipette active for %s — click a bone in hierarchy or 3D view.", g_retargetJoints[jSel].label);
					HWND hView = GetDlgItem(hWnd, VIEW_STATUS);
					if (hView) SetCursor(LoadCursor(NULL, IDC_CROSS));
					InvalidateRect((HWND)lParam, NULL, TRUE);
				}
			}
			else if (wmEvent == LBN_SELCHANGE && g_retargeter)
			{
				// Highlight the currently mapped bone for the selected joint
				int jSel = (int)SendMessage((HWND)lParam, LB_GETCURSEL, 0, 0);
				HWND hBone = GetDlgItem(hWnd, RETARGET_BONE_LIST);
				if (hBone && jSel >= 0 && jSel < g_retargetJointCount)
				{
					int jointType = g_retargetJoints[jSel].jointType;
					int boneIdx = g_retargeter->getMapping(jointType);
					SendMessage(hBone, LB_SETCURSEL, (WPARAM)boneIdx, 0);
					// Show the pairing in the viewport (green ghost / orange bone)
					if (kVisualizer)
					{
						kVisualizer->setSelection(jointType, boneIdx);
						InvalidateRect(GetDlgItem(hWnd, VIEW_STATUS), NULL, TRUE);
					}
				}
			}
			break;

		case RETARGET_BONE_LIST:
			if (wmEvent == LBN_SELCHANGE && g_retargeter)
			{
				int bSel = (int)SendMessage((HWND)lParam, LB_GETCURSEL, 0, 0);
				int jointType = -1;
				if (bSel >= 0 && bSel < g_retargeter->boneCount())
					jointType = g_retargeter->mappingJointForBone(bSel);
				// Keep the joint list in sync with the bone selection
				if (jointType >= 0)
				{
					HWND hJoint = GetDlgItem(hWnd, RETARGET_JOINT_LIST);
					if (hJoint)
					{
						for (int i = 0; i < g_retargetJointCount; ++i)
						{
							if (g_retargetJoints[i].jointType == jointType)
							{
								SendMessage(hJoint, LB_SETCURSEL, (WPARAM)i, 0);
								break;
							}
						}
					}
				}
				if (kVisualizer)
				{
					kVisualizer->setSelection(jointType, bSel);
					InvalidateRect(GetDlgItem(hWnd, VIEW_STATUS), NULL, TRUE);
				}
			}
			break;

		default:
			return DefWindowProc(hWnd, message, wParam, lParam);
		}
	}
	break;

	case WM_HSCROLL:
		if ((HWND)lParam == GetDlgItem(hWnd, SMOOTH_SLIDER))
		{
			LRESULT pos = SendMessage((HWND)lParam, TBM_GETPOS, 0, 0);
			g_session.smoothStrength = (float)pos;
			kVisualizer->setSmoothing((float)pos);
			kExporter->setSmoothing((float)pos);
			UpdateLabels(hWnd);
		}
		return 0;

	case WM_CTLCOLORSTATIC:
	case WM_CTLCOLOREDIT:
	{
		HDC hdc = (HDC)wParam;
		HWND ctl = (HWND)lParam;

		if (message == WM_CTLCOLOREDIT)
		{
			SetTextColor(hdc, Theme::TEXT);
			SetBkColor(hdc, Theme::EDIT_BG);
			return (LRESULT)g_hbrEdit;
		}

		SetBkMode(hdc, TRANSPARENT);
		int id = GetDlgCtrlID(ctl);
		if (id == LOOP_CHECK || id == GHOST_CHECK || id == PAUSE_CHECK ||
			id == DEPTH_CHECK || id == CAMERA_CHECK || id == FLOOR_CHECK ||
			id == GESTURE_CHECK)
			SetTextColor(hdc, Theme::TEXT);
		else if (id == CAPTURE_LABEL || id == PLAYBACK_LABEL || id == SMOOTH_LABEL)
			SetTextColor(hdc, Theme::ACCENT);
		else if (id == FRAME_INFO_LABEL || id == SMOOTH_VALUE || id == TAKE_INFO_LABEL ||
			id == RETARGET_TAKE_LABEL || id == RETARGET_INFO_LABEL)
			SetTextColor(hdc, Theme::TEXT);
		else
			SetTextColor(hdc, Theme::TEXT_DIM);
		return (LRESULT)g_hbrPanel;
	}

	case WM_CTLCOLORLISTBOX:
	{
		HDC hdcLb = (HDC)wParam;
		SetTextColor(hdcLb, Theme::TEXT);
		SetBkColor(hdcLb, Theme::EDIT_BG);
		return (LRESULT)g_hbrEdit;
	}

	case WM_DRAWITEM:
	{
		DRAWITEMSTRUCT *dis = (DRAWITEMSTRUCT*)lParam;
		if (dis->CtlType == ODT_BUTTON)
		{
			if (dis->CtlID == LOOP_CHECK || dis->CtlID == GHOST_CHECK || dis->CtlID == PAUSE_CHECK ||
				dis->CtlID == DEPTH_CHECK || dis->CtlID == CAMERA_CHECK || dis->CtlID == FLOOR_CHECK ||
				dis->CtlID == GESTURE_CHECK || dis->CtlID == SHOW_RETARGET_LIVE_CHECK ||
				dis->CtlID == SYMMETRY_CHECK || dis->CtlID == BVH_EXPORT_CHECK)
				DrawFlatCheck(dis);
			else
				DrawFlatButton(dis);
		}
		return TRUE;
	}

	case WM_PAINT:
	{
		PAINTSTRUCT ps;
		BeginPaint(hWnd, &ps);

		RECT rc;
		GetClientRect(hWnd, &rc);

		// Base background
		FillRect(ps.hdc, &rc, g_hbrPanel);

		// Blender-like splitter grips (subtle, brighter on hover/drag)
		{
			bool isRetarget = (g_session.workspace == 1);
			int contentTop, midH, panelX, tlineY, cw;
			MainSplitGeom(hWnd, contentTop, midH, panelX, tlineY, cw);
			int hot = (g_splitDrag ? g_splitDrag : g_splitHover);
			COLORREF gc = (hot ? Theme::ACCENT : Theme::BORDER);
			HPEN gp = CreatePen(PS_SOLID, 3, gc);
			HPEN go = (HPEN)SelectObject(ps.hdc, gp);
			int y0 = contentTop + 6, y1 = contentTop + midH - 6;
			if (y1 > y0) {
				if (isRetarget) {
					int cx = M + g_splitLeftW + M / 2;
					MoveToEx(ps.hdc, cx, (g_splitDrag == 1 || g_splitHover == 1) ? y0 : (y0 + y1) / 2 - 30, NULL);
					LineTo(ps.hdc, cx, (g_splitDrag == 1 || g_splitHover == 1) ? y1 : (y0 + y1) / 2 + 30);
				}
				int px = panelX - M / 2;
				MoveToEx(ps.hdc, px, (g_splitDrag == 2 || g_splitHover == 2) ? y0 : (y0 + y1) / 2 - 30, NULL);
				LineTo(ps.hdc, px, (g_splitDrag == 2 || g_splitHover == 2) ? y1 : (y0 + y1) / 2 + 30);
			}
			int ty = tlineY - M / 2;
			int tx0 = (g_splitDrag == 3 || g_splitHover == 3) ? M : cw / 2 - 40;
			int tx1 = (g_splitDrag == 3 || g_splitHover == 3) ? cw - M : cw / 2 + 40;
			MoveToEx(ps.hdc, tx0, ty, NULL);
			LineTo(ps.hdc, tx1, ty);
			SelectObject(ps.hdc, go); DeleteObject(gp);
		}

		RECT header = { 0, 0, rc.right, HEADER_H };
		RECT status = { 0, rc.bottom - STATUS_H, rc.right, rc.bottom };

		DrawHeader(hWnd, ps.hdc, header);
		DrawStatusBar(hWnd, ps.hdc, status);

		EndPaint(hWnd, &ps);
	}
	return 0;

	case WM_ERASEBKGND:
		return 1;

	case WM_DESTROY:
	{
		KillTimer(hWnd, TIMER_UI);

		// Stop the capture threads BEFORE the sensor or the FBX manager go
		// away: both readers keep using them until their loop has joined.
		kFrameProcessor.stop();
		kDepthReader.stop();

		// Close Kinect Sensor
		CloseDefaultSensor(gKinectSensor);

		// dont forget to delete the SdkManager
		DestroySdkObjects(gSdkManager, true);
		gSdkManager = NULL;

		// Release theme resources
		if (g_hbrPanel) DeleteObject(g_hbrPanel);
		if (g_hbrHeader) DeleteObject(g_hbrHeader);
		if (g_hbrStatus) DeleteObject(g_hbrStatus);
		if (g_hbrEdit) DeleteObject(g_hbrEdit);
		if (g_fontTitle) DeleteObject(g_fontTitle);
		if (g_fontSub) DeleteObject(g_fontSub);
		if (g_fontSmall) DeleteObject(g_fontSmall);
		if (g_fontSection) DeleteObject(g_fontSection);
		if (g_fontBtn) DeleteObject(g_fontBtn);
		if (g_fontCheck) DeleteObject(g_fontCheck);
		g_fontTitle = g_fontSub = g_fontSmall = NULL;
		g_fontSection = g_fontBtn = g_fontCheck = NULL;

		PostQuitMessage(0);
	}
	break;

	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
	}
	return 0;
}

//
//  FUNCTION: ViewProc(HWND, UINT, WPARAM, LPARAM)
//
//  PURPOSE:  Processes messages for the 3D view window
//
LRESULT CALLBACK ViewProc(
	HWND hWnd,
	UINT message,
	WPARAM wParam,
	LPARAM lParam
	)
{
	switch (message)
	{
	case WM_PAINT:
	{
		if (!kVisualizer->is_attached())
			kVisualizer->attach(hWnd);
		kVisualizer->update();
	}
	return 0;

	case WM_ERASEBKGND:
		return 1;

	case WM_SIZE:
	{
		int w = LOWORD(lParam);
		int h = HIWORD(lParam);
		kVisualizer->resize(w, h);
	}
	return 0;

	case WM_SETCURSOR:
		if (g_session.pipetteActive) { SetCursor(LoadCursor(NULL, IDC_CROSS)); return TRUE; }
		break;

	case WM_LBUTTONDOWN:
	{
		if (g_session.pipetteActive && g_session.pipetteJoint >= 0 && g_retargeter && kVisualizer) {
			int bone = kVisualizer->pickRetargetBonePublic((float)GET_X_LPARAM(lParam), (float)GET_Y_LPARAM(lParam));
			HWND hMain = GetParent(hWnd);
			if (bone >= 0) {
				g_retargeter->setMapping(g_session.pipetteJoint, bone);
				if (hMain) RefreshRetargetLists(hMain);
				for (int i=0;i<g_retargetJointCount;++i) if (g_retargetJoints[i].jointType==g_session.pipetteJoint) {
					UI_Printf("Pipette: %s -> %s.", g_retargetJoints[i].label, MakeEnglish(g_retargeter->boneName(bone)).c_str());
					break;
				}
			} else {
				UI_Printf("Pipette: no bone under cursor.");
			}
			g_session.pipetteActive = false;
			g_session.pipetteJoint = -1;
			SetCursor(LoadCursor(NULL, IDC_ARROW));
			InvalidateRect(hWnd, NULL, FALSE);
			if (hMain) {
				InvalidateRect(GetDlgItem(hMain, RETARGET_JOINT_LIST), NULL, TRUE);
				InvalidateRect(GetDlgItem(hMain, PIPETTE_BUTTON), NULL, TRUE);
			}
			return 0;
		}
		SetFocus(hWnd);
		bool shift = (GetKeyState(VK_SHIFT) & 0x8000) != 0;
		bool ctrl = (GetKeyState(VK_CONTROL) & 0x8000) != 0;
		kVisualizer->mouseDown((float)GET_X_LPARAM(lParam), (float)GET_Y_LPARAM(lParam), 0, shift, ctrl);
		SetCapture(hWnd);
	}
	return 0;

	case WM_MBUTTONDOWN:
	{
		SetFocus(hWnd);
		bool shift = (GetKeyState(VK_SHIFT) & 0x8000) != 0;
		bool ctrl = (GetKeyState(VK_CONTROL) & 0x8000) != 0;
		kVisualizer->mouseDown((float)GET_X_LPARAM(lParam), (float)GET_Y_LPARAM(lParam), 1, shift, ctrl);
		SetCapture(hWnd);
	}
	return 0;

	case WM_RBUTTONDOWN:
	{
		SetFocus(hWnd);
		bool shift = (GetKeyState(VK_SHIFT) & 0x8000) != 0;
		bool ctrl = (GetKeyState(VK_CONTROL) & 0x8000) != 0;
		kVisualizer->mouseDown((float)GET_X_LPARAM(lParam), (float)GET_Y_LPARAM(lParam), 2, shift, ctrl);
		SetCapture(hWnd);
	}
	return 0;

	case WM_MOUSEMOVE:
		if (g_session.pipetteActive) { InvalidateRect(hWnd, NULL, FALSE); return 0; }
		kVisualizer->mouseMove((float)GET_X_LPARAM(lParam), (float)GET_Y_LPARAM(lParam));
		return 0;

	case WM_LBUTTONUP:
	case WM_MBUTTONUP:
	case WM_RBUTTONUP:
		kVisualizer->mouseUp();
		if (GetCapture() == hWnd)
			ReleaseCapture();
		return 0;

	case WM_LBUTTONDBLCLK:
	case WM_MBUTTONDBLCLK:
		kVisualizer->resetCamera();
		return 0;

	case WM_MOUSEWHEEL:
	{
		int delta = GET_WHEEL_DELTA_WPARAM(wParam);
		kVisualizer->mouseWheel((float)delta);
	}
	return 0;

	case WM_KEYDOWN:
	{
		switch (wParam)
		{
		case VK_ESCAPE:
			if (g_session.pipetteActive) {
				g_session.pipetteActive = false;
				g_session.pipetteJoint = -1;
				SetCursor(LoadCursor(NULL, IDC_ARROW));
				HWND hMain = GetParent(hWnd);
				if (hMain) {
					InvalidateRect(GetDlgItem(hMain, PIPETTE_BUTTON), NULL, TRUE);
					UI_Printf("Pipette cancelled.");
				}
				InvalidateRect(hWnd, NULL, FALSE);
				return 0;
			}
			break;
		case VK_HOME:
			kVisualizer->resetCamera();
			return 0;
		case VK_SPACE:
			SendMessage(GetParent(hWnd), WM_COMMAND, MAKEWPARAM(PLAY_BUTTON, BN_CLICKED), (LPARAM)hWnd);
			return 0;
		case VK_LEFT:
			SendMessage(GetParent(hWnd), WM_COMMAND, MAKEWPARAM(PREV_FRAME_BUTTON, BN_CLICKED), 0);
			return 0;
		case VK_RIGHT:
			SendMessage(GetParent(hWnd), WM_COMMAND, MAKEWPARAM(NEXT_FRAME_BUTTON, BN_CLICKED), 0);
			return 0;
		case '1':
		case VK_NUMPAD1:
			kVisualizer->setViewPreset(0);
			return 0;
		case '3':
		case VK_NUMPAD3:
			kVisualizer->setViewPreset(1);
			return 0;
		case '7':
		case VK_NUMPAD7:
			kVisualizer->setViewPreset(2);
			return 0;
		case '0':
		case VK_NUMPAD0:
			kVisualizer->setViewPreset(3);
			return 0;
		default:
			break;
		}
	}
	return 0;

	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
	}
	return 0;
}

// Message handler for about box.
INT_PTR CALLBACK About(
	HWND hDlg,
	UINT message,
	WPARAM wParam,
	LPARAM lParam
	)
{
	UNREFERENCED_PARAMETER(lParam);
	switch (message)
	{
	case WM_INITDIALOG:
	{
		HINSTANCE hinst = (HINSTANCE)(LONG_PTR)GetWindowLongPtr(hDlg, GWLP_HINSTANCE);
		HICON hIconFBX = (HICON)LoadImage(hinst, MAKEINTRESOURCE(IDI_UI), IMAGE_ICON, 16, 16, LR_DEFAULTSIZE);
		::SendMessage(hDlg, WM_SETICON, ICON_SMALL, (LPARAM)hIconFBX);
	}
	return (INT_PTR)TRUE;

	case WM_COMMAND:
		if (LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL)
		{
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}
		break;
	}
	return (INT_PTR)FALSE;
}

// Draws the application header
void DrawHeader(HWND hWnd, HDC hdc, RECT &rc)
{
	FillRect(hdc, &rc, g_hbrHeader);

	// Bottom separator
	HPEN pen = CreatePen(PS_SOLID, 1, Theme::BORDER);
	HPEN oldPen = (HPEN)SelectObject(hdc, pen);
	MoveToEx(hdc, 0, rc.bottom - 1, NULL);
	LineTo(hdc, rc.right, rc.bottom - 1);
	SelectObject(hdc, oldPen);
	DeleteObject(pen);

	// Title
	SetBkMode(hdc, TRANSPARENT);
	SetTextColor(hdc, Theme::TEXT);
	HFONT oldFont = (HFONT)SelectObject(hdc, g_fontTitle);
	RECT title = { 14, 6, rc.right - 300, 30 };
	DrawText(hdc, L"Kinect Animation Studio Remaster", -1, &title, DT_LEFT | DT_VCENTER | DT_SINGLELINE);

	// Subtitle
	SelectObject(hdc, g_fontSub);
	SetTextColor(hdc, Theme::TEXT_DIM);
	RECT sub = { 15, 27, rc.right - 300, rc.bottom };
	DrawText(hdc, L"Realtime motion capture  \x00B7  export to FBX", -1, &sub, DT_LEFT | DT_VCENTER | DT_SINGLELINE);

	// Sensor status (right side)
	SelectObject(hdc, g_fontSmall);
	WCHAR sText[64];
	StringCchPrintf(sText, _countof(sText), g_sensorOk ? L"Sensor: connected" : L"Sensor: not found");
	SetTextColor(hdc, g_sensorOk ? Theme::GREEN : Theme::DANGER);
	RECT sRect = { rc.right - 260, 8, rc.right - 14, 36 };
	DrawText(hdc, sText, -1, &sRect, DT_RIGHT | DT_VCENTER | DT_SINGLELINE);

	// Sensor dot
	HBRUSH dotBrush = CreateSolidBrush(g_sensorOk ? Theme::GREEN : Theme::DANGER);
	HBRUSH oldBrush = (HBRUSH)SelectObject(hdc, dotBrush);
	int dotX = rc.right - 268;
	int dotY = 18;
	Ellipse(hdc, dotX - 4, dotY - 4, dotX + 4, dotY + 4);
	SelectObject(hdc, oldBrush);
	DeleteObject(dotBrush);

	SelectObject(hdc, oldFont);
}

// Draws the status bar
void DrawStatusBar(HWND hWnd, HDC hdc, RECT &rc)
{
	FillRect(hdc, &rc, g_hbrStatus);

	HPEN pen = CreatePen(PS_SOLID, 1, Theme::BORDER);
	HPEN oldPen = (HPEN)SelectObject(hdc, pen);
	MoveToEx(hdc, 0, rc.top, NULL);
	LineTo(hdc, rc.right, rc.top);
	SelectObject(hdc, oldPen);
	DeleteObject(pen);

	SetBkMode(hdc, TRANSPARENT);
	SetTextColor(hdc, Theme::TEXT_DIM);
	HFONT oldFont = (HFONT)SelectObject(hdc, g_fontSmall);

	RECT left = { 14, rc.top, rc.right / 2, rc.bottom };
	DrawText(hdc, g_statusLeft, -1, &left, DT_LEFT | DT_VCENTER | DT_SINGLELINE);

	RECT right = { rc.right / 2, rc.top, rc.right - 14, rc.bottom };
	DrawText(hdc, g_statusRight, -1, &right, DT_RIGHT | DT_VCENTER | DT_SINGLELINE);

	SelectObject(hdc, oldFont);
}

// Layouts the main child windows
void LayoutUI(HWND hWnd)
{
	RECT rc;
	GetClientRect(hWnd, &rc);

	int cw = rc.right;
	int ch = rc.bottom;

	int contentTop = HEADER_H + M;
	bool isRetarget = (g_session.workspace == 1);
	const int mappingH = 0; // mapping supprimé — drag & drop direct

	// Plus de log en bas (externalisé via Help), on rend l'espace au viewport.
	// Les dimensions viennent des splitters (redimensionnables comme Blender).
	int baseMidH = ch - HEADER_H - STATUS_H - g_splitTimelineH - M * 5;
	int midH = baseMidH;
	if (midH < 340) midH = 340;

	int panelX = cw - g_splitPanelW - M;
	int panelY = contentTop;

	// Workspace tabs (in the header)
	int tabY = 8;
	int tabH = 30;
	int tabW = 116;
	int tabX = cw - 268 - tabW * 2 - 12;
	HWND hTab1 = GetDlgItem(hWnd, WORKSPACE_MOCAP_TAB);
	if (hTab1)
		MoveWindow(hTab1, tabX, tabY, tabW, tabH, TRUE);
	HWND hTab2 = GetDlgItem(hWnd, WORKSPACE_RETARGET_TAB);
	if (hTab2)
		MoveWindow(hTab2, tabX + tabW + 8, tabY, tabW, tabH, TRUE);

	// 3D viewport + left bone hierarchy (Retarget only, Unreal-like).
	// g_splitLeftW is driven by the draggable splitter (Blender-like).
	int leftTreeW = g_splitLeftW;
	HWND hTree = GetDlgItem(hWnd, RETARGET_BONE_TREE);
	HWND hView = GetDlgItem(hWnd, VIEW_STATUS);
	if (isRetarget && hTree && hView) {
		ShowWindow(hTree, SW_SHOW);
		MoveWindow(hTree, M, contentTop, leftTreeW, midH, TRUE);
		MoveWindow(hView, M + leftTreeW + M, contentTop, panelX - (M + leftTreeW + M) - M, midH, TRUE);
	} else {
		if (hTree) ShowWindow(hTree, SW_HIDE);
		if (hView) MoveWindow(hView, M, contentTop, panelX - M * 2, midH, TRUE);
	}

	// Mapping strip supprimé sur demande — drag & drop direct entre les deux hiérarchies
	if (isRetarget)
	{
		HWND hJoint = GetDlgItem(hWnd, RETARGET_JOINT_LIST);
		if (hJoint) { MoveWindow(hJoint, 0,0,0,0, FALSE); ShowWindow(hJoint, SW_HIDE); }
		HWND hBone = GetDlgItem(hWnd, RETARGET_BONE_LIST);
		if (hBone) { MoveWindow(hBone, 0,0,0,0, FALSE); ShowWindow(hBone, SW_HIDE); }
		HWND hAssign = GetDlgItem(hWnd, MAPPING_ASSIGN_BUTTON);
		if (hAssign) { MoveWindow(hAssign, 0,0,0,0, FALSE); ShowWindow(hAssign, SW_HIDE); }
		HWND hUnassign = GetDlgItem(hWnd, MAPPING_UNASSIGN_BUTTON);
		if (hUnassign) { MoveWindow(hUnassign, 0,0,0,0, FALSE); ShowWindow(hUnassign, SW_HIDE); }
		HWND hNote = GetDlgItem(hWnd, MAPPING_NOTE_LABEL);
		if (hNote) { MoveWindow(hNote, 0,0,0,0, FALSE); ShowWindow(hNote, SW_HIDE); }
	}

	// Timeline — always visible (both workspaces), height via splitter
	int tlineY = contentTop + midH + M + (isRetarget ? (mappingH + M) : 0);
	HWND hTl = GetDlgItem(hWnd, TIMELINE_WINDOW);
	if (hTl)
		MoveWindow(hTl, M, tlineY, cw - M * 2, g_splitTimelineH, TRUE);

	// Mocap hierarchy déplacée en fenêtre externe T-Pose (plus au centre)
	HWND hJoint = GetDlgItem(hWnd, RETARGET_JOINT_LIST);
	if (hJoint) { MoveWindow(hJoint, 0,0,0,0, FALSE); ShowWindow(hJoint, SW_HIDE); }
	// Log — hidden in main window, external via Help → Show Log
	HWND hLog = GetDlgItem(hWnd, EXECUTE_STATUS);
	if (hLog) {
		MoveWindow(hLog, 0,0,0,0, FALSE);
		ShowWindow(hLog, SW_HIDE);
	}
	int logY = tlineY + TIMELINE_H + M; (void)logY;

	// Panel children per workspace
	struct PChild { int id; int x, y, w, h; };
	// Vertical rhythm: section 18 / gap 6 / buttons 30+4 / checks 20+2 / transport 26 / info 16+6.
	static const PChild mocapChildren[] = {
		{ CAPTURE_LABEL, 8, 4, 200, 18 },
		{ RECORD_BUTTON, 8, 26, 200, 30 },
		{ STOP_RECORD_BUTTON, 8, 60, 200, 30 },
		{ EXPORT_TO_BUTTON, 8, 94, 200, 30 },
		{ CLEAR_BUTTON, 8, 128, 200, 30 },
		{ PLAYBACK_LABEL, 8, 166, 200, 18 },
		{ PLAY_BUTTON, 8, 188, 200, 30 },
		{ FIRST_FRAME_BUTTON, 8, 222, 44, 26 },
		{ PREV_FRAME_BUTTON, 56, 222, 44, 26 },
		{ NEXT_FRAME_BUTTON, 104, 222, 44, 26 },
		{ LAST_FRAME_BUTTON, 152, 222, 44, 26 },
		{ FRAME_INFO_LABEL, 8, 252, 200, 16 },
		{ LOOP_CHECK, 8, 272, 200, 20 },
		{ GHOST_CHECK, 8, 294, 200, 20 },
		{ PAUSE_CHECK, 8, 316, 200, 20 },
		{ DEPTH_CHECK, 8, 338, 200, 20 },
		{ CAMERA_CHECK, 8, 360, 200, 20 },
		{ FLOOR_CHECK, 8, 382, 200, 20 },
		{ GESTURE_CHECK, 8, 404, 200, 20 },
		{ SHOW_RETARGET_LIVE_CHECK, 8, 426, 200, 20 },
		{ SMOOTH_LABEL, 8, 452, 200, 18 },
		{ SMOOTH_SLIDER, 8, 474, 200, 26 },
		{ SMOOTH_VALUE, 8, 504, 200, 16 },
		{ TAKE_INFO_LABEL, 8, 524, 200, 18 },
		{ NEW_SESSION_BUTTON, 8, 546, 200, 26 },
	};
	static const PChild retargetChildren[] = {
		{ CAPTURE_LABEL, 8, 4, 200, 18 },
		{ RECORD_BUTTON, 8, 26, 200, 30 },
		{ STOP_RECORD_BUTTON, 8, 60, 200, 30 },
		{ IMPORT_FBX_BUTTON, 8, 94, 200, 30 },
		{ AUTOMAP_BUTTON, 8, 128, 200, 30 },
		{ AUTOMAP_POS_BUTTON, 8, 162, 200, 30 },
		{ RETARGET_PLAY_BUTTON, 8, 196, 200, 30 },
		{ FIRST_FRAME_BUTTON, 8, 230, 44, 26 },
		{ PREV_FRAME_BUTTON, 56, 230, 44, 26 },
		{ NEXT_FRAME_BUTTON, 104, 230, 44, 26 },
		{ LAST_FRAME_BUTTON, 152, 230, 44, 26 },
		{ FRAME_INFO_LABEL, 8, 260, 200, 16 },
		{ EXPORT_RETARGET_BUTTON, 8, 280, 200, 30 },
		{ BVH_EXPORT_CHECK, 8, 314, 200, 20 },
		{ RETARGET_TAKE_LABEL, 8, 340, 200, 18 },
		{ RETARGET_INFO_LABEL, 8, 362, 200, 18 },
		{ SIZE_LABEL, 8, 388, 60, 18 },
		{ SIZE_EDIT, 70, 386, 130, 22 },
		{ ROT_LABEL, 8, 414, 60, 18 },
		{ ROT_X_EDIT, 70, 412, 42, 22 },
		{ ROT_Y_EDIT, 116, 412, 42, 22 },
		{ ROT_Z_EDIT, 162, 412, 38, 22 },
		{ APPLY_TRANSFORM_BUTTON, 8, 438, 200, 26 },
		{ RESET_GHOST_BUTTON, 8, 468, 200, 26 },
		{ SYMMETRY_CHECK, 8, 498, 200, 20 },
		{ RETARGET_CLEAR_BUTTON, 8, 522, 200, 26 },
		{ NEW_SESSION_BUTTON, 8, 552, 200, 26 },
	};

	const PChild *children = isRetarget ? retargetChildren : mocapChildren;
	int nChildren = isRetarget ? _countof(retargetChildren) : _countof(mocapChildren);

	for (int i = 0; i < nChildren; ++i)
	{
		HWND hCtrl = GetDlgItem(hWnd, children[i].id);
		if (hCtrl)
			MoveWindow(hCtrl, panelX + children[i].x, panelY + children[i].y,
				children[i].w, children[i].h, TRUE);
	}

	UpdateWorkspaceVisibility(hWnd);
}

// Shows / hides the controls that belong to each workspace
void UpdateWorkspaceVisibility(HWND hWnd)
{
	bool isRetarget = (g_session.workspace == 1);

	static const int mocapOnly[] = {
		EXPORT_TO_BUTTON, CLEAR_BUTTON, PLAYBACK_LABEL, PLAY_BUTTON,
		LOOP_CHECK, GHOST_CHECK, PAUSE_CHECK, DEPTH_CHECK, CAMERA_CHECK,
		FLOOR_CHECK, GESTURE_CHECK, SHOW_RETARGET_LIVE_CHECK, SMOOTH_LABEL, SMOOTH_SLIDER,
		SMOOTH_VALUE, TAKE_INFO_LABEL,
	};
	static const int retargetOnly[] = {
		IMPORT_FBX_BUTTON, AUTOMAP_BUTTON, AUTOMAP_POS_BUTTON,
		RETARGET_PLAY_BUTTON, EXPORT_RETARGET_BUTTON, BVH_EXPORT_CHECK,
		RETARGET_TAKE_LABEL, RETARGET_INFO_LABEL, SIZE_LABEL, SIZE_EDIT, ROT_LABEL,
		ROT_X_EDIT, ROT_Y_EDIT, ROT_Z_EDIT, APPLY_TRANSFORM_BUTTON,
		RESET_GHOST_BUTTON, SYMMETRY_CHECK, PIPETTE_BUTTON, RETARGET_CLEAR_BUTTON,
		RETARGET_JOINT_LIST, RETARGET_BONE_LIST, MAPPING_ASSIGN_BUTTON,
		MAPPING_UNASSIGN_BUTTON, MAPPING_NOTE_LABEL,
	};

	for (int i = 0; i < _countof(mocapOnly); ++i)
	{
		HWND h = GetDlgItem(hWnd, mocapOnly[i]);
		if (h)
			ShowWindow(h, isRetarget ? SW_HIDE : SW_SHOW);
	}
	for (int i = 0; i < _countof(retargetOnly); ++i)
	{
		HWND h = GetDlgItem(hWnd, retargetOnly[i]);
		if (h)
			ShowWindow(h, isRetarget ? SW_SHOW : SW_HIDE);
	}
}

// ----------------------------------------------------------------------------
// Display-only translation of non-English (mainly Chinese) skeleton names.
// The stored FBX names are kept untouched; only the strings shown in the
// UI are converted so they stay readable in English.
// ----------------------------------------------------------------------------

// Decodes a UTF-8 string into Unicode code points
static void DecodeUTF8(const std::string &utf8, std::vector<unsigned int> &out)
{
	for (size_t i = 0; i < utf8.size();)
	{
		unsigned char c = (unsigned char)utf8[i];
		unsigned int cp;
		if (c < 0x80) { cp = c; i += 1; }
		else if ((c >> 5) == 0x6 && i + 1 < utf8.size())
		{ cp = ((c & 0x1F) << 6) | ((unsigned char)utf8[i + 1] & 0x3F); i += 2; }
		else if ((c >> 4) == 0xE && i + 2 < utf8.size())
		{
			cp = ((c & 0x0F) << 12) | (((unsigned char)utf8[i + 1] & 0x3F) << 6) |
				((unsigned char)utf8[i + 2] & 0x3F);
			i += 3;
		}
		else { cp = c; i += 1; }
		out.push_back(cp);
	}
}

// Converts a string that may contain Chinese skeleton terms to an
// English-readable form. ASCII characters are kept; unknown CJK characters
// are dropped from the result.
std::string MakeEnglish(const std::string &utf8)
{
	std::vector<unsigned int> cps;
	DecodeUTF8(utf8, cps);

	// Multi-character phrases (checked first)
	struct Phrase { const unsigned int *cps; int n; const char *en; };
	static const unsigned int p_pelvis[]      = { 0x9AA8, 0x76C6 }; // 骨盆
	static const unsigned int p_spine1[]      = { 0x810A, 0x67F1 }; // 脊柱
	static const unsigned int p_spine2[]      = { 0x810A, 0x690E }; // 脊椎
	static const unsigned int p_clavicle[]    = { 0x9501, 0x9AA8 }; // 锁骨
	static const unsigned int p_upperarm[]    = { 0x4E0A, 0x81C2 }; // 上臂
	static const unsigned int p_forearm[]     = { 0x524D, 0x81C2 }; // 前臂
	static const unsigned int p_thigh[]       = { 0x5927, 0x817F }; // 大腿
	static const unsigned int p_lowerleg[]    = { 0x5C0F, 0x817F }; // 小腿
	static const unsigned int p_wrist[]       = { 0x624B, 0x8155 }; // 手腕
	static const unsigned int p_neck[]        = { 0x8116, 0x5B50 }; // 脖子
	static const unsigned int p_finger[]      = { 0x624B, 0x6307 }; // 手指
	static const unsigned int p_toe[]         = { 0x811A, 0x8DDE }; // 脚趾
	static const unsigned int p_ring[]        = { 0x65E0, 0x540D, 0x6307 }; // 无名指
	static const unsigned int p_middle[]      = { 0x4E2D, 0x6307 }; // 中指
	static const unsigned int p_index[]       = { 0x98DF, 0x6307 }; // 食指
	static const unsigned int p_pinky[]       = { 0x5C0F, 0x6307 }; // 小指
	static const unsigned int p_thumb[]       = { 0x62C7, 0x6307 }; // 拇指
	static const unsigned int p_tip[]         = { 0x672B, 0x7AEF }; // 末端
	static const unsigned int p_heel[]        = { 0x811A, 0x8DDF }; // 脚跟
	static const unsigned int p_elbow[]       = { 0x624B, 0x8098 }; // 手肘
	static const unsigned int p_knee[]        = { 0x819D, 0x76D6 }; // 膝盖
	static const unsigned int p_ankle[]       = { 0x811A, 0x8E1D }; // 脚踝
	static const unsigned int p_shoulder[]    = { 0x80A9, 0x5173, 0x8282 }; // 肩关节
	static const unsigned int p_elbowj[]      = { 0x8098, 0x5173, 0x8282 }; // 肘关节
	static const unsigned int p_kneej[]       = { 0x819D, 0x5173, 0x8282 }; // 膝关节
	static const unsigned int p_wristj[]      = { 0x8155, 0x5173, 0x8282 }; // 腕关节
	static const unsigned int p_hipj[]        = { 0x9ACB, 0x5173, 0x8282 }; // 髋关节
	static const Phrase phrases[] = {
		{ p_pelvis,   2, "Pelvis"      },
		{ p_spine1,   2, "Spine"       },
		{ p_spine2,   2, "Spine"       },
		{ p_clavicle, 2, "Clavicle"    },
		{ p_upperarm, 2, "UpperArm"    },
		{ p_forearm,  2, "Forearm"     },
		{ p_thigh,    2, "Thigh"       },
		{ p_lowerleg, 2, "LowerLeg"    },
		{ p_wrist,    2, "Wrist"       },
		{ p_neck,     2, "Neck"        },
		{ p_finger,   2, "Finger"      },
		{ p_toe,      2, "Toe"         },
		{ p_ring,     3, "RingFinger"  },
		{ p_middle,   2, "MiddleFinger"},
		{ p_index,    2, "IndexFinger" },
		{ p_pinky,    2, "PinkyFinger" },
		{ p_thumb,    2, "Thumb"       },
		{ p_tip,      2, "Tip"         },
		{ p_heel,     2, "Heel"        },
		{ p_elbow,    2, "Elbow"       },
		{ p_knee,     2, "Knee"        },
		{ p_ankle,    2, "Ankle"       },
		{ p_shoulder, 3, "Shoulder"    },
		{ p_elbowj,   3, "Elbow"       },
		{ p_kneej,    3, "Knee"        },
		{ p_wristj,   3, "Wrist"       },
		{ p_hipj,     3, "Hip"         },
	};

	// Single characters
	struct Map { unsigned int cp; const char *en; };
	static const Map single[] = {
		{ 0x5DE6, "Left"     }, { 0x53F3, "Right"   },   // 左 右
		{ 0x4E0A, "Upper"    }, { 0x4E0B, "Lower"   },   // 上 下
		{ 0x4E2D, "Mid"      }, { 0x6839, "Root"    },   // 中 根
		{ 0x9AA8, "Bone"     }, { 0x9ABC, "Bone"    },   // 骨 骼
		{ 0x80A9, "Shoulder" }, { 0x8098, "Elbow"   },   // 肩 肘
		{ 0x8155, "Wrist"    }, { 0x624B, "Hand"    },   // 腕 手
		{ 0x6307, "Finger"   }, { 0x819D, "Knee"    },   // 指 膝
		{ 0x8E1D, "Ankle"    }, { 0x8DB3, "Foot"    },   // 踝 足
		{ 0x811A, "Foot"     }, { 0x817F, "Leg"     },   // 脚 腿
		{ 0x8DDE, "Toe"      }, { 0x9ACB, "Hip"     },   // 趾 髋
		{ 0x81C0, "Hip"      }, { 0x76C6, "Pelvis"  },   // 臀 盆
		{ 0x8170, "Waist"    }, { 0x810A, "Spine"   },   // 腰 脊
		{ 0x9888, "Neck"     }, { 0x8116, "Neck"    },   // 颈 脖
		{ 0x5934, "Head"     }, { 0x80F8, "Chest"   },   // 头 胸
		{ 0x9501, "Clavicle" }, { 0x81C2, "Arm"     },   // 锁 臂
		{ 0x524D, "Front"    }, { 0x540E, "Back"    },   // 前 后
		{ 0x5927, "Upper"    }, { 0x5C0F, "Lower"   },   // 大 小
		{ 0x5185, "Inner"    }, { 0x5916, "Outer"   },   // 内 外
		{ 0x8FD1, "Proximal" }, { 0x8FDC, "Distal"  },   // 近 远
		{ 0x672B, "Tip"      }, { 0x7AEF, "Tip"     },   // 末 端
		{ 0x5C16, "Tip"      }, { 0x638C, "Palm"    },   // 尖 掌
		{ 0x8DDF, "Heel"     }, { 0x98DF, "Index"   },   // 跟 食
		{ 0x540D, "Ring"     }, { 0x62C7, "Thumb"   },   // 名 拇
	};

	std::string out;
	size_t i = 0;
	while (i < cps.size())
	{
		bool matched = false;
		for (size_t p = 0; p < _countof(phrases); ++p)
		{
			if (i + (size_t)phrases[p].n <= cps.size())
			{
				bool eq = true;
				for (int k = 0; k < phrases[p].n; ++k)
					if (cps[i + k] != phrases[p].cps[k]) { eq = false; break; }
				if (eq)
				{
					out += phrases[p].en;
					i += phrases[p].n;
					matched = true;
					break;
				}
			}
		}
		if (matched) continue;

		unsigned int cp = cps[i];
		bool found = false;
		for (size_t s = 0; s < _countof(single); ++s)
		{
			if (single[s].cp == cp)
			{
				out += single[s].en;
				found = true;
				break;
			}
		}
		if (found) { i++; continue; }

		if (cp < 0x80)
			out += (char)cp;   // keep ASCII punctuation / letters
		i++;
	}
	return out;
}

// Switches the active workspace (0 = Motion Capture, 1 = Retarget).
// Exactly one workspace is active at a time.
void SetWorkspace(HWND hWnd, int workspace)
{
	if (workspace != 0 && workspace != 1)
		return;
	if (g_session.workspace == workspace)
		return;

	if (g_session.pipetteActive) { g_session.pipetteActive=false; g_session.pipetteJoint=-1; }
	g_session.workspace = workspace;
	LayoutUI(hWnd);
	UpdateControlStates(hWnd);
	UpdateLabels(hWnd);
	if (workspace == 1)
		RefreshRetargetLists(hWnd);
	// Force viewport resize/recreate to avoid freeze after import+switch (shader RT lost)
	HWND hView = GetDlgItem(hWnd, VIEW_STATUS);
	if (hView && kVisualizer) {
		RECT rc; GetClientRect(hView, &rc);
		kVisualizer->resize(rc.right - rc.left, rc.bottom - rc.top);
		InvalidateRect(hView, NULL, FALSE);
	}

	// The tabs never move, so MoveWindow does not repaint them and the
	// previous highlight would linger. Redraw both tabs (and the viewport)
	// immediately so exactly one tab keeps the accent highlight.
	RedrawWindow(GetDlgItem(hWnd, WORKSPACE_MOCAP_TAB), NULL, NULL,
		RDW_INVALIDATE | RDW_UPDATENOW);
	RedrawWindow(GetDlgItem(hWnd, WORKSPACE_RETARGET_TAB), NULL, NULL,
		RDW_INVALIDATE | RDW_UPDATENOW);
	RedrawWindow(GetDlgItem(hWnd, VIEW_STATUS), NULL, NULL,
		RDW_INVALIDATE | RDW_UPDATENOW);
}

// Enables / disables the buttons depending on the current state
void UpdateControlStates(HWND hWnd)
{
	if (!kExporter)
		return;

	bool rec = kExporter->recordingStatus();
	bool hasTake = kExporter->hasTake();
	bool playing = g_session.playing;

	EnableWindow(GetDlgItem(hWnd, STOP_RECORD_BUTTON), rec);
	EnableWindow(GetDlgItem(hWnd, RECORD_BUTTON), !playing);
	EnableWindow(GetDlgItem(hWnd, PLAY_BUTTON), hasTake && !rec);
	EnableWindow(GetDlgItem(hWnd, CLEAR_BUTTON), hasTake && !rec);
	EnableWindow(GetDlgItem(hWnd, EXPORT_TO_BUTTON), hasTake && !rec);

	// Timeline transport
	bool canTransport = hasTake && !rec;
	EnableWindow(GetDlgItem(hWnd, FIRST_FRAME_BUTTON), canTransport);
	EnableWindow(GetDlgItem(hWnd, PREV_FRAME_BUTTON), canTransport);
	EnableWindow(GetDlgItem(hWnd, NEXT_FRAME_BUTTON), canTransport);
	EnableWindow(GetDlgItem(hWnd, LAST_FRAME_BUTTON), canTransport);

	// Retarget workspace
	bool hasChar = g_retargeter ? g_retargeter->hasCharacter() : false;
	EnableWindow(GetDlgItem(hWnd, AUTOMAP_BUTTON), hasChar);
	EnableWindow(GetDlgItem(hWnd, AUTOMAP_POS_BUTTON), hasChar);
	EnableWindow(GetDlgItem(hWnd, RETARGET_PLAY_BUTTON), hasTake && hasChar && !rec);
	EnableWindow(GetDlgItem(hWnd, EXPORT_RETARGET_BUTTON), hasTake && hasChar && !rec);
	EnableWindow(GetDlgItem(hWnd, APPLY_TRANSFORM_BUTTON), hasChar);
	EnableWindow(GetDlgItem(hWnd, MAPPING_ASSIGN_BUTTON), hasChar);
	EnableWindow(GetDlgItem(hWnd, MAPPING_UNASSIGN_BUTTON), hasChar);
	EnableWindow(GetDlgItem(hWnd, PIPETTE_BUTTON), hasChar);
	EnableWindow(GetDlgItem(hWnd, RETARGET_CLEAR_BUTTON), hasTake && !rec);
	EnableWindow(GetDlgItem(hWnd, NEW_SESSION_BUTTON), !rec);
	EnableWindow(GetDlgItem(hWnd, SHOW_RETARGET_LIVE_CHECK), true);

	HWND hPlay = GetDlgItem(hWnd, PLAY_BUTTON);
	if (hPlay)
		SetWindowText(hPlay, playing ? L"\x23F8 Pause" : L"\x25B6 Play");

	HWND hRPlay = GetDlgItem(hWnd, RETARGET_PLAY_BUTTON);
	if (hRPlay)
		SetWindowText(hRPlay, playing ? L"\x23F8 Pause" : L"\x25B6 Play");

	HWND hRec = GetDlgItem(hWnd, RECORD_BUTTON);
	if (hRec)
		SetWindowText(hRec, rec ? L"\x25A0 Stop" : L"\x25CF Record");

	// Menu checkmarks
	HMENU menu = GetMenu(hWnd);
	if (menu)
	{
		CheckMenuItem(menu, IDM_LOOP, MF_BYCOMMAND | (g_session.loop ? MF_CHECKED : MF_UNCHECKED));
		CheckMenuItem(menu, IDM_GHOST, MF_BYCOMMAND | (g_session.ghost ? MF_CHECKED : MF_UNCHECKED));
		CheckMenuItem(menu, IDM_PAUSE_LIVE, MF_BYCOMMAND | (g_session.livePaused ? MF_CHECKED : MF_UNCHECKED));
		CheckMenuItem(menu, IDM_TOGGLE_TRIM, MF_BYCOMMAND | (g_session.hasRange ? MF_CHECKED : MF_UNCHECKED));
	}

	// Sync the checkboxes (owner-drawn: redraw directly from session state)
	InvalidateRect(GetDlgItem(hWnd, LOOP_CHECK), NULL, TRUE);
	InvalidateRect(GetDlgItem(hWnd, GHOST_CHECK), NULL, TRUE);
	InvalidateRect(GetDlgItem(hWnd, PAUSE_CHECK), NULL, TRUE);
	InvalidateRect(GetDlgItem(hWnd, DEPTH_CHECK), NULL, TRUE);
	InvalidateRect(GetDlgItem(hWnd, CAMERA_CHECK), NULL, TRUE);
	InvalidateRect(GetDlgItem(hWnd, FLOOR_CHECK), NULL, TRUE);
	InvalidateRect(GetDlgItem(hWnd, GESTURE_CHECK), NULL, TRUE);
	InvalidateRect(GetDlgItem(hWnd, SHOW_RETARGET_LIVE_CHECK), NULL, TRUE);
}

// Refreshes the text labels
void UpdateLabels(HWND hWnd)
{
	if (!kExporter)
		return;

	INT64 dur = kExporter->takeDurationMS();
	g_session.takeDurationMS = dur;

	WCHAR buf[128];
	WCHAR durTxt[32];
	WCHAR playTxt[32];
	StringCchCopy(durTxt, _countof(durTxt), formatTimeStatic(dur));
	StringCchCopy(playTxt, _countof(playTxt), formatTimeStatic(g_session.playheadMS));

	// Take info
	size_t frames = kExporter->recordedFrameCount();
	StringCchPrintf(buf, _countof(buf), L"Take: %d frames \x00B7 %s",
		int(frames), (frames > 0 ? durTxt : L"00:00.00"));
	SetWindowText(GetDlgItem(hWnd, TAKE_INFO_LABEL), buf);

	// Current frame (nearest recorded frame at/behind the playhead)
	WCHAR frameTxt[48] = L"Frame - / -";
	if (frames > 0)
	{
		const KRecording &rec = kExporter->recording();
		const KSampleFrame *cur = rec.frameAt(g_session.playheadMS);
		size_t curIdx = cur ? (size_t)(cur - &rec.frames[0]) + 1 : 0;
		StringCchPrintf(frameTxt, _countof(frameTxt), L"Frame %d / %d", int(curIdx), int(frames));
	}
	SetWindowText(GetDlgItem(hWnd, FRAME_INFO_LABEL), frameTxt);

	// Retarget labels
	SetWindowText(GetDlgItem(hWnd, RETARGET_TAKE_LABEL), buf);

	if (g_retargeter && g_retargeter->hasCharacter())
	{
		std::string dispName = MakeEnglish(g_retargeter->characterName());
		// Strip leading/trailing punctuation left over after translation
		while (!dispName.empty())
		{
			char c = dispName[0];
			if (isalnum((unsigned char)c)) break;
			dispName.erase(0, 1);
		}
		while (!dispName.empty())
		{
			char c = dispName[dispName.size() - 1];
			if (isalnum((unsigned char)c)) break;
			dispName.pop_back();
		}
		if (dispName.empty())
		{
			SetWindowText(GetDlgItem(hWnd, RETARGET_INFO_LABEL), L"Character: loaded");
		}
		else
		{
			WCHAR nameW[128];
			MultiByteToWideChar(CP_UTF8, 0, dispName.c_str(), -1, nameW, _countof(nameW));
			StringCchPrintf(buf, _countof(buf), L"Character: %s", nameW);
			SetWindowText(GetDlgItem(hWnd, RETARGET_INFO_LABEL), buf);
		}
	}
	else
	{
		SetWindowText(GetDlgItem(hWnd, RETARGET_INFO_LABEL), L"Character: none imported");
	}

	// Smoothing value
	StringCchPrintf(buf, _countof(buf), L"Smoothing: %d%%", int(g_session.smoothStrength + 0.5f));
	SetWindowText(GetDlgItem(hWnd, SMOOTH_VALUE), buf);

	// Status bar
	StringCchPrintf(g_statusLeft, _countof(g_statusLeft),
		L"Take: %d frames \x00B7 duration %s \x00B7 playhead %s \x00B7 %s",
		int(frames), durTxt, playTxt, frameTxt);

	if (kVisualizer && kVisualizer->hasFloor())
	{
		StringCchPrintf(g_statusRight, _countof(g_statusRight),
			L"Smoothing %d%% \x00B7 floor %.2f m \x00B7 %s",
			int(g_session.smoothStrength + 0.5f),
			kVisualizer->floorHeight(),
			g_session.recording ? L"RECORDING" : (g_session.playing ? L"PLAYING" : L"READY"));
	}
	else
	{
		StringCchPrintf(g_statusRight, _countof(g_statusRight),
			L"Smoothing %d%% \x00B7 %s",
			int(g_session.smoothStrength + 0.5f),
			g_session.recording ? L"RECORDING" : (g_session.playing ? L"PLAYING" : L"READY"));
	}
}

// Helper: format ms into a static buffer
const WCHAR* formatTimeStatic(INT64 ms)
{
	static WCHAR buf[32];
	INT64 total = ms / 1000;
	INT64 min = total / 60;
	INT64 sec = total % 60;
	INT64 cent = (ms % 1000) / 10;
	swprintf_s(buf, 32, L"%02d:%02d.%02d", int(min), int(sec), int(cent));
	return buf;
}

// Create the UI child controls
void CreateUIControls(
	HWND hWndParent
	)
{
	DWORD dwStyle = WS_CHILD | WS_VISIBLE;

	// ---- 3D View ----
	CreateWindowEx(
		0,
		viewWindowClass,
		L"",
		dwStyle | WS_CLIPSIBLINGS | WS_CLIPCHILDREN,
		0, 0, 100, 100,
		hWndParent,
		(HMENU)VIEW_STATUS,
		hInst,
		NULL
		);

	// ---- Timeline ----
	CreateWindowEx(
		0,
		timelineWindowClass,
		L"",
		dwStyle | WS_CLIPSIBLINGS,
		0, 0, 100, 60,
		hWndParent,
		(HMENU)TIMELINE_WINDOW,
		hInst,
		NULL
		);

	// ---- Log console ----
	DWORD dwStyle_Edit = ES_AUTOHSCROLL | ES_MULTILINE | WS_HSCROLL | WS_VSCROLL;
	CreateWindowEx(
		WS_EX_STATICEDGE,
		L"EDIT",
		L"",
		dwStyle | dwStyle_Edit,
		0, 0, 100, 100,
		hWndParent,
		(HMENU)EXECUTE_STATUS,
		hInst,
		NULL
		);

	// ---- Right panel ----
	CreateWindowEx(0, L"STATIC", L"CAPTURE", dwStyle | SS_LEFT, 0, 0, 0, 0,
		hWndParent, (HMENU)CAPTURE_LABEL, hInst, NULL);

	CreateFlatButton(hWndParent, RECORD_BUTTON, L"\x25CF Record", 0, 0, 0, 0);
	CreateFlatButton(hWndParent, STOP_RECORD_BUTTON, L"\x25A0 Stop", 0, 0, 0, 0);
	CreateFlatButton(hWndParent, EXPORT_TO_BUTTON, L"\x2913 Export FBX", 0, 0, 0, 0);
	CreateFlatButton(hWndParent, CLEAR_BUTTON, L"Clear Take", 0, 0, 0, 0);

	CreateWindowEx(0, L"STATIC", L"PLAYBACK", dwStyle | SS_LEFT, 0, 0, 0, 0,
		hWndParent, (HMENU)PLAYBACK_LABEL, hInst, NULL);

	CreateFlatButton(hWndParent, PLAY_BUTTON, L"\x25B6 Play", 0, 0, 0, 0);

	// Timeline transport
	CreateFlatButton(hWndParent, FIRST_FRAME_BUTTON, L"|<", 0, 0, 0, 0);
	CreateFlatButton(hWndParent, PREV_FRAME_BUTTON, L"<", 0, 0, 0, 0);
	CreateFlatButton(hWndParent, NEXT_FRAME_BUTTON, L">", 0, 0, 0, 0);
	CreateFlatButton(hWndParent, LAST_FRAME_BUTTON, L">|", 0, 0, 0, 0);
	CreateWindowEx(0, L"STATIC", L"", dwStyle | SS_LEFT, 0, 0, 0, 0,
		hWndParent, (HMENU)FRAME_INFO_LABEL, hInst, NULL);

	CreateFlatCheck(hWndParent, LOOP_CHECK, L"Loop playback", 0, 0, 0, 0);
	CreateFlatCheck(hWndParent, GHOST_CHECK, L"Ghost reference", 0, 0, 0, 0);
	CreateFlatCheck(hWndParent, PAUSE_CHECK, L"Pause live view", 0, 0, 0, 0);
	CreateFlatCheck(hWndParent, DEPTH_CHECK, L"Depth / IR view", 0, 0, 0, 0);
	CreateFlatCheck(hWndParent, CAMERA_CHECK, L"Show Kinect camera", 0, 0, 0, 0);
	CreateFlatCheck(hWndParent, FLOOR_CHECK, L"Floor alignment", 0, 0, 0, 0);
	CreateFlatCheck(hWndParent, GESTURE_CHECK, L"Gesture capture", 0, 0, 0, 0);
	CreateFlatCheck(hWndParent, SHOW_RETARGET_LIVE_CHECK, L"Show Retarget Live", 0, 0, 0, 0);
	CreateFlatButton(hWndParent, NEW_SESSION_BUTTON, L"New Session", 0, 0, 0, 0);

	CreateWindowEx(0, L"STATIC", L"SMOOTHING", dwStyle | SS_LEFT, 0, 0, 0, 0,
		hWndParent, (HMENU)SMOOTH_LABEL, hInst, NULL);

	// Smoothing slider
	HWND hSlider = CreateWindowEx(
		0,
		TRACKBAR_CLASS,
		L"",
		dwStyle | TBS_HORZ | TBS_NOTICKS,
		0, 0, 0, 0,
		hWndParent,
		(HMENU)SMOOTH_SLIDER,
		hInst,
		NULL
		);
	if (hSlider)
	{
		SendMessage(hSlider, TBM_SETRANGE, TRUE, MAKELPARAM(0, 100));
		SendMessage(hSlider, TBM_SETPOS, TRUE, (LPARAM)g_session.smoothStrength);
	}

	CreateWindowEx(0, L"STATIC", L"", dwStyle | SS_LEFT, 0, 0, 0, 0,
		hWndParent, (HMENU)SMOOTH_VALUE, hInst, NULL);
	CreateWindowEx(0, L"STATIC", L"", dwStyle | SS_LEFT, 0, 0, 0, 0,
		hWndParent, (HMENU)TAKE_INFO_LABEL, hInst, NULL);

	// ---- Workspace tabs (drawn in the header area) ----
	CreateFlatButton(hWndParent, WORKSPACE_MOCAP_TAB, L"Motion Capture", 0, 0, 0, 0);
	CreateFlatButton(hWndParent, WORKSPACE_RETARGET_TAB, L"Retarget", 0, 0, 0, 0);

	// ---- Retarget panel ----
	CreateFlatButton(hWndParent, IMPORT_FBX_BUTTON, L"Import Character (FBX)", 0, 0, 0, 0);
	CreateFlatButton(hWndParent, AUTOMAP_BUTTON, L"Auto-map Bones", 0, 0, 0, 0);
	CreateFlatButton(hWndParent, AUTOMAP_POS_BUTTON, L"Auto-map by Position (Ghost)", 0, 0, 0, 0);
	CreateFlatButton(hWndParent, RETARGET_PLAY_BUTTON, L"\x25B6 Play", 0, 0, 0, 0);
	CreateFlatButton(hWndParent, EXPORT_RETARGET_BUTTON, L"\x2913 Export Retargeted", 0, 0, 0, 0);
	CreateFlatCheck(hWndParent, BVH_EXPORT_CHECK, L"Export target as BVH", 0, 0, 0, 0);

	CreateWindowEx(0, L"STATIC", L"", dwStyle | SS_LEFT, 0, 0, 0, 0,
		hWndParent, (HMENU)RETARGET_TAKE_LABEL, hInst, NULL);
	CreateWindowEx(0, L"STATIC", L"", dwStyle | SS_LEFT, 0, 0, 0, 0,
		hWndParent, (HMENU)RETARGET_INFO_LABEL, hInst, NULL);

	// Import transform (size + orientation)
	DWORD editStyle = WS_CHILD | WS_VISIBLE | WS_TABSTOP | ES_AUTOHSCROLL;
	CreateWindowEx(0, L"STATIC", L"Size", dwStyle | SS_LEFT, 0, 0, 0, 0,
		hWndParent, (HMENU)SIZE_LABEL, hInst, NULL);
	CreateWindowEx(WS_EX_CLIENTEDGE, L"EDIT", L"1.000", editStyle, 0, 0, 0, 0,
		hWndParent, (HMENU)SIZE_EDIT, hInst, NULL);
	CreateWindowEx(0, L"STATIC", L"Rot", dwStyle | SS_LEFT, 0, 0, 0, 0,
		hWndParent, (HMENU)ROT_LABEL, hInst, NULL);
	CreateWindowEx(WS_EX_CLIENTEDGE, L"EDIT", L"0.0", editStyle, 0, 0, 0, 0,
		hWndParent, (HMENU)ROT_X_EDIT, hInst, NULL);
	CreateWindowEx(WS_EX_CLIENTEDGE, L"EDIT", L"0.0", editStyle, 0, 0, 0, 0,
		hWndParent, (HMENU)ROT_Y_EDIT, hInst, NULL);
	CreateWindowEx(WS_EX_CLIENTEDGE, L"EDIT", L"0.0", editStyle, 0, 0, 0, 0,
		hWndParent, (HMENU)ROT_Z_EDIT, hInst, NULL);
	CreateFlatButton(hWndParent, APPLY_TRANSFORM_BUTTON, L"Apply Size / Rotation", 0, 0, 0, 0);
	CreateFlatButton(hWndParent, RESET_GHOST_BUTTON, L"Reset Ghost Pose", 0, 0, 0, 0);
	CreateFlatCheck(hWndParent, SYMMETRY_CHECK, L"Symmetry (mirror edit)", 0, 0, 0, 0);
	CreateFlatButton(hWndParent, PIPETTE_BUTTON, L"Pipette", 0, 0, 0, 0);
	CreateFlatButton(hWndParent, RETARGET_CLEAR_BUTTON, L"Delete Timeline", 0, 0, 0, 0);

	// ---- Bone mapping window ----
	DWORD listStyle = WS_CHILD | WS_BORDER | WS_VSCROLL | LBS_NOTIFY | LBS_NOINTEGRALHEIGHT;
	// Kinect hierarchy and imported hierarchy both as TreeView for true bone->bone drag & drop
	CreateWindowEx(0, WC_TREEVIEW, L"", WS_CHILD | WS_BORDER | TVS_HASBUTTONS | TVS_HASLINES | TVS_LINESATROOT | TVS_SHOWSELALWAYS,
		0, 0, 100, 100, hWndParent, (HMENU)RETARGET_JOINT_LIST, hInst, NULL);
	CreateWindowEx(0, L"LISTBOX", L"", listStyle, 0, 0, 100, 100,
		hWndParent, (HMENU)RETARGET_BONE_LIST, hInst, NULL);
	// Left bone hierarchy (Unreal-like, retarget only) — collapsible TreeView (target mesh)
	CreateWindowEx(0, WC_TREEVIEW, L"", WS_CHILD | WS_BORDER | TVS_HASBUTTONS | TVS_HASLINES | TVS_LINESATROOT | TVS_SHOWSELALWAYS | TVS_DISABLEDRAGDROP,
		0, 0, 100, 100, hWndParent, (HMENU)RETARGET_BONE_TREE, hInst, NULL);
	CreateFlatButton(hWndParent, 1116, L"T-Pose External", 0, 0, 0, 0); // ID reuses NEW_SESSION but text T-Pose
	CreateFlatButton(hWndParent, MAPPING_ASSIGN_BUTTON, L"Map ->", 0, 0, 0, 0);
	CreateFlatButton(hWndParent, MAPPING_UNASSIGN_BUTTON, L"Unmap", 0, 0, 0, 0);
	CreateWindowEx(0, L"STATIC",
		L"Select a joint on the left, a bone on the right, then press Map. "
		L"Use Auto-map for a first pass.", dwStyle | SS_LEFT | SS_WORDELLIPSIS, 0, 0, 0, 0,
		hWndParent, (HMENU)MAPPING_NOTE_LABEL, hInst, NULL);
}

// Applies the shared theme fonts + dark colors to native controls.
// Called once from WM_CREATE after the fonts exist.
void ApplyControlFonts(HWND hWnd)
{
	static const int sectionIds[] = { CAPTURE_LABEL, PLAYBACK_LABEL, SMOOTH_LABEL };
	for (int i = 0; i < _countof(sectionIds); ++i)
	{
		HWND h = GetDlgItem(hWnd, sectionIds[i]);
		if (h && g_fontSection) SendMessage(h, WM_SETFONT, (WPARAM)g_fontSection, TRUE);
	}
	static const int infoIds[] = {
		FRAME_INFO_LABEL, SMOOTH_VALUE, TAKE_INFO_LABEL,
		RETARGET_TAKE_LABEL, RETARGET_INFO_LABEL,
		SIZE_LABEL, ROT_LABEL, MAPPING_NOTE_LABEL,
		SIZE_EDIT, ROT_X_EDIT, ROT_Y_EDIT, ROT_Z_EDIT
	};
	for (int i = 0; i < _countof(infoIds); ++i)
	{
		HWND h = GetDlgItem(hWnd, infoIds[i]);
		if (h && g_fontSmall) SendMessage(h, WM_SETFONT, (WPARAM)g_fontSmall, TRUE);
	}
	// Dark tree views (same look as the 2D editor tree)
	HWND hTrees[] = { GetDlgItem(hWnd, RETARGET_JOINT_LIST), GetDlgItem(hWnd, RETARGET_BONE_TREE) };
	for (int i = 0; i < 2; ++i)
	{
		if (!hTrees[i]) continue;
		TreeView_SetBkColor(hTrees[i], Theme::PANEL_DARK);
		TreeView_SetTextColor(hTrees[i], Theme::TEXT);
		TreeView_SetLineColor(hTrees[i], Theme::BORDER);
		if (g_fontSmall) SendMessage(hTrees[i], WM_SETFONT, (WPARAM)g_fontSmall, TRUE);
	}
}

// Creates an owner-drawn flat button
static HWND CreateFlatButton(HWND parent, int id, LPCWSTR text, int x, int y, int w, int h)
{
	HWND hwnd = CreateWindowEx(
		0, L"BUTTON", text,
		WS_CHILD | WS_VISIBLE | BS_OWNERDRAW,
		x, y, w, h,
		parent, (HMENU)id, hInst, NULL);
	return hwnd;
}

// Creates an owner-drawn checkbox
static HWND CreateFlatCheck(HWND parent, int id, LPCWSTR text, int x, int y, int w, int h)
{
	HWND hwnd = CreateWindowEx(
		0, L"BUTTON", text,
		WS_CHILD | WS_VISIBLE | BS_OWNERDRAW | BS_CHECKBOX,
		x, y, w, h,
		parent, (HMENU)id, hInst, NULL);
	return hwnd;
}

// Mixes a color with white (0..1)
static COLORREF Lighten(COLORREF c, float amount)
{
	int r = GetRValue(c), g = GetGValue(c), b = GetBValue(c);
	r = r + int((255 - r) * amount);
	g = g + int((255 - g) * amount);
	b = b + int((255 - b) * amount);
	return RGB(r, g, b);
}

static COLORREF Darken(COLORREF c, float amount)
{
	int r = GetRValue(c), g = GetGValue(c), b = GetBValue(c);
	r = int(r * (1.0f - amount));
	g = int(g * (1.0f - amount));
	b = int(b * (1.0f - amount));
	return RGB(r, g, b);
}

// Draws one owner-drawn flat button
static void DrawFlatButton(DRAWITEMSTRUCT *dis)
{
	HDC hdc = dis->hDC;
	RECT rc = dis->rcItem;

	bool disabled = (dis->itemState & ODS_DISABLED) != 0;
	bool selected = (dis->itemState & ODS_SELECTED) != 0;
	bool hot = (dis->itemState & ODS_HOTLIGHT) != 0;

	COLORREF bg = Theme::PANEL_DARK;
	COLORREF border = Theme::BORDER;
	COLORREF fg = Theme::TEXT;

	BtnStyle style = StyleForId(dis->CtlID);

	// Workspace tabs: the active one is highlighted
	if (dis->CtlID == WORKSPACE_MOCAP_TAB || dis->CtlID == WORKSPACE_RETARGET_TAB)
	{
		bool active = (dis->CtlID == WORKSPACE_MOCAP_TAB) ? (g_session.workspace == 0) : (g_session.workspace == 1);
		if (active)
		{
			bg = Theme::ACCENT;
			fg = RGB(20, 24, 32);
			border = Darken(Theme::ACCENT, 0.3f);
		}
		else
		{
			bg = Theme::PANEL_DARK;
			fg = Theme::TEXT_DIM;
			border = Theme::BORDER;
		}
	}
	else if (dis->CtlID == PIPETTE_BUTTON && g_session.pipetteActive)
	{
		bg = Theme::ACCENT;
		fg = RGB(20, 24, 32);
		border = Darken(Theme::ACCENT, 0.3f);
	}
	else switch (style)
	{
	case BtnStyle::Record: bg = Theme::DANGER; fg = RGB(255, 255, 255); border = Darken(Theme::DANGER, 0.3f); break;
	case BtnStyle::Stop: bg = Theme::PANEL_DARK; fg = Theme::DANGER; border = Theme::DANGER; break;
	case BtnStyle::Export: bg = Theme::ACCENT; fg = RGB(20, 24, 32); border = Darken(Theme::ACCENT, 0.3f); break;
	case BtnStyle::Clear: bg = Theme::PANEL_DARK; fg = Theme::TEXT; border = Theme::BORDER; break;
	case BtnStyle::Play: bg = Theme::GREEN; fg = RGB(20, 24, 32); border = Darken(Theme::GREEN, 0.3f); break;
	default: break;
	}

	if (disabled)
	{
		bg = Theme::PANEL_DARK;
		border = Theme::BORDER;
		fg = Theme::TEXT_DIM;
	}
	else if (selected)
	{
		bg = Darken(bg, 0.25f);
	}
	else if (hot)
	{
		bg = Lighten(bg, 0.12f);
	}

	// Rounded rectangle
	HRGN rgn = CreateRoundRectRgn(rc.left + 1, rc.top + 1, rc.right, rc.bottom, 10, 10);
	HBRUSH bgBrush = CreateSolidBrush(bg);
	HBRUSH borderBrush = CreateSolidBrush(border);

	FillRgn(hdc, rgn, bgBrush);
	FrameRgn(hdc, rgn, borderBrush, 1, 1);

	DeleteObject(rgn);
	DeleteObject(bgBrush);
	DeleteObject(borderBrush);

	// Text (shared cached button font: same look everywhere, no per-draw alloc)
	SetBkMode(hdc, TRANSPARENT);
	SetTextColor(hdc, fg);

	BOOL ownFont = FALSE;
	HFONT font = g_fontBtn;
	if (!font)
	{
		font = CreateFont(15, 0, 0, 0, FW_SEMIBOLD, 0, 0, 0, DEFAULT_CHARSET,
			OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY,
			DEFAULT_PITCH | FF_DONTCARE, L"Segoe UI");
		ownFont = TRUE;
	}
	HFONT oldFont = (HFONT)SelectObject(hdc, font);

	WCHAR text[128];
	GetWindowText(dis->hwndItem, text, _countof(text));

	RECT tr = rc;
	DrawText(hdc, text, -1, &tr, DT_CENTER | DT_VCENTER | DT_SINGLELINE);

	SelectObject(hdc, oldFont);
	if (ownFont) DeleteObject(font);
}

// Draws one owner-drawn checkbox
static void DrawFlatCheck(DRAWITEMSTRUCT *dis)
{
	HDC hdc = dis->hDC;
	RECT rc = dis->rcItem;

	bool checked = (dis->itemState & ODS_CHECKED) != 0;
	bool disabled = (dis->itemState & ODS_DISABLED) != 0;
	bool hot = (dis->itemState & ODS_HOTLIGHT) != 0;

	// Owner-drawn buttons do not maintain a real check state, so read it
	// straight from the session (kept in sync by the WM_COMMAND handlers).
	switch (dis->CtlID)
	{
	case LOOP_CHECK:    checked = g_session.loop; break;
	case GHOST_CHECK:   checked = g_session.ghost; break;
	case SYMMETRY_CHECK: checked = g_session.ghostSymmetry; break;
	case PAUSE_CHECK:   checked = g_session.livePaused; break;
	case DEPTH_CHECK:   checked = g_session.depthView; break;
	case CAMERA_CHECK:  checked = g_session.cameraView; break;
	case FLOOR_CHECK:   checked = g_session.floorAlign; break;
	case GESTURE_CHECK: checked = g_session.gestureRecord; break;
	case SHOW_RETARGET_LIVE_CHECK: checked = g_session.showRetargetLive; break;
	case BVH_EXPORT_CHECK: checked = g_session.exportBVH; break;
	default: break;
	}

	// The owner-drawn button erases its background with the default
	// light button face first, so fill the whole control with the
	// panel color to keep the dark theme.
	HBRUSH bgBrush = CreateSolidBrush(Theme::PANEL);
	FillRect(hdc, &rc, bgBrush);
	DeleteObject(bgBrush);

	// Square box, centered vertically, 16 px
	int boxSize = 16;
	int boxY = rc.top + (rc.bottom - rc.top - boxSize) / 2;
	RECT box = { rc.left + 2, boxY, rc.left + 2 + boxSize, boxY + boxSize };

	// Visible in both states on the dark panel (theme-driven, same hues as before)
	COLORREF fill = checked ? Theme::ACCENT : Theme::EDIT_BG;
	COLORREF border = checked ? Lighten(Theme::ACCENT, 0.25f) : Lighten(Theme::BORDER, 0.6f);
	if (disabled)
	{
		fill = Theme::PANEL_DARK;
		border = Theme::BORDER;
	}
	else if (hot)
	{
		fill = checked ? Lighten(Theme::ACCENT, 0.12f) : Lighten(Theme::EDIT_BG, 0.12f);
		border = hot ? Lighten(Theme::BORDER, 0.75f) : border;
	}

	HBRUSH fillBrush = CreateSolidBrush(fill);
	HBRUSH borderBrush = CreateSolidBrush(border);
	FillRect(hdc, &box, fillBrush);
	FrameRect(hdc, &box, borderBrush);

	// Check mark
	if (checked)
	{
		HPEN pen = CreatePen(PS_SOLID, 2, Darken(Theme::ACCENT, 0.85f));
		HPEN oldPen = (HPEN)SelectObject(hdc, pen);
		MoveToEx(hdc, box.left + 3, box.top + boxSize / 2, NULL);
		LineTo(hdc, box.left + 6, box.bottom - 4);
		LineTo(hdc, box.right - 3, box.top + 4);
		SelectObject(hdc, oldPen);
		DeleteObject(pen);
	}

	DeleteObject(fillBrush);
	DeleteObject(borderBrush);

	// Text (shared cached check font)
	SetBkMode(hdc, TRANSPARENT);
	SetTextColor(hdc, disabled ? Theme::TEXT_DIM : Theme::TEXT);

	BOOL ownFont = FALSE;
	HFONT font = g_fontCheck;
	if (!font)
	{
		font = CreateFont(13, 0, 0, 0, FW_NORMAL, 0, 0, 0, DEFAULT_CHARSET,
			OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY,
			DEFAULT_PITCH | FF_DONTCARE, L"Segoe UI");
		ownFont = TRUE;
	}
	HFONT oldFont = (HFONT)SelectObject(hdc, font);

	WCHAR text[128];
	GetWindowText(dis->hwndItem, text, _countof(text));

	RECT tr = { box.right + 8, rc.top, rc.right, rc.bottom };
	DrawText(hdc, text, -1, &tr, DT_LEFT | DT_VCENTER | DT_SINGLELINE);

	SelectObject(hdc, oldFont);
	if (ownFont) DeleteObject(font);
}

// Opens the character (FBX) selection dialog
bool GetOpenFBXFileName(HWND hWndParent, char *outPath)
{
	outPath[0] = '\0';
	OPENFILENAMEA ofn;
	ZeroMemory(&ofn, sizeof(ofn));
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = hWndParent;
	ofn.lpstrFilter = "FBX files (*.fbx)\0*.fbx\0All files (*.*)\0*.*\0\0";
	ofn.lpstrFile = outPath;
	ofn.nMaxFile = _MAX_PATH;
	ofn.Flags = OFN_FILEMUSTEXIST | OFN_HIDEREADONLY;
	ofn.lpstrTitle = "Import character (skeleton + mesh)";
	return GetOpenFileNameA(&ofn) != FALSE;
}

// Opens a .KAS project file picker
bool GetOpenKASFileName(HWND hWndParent, char *outPath)
{
	outPath[0] = '\0';
	OPENFILENAMEA ofn;
	ZeroMemory(&ofn, sizeof(ofn));
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = hWndParent;
	ofn.lpstrFilter = "KAS project files (*.kas)\0*.kas\0All files (*.*)\0*.*\0\0";
	ofn.lpstrFile = outPath;
	ofn.nMaxFile = _MAX_PATH;
	ofn.Flags = OFN_FILEMUSTEXIST | OFN_HIDEREADONLY | OFN_PATHMUSTEXIST;
	ofn.lpstrTitle = "Open project";
	return GetOpenFileNameA(&ofn) != FALSE;
}

// Opens a save-as dialog for a .KAS project file
bool GetSaveKASFileName(HWND hWndParent, char *outPath)
{
	outPath[0] = '\0';
	OPENFILENAMEA ofn;
	ZeroMemory(&ofn, sizeof(ofn));
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = hWndParent;
	ofn.lpstrFilter = "KAS project files (*.kas)\0*.kas\0All files (*.*)\0*.*\0\0";
	ofn.lpstrFile = outPath;
	ofn.nMaxFile = _MAX_PATH;
	ofn.Flags = OFN_OVERWRITEPROMPT | OFN_HIDEREADONLY | OFN_PATHMUSTEXIST;
	ofn.lpstrDefExt = "kas";
	ofn.lpstrTitle = "Save project as";
	return GetSaveFileNameA(&ofn) != FALSE;
}

// ============================================================================
// 2D Body Editor — modern dark UI (same theme as the main app).
// Layout: header (54) + left imported-bones tree (270) + center T-pose canvas
// + right details (240) + status (30). Drag: imported bone -> mocap joint.
// ============================================================================
#define IDC_ED_TREE    7001
#define IDC_ED_AUTOMAP 7002
#define IDC_ED_CLEAR   7003
#define IDC_ED_UNMAP   7004
#define IDC_ED_CLOSE   7005

static HWND  s_edTree = NULL;
static HWND  s_edBtnAuto = NULL, s_edBtnClear = NULL, s_edBtnUnmap = NULL, s_edBtnClose = NULL;
static HFONT s_edFontTitle = NULL, s_edFontUI = NULL, s_edFontSmall = NULL;
static int   s_edSelJoint = -1;    // selected mocap JointType
static int   s_edHoverJoint = -1;  // hovered mocap JointType
static int   s_edDragBone = -1;    // imported bone idx being dragged (from tree)
static int   s_edDragJoint = -1;   // mocap joint being dragged (from canvas)
static bool  s_edDragging = false;
// Blender-like splitters: left tree / canvas / right panel
static int   s_edLeftW = 270, s_edRightW = 240;
static int   s_edSplitDrag = 0, s_edSplitHover = 0; // 1 left, 2 right
static POINT s_edSplitStart = {0,0};
static int   s_edSplitStartL = 0, s_edSplitStartR = 0;

ATOM TPoseRegisterClass(HINSTANCE hInst) {
	WNDCLASSEX wcex={0}; wcex.cbSize=sizeof(wcex); wcex.style=CS_HREDRAW|CS_VREDRAW;
	wcex.lpfnWndProc=TPoseProc; wcex.hInstance=hInst; wcex.hCursor=LoadCursor(NULL,IDC_ARROW);
	wcex.hbrBackground=NULL; wcex.lpszClassName=L"KAS_TPoseClass"; // we paint everything
	wcex.hIcon = LoadIcon(hInst, MAKEINTRESOURCE(IDI_UI));
	return RegisterClassEx(&wcex);
}
HWND CreateTPoseWindow(HWND hParent) {
	if (g_hTPoseWnd && IsWindow(g_hTPoseWnd)) { ShowWindow(g_hTPoseWnd, SW_RESTORE); SetForegroundWindow(g_hTPoseWnd); return g_hTPoseWnd; }
	g_hTPoseWnd = CreateWindowEx(0, L"KAS_TPoseClass", L"Editeur Corps 2D — Mocap T-Pose", WS_OVERLAPPEDWINDOW|WS_CLIPCHILDREN,
		120,80,1020,640,hParent,NULL,hInst,NULL);
	if (g_hTPoseWnd) ShowWindow(g_hTPoseWnd, SW_SHOW);
	return g_hTPoseWnd;
}
// T-pose 2D layout for 25 Kinect joints (normalized 0..1, y DOWN: 0 top, 1 bottom)
static void GetTPosePos(int joint, float &x, float &y) {
	// Upright T-pose: head top, spine vertical, arms horizontal, legs vertical
	struct P{int j; float x,y;};
	static const P map[]={
		{3,0.50f,0.075f},{2,0.50f,0.155f},{20,0.50f,0.205f},
		{4,0.375f,0.215f},{5,0.245f,0.215f},{6,0.125f,0.215f},{7,0.045f,0.215f},{21,0.045f,0.165f},{22,0.075f,0.180f},
		{8,0.625f,0.215f},{9,0.755f,0.215f},{10,0.875f,0.215f},{11,0.955f,0.215f},{23,0.955f,0.165f},{24,0.925f,0.180f},
		{1,0.50f,0.315f},{0,0.50f,0.440f},
		{12,0.435f,0.455f},{13,0.435f,0.640f},{14,0.435f,0.815f},{15,0.435f,0.875f},
		{16,0.565f,0.455f},{17,0.565f,0.640f},{18,0.565f,0.815f},{19,0.565f,0.875f},
	};
	for(auto &p: map) if(p.j==joint){ x=p.x; y=p.y; return; }
	x=0.5f; y=0.5f;
}
static void EdRefreshTree(HWND hTree) {
	if (hTree) TreeView_DeleteAllItems(hTree);
	if (!g_retargeter || g_retargeter->boneCount() == 0) return;
	int n = g_retargeter->boneCount();
	// New skeleton (import): reset memory, expand all parents by default
	if (s_expandedEdBones != n) {
		s_expandedEd.clear();
		for (int i = 0; i < n; ++i) {
			bool isParent = false;
			for (int k = 0; k < n; ++k) if (g_retargeter->bone(k).parent == i) { isParent = true; break; }
			if (isParent) s_expandedEd.insert(i);
		}
		s_expandedEdBones = n;
	}
	if (!hTree) return;
	std::vector<HTREEITEM> items(n, NULL);
	for (int i = 0; i < n; ++i) {
		int par = g_retargeter->bone(i).parent;
		TVINSERTSTRUCT tvis = {0};
		tvis.hParent = (par >= 0 && par < n && items[par]) ? items[par] : TVI_ROOT;
		tvis.hInsertAfter = TVI_LAST;
		tvis.item.mask = TVIF_TEXT | TVIF_PARAM;
		std::string en = MakeEnglish(g_retargeter->boneName(i));
		if (en.empty()) en = "?";
		WCHAR buf[128]; MultiByteToWideChar(CP_UTF8, 0, en.c_str(), -1, buf, 128);
		// show mapped joint after the name: "Humerus  [Epaule G]"
		WCHAR label[192]; wcsncpy_s(label, buf, _TRUNCATE);
		int jt = g_retargeter->mappingJointForBone(i);
		if (jt >= 0) {
			const char *jl = "?";
			for (int k = 0; k < g_retargetJointCount; ++k) if (g_retargetJoints[k].jointType == jt) { jl = g_retargetJoints[k].label; break; }
			WCHAR jw[64]; MultiByteToWideChar(CP_UTF8, 0, jl, -1, jw, 64);
			WCHAR tmp[192]; StringCchPrintf(tmp, 192, L"%s  [%s]", buf, jw);
			wcsncpy_s(label, tmp, _TRUNCATE);
		}
		tvis.item.pszText = label;
		tvis.item.lParam = i;
		items[i] = TreeView_InsertItem(hTree, &tvis);
	}
	// Re-apply the memorized expanded groups (mapping no longer resets them)
	for (int idx : s_expandedEd)
		if (idx >= 0 && idx < n && items[idx]) TreeView_Expand(hTree, items[idx], TVE_EXPAND);
}
static int EdMappedCount() {
	if (!g_retargeter) return 0;
	int c = 0;
	for (int i = 0; i < g_retargetJointCount; ++i)
		if (g_retargeter->getMapping(g_retargetJoints[i].jointType) >= 0) ++c;
	return c;
}
	// ---- Editeur 2D : helpers de mise en page ----
	static void EdLayout(HWND hWnd, RECT &rcHead, RECT &rcLeft, RECT &rcCanvas, RECT &rcRight, RECT &rcStatus) {
		RECT rc; GetClientRect(hWnd, &rc);
		int W = rc.right - rc.left, H = rc.bottom - rc.top;
		rcHead = { 0, 0, W, 54 };
		rcStatus = { 0, H - 30, W, H };
		int y0 = 54, y1 = H - 30;
		int leftW = s_edLeftW, rightW = s_edRightW;
		if (W < 700) { leftW = 220; rightW = 200; }
		rcLeft = { 8, y0 + 30, 8 + leftW, y1 - 8 };
		rcRight = { W - rightW - 8, y0 + 30, W - 8, y1 - 8 };
		rcCanvas = { 8 + leftW + 8, y0 + 8, W - rightW - 16, y1 - 8 };
	}
	// Hit test splitters: 1 left tree/canvas, 2 canvas/right panel
	static int EdSplitHit(HWND hWnd, int x, int y) {
		RECT rh, rl, rc, rr, rs; EdLayout(hWnd, rh, rl, rc, rr, rs);
		int y0 = 54 + 8, y1 = rs.top - 8;
		if (y < y0 || y > y1) return 0;
		int cxL = rl.right + 4;   // middle of the 8px gap
		int cxR = rc.right + 4;
		if (x >= cxL - 4 && x <= cxL + 4) return 1;
		if (x >= cxR - 4 && x <= cxR + 4) return 2;
		return 0;
	}
	static void EdJointPoint(HWND hWnd, int joint, POINT &pt) {
		RECT rh, rl, rc, rr, rs; EdLayout(hWnd, rh, rl, rc, rr, rs);
		float x, y; GetTPosePos(joint, x, y);
		int pad = 46;
		int cw = (rc.right - rc.left) - pad * 2;
		int ch = (rc.bottom - rc.top) - pad * 2;
		if (cw < 40) cw = 40; if (ch < 40) ch = 40;
		pt.x = rc.left + pad + int(x * cw);
		pt.y = rc.top + pad + int(y * ch);
	}
	static int EdHitJoint(HWND hWnd, int sx, int sy) {
		for (int i = 0; i < g_retargetJointCount; ++i) {
			POINT pt; EdJointPoint(hWnd, g_retargetJoints[i].jointType, pt);
			int dx = sx - pt.x, dy = sy - pt.y;
			if (dx*dx + dy*dy <= 17*17) return g_retargetJoints[i].jointType;
		}
		return -1;
	}
	static const char* EdJointLabel(int jt) {
		for (int i = 0; i < g_retargetJointCount; ++i)
			if (g_retargetJoints[i].jointType == jt) return g_retargetJoints[i].label;
		return "?";
	}
	static void EdPositionChildren(HWND hWnd) {
		RECT rh, rl, rc, rr, rs; EdLayout(hWnd, rh, rl, rc, rr, rs);
		if (s_edTree) MoveWindow(s_edTree, rl.left, rl.top + 24, rl.right - rl.left, rl.bottom - rl.top - 24, TRUE);
		int bw = 106, bh = 26, gap = 8;
		int bx = rr.right - bw;
		if (s_edBtnUnmap) MoveWindow(s_edBtnUnmap, rr.left, rr.bottom - bh, rr.right - rr.left, bh, TRUE);
		if (s_edBtnClose) MoveWindow(s_edBtnClose, bx, 12, bw, 30, TRUE);
		if (s_edBtnClear) MoveWindow(s_edBtnClear, bx - bw - gap, 12, bw, 30, TRUE);
		if (s_edBtnAuto) MoveWindow(s_edBtnAuto, bx - bw*2 - gap*2 - 20, 12, bw + 20, 30, TRUE);
	}

	LRESULT CALLBACK TPoseProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
	switch(msg){
	case WM_CREATE: {
		s_edFontTitle = CreateFont(16,0,0,0,FW_SEMIBOLD,0,0,0,DEFAULT_CHARSET,OUT_DEFAULT_PRECIS,CLIP_DEFAULT_PRECIS,CLEARTYPE_QUALITY,DEFAULT_PITCH,L"Segoe UI");
		s_edFontUI = CreateFont(13,0,0,0,FW_NORMAL,0,0,0,DEFAULT_CHARSET,OUT_DEFAULT_PRECIS,CLIP_DEFAULT_PRECIS,CLEARTYPE_QUALITY,DEFAULT_PITCH,L"Segoe UI");
		s_edFontSmall = CreateFont(11,0,0,0,FW_NORMAL,0,0,0,DEFAULT_CHARSET,OUT_DEFAULT_PRECIS,CLIP_DEFAULT_PRECIS,CLEARTYPE_QUALITY,DEFAULT_PITCH,L"Segoe UI");
		s_edTree = CreateWindowEx(0, WC_TREEVIEW, L"", WS_CHILD|WS_VISIBLE|WS_BORDER|TVS_HASBUTTONS|TVS_HASLINES|TVS_LINESATROOT|TVS_SHOWSELALWAYS,
			0,0,100,100, hWnd, (HMENU)IDC_ED_TREE, hInst, NULL);
		if (s_edTree) {
			TreeView_SetBkColor(s_edTree, Theme::PANEL_DARK);
			TreeView_SetTextColor(s_edTree, Theme::TEXT);
			TreeView_SetLineColor(s_edTree, Theme::BORDER);
			TreeView_SetInsertMarkColor(s_edTree, Theme::ACCENT);
		}
		s_edBtnAuto = CreateWindow(L"BUTTON", L"Auto-map", WS_CHILD|WS_VISIBLE|BS_OWNERDRAW, 0,0,100,30, hWnd, (HMENU)IDC_ED_AUTOMAP, hInst, NULL);
		s_edBtnClear = CreateWindow(L"BUTTON", L"Effacer", WS_CHILD|WS_VISIBLE|BS_OWNERDRAW, 0,0,100,30, hWnd, (HMENU)IDC_ED_CLEAR, hInst, NULL);
		s_edBtnClose = CreateWindow(L"BUTTON", L"Fermer", WS_CHILD|WS_VISIBLE|BS_OWNERDRAW, 0,0,100,30, hWnd, (HMENU)IDC_ED_CLOSE, hInst, NULL);
		s_edBtnUnmap = CreateWindow(L"BUTTON", L"Retirer ce lien", WS_CHILD|WS_VISIBLE|BS_OWNERDRAW, 0,0,100,30, hWnd, (HMENU)IDC_ED_UNMAP, hInst, NULL);
		HFONT hf = s_edFontUI ? s_edFontUI : (HFONT)GetStockObject(DEFAULT_GUI_FONT);
		if (s_edBtnAuto) SendMessage(s_edBtnAuto, WM_SETFONT, (WPARAM)hf, TRUE);
		if (s_edBtnClear) SendMessage(s_edBtnClear, WM_SETFONT, (WPARAM)hf, TRUE);
		if (s_edBtnClose) SendMessage(s_edBtnClose, WM_SETFONT, (WPARAM)hf, TRUE);
		if (s_edBtnUnmap) SendMessage(s_edBtnUnmap, WM_SETFONT, (WPARAM)hf, TRUE);
		EdRefreshTree(s_edTree);
		EdPositionChildren(hWnd);
		return 0;
	}
	case WM_SIZE: EdPositionChildren(hWnd); InvalidateRect(hWnd, NULL, FALSE); return 0;
	case WM_NOTIFY: {
		LPNMHDR hdr = (LPNMHDR)lParam;
		if (hdr && hdr->hwndFrom == s_edTree && hdr->code == TVN_BEGINDRAG) {
			LPNMTREEVIEW tv = (LPNMTREEVIEW)lParam;
			TVITEM item = {0}; item.hItem = tv->itemNew.hItem; item.mask = TVIF_PARAM | TVIF_TEXT;
			WCHAR buf[128]; item.pszText = buf; item.cchTextMax = 128;
			TreeView_GetItem(s_edTree, &item);
			s_edDragBone = (int)item.lParam; s_edDragJoint = -1; s_edDragging = true;
			POINT ps = tv->ptDrag; ClientToScreen(s_edTree, &ps);
			CreateDragWindow(hWnd, buf, ps);
			SetCapture(hWnd); SetCursor(LoadCursor(NULL, IDC_HAND));
			return 0;
		}
		if (hdr && hdr->hwndFrom == s_edTree && hdr->code == TVN_SELCHANGED) {
			InvalidateRect(hWnd, NULL, FALSE);
		}
		// Remember expanded/collapsed groups of the editor tree
		if (hdr && hdr->hwndFrom == s_edTree && hdr->code == TVN_ITEMEXPANDED) {
			LPNMTREEVIEW tv = (LPNMTREEVIEW)lParam;
			int bone = (int)tv->itemNew.lParam;
			if (tv->action & TVE_EXPAND) s_expandedEd.insert(bone);
			else s_expandedEd.erase(bone);
			if (g_retargeter) s_expandedEdBones = g_retargeter->boneCount();
		}
		break;
	}
	case WM_COMMAND: {
		int id = LOWORD(wParam);
		if (id == IDC_ED_AUTOMAP && g_retargeter) {
			g_retargeter->autoMap();
			EdRefreshTree(s_edTree);
			if (ghWnd) RefreshRetargetLists(ghWnd);
			UI_Printf("Auto-map depuis l'editeur 2D.");
			InvalidateRect(hWnd, NULL, FALSE);
			return 0;
		}
		if (id == IDC_ED_CLEAR && g_retargeter) {
			g_retargeter->clearMapping();
			EdRefreshTree(s_edTree);
			if (ghWnd) RefreshRetargetLists(ghWnd);
			UI_Printf("Mapping efface.");
			InvalidateRect(hWnd, NULL, FALSE);
			return 0;
		}
		if (id == IDC_ED_UNMAP && g_retargeter && s_edSelJoint >= 0) {
			g_retargeter->setMapping(s_edSelJoint, -1);
			EdRefreshTree(s_edTree);
			if (ghWnd) RefreshRetargetLists(ghWnd);
			UI_Printf("Lien retire.");
			InvalidateRect(hWnd, NULL, FALSE);
			return 0;
		}
		if (id == IDC_ED_CLOSE) { ShowWindow(hWnd, SW_HIDE); return 0; }
		break;
	}
	case WM_DRAWITEM: {
		DRAWITEMSTRUCT *dis = (DRAWITEMSTRUCT*)lParam;
		if (dis && dis->CtlType == ODT_BUTTON) { DrawFlatButton(dis); return TRUE; }
		break;
	}
	case WM_PAINT: {
		PAINTSTRUCT ps; HDC hdc = BeginPaint(hWnd, &ps);
		RECT rc; GetClientRect(hWnd, &rc);
		HDC mem = CreateCompatibleDC(hdc);
		if (!mem) { EndPaint(hWnd, &ps); return 0; }
		HBITMAP bmp = CreateCompatibleBitmap(hdc, rc.right > 1 ? rc.right : 1, rc.bottom > 1 ? rc.bottom : 1);
		if (!bmp) { DeleteDC(mem); EndPaint(hWnd, &ps); return 0; }
		HBITMAP oldBmp = (HBITMAP)SelectObject(mem, bmp);
		if (!oldBmp) { DeleteObject(bmp); DeleteDC(mem); EndPaint(hWnd, &ps); return 0; }
		HBRUSH bgB = CreateSolidBrush(Theme::BG);
		FillRect(mem, &rc, bgB); DeleteObject(bgB);

		RECT rh, rl, rcv, rr, rs; EdLayout(hWnd, rh, rl, rcv, rr, rs);
		// ---- Header ----
		HBRUSH hdB = CreateSolidBrush(Theme::PANEL);
		FillRect(mem, &rh, hdB); DeleteObject(hdB);
		HPEN hdP = CreatePen(PS_SOLID, 1, Theme::BORDER);
		HPEN hdOld = (HPEN)SelectObject(mem, hdP);
		MoveToEx(mem, 0, rh.bottom - 1, NULL); LineTo(mem, rc.right, rh.bottom - 1);
		SelectObject(mem, hdOld); DeleteObject(hdP);
		HFONT of = (HFONT)SelectObject(mem, s_edFontTitle ? s_edFontTitle : (HFONT)GetStockObject(DEFAULT_GUI_FONT));
		SetBkMode(mem, TRANSPARENT); SetTextColor(mem, Theme::TEXT);
		RECT tR = { 14, 6, rcv.left - 8, 28 };
		DrawText(mem, L"EDITEUR CORPS 2D  —  T-POSE MOCAP", -1, &tR, DT_LEFT | DT_SINGLELINE | DT_VCENTER);
		SelectObject(mem, s_edFontSmall ? s_edFontSmall : (HFONT)GetStockObject(DEFAULT_GUI_FONT));
		SetTextColor(mem, Theme::TEXT_DIM);
		RECT sR = { 14, 28, rcv.left - 8, 50 };
		DrawText(mem, L"Glissez un os import\u00E9 (gauche) vers un cercle du mannequin  •  mocap = parent", -1, &sR, DT_LEFT | DT_SINGLELINE | DT_VCENTER);
		// mapped pill
		WCHAR pill[32]; StringCchPrintf(pill, 32, L"%d / %d", EdMappedCount(), g_retargetJointCount);
		SIZE sz = {0}; GetTextExtentPoint32(mem, pill, (int)wcslen(pill), &sz);
		int pw = sz.cx + 28, ph = 24, px = rcv.left - pw - 12, py = 15;
		HBRUSH plB = CreateSolidBrush(Theme::TAKE_FILL);
		HPEN plP = CreatePen(PS_SOLID, 1, Theme::ACCENT);
		HGDIOBJ oB = SelectObject(mem, plB), oP = SelectObject(mem, plP);
		RoundRect(mem, px, py, px + pw, py + ph, 10, 10);
		SelectObject(mem, oB); SelectObject(mem, oP); DeleteObject(plB); DeleteObject(plP);
		SetTextColor(mem, Theme::TEXT);
		RECT pR = { px, py, px + pw, py + ph };
		DrawText(mem, pill, -1, &pR, DT_CENTER | DT_SINGLELINE | DT_VCENTER);
		SelectObject(mem, of);

		// ---- Left panel ----
		HBRUSH lpB = CreateSolidBrush(Theme::PANEL_DARK);
		RECT lpR = { rl.left - 4, rh.bottom + 4, rl.right + 4, rs.top - 4 };
		FillRect(mem, &lpR, lpB); DeleteObject(lpB);
		SelectObject(mem, s_edFontUI ? s_edFontUI : (HFONT)GetStockObject(DEFAULT_GUI_FONT));
		SetTextColor(mem, Theme::TEXT);
		RECT llR = { rl.left, rh.bottom + 8, rl.right, rh.bottom + 28 };
		DrawText(mem, L"OS IMPORT\u00C9", -1, &llR, DT_LEFT | DT_SINGLELINE);

		// ---- Right panel ----
		HBRUSH rpB = CreateSolidBrush(Theme::PANEL);
		RECT rpR = { rr.left - 4, rh.bottom + 4, rr.right + 4, rs.top - 4 };
		FillRect(mem, &rpR, rpB); DeleteObject(rpB);
		SelectObject(mem, s_edFontUI ? s_edFontUI : (HFONT)GetStockObject(DEFAULT_GUI_FONT));
		SetTextColor(mem, Theme::TEXT);
		RECT rl1 = { rr.left, rh.bottom + 8, rr.right, rh.bottom + 30 };
		DrawText(mem, L"D\u00C9TAIL", -1, &rl1, DT_LEFT | DT_SINGLELINE);
		{
			WCHAR jn[64] = L"(cliquez un cercle)", bn[128] = L"";
			if (s_edSelJoint >= 0) {
				MultiByteToWideChar(CP_UTF8, 0, EdJointLabel(s_edSelJoint), -1, jn, 64);
				if (g_retargeter) {
					int b = g_retargeter->getMapping(s_edSelJoint);
					if (b >= 0) {
						std::string en = MakeEnglish(g_retargeter->boneName(b));
						MultiByteToWideChar(CP_UTF8, 0, en.c_str(), -1, bn, 128);
					} else wcsncpy_s(bn, L"(vide — glissez un os)", _TRUNCATE);
				}
			}
			SelectObject(mem, s_edFontTitle ? s_edFontTitle : (HFONT)GetStockObject(DEFAULT_GUI_FONT));
			SetTextColor(mem, Theme::TEXT);
			RECT jr = { rr.left, rh.bottom + 34, rr.right, rh.bottom + 58 };
			DrawText(mem, jn, -1, &jr, DT_LEFT | DT_SINGLELINE | DT_WORD_ELLIPSIS);
			SelectObject(mem, s_edFontUI ? s_edFontUI : (HFONT)GetStockObject(DEFAULT_GUI_FONT));
			SetTextColor(mem, Theme::ACCENT);
			RECT br2 = { rr.left, rh.bottom + 58, rr.right, rh.bottom + 80 };
			DrawText(mem, bn, -1, &br2, DT_LEFT | DT_SINGLELINE | DT_WORD_ELLIPSIS);
			SelectObject(mem, s_edFontSmall ? s_edFontSmall : (HFONT)GetStockObject(DEFAULT_GUI_FONT));
			SetTextColor(mem, Theme::TEXT_DIM);
			RECT hint = { rr.left, rh.bottom + 118, rr.right, rs.top - 44 };
			DrawText(mem, L"Vert = mapp\u00E9\nGris = vide\nCercle bleu = cible du d\u00E9p\u00F4t\n\nAstuce : partez de la liste de gauche.", -1, &hint, DT_LEFT | DT_WORDBREAK);
		}

		// ---- Canvas ----
		HBRUSH cvB = CreateSolidBrush(Theme::BG);
		FillRect(mem, &rcv, cvB); DeleteObject(cvB);
		HPEN cvP = CreatePen(PS_SOLID, 1, Theme::BORDER);
		HGDIOBJ cvO = SelectObject(mem, cvP);
		Rectangle(mem, rcv.left, rcv.top, rcv.right, rcv.bottom);
		SelectObject(mem, cvO); DeleteObject(cvP);
		// dotted grid
		{
			HPEN gP = CreatePen(PS_DOT, 1, Theme::BORDER);
			HGDIOBJ gO = SelectObject(mem, gP);
			for (int gx = rcv.left + 20; gx < rcv.right; gx += 28) { MoveToEx(mem, gx, rcv.top + 4, NULL); LineTo(mem, gx, rcv.bottom - 4); }
			for (int gy = rcv.top + 20; gy < rcv.bottom; gy += 28) { MoveToEx(mem, rcv.left + 4, gy, NULL); LineTo(mem, rcv.right - 4, gy); }
			SelectObject(mem, gO); DeleteObject(gP);
		}
		auto proj = [&](int j, POINT &pt) {
			float x, y; GetTPosePos(j, x, y);
			int pad = 46;
			int cw = (rcv.right - rcv.left) - pad * 2;
			int ch = (rcv.bottom - rcv.top) - pad * 2;
			if (cw < 40) cw = 40; if (ch < 40) ch = 40;
			pt.x = rcv.left + pad + int(x * cw);
			pt.y = rcv.top + pad + int(y * ch);
		};
		auto isMapped = [&](int jt) -> bool {
			return g_retargeter && g_retargeter->getMapping(jt) >= 0;
		};
		struct Seg { int a, b, w; COLORREF on, off; };
		const Seg segs[] = {
			{20,2,15,RGB(126,156,214),RGB(72,80,98)}, {2,1,17,RGB(126,156,214),RGB(72,80,98)}, {1,0,18,RGB(126,156,214),RGB(72,80,98)},
			{20,4,10,RGB(110,190,140),RGB(66,86,74)}, {4,5,9,RGB(110,190,140),RGB(66,86,74)}, {5,6,8,RGB(110,190,140),RGB(66,86,74)}, {6,7,7,RGB(110,190,140),RGB(66,86,74)},
			{20,8,10,RGB(110,190,140),RGB(66,86,74)}, {8,9,9,RGB(110,190,140),RGB(66,86,74)}, {9,10,8,RGB(110,190,140),RGB(66,86,74)}, {10,11,6,RGB(110,190,140),RGB(66,86,74)},
			{0,12,11,RGB(122,162,230),RGB(70,80,100)}, {12,13,10,RGB(122,162,230),RGB(70,80,100)}, {13,14,9,RGB(122,162,230),RGB(70,80,100)},
			{0,16,11,RGB(122,162,230),RGB(70,80,100)}, {16,17,10,RGB(122,162,230),RGB(70,80,100)}, {17,18,9,RGB(122,162,230),RGB(70,80,100)},
		};
		for (auto &s : segs) {
			POINT pa, pb; proj(s.a, pa); proj(s.b, pb);
			bool m = isMapped(s.b);
			LOGBRUSH lb = { BS_SOLID, m ? s.on : s.off, 0 };
			HPEN p = ExtCreatePen(PS_GEOMETRIC | PS_SOLID | PS_ENDCAP_ROUND | PS_JOIN_ROUND, s.w, &lb, 0, NULL);
			if (!p) continue;
			HPEN oo = (HPEN)SelectObject(mem, p);
			MoveToEx(mem, pa.x, pa.y, NULL); LineTo(mem, pb.x, pb.y);
			SelectObject(mem, oo); DeleteObject(p);
		}
		// pelvis bar + head
		{
			POINT pl, pr; proj(12, pl); proj(16, pr);
			LOGBRUSH lb = { BS_SOLID, (isMapped(12) || isMapped(16)) ? RGB(126,156,214) : RGB(72,80,98), 0 };
			HPEN p = ExtCreatePen(PS_GEOMETRIC | PS_SOLID | PS_ENDCAP_ROUND | PS_JOIN_ROUND, 13, &lb, 0, NULL);
			if (p) {
				HPEN oo = (HPEN)SelectObject(mem, p);
				MoveToEx(mem, pl.x, pl.y, NULL); LineTo(mem, pr.x, pr.y);
				SelectObject(mem, oo); DeleteObject(p);
			}
			POINT hd; proj(3, hd);
			HBRUSH hb = CreateSolidBrush(isMapped(3) ? RGB(238,205,160) : RGB(120,110,100));
			HPEN hp = CreatePen(PS_SOLID, 2, RGB(230,232,240));
			HGDIOBJ ob = SelectObject(mem, hb), op = SelectObject(mem, hp);
			Ellipse(mem, hd.x - 17, hd.y - 19, hd.x + 17, hd.y + 19);
			SelectObject(mem, ob); SelectObject(mem, op); DeleteObject(hb); DeleteObject(hp);
		}
		// joints on top
		for (int i = 0; i < g_retargetJointCount; ++i) {
			int jt = g_retargetJoints[i].jointType;
			POINT pt; proj(jt, pt);
			bool mapped = isMapped(jt);
			bool sel = (jt == s_edSelJoint);
			bool hov = (jt == s_edHoverJoint);
			bool dropT = (s_edDragging && s_edDragBone >= 0 && jt == s_edHoverJoint);
			if (dropT) {
				HPEN dp = CreatePen(PS_DOT, 2, Theme::ACCENT);
				HGDIOBJ doo = SelectObject(mem, dp);
				HBRUSH nll = (HBRUSH)GetStockObject(NULL_BRUSH);
				HGDIOBJ dbo = SelectObject(mem, nll);
				Ellipse(mem, pt.x - 20, pt.y - 20, pt.x + 20, pt.y + 20);
				SelectObject(mem, doo); SelectObject(mem, dbo); DeleteObject(dp);
			}
			HBRUSH fb = CreateSolidBrush(dropT || hov ? Theme::ACCENT : (mapped ? RGB(80,220,130) : RGB(88,94,110)));
			HPEN fp2 = CreatePen(PS_SOLID, sel ? 3 : 2, sel ? Theme::ACCENT_HOT : RGB(228,230,238));
			HGDIOBJ ob2 = SelectObject(mem, fb), op2 = SelectObject(mem, fp2);
			Ellipse(mem, pt.x - 10, pt.y - 10, pt.x + 10, pt.y + 10);
			SelectObject(mem, ob2); SelectObject(mem, op2); DeleteObject(fb); DeleteObject(fp2);
			// labels
			WCHAR lab[64]; MultiByteToWideChar(CP_UTF8, 0, g_retargetJoints[i].label, -1, lab, 64);
			SelectObject(mem, s_edFontSmall ? s_edFontSmall : (HFONT)GetStockObject(DEFAULT_GUI_FONT));
			SetBkMode(mem, TRANSPARENT);
			SetTextColor(mem, (sel || hov) ? Theme::ACCENT_HOT : Theme::TEXT);
			bool leftSide = false;
			{
				float lx, ly; GetTPosePos(jt, lx, ly);
				leftSide = (lx < 0.5f);
			}
			RECT lr;
			if (jt == 3) lr = { pt.x - 60, pt.y - 44, pt.x + 60, pt.y - 24 };
			else if (leftSide) lr = { pt.x - 118, pt.y - 8, pt.x - 14, pt.y + 8 };
			else lr = { pt.x + 14, pt.y - 8, pt.x + 118, pt.y + 8 };
			DrawText(mem, lab, -1, &lr, DT_SINGLELINE | DT_VCENTER | (leftSide || jt == 3 ? DT_CENTER : DT_LEFT));
			if (mapped && g_retargeter) {
				int b = g_retargeter->getMapping(jt);
				if (b >= 0) {
					std::string en = MakeEnglish(g_retargeter->boneName(b));
					WCHAR bw[96]; MultiByteToWideChar(CP_UTF8, 0, en.c_str(), -1, bw, 96);
					SetTextColor(mem, Theme::TEXT_DIM);
					RECT br2;
					if (jt == 3) br2 = { pt.x - 60, pt.y + 20, pt.x + 60, pt.y + 34 };
					else if (leftSide) br2 = { pt.x - 118, pt.y + 8, pt.x - 14, pt.y + 22 };
					else br2 = { pt.x + 14, pt.y + 8, pt.x + 118, pt.y + 22 };
					DrawText(mem, bw, -1, &br2, DT_SINGLELINE | DT_VCENTER | (leftSide || jt == 3 ? DT_CENTER : DT_LEFT));
				}
			}
		}
		// ---- Status bar ----
		HBRUSH stB = CreateSolidBrush(Theme::PANEL);
		FillRect(mem, &rs, stB); DeleteObject(stB);
		HPEN stP = CreatePen(PS_SOLID, 1, Theme::BORDER);
		HGDIOBJ stO = SelectObject(mem, stP);
		MoveToEx(mem, 0, rs.top, NULL); LineTo(mem, rc.right, rs.top);
		SelectObject(mem, stO); DeleteObject(stP);
		{
			WCHAR st[256];
			if (s_edDragging && s_edDragBone >= 0 && g_retargeter) {
				std::string en = MakeEnglish(g_retargeter->boneName(s_edDragBone));
				WCHAR bw[96]; MultiByteToWideChar(CP_UTF8, 0, en.c_str(), -1, bw, 96);
				if (s_edHoverJoint >= 0) {
					WCHAR jw[64]; MultiByteToWideChar(CP_UTF8, 0, EdJointLabel(s_edHoverJoint), -1, jw, 64);
					StringCchPrintf(st, 256, L"D\u00E9posez  \"%s\"  \u2192  %s  (rel\u00E2chez pour lier)", bw, jw);
				} else StringCchPrintf(st, 256, L"Glissez \"%s\" vers un cercle du mannequin…", bw);
			} else if (s_edDragging && s_edDragJoint >= 0) {
				WCHAR jw[64]; MultiByteToWideChar(CP_UTF8, 0, EdJointLabel(s_edDragJoint), -1, jw, 64);
				StringCchPrintf(st, 256, L"Glissez \"%s\" vers un os de la liste de gauche…", jw);
			} else if (s_edHoverJoint >= 0) {
				WCHAR jw[64]; MultiByteToWideChar(CP_UTF8, 0, EdJointLabel(s_edHoverJoint), -1, jw, 64);
				StringCchPrintf(st, 256, L"%s — cliquez-glissez vers la liste, ou d\u00E9posez un os import\u00E9 dessus", jw);
			} else StringCchPrintf(st, 256, L"%d / %d mapp\u00E9s  •  mocap = parent, mesh = enfant", EdMappedCount(), g_retargetJointCount);
			SelectObject(mem, s_edFontSmall ? s_edFontSmall : (HFONT)GetStockObject(DEFAULT_GUI_FONT));
			SetTextColor(mem, Theme::TEXT_DIM);
			RECT sr2 = { 12, rs.top, rc.right - 12, rs.bottom };
			DrawText(mem, st, -1, &sr2, DT_LEFT | DT_SINGLELINE | DT_VCENTER);
		}
		// Splitter grips (brighter on hover/drag)
		{
			RECT gh, gl, gc, gr, gs; EdLayout(hWnd, gh, gl, gc, gr, gs);
			int hot = s_edSplitDrag ? s_edSplitDrag : s_edSplitHover;
			COLORREF gcol = hot ? Theme::ACCENT : Theme::BORDER;
			HPEN gp = CreatePen(PS_SOLID, 3, gcol);
			HGDIOBJ go = SelectObject(mem, gp);
			int y0 = 54 + 14, y1 = gs.top - 14, ym = (y0 + y1) / 2;
			int cxL = gl.right + 4, cxR = gc.right + 4;
			bool h1 = (hot == 1), h2 = (hot == 2);
			MoveToEx(mem, cxL, h1 ? y0 : ym - 30, NULL); LineTo(mem, cxL, h1 ? y1 : ym + 30);
			MoveToEx(mem, cxR, h2 ? y0 : ym - 30, NULL); LineTo(mem, cxR, h2 ? y1 : ym + 30);
			SelectObject(mem, go); DeleteObject(gp);
		}
		BitBlt(hdc, 0, 0, rc.right, rc.bottom, mem, 0, 0, SRCCOPY);
		SelectObject(mem, oldBmp); DeleteObject(bmp); DeleteDC(mem);
		EndPaint(hWnd, &ps); return 0;
	}
	case WM_SETCURSOR:
		if ((HWND)wParam == hWnd) {
			if (s_edSplitDrag) { SetCursor(LoadCursor(NULL, IDC_SIZEWE)); return TRUE; }
			if (!s_edDragging) {
				POINT pt; GetCursorPos(&pt); ScreenToClient(hWnd, &pt);
				int z = EdSplitHit(hWnd, pt.x, pt.y);
				if (z != s_edSplitHover) { s_edSplitHover = z; InvalidateRect(hWnd, NULL, FALSE); }
				if (z) { SetCursor(LoadCursor(NULL, IDC_SIZEWE)); return TRUE; }
				if (s_edSplitHover) { s_edSplitHover = 0; InvalidateRect(hWnd, NULL, FALSE); }
			}
		}
		break;
	case WM_LBUTTONDOWN: {
		SetFocus(hWnd);
		int sx = GET_X_LPARAM(lParam), sy = GET_Y_LPARAM(lParam);
		int sp = EdSplitHit(hWnd, sx, sy);
		if (sp && !s_edDragging) {
			s_edSplitDrag = sp; s_edSplitHover = sp;
			s_edSplitStart.x = sx; s_edSplitStart.y = sy;
			s_edSplitStartL = s_edLeftW; s_edSplitStartR = s_edRightW;
			SetCapture(hWnd); SetCursor(LoadCursor(NULL, IDC_SIZEWE));
			return 0;
		}
		int jt = EdHitJoint(hWnd, sx, sy);
		if (jt >= 0) {
			s_edSelJoint = jt; s_edDragJoint = jt; s_edDragBone = -1; s_edDragging = true;
			WCHAR jw[64]; MultiByteToWideChar(CP_UTF8, 0, EdJointLabel(jt), -1, jw, 64);
			CreateDragWindow(hWnd, jw, ([&]{ POINT p={sx,sy}; ClientToScreen(hWnd,&p); return p; })());
			SetCapture(hWnd); SetCursor(LoadCursor(NULL, IDC_HAND));
			InvalidateRect(hWnd, NULL, FALSE);
			return 0;
		}
		s_edSelJoint = -1; InvalidateRect(hWnd, NULL, FALSE);
		break;
	}
	case WM_MOUSEMOVE: {
		if (s_edSplitDrag) {
			POINT pt; GetCursorPos(&pt); ScreenToClient(hWnd, &pt);
			int dx = pt.x - s_edSplitStart.x;
			RECT rc; GetClientRect(hWnd, &rc);
			int W = rc.right - rc.left;
			if (s_edSplitDrag == 1) {
				int v = s_edSplitStartL + dx;
				int maxL = W - s_edRightW - 16 - 8 - 220;
				if (v < 170) v = 170;
				if (v > maxL && maxL > 170) v = maxL;
				if (v > 460) v = 460;
				s_edLeftW = v;
			} else if (s_edSplitDrag == 2) {
				int v = s_edSplitStartR - dx;
				int maxR = W - s_edLeftW - 16 - 8 - 220;
				if (v < 170) v = 170;
				if (v > maxR && maxR > 170) v = maxR;
				if (v > 380) v = 380;
				s_edRightW = v;
			}
			EdPositionChildren(hWnd);
			InvalidateRect(hWnd, NULL, FALSE);
			SetCursor(LoadCursor(NULL, IDC_SIZEWE));
			return 0;
		}
		int sx = GET_X_LPARAM(lParam), sy = GET_Y_LPARAM(lParam);
		int hov = EdHitJoint(hWnd, sx, sy);
		if (hov != s_edHoverJoint) { s_edHoverJoint = hov; InvalidateRect(hWnd, NULL, FALSE); }
		if (s_edDragging) {
			POINT ps; GetCursorPos(&ps);
			UpdateDragWindow(ps);
			if (s_edDragJoint >= 0 && s_edTree) {
				POINT pc = ps; ScreenToClient(s_edTree, &pc);
				TVHITTESTINFO ht = {0}; ht.pt = pc;
				HTREEITEM hIt = TreeView_HitTest(s_edTree, &ht);
				if (hIt && (ht.flags & (TVHT_ONITEM | TVHT_ONITEMINDENT)))
					TreeView_SelectDropTarget(s_edTree, hIt);
				else
					TreeView_SelectDropTarget(s_edTree, NULL);
			}
			SetCursor(LoadCursor(NULL, IDC_HAND));
			InvalidateRect(hWnd, NULL, FALSE);
			return 0;
		}
		break;
	}
	case WM_LBUTTONUP: {
		if (s_edSplitDrag) {
			s_edSplitDrag = 0;
			if (GetCapture() == hWnd) ReleaseCapture();
			SetCursor(LoadCursor(NULL, IDC_ARROW));
			InvalidateRect(hWnd, NULL, FALSE);
			return 0;
		}
		bool wasDrag = s_edDragging;
		int db = s_edDragBone, dj = s_edDragJoint;
		int sx = GET_X_LPARAM(lParam), sy = GET_Y_LPARAM(lParam);
		if (s_edTree) TreeView_SelectDropTarget(s_edTree, NULL);
		DestroyDragWindow();
		s_edDragging = false;
		if (GetCapture() == hWnd) ReleaseCapture();
		SetCursor(LoadCursor(NULL, IDC_ARROW));
		if (wasDrag && db >= 0) {
			// imported bone -> mocap joint under cursor
			int jt = EdHitJoint(hWnd, sx, sy);
			if (jt >= 0 && g_retargeter) {
				g_retargeter->setMapping(jt, db);
				EdRefreshTree(s_edTree);
				if (ghWnd) RefreshRetargetLists(ghWnd);
				UI_Printf("Editeur: %s -> %s.", EdJointLabel(jt), MakeEnglish(g_retargeter->boneName(db)).c_str());
				s_edSelJoint = jt;
			}
			s_edDragBone = -1;
			InvalidateRect(hWnd, NULL, FALSE);
			return 0;
		}
		if (wasDrag && dj >= 0 && s_edTree) {
			// mocap joint -> imported bone under cursor (tree hit)
			POINT ps; GetCursorPos(&ps);
			POINT pc = ps; ScreenToClient(s_edTree, &pc);
			TVHITTESTINFO ht = {0}; ht.pt = pc;
			HTREEITEM hIt = TreeView_HitTest(s_edTree, &ht);
			if (hIt && g_retargeter) {
				TVITEM tvi = {0}; tvi.mask = TVIF_PARAM; tvi.hItem = hIt;
				TreeView_GetItem(s_edTree, &tvi);
				int bone = (int)tvi.lParam;
				if (bone >= 0) {
					g_retargeter->setMapping(dj, bone);
					EdRefreshTree(s_edTree);
					if (ghWnd) RefreshRetargetLists(ghWnd);
					UI_Printf("Editeur: %s -> %s.", EdJointLabel(dj), MakeEnglish(g_retargeter->boneName(bone)).c_str());
					s_edSelJoint = dj;
				}
			}
		s_edDragJoint = -1;
		InvalidateRect(hWnd, NULL, FALSE);
		return 0;
		}
		break;
	}
	case WM_CLOSE: ShowWindow(hWnd, SW_HIDE); return 0;
	case WM_DESTROY:
		if (s_edFontTitle) DeleteObject(s_edFontTitle);
		if (s_edFontUI) DeleteObject(s_edFontUI);
		if (s_edFontSmall) DeleteObject(s_edFontSmall);
		s_edFontTitle = s_edFontUI = s_edFontSmall = NULL;
		s_edTree = NULL; s_edBtnAuto = s_edBtnClear = s_edBtnUnmap = s_edBtnClose = NULL;
		DestroyDragWindow();
		s_edDragging = false; s_edDragBone = s_edDragJoint = -1;
		s_edSplitDrag = 0; s_edSplitHover = 0;
		break;
	}
	return DefWindowProc(hWnd,msg,wParam,lParam);
}

// Reads a float from one of the retarget edit boxes
static float ReadRetargetEdit(HWND hWnd, int id, float def)
{
	HWND hEdit = GetDlgItem(hWnd, id);
	if (!hEdit)
		return def;
	char buf[64];
	GetWindowTextA(hEdit, buf, _countof(buf));
	float v = (float)atof(buf);
	if (v != v)   // NaN
		v = def;
	return v;
}

// Applies the values currently typed in the size / rotation edit boxes
static void ApplyImportTransformFromEdits(HWND hWnd)
{
	if (!g_retargeter)
		return;
	float s = ReadRetargetEdit(hWnd, SIZE_EDIT, 1.0f);
	float rx = ReadRetargetEdit(hWnd, ROT_X_EDIT, 0.0f);
	float ry = ReadRetargetEdit(hWnd, ROT_Y_EDIT, 0.0f);
	float rz = ReadRetargetEdit(hWnd, ROT_Z_EDIT, 0.0f);
	g_retargeter->setImportScale(s);
	g_retargeter->setImportRotation(rx, ry, rz);
	InvalidateRect(GetDlgItem(hWnd, VIEW_STATUS), NULL, TRUE);
}

// Writes the current import transform values into the edit boxes
static void SyncImportTransformEdits(HWND hWnd)
{
	if (!g_retargeter)
		return;
	char buf[64];
	sprintf_s(buf, "%.3f", g_retargeter->importScale());
	SetWindowTextA(GetDlgItem(hWnd, SIZE_EDIT), buf);
	sprintf_s(buf, "%.1f", g_retargeter->importRotX());
	SetWindowTextA(GetDlgItem(hWnd, ROT_X_EDIT), buf);
	sprintf_s(buf, "%.1f", g_retargeter->importRotY());
	SetWindowTextA(GetDlgItem(hWnd, ROT_Y_EDIT), buf);
	sprintf_s(buf, "%.1f", g_retargeter->importRotZ());
	SetWindowTextA(GetDlgItem(hWnd, ROT_Z_EDIT), buf);
}

// Refreshes the joint / bone lists of the mapping window
void RefreshRetargetLists(HWND hWnd)
{
	HWND hJoint = GetDlgItem(hWnd, RETARGET_JOINT_LIST);
	HWND hBone = GetDlgItem(hWnd, RETARGET_BONE_LIST);
	if (!g_retargeter)
		return;

	// Joint TreeView uses lParam = jointType, Bone ListBox uses index
	int jSel = -1, bSel = -1;
	if (hJoint) {
		HTREEITEM hSel = TreeView_GetSelection(hJoint);
		if (hSel) { TVITEM tvi={0}; tvi.mask=TVIF_PARAM; tvi.hItem=hSel; TreeView_GetItem(hJoint,&tvi); jSel=(int)tvi.lParam; }
		// fallback for old ListBox path (if not yet converted)
		if (jSel==-1) jSel = (int)SendMessage(hJoint, LB_GETCURSEL, 0, 0);
	}
	if (hBone) bSel = (int)SendMessage(hBone, LB_GETCURSEL, 0, 0);

	if (hJoint)
	{
		// Kinect joints as hierarchical TreeView (if TreeView, else ListBox compat)
		bool isTree = (GetWindowLong(hJoint, GWL_STYLE) & TVS_HASBUTTONS) != 0;
		if (isTree) {
			TreeView_DeleteAllItems(hJoint);
			// Kinect parent map (JointType -> parent JointType)
			const int kinectParent[JointType_Count] = {
				-1, 0, 1, 2, 20, 4, 5, 6, 20, 8, 9, 10, 0, 12,13,14, 0,16,17,18, 2, 7,7,11,11
			};
			std::vector<HTREEITEM> hJoints(JointType_Count, NULL);
			// Need map from JointType to display index
			for (int i=0;i<g_retargetJointCount;++i) {
				int jt = g_retargetJoints[i].jointType;
				int par = (jt>=0 && jt<JointType_Count) ? kinectParent[jt] : -1;
				HTREEITEM hPar = NULL;
				if (par>=0) {
					// find parent item already inserted
					for(int k=0;k<g_retargetJointCount;++k) if(g_retargetJoints[k].jointType==par && hJoints[g_retargetJoints[k].jointType]) { hPar=hJoints[par]; break; }
				}
				TVINSERTSTRUCT tvis={0};
				tvis.hParent = hPar ? hPar : TVI_ROOT;
				tvis.hInsertAfter = TVI_LAST;
				tvis.item.mask = TVIF_TEXT | TVIF_PARAM;
				WCHAR label[128];
				int boneIdx = g_retargeter->getMapping(jt);
				WCHAR boneW[64]={0};
				if (boneIdx>=0) { std::string en=MakeEnglish(g_retargeter->boneName(boneIdx)); MultiByteToWideChar(CP_UTF8,0,en.c_str(),-1,boneW,64); }
				WCHAR labelA[64]; MultiByteToWideChar(CP_UTF8,0,g_retargetJoints[i].label,-1,labelA,64);
				if (boneIdx>=0) StringCchPrintf(label,128,L"%s -> %s",labelA,boneW);
				else StringCchPrintf(label,128,L"%s -> (none)",labelA);
				tvis.item.pszText = label;
				tvis.item.lParam = jt;
				HTREEITEM hIt = TreeView_InsertItem(hJoint, &tvis);
				hJoints[jt]=hIt;
				if (hPar) TreeView_Expand(hJoint, hPar, TVE_EXPAND);
			}
			// restore selection
			if (jSel>=0) {
				// find item with lParam == jSel
				HTREEITEM hIt = TreeView_GetRoot(hJoint);
				std::function<HTREEITEM(HTREEITEM)> find = [&](HTREEITEM h)->HTREEITEM{
					while(h){ TVITEM t={0}; t.mask=TVIF_PARAM; t.hItem=h; TreeView_GetItem(hJoint,&t); if((int)t.lParam==jSel) return h; HTREEITEM c=TreeView_GetChild(hJoint,h); if(c){ HTREEITEM r=find(c); if(r) return r; } h=TreeView_GetNextSibling(hJoint,h); } return NULL;
				};
				HTREEITEM sel = find(hIt);
				if (sel) TreeView_SelectItem(hJoint, sel);
			}
		} else {
			SendMessage(hJoint, LB_RESETCONTENT, 0, 0);
			for (int i = 0; i < g_retargetJointCount; ++i)
			{
				WCHAR label[128];
				int boneIdx = g_retargeter->getMapping(g_retargetJoints[i].jointType);
				if (boneIdx >= 0)
				{
					WCHAR boneW[96];
					std::string en = MakeEnglish(g_retargeter->boneName(boneIdx));
					if (en.empty()) en = "?";
					MultiByteToWideChar(CP_UTF8, 0, en.c_str(), -1, boneW, _countof(boneW));
					WCHAR labelA[64];
					MultiByteToWideChar(CP_UTF8, 0, g_retargetJoints[i].label, -1, labelA, _countof(labelA));
					StringCchPrintf(label, _countof(label), L"%s  ->  %s", labelA, boneW);
				}
				else
				{
					WCHAR labelA[64];
					MultiByteToWideChar(CP_UTF8, 0, g_retargetJoints[i].label, -1, labelA, _countof(labelA));
					StringCchPrintf(label, _countof(label), L"%s  ->  (none)", labelA);
				}
				SendMessage(hJoint, LB_ADDSTRING, 0, (LPARAM)label);
			}
		}
	}

	// Left hierarchical TreeView (Unreal-like, retarget only) — restore memorized groups
	HWND hTree = GetDlgItem(hWnd, RETARGET_BONE_TREE);
	if (hTree)
	{
		TreeView_DeleteAllItems(hTree);
		int n = g_retargeter->boneCount();
		// New skeleton (import): reset memory, expand all parents by default
		if (s_expandedMainBones != n) {
			s_expandedMain.clear();
			for (int i = 0; i < n; ++i) {
				bool isParent = false;
				for (int k = 0; k < n; ++k) if (g_retargeter->bone(k).parent == i) { isParent = true; break; }
				if (isParent) s_expandedMain.insert(i);
			}
			s_expandedMainBones = n;
		}
		std::vector<HTREEITEM> hItems(n, NULL);
		for (int i = 0; i < n; ++i)
		{
			TVINSERTSTRUCT tvis = {0};
			tvis.hParent = (g_retargeter->bone(i).parent >= 0) ? hItems[g_retargeter->bone(i).parent] : TVI_ROOT;
			tvis.hInsertAfter = TVI_LAST;
			tvis.item.mask = TVIF_TEXT | TVIF_PARAM;
			WCHAR boneW[128];
			std::string en = MakeEnglish(g_retargeter->boneName(i));
			if (en.empty()) en = "?";
			MultiByteToWideChar(CP_UTF8, 0, en.c_str(), -1, boneW, _countof(boneW));
			tvis.item.pszText = boneW;
			tvis.item.lParam = i;
			hItems[i] = TreeView_InsertItem(hTree, &tvis);
		}
		// Re-apply the memorized expanded groups (mapping no longer resets them)
		for (int idx : s_expandedMain)
			if (idx >= 0 && idx < n && hItems[idx]) TreeView_Expand(hTree, hItems[idx], TVE_EXPAND);
		// Keep selection
		if (bSel >=0 && bSel < n && hItems[bSel]) TreeView_SelectItem(hTree, hItems[bSel]);
	}
	if (hBone) // legacy bottom bone list (hidden, kept for mapping compat)
	{
		SendMessage(hBone, LB_RESETCONTENT, 0, 0);
		for (int i = 0; i < g_retargeter->boneCount(); ++i)
		{
			WCHAR boneW[128];
			std::string en = MakeEnglish(g_retargeter->boneName(i));
			if (en.empty()) en = "?";
			MultiByteToWideChar(CP_UTF8, 0, en.c_str(), -1, boneW, _countof(boneW));
			SendMessage(hBone, LB_ADDSTRING, 0, (LPARAM)boneW);
		}
		if (bSel >= 0) SendMessage(hBone, LB_SETCURSEL, bSel, 0);
	}

	if (hJoint && jSel >= 0 && jSel < g_retargetJointCount)
		SendMessage(hJoint, LB_SETCURSEL, (WPARAM)jSel, 0);
	if (hBone && bSel >= 0 && bSel < g_retargeter->boneCount())
		SendMessage(hBone, LB_SETCURSEL, (WPARAM)bSel, 0);
}
