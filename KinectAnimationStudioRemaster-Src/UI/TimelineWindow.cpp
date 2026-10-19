#include "TimelineWindow.h"
#include "Theme.h"
#include "..\kinect\Session.h"
#include "..\kinect\KBodyExporter.h"
#include <windowsx.h>

// Global exporter pointer (defined in UI.cpp)
extern KBodyExporter *g_pExporter;

// Format a time as mm:ss.cc
static void FormatTime(INT64 ms, WCHAR *out, int outLen)
{
	INT64 total = ms / 1000;
	INT64 min = total / 60;
	INT64 sec = total % 60;
	INT64 cent = (ms % 1000) / 10;
	swprintf_s(out, outLen, L"%02d:%02d.%02d", int(min), int(sec), int(cent));
}

static HBRUSH MakeBrush(COLORREF c)
{
	return CreateSolidBrush(c);
}

/// <summary>
/// Registers the timeline window class
/// </summary>
ATOM TimelineRegisterClass(HINSTANCE hInstance)
{
	WNDCLASSEX wcex;
	wcex.cbSize        = sizeof(WNDCLASSEX);
	wcex.style         = CS_HREDRAW | CS_VREDRAW;
	wcex.lpfnWndProc   = TimelineProc;
	wcex.cbClsExtra    = 0;
	wcex.cbWndExtra    = 0;
	wcex.hInstance     = hInstance;
	wcex.hIcon         = NULL;
	wcex.hCursor       = LoadCursor(NULL, IDC_SIZEWE);
	wcex.hbrBackground = MakeBrush(Theme::TIMELINE_BG);
	wcex.lpszMenuName  = NULL;
	wcex.lpszClassName = L"KAS_TimelineClass";
	wcex.hIconSm       = NULL;

	ATOM result = RegisterClassEx(&wcex);
	DeleteObject(wcex.hbrBackground);
	return result;
}

/// <summary>
/// Maps a client X coordinate to a relative time (0..duration)
/// </summary>
static INT64 ClientToTime(HWND hWnd, int x, INT64 duration)
{
	RECT rc;
	GetClientRect(hWnd, &rc);

	const int left = 12;
	const int right = rc.right - 12;
	int width = right - left;
	if (width <= 0 || duration <= 0)
		return 0;

	float t = float(x - left) / float(width);
	if (t < 0.0f) t = 0.0f;
	if (t > 1.0f) t = 1.0f;

	return INT64(t * duration);
}

/// <summary>
/// Draws the timeline content with double buffering
/// </summary>
static void DrawTimeline(HWND hWnd, HDC hdc, RECT &rc)
{
	HBRUSH bgBrush = MakeBrush(Theme::TIMELINE_BG);
	HBRUSH fillBrush = MakeBrush(Theme::TAKE_FILL);
	HBRUSH borderBrush = MakeBrush(Theme::BORDER);
	HBRUSH playheadBrush = MakeBrush(Theme::PLAYHEAD);

	FillRect(hdc, &rc, bgBrush);

	INT64 duration = g_pExporter ? g_pExporter->takeDurationMS() : 0;
	bool hasTake = g_pExporter ? g_pExporter->hasTake() : false;

	const int left = 12;
	const int right = rc.right - 12;
	const int top = 12;
	const int barBottom = rc.bottom - 26;
	int barWidth = right - left;

	if (duration > 0 && barWidth > 0)
	{
		// Take region
		RECT takeRect = { left, top, right, barBottom };
		FrameRect(hdc, &takeRect, borderBrush);

		// Fill the take region
		RECT fillRect = { left + 1, top + 1, right - 1, barBottom - 1 };
		FillRect(hdc, &fillRect, fillBrush);

		// Resolve the in/out trim markers
		INT64 inMS = 0, outMS = duration;
		bool hasTrim = false;
		if (g_session.hasRange)
		{
			inMS = g_session.playInMS; if (inMS < 0) inMS = 0; if (inMS > duration) inMS = duration;
			outMS = g_session.playOutMS; if (outMS < 0) outMS = 0; if (outMS > duration) outMS = duration;
			if (inMS > outMS) { INT64 t = inMS; inMS = outMS; outMS = t; }
			if (inMS > 0 || outMS < duration)
				hasTrim = true;
		}

		// Keyframe ticks (thin vertical lines per recorded frame)
		const KRecording &rec = g_pExporter->recording();
		size_t frameCount = rec.size();
		if (frameCount > 0)
		{
			HBRUSH tickBrush = MakeBrush(Theme::ACCENT);
			HBRUSH tickDark = MakeBrush(Theme::PANEL_DARK);

			RECT tickArea = { left + 1, top + 1, right - 1, barBottom - 1 };
			// Only draw ticks when there are not too many of them
			if (frameCount < 20000)
			{
				for (size_t i = 0; i < frameCount; ++i)
				{
					INT64 t = rec.frames[i].timeMS - rec.startTimeMS();
					int x = left + 1 + int((double(t) / double(duration)) * (barWidth - 2));

					RECT tick = { x, tickArea.top + 2, x + 1, tickArea.bottom - 2 };
					FillRect(hdc, &tick, tickBrush);
				}
			}
			else
			{
				FillRect(hdc, &tickArea, tickBrush);
			}

			DeleteObject(tickBrush);
			DeleteObject(tickDark);

			// Progress region up to the playhead
			INT64 play = g_session.playheadMS;
			if (play > 0 && play <= duration)
			{
				int px = left + int((double(play) / double(duration)) * barWidth);
				RECT prog = { left + 1, top + 1, px, barBottom - 1 };
				HBRUSH progBrush = MakeBrush(RGB(70, 90, 130));
				FillRect(hdc, &prog, progBrush);
				DeleteObject(progBrush);
			}
		}

		// In/out trim: shade the excluded regions and draw the boundary lines
		if (hasTrim)
		{
			int xIn = left + int((double(inMS) / double(duration)) * barWidth);
			int xOut = left + int((double(outMS) / double(duration)) * barWidth);

			HBRUSH dimBrush = MakeBrush(RGB(26, 30, 40));
			if (xIn > left)
			{
				RECT r = { left + 1, top + 1, xIn, barBottom - 1 };
				FillRect(hdc, &r, dimBrush);
			}
			if (xOut < right)
			{
				RECT r = { xOut, top + 1, right - 1, barBottom - 1 };
				FillRect(hdc, &r, dimBrush);
			}
			DeleteObject(dimBrush);

			HPEN inPen = CreatePen(PS_SOLID, 2, Theme::ACCENT);
			HPEN outPen = CreatePen(PS_SOLID, 2, Theme::DANGER);
			HPEN oldPen = (HPEN)SelectObject(hdc, inPen);
			MoveToEx(hdc, xIn, top - 1, NULL);
			LineTo(hdc, xIn, barBottom);
			SelectObject(hdc, outPen);
			MoveToEx(hdc, xOut, top - 1, NULL);
			LineTo(hdc, xOut, barBottom);
			SelectObject(hdc, oldPen);
			DeleteObject(inPen);
			DeleteObject(outPen);
		}

		// Playhead
		INT64 play = g_session.playheadMS;
		int px = left + int((double(play) / double(duration)) * barWidth);

		// Handle (triangle)
		POINT handle[3];
		handle[0] = { px - 5, top - 1 };
		handle[1] = { px + 5, top - 1 };
		handle[2] = { px, top + 7 };
		HPEN playPen = CreatePen(PS_SOLID, 1, Theme::PLAYHEAD);
		HPEN oldPen = (HPEN)SelectObject(hdc, playPen);
		HBRUSH oldBrush = (HBRUSH)SelectObject(hdc, playheadBrush);
		Polygon(hdc, handle, 3);

		// Line
		MoveToEx(hdc, px, top + 7, NULL);
		LineTo(hdc, px, barBottom);
		SelectObject(hdc, oldPen);
		SelectObject(hdc, oldBrush);
		DeleteObject(playPen);

		// Time labels (same scale as the rest of the UI)
		SetBkMode(hdc, TRANSPARENT);
		HFONT font = CreateFont(13, 0, 0, 0, FW_SEMIBOLD, 0, 0, 0, DEFAULT_CHARSET,
			OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY,
			DEFAULT_PITCH | FF_DONTCARE, L"Segoe UI");
		HFONT oldFont = (HFONT)SelectObject(hdc, font);
		SetTextColor(hdc, Theme::TEXT);

		WCHAR startTxt[32], endTxt[32], midTxt[32];
		FormatTime(0, startTxt, 32);
		FormatTime(duration, endTxt, 32);
		FormatTime(duration / 2, midTxt, 32);

		RECT tl = { left, barBottom + 2, left + 90, rc.bottom };
		DrawText(hdc, startTxt, -1, &tl, DT_LEFT | DT_VCENTER | DT_SINGLELINE);

		RECT tr = { right - 90, barBottom + 2, right, rc.bottom };
		DrawText(hdc, endTxt, -1, &tr, DT_RIGHT | DT_VCENTER | DT_SINGLELINE);

		RECT tm = { left + 90, barBottom + 2, right - 90, rc.bottom };
		DrawText(hdc, midTxt, -1, &tm, DT_CENTER | DT_VCENTER | DT_SINGLELINE);

		SelectObject(hdc, oldFont);
		DeleteObject(font);
	}
	else
	{
		SetBkMode(hdc, TRANSPARENT);
		HFONT font = CreateFont(13, 0, 0, 0, FW_NORMAL, 0, 0, 0, DEFAULT_CHARSET,
			OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY,
			DEFAULT_PITCH | FF_DONTCARE, L"Segoe UI");
		HFONT oldFont = (HFONT)SelectObject(hdc, font);
		SetTextColor(hdc, Theme::TEXT_DIM);

		RECT empty = rc;
		LPCWSTR msg = hasTake ? L"Take is empty" : L"Nothing recorded yet - press Record to capture a take";
		DrawText(hdc, msg, -1, &empty, DT_CENTER | DT_VCENTER | DT_SINGLELINE);

		SelectObject(hdc, oldFont);
		DeleteObject(font);
	}

	DeleteObject(bgBrush);
	DeleteObject(fillBrush);
	DeleteObject(borderBrush);
	DeleteObject(playheadBrush);
}

/// <summary>
/// Window procedure for the timeline strip
/// </summary>
LRESULT CALLBACK TimelineProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	switch (message)
	{
	case WM_PAINT:
		{
			PAINTSTRUCT ps;
			BeginPaint(hWnd, &ps);

			RECT rc;
			GetClientRect(hWnd, &rc);

			// Double buffering
			HDC memDC = CreateCompatibleDC(ps.hdc);
			HBITMAP memBmp = CreateCompatibleBitmap(ps.hdc, rc.right - rc.left, rc.bottom - rc.top);
			HBITMAP oldBmp = (HBITMAP)SelectObject(memDC, memBmp);

			DrawTimeline(hWnd, memDC, rc);

			BitBlt(ps.hdc, 0, 0, rc.right, rc.bottom, memDC, 0, 0, SRCCOPY);

			SelectObject(memDC, oldBmp);
			DeleteObject(memBmp);
			DeleteDC(memDC);

			EndPaint(hWnd, &ps);
		}
		return 0;

	case WM_ERASEBKGND:
		return 1;

	case WM_LBUTTONDOWN:
		{
			INT64 duration = g_pExporter ? g_pExporter->takeDurationMS() : 0;
			if (duration > 0)
			{
				SetCapture(hWnd);
				int x = GET_X_LPARAM(lParam);
				g_session.playheadMS = ClientToTime(hWnd, x, duration);
				InvalidateRect(hWnd, NULL, FALSE);
			}
		}
		return 0;

	case WM_MOUSEMOVE:
		if (GetCapture() == hWnd)
		{
			INT64 duration = g_pExporter ? g_pExporter->takeDurationMS() : 0;
			if (duration > 0)
			{
				int x = GET_X_LPARAM(lParam);
				g_session.playheadMS = ClientToTime(hWnd, x, duration);
				InvalidateRect(hWnd, NULL, FALSE);
			}
		}
		return 0;

	case WM_LBUTTONUP:
		if (GetCapture() == hWnd)
			ReleaseCapture();
		return 0;

	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
	}
}
