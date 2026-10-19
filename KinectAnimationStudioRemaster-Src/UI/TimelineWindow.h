#pragma once

#include "..\common\stdafx.h"

// Registers the timeline window class
ATOM TimelineRegisterClass(HINSTANCE hInstance);

// Window procedure for the timeline strip
LRESULT CALLBACK TimelineProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
