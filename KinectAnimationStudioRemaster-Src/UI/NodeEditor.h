#pragma once

#include "..\common\stdafx.h"

// Separate top-level window hosting the Node Motion Editor:
// canvas (nodes show their name only) + left details panel
// (Unreal-style parameters + bone array with + / dropdown).
ATOM NodeEditorRegisterClass(HINSTANCE hInstance);
HWND CreateNodeEditorWindow(HWND hParent);
LRESULT CALLBACK NodeEditorProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);
