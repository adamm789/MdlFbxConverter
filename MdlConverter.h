#pragma once
#include <string>
#include <wtypes.h>
extern "C" {
	__declspec(dllexport) int ConvertToFbx(const wchar_t* mdlFilePath);
	__declspec(dllexport) int ConvertToFbxWithOutput(const wchar_t* mdlFilePath, const wchar_t* outputPath);
}
