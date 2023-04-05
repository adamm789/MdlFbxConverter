#include "MdlConverter.h"
#include "MdlToFbxConverter.h"
#include <stdlib.h>

int ConvertToFbx(const wchar_t* wideStr)
{
	char buffer[500];
	size_t charsConverted = 0;
	wcstombs_s(&charsConverted, buffer, 500, wideStr, 500);
	MdlToFbxConverter converter(buffer);

	return 0;
}

int ConvertToFbxWithOutput(const wchar_t* mdlFilePath, const wchar_t* outputPath)
{
	char mdlBuffer[500];
	char outputBuffer[500];
	size_t mdlCharsConverted = 0;
	size_t outputCharsConverted = 0;

	wcstombs_s(&mdlCharsConverted, mdlBuffer, 500, mdlFilePath, 500);
	wcstombs_s(&outputCharsConverted, outputBuffer, 500, outputPath, 500);
	MdlToFbxConverter converter(mdlBuffer, outputBuffer);
	return 0;
}