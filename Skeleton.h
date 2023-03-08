#pragma once
#include <string>
#include "Bone.h"
static class Skeleton
{
public:
	// TODO: Build skeleton from other sources
	static Bone* BuildSkeletonFromFile(std::string filePath);
	static Bone* BuildSkeletonFromData(const char* data);
};

