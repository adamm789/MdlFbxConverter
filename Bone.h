#pragma once
#include <string>
#include <vector>
#include "Eigen/dense"
class Bone
{
public:
	Bone();
	Bone(std::string name);

	std::string Name = "";
	int Number = 0;
	Bone* Parent = nullptr;
	std::vector<Bone*> Children;
	Eigen::Transform<double, 3, Eigen::Affine> PoseMatrix = Eigen::Affine3d::Identity();

	Bone* GetBone(std::string name);
};

