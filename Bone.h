#pragma once
#include <string>
#include <vector>
#include "../eigen-3.4.0/Eigen/dense"
class Bone
{
public:
	std::string Name;
	int Number;
	Bone* Parent;
	std::vector<Bone*> Children;
	Eigen::Transform<double, 3, Eigen::Affine> PoseMatrix;

	Bone* GetBone(std::string name);
};

