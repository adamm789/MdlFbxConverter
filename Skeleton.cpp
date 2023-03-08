#include "Skeleton.h"
#include "include/json.hpp"
#include <iostream>
#include <fstream>
using json = nlohmann::json;

Bone* Skeleton::BuildSkeletonFromFile(std::string filePath) {

	std::ifstream ifs;
	ifs.open(filePath);
	fprintf(stdout, "Trying to read skeleton from %s\n", std::string(filePath).c_str());

	std::string s;
	std::map<int, Bone*> boneNumbers;
	std::map<int, int> boneNumberToParent;
	Bone* ret = NULL;

	while (std::getline(ifs, s)) {
		json data;
		try {
			data = json::parse(s);
			int number = data.at("BoneNumber");
			int parent = data.at("BoneParent");
			std::string name = data.at("BoneName");
			std::vector<double> matrixArr = data.at("PoseMatrix");

			Bone* b = new Bone();
			b->Name = name;
			b->Number = number;

			Eigen::Transform<double, 3, Eigen::Affine> matrix;

			for (int row = 0; row < 4; row++) {
				for (int col = 0; col < 4; col++) {
					matrix(row, col) = matrixArr[(row * 4) + col];
				}
			}

			b->PoseMatrix = matrix;
			boneNumbers.emplace(number, b);

			if (name == "n_root") {
				ret = boneNumbers[number];
			}

			if (parent != -1) {
				boneNumberToParent.emplace(number, parent);
			}
		}
		catch (json::parse_error& ex) {
			std::cerr << "parse error at byte " << ex.byte << std::endl;
		}
	}
	ifs.close();
	std::map<int, Bone*>::iterator it;
	for (it = boneNumbers.begin(); it != boneNumbers.end(); it++) {
		int num = it->first;
		Bone* b = it->second;

		auto parentIt = boneNumberToParent.find(num);
		if (parentIt != boneNumberToParent.end()) {
			Bone* p = boneNumbers.find(parentIt->second)->second;
			p->Children.push_back(b);
			b->Parent = p;
		}
	}
	

	return ret;
}
Bone* Skeleton::BuildSkeletonFromData(const char* data) {
	return NULL;
}
