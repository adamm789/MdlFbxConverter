#pragma once

#include <string>
#include "../LuminaPlusPlus/Models/Models/Model.h"
#include "include/fbxsdk.h"
#include "Skeleton.h"


class MdlToFbxConverter
{
public:
	MdlToFbxConverter(const char* filePath, const char* outputPath = "output.fbx");
	~MdlToFbxConverter();

	void SetSkeletonFromFile(std::string filePath);
	void SetSkeletonFromData(const char* data);

private:
	Model* model;
	MdlFile* mdlFile;
	FbxManager* manager;
	FbxScene* scene;
	std::map<std::string, FbxSurfaceMaterial*> MaterialPathToSurfaceMaterial;
	std::map<Bone*, FbxNode*> BoneToNode;
	Bone* n_root;
	std::string outputPath;

	void InitScene();
	void CreateScene(Model* model);
	int ExportScene();
	void AddPartToScene(Mesh* group, Submesh* part, FbxNode* parent, int indicesOffset, int partNumber);
	FbxMesh* MakeMesh(std::vector<Vertex>& vertices, std::vector<unsigned short>& indices, std::string meshName, FbxNode* parent, FbxSurfaceMaterial* material);
	void AddBoneToScene(Bone*, FbxPose* bindPose, FbxNode* parentNode);
	void CreateMaterials();
	FbxShape* MakeShape(std::vector<Vertex>& vertices, std::string meshName);

	void AddShapeToScene(std::vector<Vertex>& vertices, std::vector<Shape> shapes, FbxNode* parent);
};

