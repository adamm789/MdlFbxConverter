#pragma once

#include <string>
#include <map>
#include <fbxsdk.h>
#include "LuminaPlusPlus/Models/Models/Mesh.h"
//#include <LuminaPlusPlus/Data/Files/MdlFile.h>
class FbxToMdlConverter
{
public:
	__declspec(dllexport) int ImportFbx(std::string fbxFilePath);

private:
	FbxManager* manager;
	FbxScene* scene;
	std::vector<std::string> BoneNames;
	//MdlFile* mdlFile;

	void TestNode(FbxNode* pNode);
	void SaveNode(Mesh* parent, FbxNode* pNode, int subMeshIndex);

	FbxSkin* GetSkin(FbxMesh* mesh);
	FbxBlendShape* GetMorpher(FbxMesh* mesh);

	void IterateNode(FbxNode* pNode);

	int GetDirectIndex(FbxMesh* mesh, FbxLayerElementTemplate<FbxVector4>* layerElement, int index_id);
	int GetDirectIndex(FbxMesh* mesh, FbxLayerElementTemplate<FbxVector2>* layerElement, int index_id);
	int GetDirectIndex(FbxMesh* mesh, FbxLayerElementTemplate<FbxColor>* layerElement, int index_id);
	FbxVector4 GetPosition(FbxMesh* const mesh, int index_id);
	FbxVector4 GetNormal(FbxMesh* const mesh, int index_id);
	FbxVector2 GetUV1(FbxMesh* const mesh, int index_id);
	FbxVector2 GetUV2(FbxMesh* const mesh, int index_id);
	FbxColor GetVertexColor(FbxMesh* const mesh, int index_id);
};

struct Weight {
	int BoneIndex;
	double Value;

	Weight(int idx, double v) {
		BoneIndex = idx;
		Value = v;
	}
};
