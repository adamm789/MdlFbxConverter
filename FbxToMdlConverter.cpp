#include "FbxToMdlConverter.h"
#include <regex>
#include <Models/Models/Model.h>
#include <Models/Models/Vertex.h>

// https://github.com/TexTools/TT_FBX_Reader/blob/master/TT_FBX/src/fbx_importer.cpp

const std::regex meshRegex(".*[_ ^][0-9]+[\\.\\-]?([0-9]+)?$");
const std::regex extractMeshInfoRegex(".*[_ ^]([0-9]+)[\\.\\-]([0-9]+)$");

std::map<int, std::map<int, FbxNode*>> groupPartToNode;


int FbxToMdlConverter::ImportFbx(std::string fbxFilePath) {
	fprintf(stdout, "Attempting to process fbx: %s\n", fbxFilePath.c_str());

	manager = FbxManager::Create();
	FbxIOSettings* ios = FbxIOSettings::Create(manager, IOSROOT);
	manager->SetIOSettings(ios);

	FbxImporter* importer = FbxImporter::Create(manager, "");
	bool success = importer->Initialize(fbxFilePath.c_str(), -1, manager->GetIOSettings());
	if (!success) {
		fprintf(stderr, "Could not load FBX file\n");
		return -1;
	}

	scene = FbxScene::Create(manager, "root");
	importer->Import(scene);
	importer->Destroy();

	FbxSystemUnit::m.ConvertScene(scene);
	auto up = FbxAxisSystem::EUpVector::eYAxis;
	auto front = FbxAxisSystem::EFrontVector::eParityOdd;
	auto handedness = FbxAxisSystem::eRightHanded;
	FbxAxisSystem dbAxis(up, front, handedness);
	dbAxis.ConvertScene(scene);

	FbxNode* root = scene->GetRootNode();
	if (root) {
		for (int i = 0; i < root->GetChildCount(); i++) {
			IterateNode(root->GetChild(i));
		}
	}

	Model* model = new Model();

	for (int groupNum = 0; groupNum < groupPartToNode.size(); groupNum++) {
		std::map<int, FbxNode*> partToNode = groupPartToNode[groupNum];
		Mesh* group = new Mesh(groupNum);
		for (int partNum = 0; partNum < partToNode.size(); partNum++) {
			SaveNode(group, partToNode[partNum], partNum);
		}
		model->Meshes.push_back(*group);
	}

	return 0;
}

void FbxToMdlConverter::IterateNode(FbxNode* pNode) {
	if (pNode == NULL) {
		return;
	}

	FbxMesh* mesh = pNode->GetMesh();
	std::string meshName = pNode->GetName();

	std::smatch m;

	bool success = std::regex_match(meshName, m, extractMeshInfoRegex);

	// Somehow we got here with a badly named mesh.
	if (success) {
		std::string meshMatch = m[1];
		std::string partMatch = m[2];

		int meshNum = std::atoi(meshMatch.c_str());;
		int partNum = 0;
		if (partMatch != "") {
			partNum = std::atoi(partMatch.c_str());;
		}

		auto group = groupPartToNode.find(meshNum);
		if (group == groupPartToNode.end()) {
			std::map<int, FbxNode*> part;
			part.emplace(partNum, pNode);
			groupPartToNode.emplace(meshNum, part);
		}
		else {
			auto part = group->second.find(partNum);
			if (part == group->second.end()) {
				group->second.emplace(partNum, pNode);
			}
			else {
				fprintf(stderr, "Mesh %i.%i already exists.\n", meshNum, partNum);
				return;
			}
		}
	}

	for (int i = 0; i < pNode->GetChildCount(); i++) {
		IterateNode(pNode->GetChild(i));
	}
}

static bool NearlyEqual(FbxVector4 a, FbxVector4 b) {
	double epsilon = 0.000001;

	if (std::abs(a[0] - b[0]) > epsilon) {
		return false;
	}
	if (std::abs(a[1] - b[1]) > epsilon) {
		return false;
	}
	if (std::abs(a[2] - b[2]) > epsilon) {
		return false;
	}
	if (std::abs(a[3] - b[3]) > epsilon) {
		return false;
	}

	return true;
}

void FbxToMdlConverter::SaveNode(Mesh* parent, FbxNode* node, int subMeshIndex) {
	FbxMesh* mesh = node->GetMesh();

	int numVertices = mesh->GetControlPointsCount();
	int numIndices = mesh->GetPolygonVertexCount();
	if (numIndices == 0 || numVertices == 0) {
		// Mesh does not actually have any tris.
		fprintf(stderr, "Mesh has no vertices/trianges\n");
		return;
	}

	FbxSkin* skin = GetSkin(mesh);
	if (skin == NULL) {
		// Mesh does not actually have a skin.
		fprintf(stderr, "Mesh does not have a valid skin element. Armature?\n");
	}

	// TODO: BoneTable...
	std::map<std::string, uint16_t> boneNameToBoneTableIndex;
	std::map<int, std::vector<Weight>> weights;

	// Vector of [control point index] => [Set of tri indexes that reference it.
	std::vector<std::vector<int>> controlToPolyArray;
	controlToPolyArray.resize(mesh->GetControlPointsCount());
	int polys = mesh->GetPolygonCount();
	if (polys != (numIndices / 3.0f)) {
		fprintf(stderr, "FBX is not fully triangulated.\n");
		return;
	}

	for (int i = 0; i < numIndices; i++) {
		int controlPointIndex = mesh->GetPolygonVertex(i / 3, i % 3);
		controlToPolyArray[controlPointIndex].push_back(i);
	}

	if (skin != NULL) {
		int numClusters = skin->GetClusterCount();
		for (int i = 0; i < skin->GetClusterCount(); i++) {
			FbxCluster::ELinkMode mode = skin->GetCluster(i)->GetLinkMode();
			std::string name = skin->GetCluster(i)->GetLink()->GetName();

			int boneIdx = 0;
			auto it = std::find(BoneNames.begin(), BoneNames.end(), name);
			if (it == BoneNames.end()) {
				boneIdx = BoneNames.size();
				BoneNames.push_back(name);
			}
			else {
				boneIdx = it->npos;
			}

			int affectedVertCount = skin->GetCluster(i)->GetControlPointIndicesCount();
			if (affectedVertCount == 0) continue;

			for (int vi = 0; vi < affectedVertCount; vi++) {
				int cpIndex = skin->GetCluster(i)->GetControlPointIndices()[vi];
				double weight = skin->GetCluster(i)->GetControlPointWeights()[vi];

				// TODO: This currently allows more than four weights to be added (when xivmdls can only reference four bones)
				weights[cpIndex].push_back(Weight(boneIdx, weight));
			}
		}
	}

	FbxBlendShape* morpher = GetMorpher(mesh);
	int deformerCount = mesh->GetDeformerCount();

	int vertexCount = mesh->GetControlPointsCount();
	auto meshVerts = mesh->GetControlPoints();

	auto vertArray = new FbxVector4[vertexCount];
	memcpy(vertArray, meshVerts, vertexCount * sizeof(FbxVector4));

	bool anyActiveBlends = false;

	auto worldTransform = node->EvaluateGlobalTransform();
	auto normalMatri = node->EvaluateGlobalTransform().Inverse().Transpose();

	// Loop Deformation Blends first
	for (int i = 0; i < deformerCount; i++) {
		FbxDeformer* d = mesh->GetDeformer(i);
		FbxDeformer::EDeformerType dType = d->GetDeformerType();
		if (dType == FbxDeformer::eBlendShape) {
			auto morpher = (FbxBlendShape*)d;

			if (morpher != NULL) {
				int channelCount = morpher->GetBlendShapeChannelCount();
				for (int j = 0; j < channelCount; j++) {
					FbxBlendShapeChannel* channel = morpher->GetBlendShapeChannel(j);
					FbxSubDeformer::EType subType = channel->GetSubDeformerType();

					int shapeCount = channel->GetTargetShapeCount();
					if (shapeCount == 0) {
						continue;
					}
					else if (shapeCount > 1) {
						fprintf(stderr, "Invalid shape channel. Skipping.\n");
						continue;
					}

					FbxShape* fbxShape = channel->GetTargetShape(0);
					auto name = std::string(fbxShape->GetName());
					if (name.find("shp_")) {

					}
					else {
						double pct = channel->DeformPercent;
						if (pct == 0.0) {
							continue;
						}
						fprintf(stdout, "Applying blend shape %s\n", name.c_str());
						anyActiveBlends = true;

						for (int k = 0; k < vertexCount; k++) {
							auto original = fbxShape->GetControlPoints()[k];
							auto target = vertArray[k];
							auto diff = original - target;
							FbxVector4 lInfluence = diff * pct * 0.01;
							auto len = std::abs(lInfluence.Length());
							vertArray[k] += lInfluence;
						}
					}
				}
			}
		}
	}

	if (anyActiveBlends) {

		// If we had any deforming blends, copy the now deformed vertices
		// into the base array.
		auto vertexCount = mesh->GetControlPointsCount();
		auto meshVerts = mesh->GetControlPoints();

		// Setup our vertex deformation array.
		memcpy(meshVerts, vertArray, vertexCount * sizeof(FbxVector4));
	}


	std::vector<std::string> shapeNames;
	std::map<std::string, std::map<int, Vertex>> shapeVertices;
	// Handle "shp_" deformations
	for (int i = 0; i < deformerCount; i++) {
		FbxDeformer* d = mesh->GetDeformer(i);
		FbxDeformer::EDeformerType dType = d->GetDeformerType();
		if (dType == FbxDeformer::eBlendShape) {
			auto morpher = (FbxBlendShape*)d;
			if (morpher != NULL) {
				int channelCount = morpher->GetBlendShapeChannelCount();

				for (int i = 0; i < morpher->GetBlendShapeChannelCount(); i++) {
					FbxBlendShapeChannel* channel = morpher->GetBlendShapeChannel(i);
					FbxSubDeformer::EType subtype = channel->GetSubDeformerType();

					int shapeCount = channel->GetTargetShapeCount();
					if (shapeCount == 0) {
						continue;
					}
					else if (shapeCount > 1) {
						fprintf(stderr, "%s contains invalid shape channel\n", node->GetName());
						continue;
					}

					FbxShape* fbxShape = channel->GetTargetShape(0);

					auto name = std::string(fbxShape->GetName());
					if (name.rfind("shp_", 0) == 0) {
						auto skip = false;
						if (std::find(shapeNames.begin(), shapeNames.end(), name) != shapeNames.end()) {
							skip = true;
						}

						if (skip) {
							fprintf(stderr, "Shape name: %s is included more than once.\n", name.c_str());
							continue;
						}

						shapeNames.push_back(name);
						for (int j = 0; j < vertexCount; j++) {
							auto shapeVert = fbxShape->GetControlPointAt(j);
							auto baseVert = mesh->GetControlPointAt(j);

							if (!NearlyEqual(shapeVert, baseVert)) {
								Vertex sVert = Vertex();
								auto worldPos = worldTransform.MultT(shapeVert);

								for (int k = 0; k < 4; k++) {
									sVert.Position[k] = worldPos.mData[k];
								}
								if (shapeVertices.find(name) == shapeVertices.end()) {
									shapeVertices.emplace(name, std::map<int, Vertex>());
								}
								shapeVertices[name].insert({ j, sVert });
							}
						}
					}
				}
			}
		}
	}

	std::vector<Vertex> vertices;
	std::vector<int> triIndices;
	triIndices.resize(numIndices);
	std::map<int, std::vector<int>> controlPointToVertexMapping;

	for (int cpi = 0; cpi < controlToPolyArray.size(); cpi++) {
		int sharedIndexCount = controlToPolyArray[cpi].size();
		int oldSize = vertices.size();

		if (sharedIndexCount == 0) {
			continue;
		}

		std::vector<Vertex> sharedVerts;
		for (int ti = 0; ti < sharedIndexCount; ti++) {
			Vertex myVert = Vertex();
			int indexId = controlToPolyArray[cpi][ti];

			auto vertWorldPosition = worldTransform.MultT(GetPosition(mesh, indexId));
			auto vertWorldNormal = normalMatri.MultT(GetNormal(mesh, indexId));
			auto localNormal = GetNormal(mesh, indexId);
			vertWorldNormal.Normalize();

			auto vertexColor = GetVertexColor(mesh, indexId);

			for (int i = 0; i < 4; i++) {
				myVert.Position[i] = vertWorldPosition.mData[i];

				if (i != 3) {
					myVert.Normal[i] = vertWorldNormal.mData[i];
				}
			}

			myVert.Color[0] = vertexColor.mRed;
			myVert.Color[1] = vertexColor.mGreen;
			myVert.Color[2] = vertexColor.mBlue;
			myVert.Color[3] = vertexColor.mAlpha;

			auto uv1 = GetUV1(mesh, indexId);
			auto uv2 = GetUV2(mesh, indexId);

			// Guess we have to flip the "v" value
			myVert.UV[0] = uv1[0];
			myVert.UV[1] = -uv1[1];
			myVert.UV[2] = uv2[0];
			myVert.UV[3] = -uv2[1];

			// TODO: BlendIndices
			// TODO: Bone names seem to be sorted alphabetically
			std::vector<Weight> weightSet = weights[cpi];
			for (int i = 0; i < 4; i++) {
				if (weightSet.size() > i) {
					myVert.BlendWeights[i] = weightSet[i].Value;
				}
				else {
					myVert.BlendWeights[i] = 0;
				}
			}

			// TODO: Tangent2?
			// TODO: Tangent1... Calculate tangents...?

			int sharedVertToUse = -1;
			for (int svi = 0; svi < sharedVerts.size(); svi++) {
				Vertex* other = &sharedVerts[svi];
				// TODO: Check to see if vertex is already there
				if (other->Position[0] == sharedVerts[svi].Position[0] &&
					other->Position[1] == sharedVerts[svi].Position[1] &&
					other->Position[2] == sharedVerts[svi].Position[2] &&
					other->Position[3] == sharedVerts[svi].Position[3]) {
					sharedVertToUse = svi;
					break;
				}
			}

			if (sharedVertToUse == -1) {
				sharedVerts.push_back(myVert);
				sharedVertToUse = sharedVerts.size() - 1;
			}
			triIndices[indexId] = sharedVertToUse + oldSize;
		}

		vertices.resize(oldSize + sharedVerts.size());
		controlPointToVertexMapping.insert({ cpi, std::vector<int>() });
		for (int svi = 0; svi < sharedVerts.size(); svi++) {
			vertices[oldSize + svi] = sharedVerts[svi];
			controlPointToVertexMapping[cpi].push_back(oldSize + svi);
		}
	}

	for (auto sName = shapeVertices.begin(); sName != shapeVertices.end(); sName++) {
		std::map<int, Vertex> newMapping;
		for (auto newIt = sName->second.begin(); newIt != sName->second.end(); newIt++) {
			auto cpi = newIt->first;
			auto arr = controlPointToVertexMapping[cpi];
			for (int i = 0; i < arr.size(); i++) {
				newMapping.insert({ arr[i], newIt->second });
			}
		}
		sName->second = newMapping;
	}

	Submesh child = Submesh();

	child.IndexOffset = parent->Indices.size();
	child.IndexNum = triIndices.size();

	parent->Vertices.insert(parent->Vertices.end(), vertices.begin(), vertices.end());
	parent->Indices.insert(parent->Indices.end(), triIndices.begin(), triIndices.end());
	parent->Submeshes.push_back(child);

	for (auto sName = shapeVertices.begin(); sName != shapeVertices.end(); sName++) {
		uint16_t startIndex[3] = { parent->Vertices.size(), 0, 0 };
		// TODO: What is meshCount?
		uint16_t meshCount[3] = { 0,0,0 };
		Shape s(sName->first, startIndex, meshCount);
		auto& mapping = sName->second;
		for (auto it = mapping.begin(); it != mapping.end(); it++) {
			parent->Vertices.push_back(it->second);
		}

	}

	// TODO: Add shapes
	// TODO: Add bones
}

FbxSkin* FbxToMdlConverter::GetSkin(FbxMesh* mesh) {
	int count = mesh->GetDeformerCount();
	for (int i = 0; i < count; i++) {
		FbxDeformer* d = mesh->GetDeformer(i);
		FbxDeformer::EDeformerType dType = d->GetDeformerType();
		if (dType == FbxDeformer::eSkin) {
			return (FbxSkin*)d;
		}
	}
	return NULL;
}

// Gets the first Skin element in a mesh.
FbxBlendShape* FbxToMdlConverter::GetMorpher(FbxMesh* mesh) {
	int count = mesh->GetDeformerCount();
	for (int i = 0; i < count; i++) {
		FbxDeformer* d = mesh->GetDeformer(i);
		FbxDeformer::EDeformerType dType = d->GetDeformerType();
		if (dType == FbxDeformer::eBlendShape) {
			return (FbxBlendShape*)d;
		}
	}
	return NULL;
}

int FbxToMdlConverter::GetDirectIndex(FbxMesh* mesh, FbxLayerElementTemplate<FbxVector4>* layerElement, int index_id) {

	if (layerElement == NULL) {
		return -1;
	}

	FbxLayerElement::EMappingMode mapMode = layerElement->GetMappingMode();
	FbxLayerElement::EReferenceMode refMode = layerElement->GetReferenceMode();
	// Pick which index we're using.
	int index = 0;
	if (mapMode == FbxLayerElement::eByControlPoint) {
		index = mesh->GetPolygonVertex(index_id / 3, index_id % 3);
	}
	else if (mapMode == FbxLayerElement::eByPolygonVertex) {
		index = index_id;
	}
	else {
		return -1;
	}


	FbxVector4 Normal;
	// Run it through the appropriate direct/indirect
	if (refMode == FbxLayerElement::eDirect) {
		index = index;
	}
	else if (refMode == FbxLayerElement::eIndexToDirect) {
		index = layerElement->GetIndexArray().GetAt(index);
	}
	else {
		return -1;
	}
	return index;
}

int FbxToMdlConverter::GetDirectIndex(FbxMesh* mesh, FbxLayerElementTemplate<FbxVector2>* layerElement, int index_id) {

	if (layerElement == NULL) {
		return -1;
	}

	FbxLayerElement::EMappingMode mapMode = layerElement->GetMappingMode();
	FbxLayerElement::EReferenceMode refMode = layerElement->GetReferenceMode();
	// Pick which index we're using.
	int index = 0;
	if (mapMode == FbxLayerElement::eByControlPoint) {
		index = mesh->GetPolygonVertex(index_id / 3, index_id % 3);
	}
	else if (mapMode == FbxLayerElement::eByPolygonVertex) {
		index = index_id;
	}
	else {
		return -1;
	}


	FbxVector4 Normal;
	// Run it through the appropriate direct/indirect
	if (refMode == FbxLayerElement::eDirect) {
		index = index;
	}
	else if (refMode == FbxLayerElement::eIndexToDirect) {
		index = layerElement->GetIndexArray().GetAt(index);
	}
	else {
		return -1;
	}
	return index;
}

int FbxToMdlConverter::GetDirectIndex(FbxMesh* mesh, FbxLayerElementTemplate<FbxColor>* layerElement, int index_id) {

	if (layerElement == NULL) {
		return -1;
	}

	FbxLayerElement::EMappingMode mapMode = layerElement->GetMappingMode();
	FbxLayerElement::EReferenceMode refMode = layerElement->GetReferenceMode();
	// Pick which index we're using.
	int index = 0;
	if (mapMode == FbxLayerElement::eByControlPoint) {
		index = mesh->GetPolygonVertex(index_id / 3, index_id % 3);
	}
	else if (mapMode == FbxLayerElement::eByPolygonVertex) {
		index = index_id;
	}
	else {
		return -1;
	}


	FbxVector4 Normal;
	// Run it through the appropriate direct/indirect
	if (refMode == FbxLayerElement::eDirect) {
		index = index;
	}
	else if (refMode == FbxLayerElement::eIndexToDirect) {
		index = layerElement->GetIndexArray().GetAt(index);
	}
	else {
		return -1;
	}
	return index;
}

// Get the raw position value for a triangle index.
FbxVector4 FbxToMdlConverter::GetPosition(FbxMesh* const mesh, int index_id) {
	FbxVector4 def = FbxVector4(0, 0, 0, 0);
	if (mesh->GetLayerCount() < 1) {
		return def;
	}
	FbxLayer* layer = mesh->GetLayer(0);
	int vertex_id = mesh->GetPolygonVertex(index_id / 3, index_id % 3);
	FbxVector4 position = mesh->GetControlPointAt(vertex_id);
	return position;
}

// Get the raw normal value for a triangle index.
FbxVector4 FbxToMdlConverter::GetNormal(FbxMesh* const mesh, int index_id) {
	FbxVector4 def = FbxVector4(0, 0, 0, 1.0);
	if (mesh->GetLayerCount() < 1) {
		return def;
	}
	FbxLayer* layer = mesh->GetLayer(0);
	FbxLayerElementNormal* layerElement = layer->GetNormals();
	int index = GetDirectIndex(mesh, layerElement, index_id);
	return index == -1 ? def : layerElement->GetDirectArray().GetAt(index);
}

// Get the raw uv1 value for a triangle index.
FbxVector2 FbxToMdlConverter::GetUV1(FbxMesh* const mesh, int index_id) {
	FbxVector2 def = FbxVector2(0, 0);
	if (mesh->GetLayerCount() < 1) {
		return def;
	}
	FbxLayer* layer = mesh->GetLayer(0);
	FbxLayerElementUV* uvs = layer->GetUVs();
	int index = GetDirectIndex(mesh, uvs, index_id);
	return index == -1 ? def : uvs->GetDirectArray().GetAt(index);
}

// Get the raw uv2 value for a triangle index.
FbxVector2 FbxToMdlConverter::GetUV2(FbxMesh* const mesh, int index_id) {
	FbxVector2 def = FbxVector2(0, 0);
	if (mesh->GetLayerCount() < 2) {
		return def;
	}
	FbxLayer* layer = mesh->GetLayer(1);
	FbxLayerElementUV* uvs = layer->GetUVs();
	int index = GetDirectIndex(mesh, uvs, index_id);
	return index == -1 ? def : uvs->GetDirectArray().GetAt(index);
}

// Gets the raw vertex color value for a triangle index.
FbxColor FbxToMdlConverter::GetVertexColor(FbxMesh* const mesh, int index_id) {
	FbxColor def = FbxColor(1, 1, 1, 1);
	if (mesh->GetLayerCount() < 1) {
		return def;
	}
	FbxLayer* layer = mesh->GetLayer(0);
	FbxLayerElementVertexColor* layerElement = layer->GetVertexColors();
	int index = GetDirectIndex(mesh, layerElement, index_id);
	return index == -1 ? def : layerElement->GetDirectArray().GetAt(index);
}