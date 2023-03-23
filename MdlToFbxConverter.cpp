#include "MdlToFbxConverter.h"
#include "../eigen-3.4.0/Eigen/Dense"

// Pretty much entirely from https://github.com/TexTools/TT_FBX_Reader/blob/master/TT_FBX/src/db_converter.cpp
MdlToFbxConverter::MdlToFbxConverter(std::string mdlFilePath, std::string outputPath) {
	fprintf(stdout, "Converting %s to %s\n", mdlFilePath.c_str(), outputPath.c_str());
	this->outputPath = outputPath;

	mdlFile = new MdlFile();
	mdlFile->LoadFromFile(mdlFilePath);

	//mdlFile = MdlFile::LoadFromData();
	//mdlFile = MdlFile::LoadFromFile2(mdlFilePath);
	
	model = new Model(mdlFile);
	manager = FbxManager::Create();

	FbxIOSettings* ios = FbxIOSettings::Create(manager, IOSROOT);
	manager->SetIOSettings(ios);

	CreateScene(model);
	ExportScene();

	scene->Destroy();
	manager->Destroy();
}

MdlToFbxConverter::~MdlToFbxConverter() {
	delete mdlFile;
	delete model;

	std::map<Bone*, FbxNode*>::iterator it;
	for (it = BoneToNode.begin(); it != BoneToNode.end(); it++) {
		delete it->first;
	}
}

void MdlToFbxConverter::SetSkeletonFromFile(std::string filePath)
{
}

void MdlToFbxConverter::SetSkeletonFromData(const char* data)
{
}

void MdlToFbxConverter::CreateScene(Model* model) {
	scene = FbxScene::Create(manager, "fbx export");
	FbxNode* root = scene->GetRootNode();

	// Initialize axis
	auto up = FbxAxisSystem::EUpVector::eYAxis;
	auto front = FbxAxisSystem::EFrontVector::eParityOdd;
	auto handedness = FbxAxisSystem::eRightHanded;

	FbxAxisSystem dbAxis(up, front, handedness);
	dbAxis.ConvertScene(scene);

	scene->GetGlobalSettings().SetSystemUnit(FbxSystemUnit::m);

	FbxNode* firstNode = FbxNode::Create(manager, "root name");
	FbxDouble3 rootScale = firstNode->LclScaling.Get();
	root->AddChild(firstNode);

	// TODO: Build a skeleton from something other than a file that came from TexTools
	n_root = Skeleton::BuildSkeletonFromFile("..\\Skeletons\\c0101b0001.skel");

	FbxPose* pose = FbxPose::Create(manager, "Bindpose");
	pose->SetIsBindPose(true);
	AddBoneToScene(n_root, pose, firstNode);
	scene->AddPose(pose);

	CreateMaterials();

	std::vector<FbxNode*> meshGroupNodes;
	for (int i = 0; i < model->Meshes.size(); i++) {
		FbxNode* node = FbxNode::Create(manager, ("Group" + std::to_string(i)).c_str());
		meshGroupNodes.push_back(node);
		firstNode->AddChild(node);

		std::vector<Shape> shapes;

		pose->Add(node, node->EvaluateGlobalTransform());
		for (int p = 0; p < model->Meshes[i].Submeshes.size(); p++) {
			AddPartToScene(&model->Meshes[i], &model->Meshes[i].Submeshes[p], node, model->Meshes[i].Submeshes[0].IndexOffset, p);
		}
	}
	pose->Add(firstNode, firstNode->EvaluateGlobalTransform());
}

FbxDouble3 MatrixToScale(Eigen::Transform<double, 3, Eigen::Affine> affineMatrix) {

	Eigen::Matrix4d m = affineMatrix.matrix();

	FbxVector4 row1 = FbxVector4(m(0, 0), m(1, 0), m(2, 0));
	FbxVector4 row2 = FbxVector4(m(0, 1), m(1, 1), m(2, 1));
	FbxVector4 row3 = FbxVector4(m(0, 2), m(1, 2), m(2, 2));
	FbxDouble3 ret;
	ret[0] = row1.Length();
	ret[1] = row2.Length();
	ret[2] = row3.Length();
	return ret;
}

void MdlToFbxConverter::AddBoneToScene(Bone* bone, FbxPose* bindPose, FbxNode* parentNode) {
	// TODO: Bones seem to be in position, but all facing the wrong directions (seems to be "outwards")
	if (bone == NULL) return;

	FbxNode* node = FbxNode::Create(manager, bone->Name.c_str());
	FbxSkeleton* skeletonAttribute = FbxSkeleton::Create(scene, "Skeleton");

	BoneToNode.emplace(bone, node);

	if (bone->Parent == NULL) {
		skeletonAttribute->SetSkeletonType(FbxSkeleton::eRoot);
	}
	else {
		skeletonAttribute->SetSkeletonType(FbxSkeleton::eLimbNode);
	}

	skeletonAttribute->LimbLength.Set(1.0);
	skeletonAttribute->Size.Set(1.0);
	node->SetNodeAttribute(skeletonAttribute);

	Eigen::Block<Eigen::Matrix4d, 3, 1, true> t = bone->PoseMatrix.translation();
	FbxDouble3 translation = FbxDouble3(t.x(), t.y(), t.z());

	// according to https://github.com/TexTools/TT_FBX_Reader/blob/master/TT_FBX/src/db_converter.cpp this just works?
	Eigen::Vector3d rot = bone->PoseMatrix.rotation().eulerAngles(2, 1, 0);
	FbxDouble3 degRot = FbxDouble3(FBXSDK_RAD_TO_DEG * rot[2], FBXSDK_RAD_TO_DEG * rot[1], FBXSDK_RAD_TO_DEG * rot[0]);

	FbxDouble3 scale = MatrixToScale(bone->PoseMatrix);

	node->LclTranslation.Set(translation);
	node->LclRotation.Set(degRot);
	node->LclScaling.Set(scale);

	parentNode->AddChild(node);
	bindPose->Add(node, node->EvaluateGlobalTransform());

	for (int i = 0; i < bone->Children.size(); i++) {
		AddBoneToScene(bone->Children[i], bindPose, node);
	}
}

// TODO: Allow providing a material to assign to the model (MtrlFile?)
void MdlToFbxConverter::CreateMaterials() {
	for (int i = 0; i < model->Materials.size(); i++) {
		Material mat = model->Materials[i];
		FbxString lMaterialName = mat.MaterialPath.c_str();
		FbxDouble3 white = FbxDouble3(1.0f, 1.0f, 1.0f);
		FbxDouble3 black = FbxDouble3(0.f, 0.f, 0.f);
		FbxSurfacePhong* lMaterial = FbxSurfacePhong::Create(scene, lMaterialName.Buffer());

		lMaterial->Emissive.Set(black);
		lMaterial->Diffuse.Set(white);
		lMaterial->TransparencyFactor.Set(0.0);
		lMaterial->ShadingModel.Set("Phong");
		lMaterial->Shininess.Set(0.5);

		lMaterial->TransparentColor.Set(FbxDouble3(1.0f, 1.0f, 1.0f));
		lMaterial->TransparencyFactor.Set(0.0f);

		// TODO: Allow way to assign textures some other way?
		for (int j = 0; j < mat.Textures.size(); j++) {
			Texture tex = mat.Textures[j];
			FbxFileTexture* texture = NULL;
			if (tex.TextureUsageSimple == Texture::Usage::Diffuse) {
				texture = FbxFileTexture::Create(scene, std::string(mat.MaterialPath + " Diffuse").c_str());
			}
			else if (tex.TextureUsageSimple == Texture::Usage::Specular) {
				texture = FbxFileTexture::Create(scene, std::string(mat.MaterialPath + " Specular").c_str());
			}
			else if (tex.TextureUsageSimple == Texture::Usage::Normal) {
				texture = FbxFileTexture::Create(scene, std::string(mat.MaterialPath + " Normal").c_str());
			}
			else {
				fprintf(stderr, "Could not create FbxFileTexture from usage: %i\n", tex.TextureUsageSimple);
			}
			// TODO: Not sure about these two
			// Emissive
			// Opacity

			if (texture != NULL) {
				texture->SetFileName(tex.TexturePath.c_str());
				texture->SetTextureUse(FbxTexture::eStandard);
				texture->SetMappingType(FbxTexture::eUV);
				texture->SetMaterialUse(FbxFileTexture::eModelMaterial);
				texture->Alpha.Set(1.0);

				if (tex.TextureUsageSimple == Texture::Usage::Diffuse) {
					lMaterial->Diffuse.ConnectSrcObject(texture);
				}
				if (tex.TextureUsageSimple == Texture::Usage::Specular) {
					lMaterial->Specular.ConnectSrcObject(texture);
				}
				if (tex.TextureUsageSimple == Texture::Usage::Normal) {
					lMaterial->TransparentColor.ConnectSrcObject(texture);
					lMaterial->TransparencyFactor.ConnectSrcObject(texture);
				}
			}
		}

		MaterialPathToSurfaceMaterial.emplace(mat.MaterialPath, lMaterial);
	}
}

bool CompareShape(const Shape& lhs, const Shape& rhs) {
	return lhs.ShapeValuesStartIndex > rhs.ShapeValuesStartIndex;
}

void MdlToFbxConverter::AddPartToScene(Mesh* group, Submesh* part, FbxNode* parent, int indicesOffset, int partNumber) {
	std::string modelName = "model name";
	std::string partName = std::string(modelName + " Part " + std::to_string(group->MeshIndex) + "." + std::to_string(partNumber));

	FbxNode* node = FbxNode::Create(manager, partName.c_str());
	parent->AddChild(node);

	FbxSurfaceMaterial* lMaterial = NULL;

	std::map<std::string, FbxSurfaceMaterial*>::iterator it = MaterialPathToSurfaceMaterial.find(group->Material->MaterialPath);
	if (it != MaterialPathToSurfaceMaterial.end()) {
		lMaterial = it->second;
	}
	else {
		lMaterial = FbxSurfacePhong::Create(scene, "material name");
		fprintf(stderr, "Could not find material: %s\n", group->Material->MaterialPath.c_str());
	}

	std::vector<Vertex> uniquePartVertices;
	std::vector<uint16_t> uniquePartVerticesIndices;	// This is so I just compare the vertex indices
	std::vector<uint16_t> indicesToUniqueVertices;
	std::map<uint16_t, uint16_t> oldIndicesToNewIndices;

	// Get a list of unique vertices that belong to this part
	for (uint32_t i = 0; i < part->IndexNum; i++) {
		uint16_t currIndex = part->IndexOffset - indicesOffset + i;
		uint16_t vertexNum = group->Indices[currIndex];

		auto it = std::find(uniquePartVerticesIndices.begin(), uniquePartVerticesIndices.end(), vertexNum);
		int existingIndex = it - uniquePartVerticesIndices.begin();

		if (it != uniquePartVerticesIndices.end()) {
			indicesToUniqueVertices.push_back(existingIndex);
			oldIndicesToNewIndices.emplace(currIndex, existingIndex);
		}
		else {
			uint16_t size = uniquePartVerticesIndices.size();
			indicesToUniqueVertices.push_back(size);
			oldIndicesToNewIndices.emplace(currIndex, size);

			uniquePartVerticesIndices.push_back(vertexNum);
			uniquePartVertices.push_back(group->Vertices[vertexNum]);
		}
	}
	FbxMesh* mesh = MakeMesh(uniquePartVertices, indicesToUniqueVertices, std::string(partName + " Mesh Attribute"), node, lMaterial);

	if (part->Shapes.size() > 0) {
		auto blendShape = FbxBlendShape::Create(scene, std::string(partName + " Blend Shapes").c_str());
		mesh->AddDeformer(blendShape);

		// Sort shapes by ShapeValueStartIndex descending
		std::sort(part->Shapes.begin(), part->Shapes.end(), CompareShape);
		int prevValue = INT_MAX;
		for (int i = 0; i < part->Shapes.size(); i++) {
			Shape s = part->Shapes[i];
			auto channel = FbxBlendShapeChannel::Create(blendShape, std::string("channel_" + s.ShapeName).c_str());

			std::vector<uint16_t> shapeIndices = std::vector<uint16_t>();
			std::vector<Vertex> uniqueShapeVertices = std::vector<Vertex>();

			for (int j = 0; j < uniquePartVertices.size(); j++) {
				uniqueShapeVertices.push_back(uniquePartVertices[j]);
			}

			for (int j = 0; j < part->IndexNum; j++) {
				int currIndex = part->IndexOffset - indicesOffset + j;
				int newIndex = oldIndicesToNewIndices.find(currIndex)->second;

				for (int k = 0; k < s.ShapeValueStructs.size(); k++) {
					if (s.ShapeValueStructs[k].Offset == currIndex && currIndex >= s.ShapeValuesStartIndex && currIndex < prevValue) {
						Vertex newVertex = group->Vertices[s.ShapeValueStructs[k].Value];
						uniqueShapeVertices[newIndex] = newVertex;
					}
				}
			}
			// We don't want later processed shapes to include vertices from already processed shapes
			prevValue = s.ShapeValuesStartIndex;

			if (uniqueShapeVertices.size() != uniquePartVertices.size()) {
				fprintf(stderr, "Number of shape vertices (%i) does not match the number of unique part vertices (%i)\n", (int)uniqueShapeVertices.size(), (int)uniquePartVertices.size());
				fprintf(stderr, "Could not add shape: %s\n", s.ShapeName.c_str());
			}
			else {
				FbxShape* shapeMesh = MakeShape(uniqueShapeVertices, s.ShapeName);
				channel->SetMultiLayer(false);
				channel->AddTargetShape(shapeMesh);
			}
		}
	}

	FbxSkin* skin = FbxSkin::Create(scene, std::string(partName + "Skin Attribute").c_str());
	auto typeName = skin->GetTypeName();
	auto defType = skin->GetDeformerType();
	skin->SetSkinningType(FbxSkin::eLinear);
	mesh->AddDeformer(skin);

	// Set weights
	std::map<int, std::string>::iterator it2;
	int boneNameIndex = 0;
	for (it2 = group->Parent->StringOffsetToStringMap.begin(); it2 != group->Parent->StringOffsetToStringMap.end(); it2++) {
		std::string boneName = it2->second;
		Bone* b = n_root->GetBone(boneName);
		if (b == NULL) {
			continue;
		}
		FbxCluster* cluster = FbxCluster::Create(scene, std::string(partName + " " + boneName + " Cluster").c_str());

		cluster->SetLink(BoneToNode[b]);
		cluster->SetLinkMode(FbxCluster::ELinkMode::eNormalize);

		cluster->SetTransformMatrix(node->EvaluateGlobalTransform());
		cluster->SetTransformLinkMatrix(BoneToNode[b]->EvaluateGlobalTransform());

		for (int vi = 0; vi < uniquePartVertices.size(); vi++) {
			Vertex v = uniquePartVertices[vi];

			for (int wi = 0; wi < 4; wi++) {
				unsigned char set = group->BoneTable[v.BlendIndices[wi]];

				if (set == boneNameIndex && v.BlendWeights[wi] > 0) {
					cluster->AddControlPointIndex(vi, v.BlendWeights[wi]);
				}
			}
		}
		if (cluster->GetControlPointIndicesCount() > 0) {
			skin->AddCluster(cluster);
		}
		else {
			cluster->Destroy();
		}

		boneNameIndex++;
	}

	FbxPose* pose = scene->GetPose(0);
	FbxMatrix bind = node->EvaluateGlobalTransform();
	pose->Add(node, bind);
}

FbxMesh* MdlToFbxConverter::MakeMesh(std::vector<Vertex>& vertices, std::vector<unsigned short>& indices, std::string meshName, FbxNode* parent, FbxSurfaceMaterial* material) {
	FbxMesh* mesh = FbxMesh::Create(manager, meshName.c_str());
	parent->SetShadingMode(FbxNode::eTextureShading);

	parent->AddMaterial(material);
	parent->SetNodeAttribute(mesh);
	FbxGeometryElementMaterial* lMaterialElement = mesh->CreateElementMaterial();
	lMaterialElement->SetMappingMode(FbxGeometryElement::eAllSame);

	mesh->InitControlPoints(vertices.size());
	mesh->InitNormals(vertices.size());

	FbxGeometryElementVertexColor* colorElement = mesh->CreateElementVertexColor();
	colorElement->SetMappingMode(FbxLayerElement::EMappingMode::eByPolygonVertex);
	colorElement->SetReferenceMode(FbxLayerElement::EReferenceMode::eIndexToDirect);

	FbxGeometryElementUV* uvElement = mesh->CreateElementUV("uv1");
	uvElement->SetMappingMode(FbxLayerElement::EMappingMode::eByControlPoint);

	// Create a new layer and stick the UV2 Element on it
	auto newLayerId = mesh->CreateLayer();
	auto* layer2 = mesh->GetLayer(newLayerId);
	auto* uv2Layer = FbxLayerElementUV::Create(mesh, "uv2");
	uv2Layer->SetMappingMode(FbxLayerElement::EMappingMode::eByControlPoint);
	layer2->SetUVs(uv2Layer);

	auto worldTransform = parent->EvaluateGlobalTransform();
	auto normalMatrix = parent->EvaluateLocalTransform().Inverse().Transpose();

	for (int i = 0; i < vertices.size(); i++) {
		Vertex v = vertices[i];
		FbxVector4 pos = FbxVector4(v.Position[0], v.Position[1], v.Position[2], v.Position[3]);
		FbxVector4 normal = FbxVector4(v.Normal[0], v.Normal[1], v.Normal[2]);

		mesh->SetControlPointAt(pos, normal, i);

		// ffxiv uvs are in [1, -1] and inverted vertically
		uvElement->GetDirectArray().Add(FbxVector2(v.UV[0], 1 -  v.UV[1]));
		uv2Layer->GetDirectArray().Add(FbxVector2(v.UV[2], 1 -  v.UV[3]));
	}

	for (int i = 0; i < indices.size(); i += 3) {
		mesh->BeginPolygon();
		for (int j = 0; j < 3; j++) {
			mesh->AddPolygon(indices[i + j]);
			Vertex vert = vertices[indices[i + j]];
			colorElement->GetDirectArray().Add(FbxColor(vert.Color[0], vert.Color[1], vert.Color[2], vert.Color[3]));
			colorElement->GetIndexArray().Add(i + j);
		}
		mesh->EndPolygon();
	}

	return mesh;
}


FbxShape* MdlToFbxConverter::MakeShape(std::vector<Vertex>& vertices, std::string meshName) {
	FbxShape* shapeMesh = FbxShape::Create(manager, meshName.c_str());

	shapeMesh->InitControlPoints(vertices.size());
	shapeMesh->InitNormals(vertices.size());
	FbxVector4* controlPoints = shapeMesh->GetControlPoints();

	for (int i = 0; i < vertices.size(); i++) {
		Vertex v = vertices[i];
		FbxVector4 pos = FbxVector4(v.Position[0], v.Position[1], v.Position[2], v.Position[3]);
		FbxVector4 normal = FbxVector4(v.Normal[0], v.Normal[1], v.Normal[2]);

		shapeMesh->SetControlPointAt(pos, normal, i);
	}

	return shapeMesh;
}

int MdlToFbxConverter::ExportScene() {
	auto ios = manager->GetIOSettings();
	ios->SetBoolProp(EXP_FBX_MATERIAL, true);
	ios->SetBoolProp(EXP_FBX_TEXTURE, true);
	ios->SetBoolProp(EXP_FBX_EMBEDDED, true);
	ios->SetBoolProp(EXP_FBX_SHAPE, true);
	ios->SetBoolProp(EXP_FBX_GOBO, true);
	ios->SetBoolProp(EXP_FBX_ANIMATION, true);
	ios->SetBoolProp(EXP_FBX_GLOBAL_SETTINGS, true);

	FbxExporter* exporter = FbxExporter::Create(manager, "");

	char* lFileName = const_cast<char*>(outputPath.c_str());
	manager->SetIOSettings(ios);
	bool exportStatus = exporter->Initialize(lFileName, -1, ios);
	if (!exportStatus) {
		std::cout << "Call to FbxExporter::Initialize failed" << std::endl;
		return -1;
	}
	exporter->Export(scene);
	exporter->Destroy();

	return 0;
}