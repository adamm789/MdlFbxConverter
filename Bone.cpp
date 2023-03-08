#include "Bone.h"

Bone* Bone::GetBone(std::string name) {
	if (this->Name == name) {
		return this;
	}
	else {
		for (int i = 0; i < Children.size(); i++) {
			Bone* b = Children[i]->GetBone(name);
			if (b != NULL) {
				return b;
			}
		}
		return NULL;
	}
}