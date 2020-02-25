#pragma once

#include <sl/Camera.hpp>

#include "ofMain.h"
#include "ofxCv.h"

class ofxZedPosData
{
public:

	void update( sl::Pose & zedPose );

	glm::vec3 getPos() { return cameraPos; }
	glm::vec4 getOri() { return cameraOri; }
	glm::vec3 getRot() { return cameraRot; }

private:

	glm::vec3 cameraPos;
	glm::vec4 cameraOri;
	glm::vec3 cameraRot;
};