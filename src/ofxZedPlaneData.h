#pragma once

#include <sl/Camera.hpp>

#include "ofMain.h"
#include "ofxCv.h"

class ofxZedPlaneData
{
public:

	void update( sl::uint2 & coord, sl::Plane &plane );

	ofMesh & getMesh() { return detPlaneMesh; }

private:

	ofMesh detPlaneMesh;
};
