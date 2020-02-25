#pragma once

#include <sl/Camera.hpp>

#include "ofMain.h"
#include "ofxCv.h"

class ofxZedSpatialData
{
public:
	void	 update( sl::Mesh slMesh );

	ofMesh & getMesh() { return spatialMesh; }

private:

	ofMesh spatialMesh;
};