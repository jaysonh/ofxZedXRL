#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include <sl/Camera.hpp>

using namespace sl;

class ofxZedXRL
{
public:

	struct CameraOptions
	{
		bool usePosTrack = false;
		bool usePlaneDet = false;
		bool useSpatMap  = false;
	};

	ofxZedXRL();

	void setup( bool usePosTrack = false, 
				bool usePlaneDet = false, 
				bool useSpatMap  = false, 
				UNIT unitMesure  = UNIT::MILLIMETER );

	void update();

	void drawLeft(  int x, int y, int w = 1280, int h = 720 );
	void drawRight( int x, int y, int w = 1280, int h = 720  );
	void drawDepth( int x, int y, int w = 1280, int h = 720  );
	void drawDetPlane();

	float getDepthAt(int x, int y);

	ofMesh & getMesh(    int meshStep = 4 );
	ofMesh & getMeshRGB( int meshStep = 4 );
	ofMesh & getSpatialMesh();
	ofMesh & getDetPlane();

	ofPixels getImageColor();
	ofPixels getImageCalibRGB(float depthThresh = 0.0);

	glm::vec3 getTranslatedPos(int x, int y);
	
	glm::vec3 & getCameraRot();
	glm::vec3 & getCameraPos();
	glm::vec4 & getCameraOri();

	cv::Mat getRGBDMat();
	cv::Mat getDepthMat();

	int getMeshHeight();
	int getMeshWidth();

	glm::vec3 getWorldCoordAt(glm::vec2 pos);

private:
	ofColor getRGBFromFloat(float v);
	void addPointToMesh(sl::float3 v);
	cv::Mat slMat2cvMat(sl::Mat& input, sl::MEM memType = sl::MEM::CPU); // converts between sl mat and opencv mat types

	glm::vec3 cameraPos; 
	glm::vec3 cameraRot;
	glm::vec4 cameraOri;

	glm::vec3 trackedPos;
	glm::vec4 trackedOri;

	bool usePosTrack = false;
	bool usePlaneDet = false;
	bool useSpatMap  = false;

	sl::Mesh spatialMeshsl;
	float ts_last = 0.;
	ofMesh pointMesh;
	ofMesh   detPlaneMesh;
	ofMesh   spatialMesh;
	vector<ofVec3f> detPlaneBounds;

	
	sl::Camera zed;
	int zedWidth, zedHeight;

	sl::Mat matDepthMesRGB;
	sl::Mat matDepthMes;
	cv::Mat cvMatLeft;
	cv::Mat cvMatRight;
	cv::Mat cvMatDepth;
	cv::Mat cvMatCloud;
	cv::Mat cvMatDepthMes;
	cv::Mat cvPointCloud;

	vector<ofVec3f> pointCloudVerts;
	

};