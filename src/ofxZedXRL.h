#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include <sl/Camera.hpp>

using namespace sl;
class ofxZedXRL
{
public:

	struct DetPlane
	{
		DetPlane()
		{

		}

		DetPlane(sl::float3 _normal, sl::float4 _planeEq)
		{
			normal  = _normal;
			planeEq = _planeEq;

		}

		sl::float3 normal;
		sl::float4 planeEq;
	};

	ofxZedXRL();

	void setup( bool _usePosTrack = false, bool _usePlaneDet = false, bool _useSpatialMap =false);
	void update();

	void drawLeft(int x, int y,  int w = 1280, int h = 720);
	void drawRight(int x, int y, int w = 1280, int h = 720);
	void drawDepth(int x, int y, int w = 1280, int h = 720);
	void drawDetPlane();

	float getDepthAt(int x, int y);

	ofMesh * getPointMesh();
	ofMesh * getPointMeshRGB();
	ofMesh * getSpatialMesh();
	ofMesh * getDetPlane();

private:
	const float zScale = 100.0;
	bool usePosTrack   = false;
	bool usePlaneDet   = false;
	bool useSpatialMap = false;
	ofMesh pointMesh;
	DetPlane detPlane;
	ofMesh   detPlaneMesh;
	ofMesh   spatialMesh;
	vector<ofVec3f> detPlaneBounds;

	cv::Mat slMat2cvMat(sl::Mat& input, sl::MEM memType = sl::MEM::CPU); // converts between sl mat and opencv mat types

	sl::Camera zed;
	int zedWidth, zedHeight;

	cv::Mat cvMatLeft;
	cv::Mat cvMatRight;
	cv::Mat cvMatDepth;
	cv::Mat cvMatCloud;
	cv::Mat cvMatDepthMes;
	cv::Mat cvPointCloud;


	vector<ofVec3f> pointCloudVerts;
	
	ofVec3f trackedPos;
	ofVec4f trackedOri;

};