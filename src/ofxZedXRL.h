#pragma once

#include <sl/Camera.hpp>

#include "ofMain.h"
#include "ofxCv.h"

#include "ofxZedPosData.h"
#include "ofxZedPlaneData.h"
#include "ofxZedSpatialData.h"

namespace ofxZedXRL
{
	using namespace sl;

	class ofxZed
	{
	public:

		struct CameraOptions
		{
			bool usePosTrack = false;
			bool usePlaneDet = false;
			bool useSpatMap  = false;
			UNIT unitMeasure = UNIT::MILLIMETER;
			int width        = 1280;
			int height       = 720;
		};

		ofxZed();
		~ofxZed();

		void setup( CameraOptions options );

		void update();

		void drawLeft(  int x, int y, int w = 1280, int h = 720 );
		void drawRight( int x, int y, int w = 1280, int h = 720 );
		void drawDepth( int x, int y, int w = 1280, int h = 720 );

		float     getDepthAt( int x, int y );
		ofMesh &  getMesh( int meshStep = ofxZed::meshDispRes, bool saveRGB = false );
		ofPixels  getColPixs();
		ofPixels  getCalibPixs( float depthThresh = 0.0 );
		cv::Mat   getRGBDMat();
		cv::Mat   getDepthMat();
		glm::vec3 getWorldFromMat(  int x, int y );
		glm::vec3 getWorldFromMesh( int x, int y );

		// Get the result data from the zed camera
		ofxZedPosData	  & getPosData()     { return position;   }
		ofxZedSpatialData & getSpatialData() { return spatDet;    }
		ofxZedPlaneData   & getPlaneData()   { return planeDet;   }
		CameraOptions	  & getCamOptions()  { return camOptions; }

		// Mesh weight params
		static const int meshFullRes = 1;
		static const int meshDispRes = 4;

	private:

		const int   matChannels = 4;	
		const float spatMapInt  = 0.5; // time between taking a spatial map

		ofColor getRGBValue( float v );
		cv::Mat slMat2cvMat( sl::Mat& input, 
							 sl::MEM memType = sl::MEM::CPU ); // converts between sl mat and opencv mat types

		sl::Camera		  camera;
		CameraOptions     camOptions;
		ofxZedPosData	  position;
		ofxZedSpatialData spatDet;
		ofxZedPlaneData   planeDet;

		float spatMapStart = 0.0;

		// Keep these available for possible use later
		ofMesh pointMesh;

		sl::Mat matDepthMesRGB;
		sl::Mat matDepthMes;

		cv::Mat cvMatLeft;
		cv::Mat cvMatRight;
		cv::Mat cvMatDepth;
		cv::Mat cvMatCloud;
		cv::Mat cvMatDepthMes;
		cv::Mat cvPointCloud;
	};
}