#include "ofxZedXRL.h"

namespace ofxZedXRL
{
	// default constructor
	ofxZed::ofxZed()
	{

	}

	// default deconstructor
	ofxZed::~ofxZed()
	{
		// shutdown the zed nicely
		camera.close();
	}

	// setup the camera
	void ofxZed::setup( CameraOptions camOptions )
	{
		this->camOptions = camOptions;

		InitParameters init_params;
		init_params.camera_resolution = RESOLUTION::HD720;
		init_params.depth_mode		  = DEPTH_MODE::ULTRA;		// PERFORMANCE QUALITY 
		init_params.coordinate_units  = camOptions.unitMeasure;	// UNIT::MILLIMETER;// UNIT::METER;
		init_params.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;

		ERROR_CODE err = camera.open( init_params );

		if ( err != ERROR_CODE::SUCCESS ) 
		{
			ofLog(OF_LOG_ERROR, "Error " + ofToString(err));
		}
		else
		{
			auto camera_infos = camera.getCameraInformation();
			ofLog(OF_LOG_NOTICE, "ZED Opened: " + ofToString(camera_infos.serial_number));
		}

		// if using plane detection we need to enable the positional tracking
		camOptions.usePosTrack = ( camOptions.usePosTrack || camOptions.usePlaneDet || camOptions.useSpatMap );

		// setup spatial mapping
		if ( camOptions.useSpatMap )
		{
			sl::SpatialMappingParameters mapping_parameters;

			mapping_parameters.resolution_meter = 0.03;  // Set resolution to 3cm
			mapping_parameters.map_type			= sl::SpatialMappingParameters::SPATIAL_MAP_TYPE::MESH; // FUSED_POINT_CLOUD or MESH
			mapping_parameters.save_texture		= false; // save resources by not generating the texture

			camera.enableSpatialMapping( mapping_parameters );
		}
	}

	// update the camera
	void ofxZed::update()
	{
		RuntimeParameters runtime_parameters;
		runtime_parameters.sensing_mode = SENSING_MODE::FILL; // Use STANDARD sensing mode FILL/STANDARD/LAST
		runtime_parameters.measure3D_reference_frame = REFERENCE_FRAME::WORLD;

		if (camera.grab( runtime_parameters ) == ERROR_CODE::SUCCESS )
		{
			// grab images from camera
			sl::Mat matLeft;
			sl::Mat matRight;
			sl::Mat matDepth;

			camera.retrieveImage( matLeft,  VIEW::LEFT  );
			camera.retrieveImage( matRight, VIEW::RIGHT );
			camera.retrieveImage( matDepth, VIEW::DEPTH );

			camera.retrieveMeasure( matDepthMesRGB, MEASURE::XYZRGBA );
			camera.retrieveMeasure( matDepthMes,    
								    MEASURE::XYZ, 
								    MEM::CPU, 
								    sl::Resolution( camOptions.width, camOptions.height ) );

			cvMatLeft  = slMat2cvMat( matLeft  );
			cvMatRight = slMat2cvMat( matRight );
			cvMatDepth = slMat2cvMat( matDepth );

			// Need to shift color channels 
			cv::cvtColor( cvMatLeft,  cvMatLeft,  cv::COLOR_BGR2RGB  );  
			cv::cvtColor( cvMatRight, cvMatRight, cv::COLOR_BGR2RGB  );  

			// Save camera positional/rotation/orientation data
			if ( camOptions.usePosTrack )
			{
				sl::Pose zedPos;
				camera.getPosition( zedPos );

				position.update( zedPos );
			}

			if ( camOptions.useSpatMap )
			{
				// Only grab the spatial map from timer
				if ( ofGetElapsedTimef() - spatMapStart > spatMapInt)
				{
					camera.requestSpatialMapAsync();
					spatMapStart = ofGetElapsedTimef();
				}

				// Rebuild mesh with updated data
				if (camera.getSpatialMapRequestStatusAsync() == ERROR_CODE::SUCCESS )
				{
					sl::Mesh slMesh;
					camera.retrieveSpatialMapAsync( slMesh );
					spatDet.update( slMesh );
				}
			}

			// Update plane detection
			if ( camOptions.usePlaneDet )
			{
				Plane plane; // tracked plane
				Pose  pose;  // positional tracking data

				if ( camera.getPosition( pose ) == POSITIONAL_TRACKING_STATE::OK )
				{
					// This is the position from which we search for the plane
					sl::uint2 coord = sl::uint2( camOptions.width / 2, camOptions.width / 2 );
					
					if ( camera.findPlaneAtHit( coord, plane ) == ERROR_CODE::SUCCESS )
					{
						planeDet.update( coord, plane );
					}
				}
			}
		}
	}

	// Get point cloud mesh
	ofMesh & ofxZed::getMesh( int meshStep, bool saveRGB )
	{
		// make point cloud
		int w = matDepthMes.getWidth();
		int h = matDepthMes.getHeight();

		float *data = (float*)( matDepthMes.getPtr<sl::uchar1>( MEM::CPU ) );

		pointMesh.clear();

		pointMesh.setMode( OF_PRIMITIVE_POINTS );
		
		for ( int y = 0; y < h; y += meshStep )
		{
			for ( int x = 0; x < w; x += meshStep )
			{
				int indx = x * matChannels + y * w*matChannels;

				glm::vec3 p( data[indx + 0],
							 data[indx + 1],
							 data[indx + 2] );

				ofColor c = ofColor( 0, 125, ofMap( p.x, -10000, 10000, 0, 255 ) );

				if ( saveRGB ) // grab colour from 4th pixel
				{
					c = getRGBValue( data[indx + 3] );
				}

				pointMesh.addVertex( p );
				pointMesh.addColor ( c );
			}
		}

		return pointMesh;
	}
	
	// get depth at a specified pos
	float ofxZed::getDepthAt( int x, int y )
	{
		return cvMatDepthMes.at<float>( x, y );
	}

	// drag left camera rgb image
	void ofxZed::drawLeft( int x, int y, int w, int h )
	{
		ofxCv::drawMat( cvMatLeft, x, y, w, h );
	}

	// draw right camera rgb image
	void ofxZed::drawRight( int x, int y, int w, int h )
	{
		ofxCv::drawMat( cvMatRight, x, y, w, h );
	}

	// draw depth camera image
	void ofxZed::drawDepth( int x, int y, int w, int h )
	{
		ofxCv::drawMat( cvMatDepth, x, y, w, h );
	}

	// convert an sl::mat to cv::mat
	cv::Mat ofxZed::slMat2cvMat( Mat& input, sl::MEM memType )
	{
		// Mapping between MAT_TYPE and CV_TYPE
		int cv_type = -1;

		switch ( input.getDataType() )
		{
			case MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
			case MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
			case MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
			case MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
			case MAT_TYPE::U8_C1: cv_type = CV_8UC1;   break;
			case MAT_TYPE::U8_C2: cv_type = CV_8UC2;   break;
			case MAT_TYPE::U8_C3: cv_type = CV_8UC3;   break;
			case MAT_TYPE::U8_C4: cv_type = CV_8UC4;   break;
			default: break;
		}

		// Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
		// cv::Mat and sl::Mat will share a single memory structure
		return cv::Mat( input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(memType) );
	}
	
	ofPixels ofxZed::getColPixs()
	{
		ofImage img;

		ofxCv::toOf( cvMatLeft, img );

		return img.getPixels();
	}

	ofPixels ofxZed::getCalibPixs( float depthThresh )
	{
		ofPixels pixs;

		// make point cloud
		int w = matDepthMesRGB.getWidth();
		int h = matDepthMesRGB.getHeight();

		pixs.allocate(w, h, OF_PIXELS_RGB);

		float *data = ( float* )( matDepthMesRGB.getPtr<float>(MEM::CPU) );

		for (int y = 0; y < h; y += 1)
		{
			for (int x = 0; x < w; x += 1)
			{
				int indx = x * matChannels + y * w * matChannels;

				float depth = data[indx + 2];

				// If pixel is valid world point
				if (isnan( depth ) || abs( depth ) < depthThresh || depth > 0.0)
				{
					pixs.setColor( x, y, ofColor(0) );
				}
				else
				{
					pixs.setColor( x, y, getRGBValue( data[indx + 3] ) );
				}
			}
		}

		return pixs;
	}

	glm::vec3 ofxZed::getWorldFromMesh( int x, int y )
	{
		return pointMesh.getVertices()[x + y * camOptions.width];
	}

	cv::Mat ofxZed::getRGBDMat()
	{
		return slMat2cvMat( matDepthMesRGB );
	}

	glm::vec3 ofxZed::getWorldFromMat( int x, int y )
	{
		float *data = (float*)(matDepthMes.getPtr<sl::uchar1>( MEM::CPU ));

		int numChannels = 4;

		int w = matDepthMes.getWidth();
		int h = matDepthMes.getHeight();

		int indx = x * numChannels + y * w*numChannels;

		return glm::vec3( data[indx + 0],
						  data[indx + 1],
						  data[indx + 2]  );
	}

	cv::Mat ofxZed::getDepthMat()
	{
		return cvMatDepth;
	}

	ofColor ofxZed::getRGBValue( float v )
	{
		// decompose 4th float to rgb values
		unsigned char color[sizeof(float)];
		memcpy( color, &v, sizeof(float) );

		return ofColor( (int)color[0], (int)color[1], (int)color[2] );
	}
}