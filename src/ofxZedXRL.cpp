#include "ofxZedXRL.h"


ofxZedXRL::ofxZedXRL()
{

}

void ofxZedXRL::setup( bool _usePosTrack, bool _usePlaneDet, bool _useSpatialMap )
{
	InitParameters init_params;
	init_params.camera_resolution = RESOLUTION::HD720;
	init_params.depth_mode		  = DEPTH_MODE::PERFORMANCE;
	init_params.coordinate_units  = UNIT::METER;
	init_params.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP; 
	
	ERROR_CODE err = zed.open(init_params);
	
	if (err != ERROR_CODE::SUCCESS) {
		ofLog(OF_LOG_ERROR, "Error " + ofToString(err));
	}
	else
	{
		auto camera_infos = zed.getCameraInformation();
		ofLog(OF_LOG_NOTICE, "ZED Opened: " + ofToString(camera_infos.serial_number));
	}
	if (_usePosTrack || _usePlaneDet || _useSpatialMap) // if using plane detection we need to enable the positional tracking
	{
		usePosTrack = true;

		sl::PositionalTrackingParameters tracking_parameters;
		err = zed.enablePositionalTracking(tracking_parameters);
	}


	if (_useSpatialMap)
	{
		sl::SpatialMappingParameters mapping_parameters;
		mapping_parameters.resolution_meter = 0.1;  // Set resolution to 3cm
		err = zed.enableSpatialMapping(mapping_parameters);

		useSpatialMap = _useSpatialMap;
	}

	if (_usePlaneDet)
	{
		usePlaneDet = true;

	}
}
ofMesh * ofxZedXRL::getDetPlane()
{
	return &detPlaneMesh;
}

void ofxZedXRL::update()
{
	RuntimeParameters runtime_parameters;
	runtime_parameters.sensing_mode = SENSING_MODE::STANDARD; // Use STANDARD sensing mode

	if (zed.grab(runtime_parameters) == ERROR_CODE::SUCCESS)
	{

		sl::Mat matLeft;
		sl::Mat matRight;
		sl::Mat matDepth;
		sl::Mat pointCloud;

		zed.retrieveImage(   matLeft,     VIEW::LEFT   );
		zed.retrieveImage(   matRight,    VIEW::RIGHT  );
		zed.retrieveImage(   matDepth,    VIEW::DEPTH);
		zed.retrieveMeasure( pointCloud,  MEASURE::XYZRGBA);
		cvMatLeft     = slMat2cvMat( matLeft     );
		cvMatRight    = slMat2cvMat( matRight    );
		cvMatDepth    = slMat2cvMat( matDepth    );
		cvMatCloud    = slMat2cvMat( pointCloud  );
									 
		cv::cvtColor(cvMatLeft,  cvMatLeft,  cv::COLOR_BGR2RGB);
		cv::cvtColor(cvMatRight, cvMatRight, cv::COLOR_BGR2RGB);		

		/*if (useSpatialMap)
		{
			sl::SPATIAL_MAPPING_STATE mapping_state = zed.getSpatialMappingState();

			sl::Mesh mesh;

			zed.extractWholeSpatialMap(mesh);
			mesh.filter(sl::MeshFilterParameters::MESH_FILTER::LOW);
			
			spatialMesh.clear();
			spatialMesh.setMode(OF_PRIMITIVE_POINTS);
			//cout << "num vertices: " <<  mesh.vertices.size() << endl;
			if(mesh.vertices.size() < 10000)
			{
				for (auto v : mesh.vertices)
				{
					spatialMesh.addVertex(ofVec3f(v.x *100.0, v.y*100.0, v.z*100.0));
				}
			}
		}*/
		if (usePosTrack)
		{
			sl::Pose zed_pose;
			POSITIONAL_TRACKING_STATE state = zed.getPosition(zed_pose, REFERENCE_FRAME::WORLD);

			trackedPos = ofVec3f(zed_pose.getTranslation().tx, zed_pose.getTranslation().ty, zed_pose.getTranslation().tz);
			trackedOri = ofVec4f(zed_pose.getOrientation().ox, zed_pose.getOrientation().oy, zed_pose.getOrientation().oz, zed_pose.getOrientation().ow);
		}

		if (usePlaneDet)
		{
			Plane plane;
			Pose pose; // positional tracking data
			ERROR_CODE find_plane_status = ERROR_CODE::SUCCESS;
			POSITIONAL_TRACKING_STATE tracking_state = POSITIONAL_TRACKING_STATE::OFF;

			sl::RuntimeParameters runtime_parameters;
			runtime_parameters.measure3D_reference_frame = REFERENCE_FRAME::WORLD;

			tracking_state = zed.getPosition(pose);
						
			if (tracking_state == POSITIONAL_TRACKING_STATE::OK)
			{
				find_plane_status = zed.findPlaneAtHit(sl::uint2(640,480), plane);
				if (find_plane_status == ERROR_CODE::SUCCESS) {
					sl::Mesh mesh = plane.extractMesh();

					mesh.filter(sl::MeshFilterParameters::MESH_FILTER::LOW);
					cout << mesh.vertices.size() << endl;
					if (mesh.vertices.size() < 1000000000)
					{
						detPlaneMesh.clear();
						detPlaneMesh.setMode(OF_PRIMITIVE_POINTS);
						for (auto v : mesh.vertices)
						{
							detPlaneMesh.addVertex(ofVec3f(v.x *zScale, v.y*zScale, v.z*zScale));
							detPlaneMesh.addColor(ofColor::green);
						}
					}
				}
			}
		}
	}
}

ofMesh * ofxZedXRL::getPointMeshRGB()
{
	sl::Mat matDepthMes;
	zed.retrieveMeasure(matDepthMes, MEASURE::XYZRGBA);

	// make point cloud
	int w = matDepthMes.getWidth();
	int h = matDepthMes.getHeight();

	float *data = (float*)(matDepthMes.getPtr<float>(MEM::CPU));

	int step = 4;
	pointCloudVerts.resize(w*h);

	pointMesh.clear();
	pointMesh.setMode(OF_PRIMITIVE_POINTS);

	bool invX = true;
	bool invY = true;

	int numChannels = 4;
	for (int y = 0; y < h; y += step)
	{
		for (int x = 0; x < w; x += step)
		{
			int indx = x * numChannels + y * w*numChannels;

			ofVec3f p( data[indx + 0] * zScale,
					   data[indx + 1] * zScale,
				       data[indx + 2] * zScale);
			
			// decompose 4th float to rgb values
			float col = data[indx + 3];					
			unsigned char color[sizeof(float)];
			memcpy(color, &col, sizeof(float));
			ofColor c((int)color[0], (int)color[1], (int)color[2]);

			if (invX) p.x = -p.x;
			if (invY) p.y = -p.y;

			pointMesh.addVertex( p );
			pointMesh.addColor(  c );
		}
	}
	return &pointMesh;
}
ofMesh * ofxZedXRL::getPointMesh()
{
	sl::Mat matDepthMes;
	zed.retrieveMeasure(matDepthMes, MEASURE::XYZ);

	// make point cloud
	int w = matDepthMes.getWidth();
	int h = matDepthMes.getHeight();

	float *data = (float*)(matDepthMes.getPtr<sl::uchar1>(MEM::CPU));

	int step = 4; 
	float zScale = 100.0;
	pointCloudVerts.resize(w*h);

	pointMesh.clear();
	pointMesh.setMode(OF_PRIMITIVE_POINTS);
	float scale = 100.0;

	bool invX = true;
	bool invY = true;

	int numChannels = 3;
	
	for (int y = 0; y < h; y += step)
	{
		for (int x = 0; x < w; x += step)
		{
			int indx = x * numChannels + y * w*numChannels;

			ofVec3f p( data[indx + 0] * zScale,
				       data[indx + 1] * zScale,
				       data[indx + 2] * zScale  );

			if (invX) p.x = -p.x;
			if (invY) p.y = -p.y;

			pointMesh.addVertex(p);

		}
	}

	return &pointMesh;
}
ofMesh *ofxZedXRL::getSpatialMesh()
{
	return &spatialMesh;
}

void ofxZedXRL::drawDetPlane()
{
	
	detPlaneMesh.draw();
}

float ofxZedXRL::getDepthAt(int x, int y)
{

	return cvMatDepthMes.at<float>(x, y);
}

void ofxZedXRL::drawLeft(int x, int y, int w, int h)
{

	ofxCv::drawMat(cvMatLeft, x, y, w, h);
}

void ofxZedXRL::drawRight(int x, int y, int w, int h)
{

	ofxCv::drawMat(cvMatRight, x, y, w, h);
}

void ofxZedXRL::drawDepth(int x, int y, int w, int h)
{
	ofxCv::drawMat(cvMatDepth, x, y, w, h);

}


cv::Mat ofxZedXRL::slMat2cvMat(Mat& input, sl::MEM memType) {
	// Mapping between MAT_TYPE and CV_TYPE
	int cv_type = -1;
	switch (input.getDataType()) {
	case MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
	case MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
	case MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
	case MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
	case MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
	case MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
	case MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
	case MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
	default: break;
	}

	// Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
	// cv::Mat and sl::Mat will share a single memory structure
	return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(memType));
}


