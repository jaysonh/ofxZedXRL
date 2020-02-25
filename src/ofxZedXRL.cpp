#include "ofxZedXRL.h"


ofxZedXRL::ofxZedXRL()
{

}

void ofxZedXRL::setup( bool usePosTrack, bool usePlaneDet, bool useSpatialMap, UNIT unit )
{

	this->useSpatMap  = useSpatMap;
	this->usePosTrack = usePosTrack;
	this->usePlaneDet = usePlaneDet;

	InitParameters init_params;
	init_params.camera_resolution = RESOLUTION::HD720;
	init_params.depth_mode		  = DEPTH_MODE::ULTRA;  // PERFORMANCE QUALITY 
	init_params.coordinate_units  = unit;				// UNIT::MILLIMETER;// UNIT::METER;
	init_params.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP; 
	
	ERROR_CODE err = zed.open(init_params);
	
	if (err != ERROR_CODE::SUCCESS) {
		ofLog(OF_LOG_ERROR, "Error " + ofToString(err));
	}else
	{
		auto camera_infos = zed.getCameraInformation();
		ofLog(OF_LOG_NOTICE, "ZED Opened: " + ofToString(camera_infos.serial_number));
	}

	// if using plane detection we need to enable the positional tracking
	usePosTrack = (usePosTrack || usePlaneDet || useSpatialMap);

	if (useSpatialMap)
	{
		sl::SpatialMappingParameters mapping_parameters;
		mapping_parameters.resolution_meter = 0.03;  // Set resolution to 1cm
		mapping_parameters.map_type		    = sl::SpatialMappingParameters::SPATIAL_MAP_TYPE::MESH; // FUSED_POINT_CLOUD or MESH
		mapping_parameters.save_texture     = true;
		
		err = zed.enableSpatialMapping(mapping_parameters);
	}
}
ofMesh & ofxZedXRL::getDetPlane()
{
	return detPlaneMesh;
}

glm::vec3 & ofxZedXRL::getCameraPos()
{
	return cameraPos;
}

glm::vec4 & ofxZedXRL::getCameraOri()
{
	return cameraOri;
}

glm::vec3 & ofxZedXRL::getCameraRot()
{
	return cameraRot;
}
void ofxZedXRL::update()
{
	RuntimeParameters runtime_parameters;
	runtime_parameters.sensing_mode = SENSING_MODE::FILL; // Use STANDARD sensing mode FILL/STANDARD/LAST
	runtime_parameters.measure3D_reference_frame = REFERENCE_FRAME::WORLD;
	
	if (zed.grab(runtime_parameters) == ERROR_CODE::SUCCESS)
	{
		sl::Mat matLeft;
		sl::Mat matRight;
		sl::Mat matDepth;

		zed.retrieveImage(   matLeft,       VIEW::LEFT  );
		zed.retrieveImage(   matRight,      VIEW::RIGHT );
		zed.retrieveImage(   matDepth,      VIEW::DEPTH );
		
		zed.retrieveMeasure(matDepthMesRGB, MEASURE::XYZRGBA);
		zed.retrieveMeasure(matDepthMes,    MEASURE::XYZ,MEM::CPU, sl::Resolution(1280,720));
		cvMatLeft     = slMat2cvMat( matLeft     );
		cvMatRight    = slMat2cvMat( matRight    );
		cvMatDepth    = slMat2cvMat( matDepth    );
									 
		cv::cvtColor(cvMatLeft,  cvMatLeft,  cv::COLOR_BGR2RGB); // need to shift color to rgb
		cv::cvtColor(cvMatRight, cvMatRight, cv::COLOR_BGR2RGB);		
		if (usePosTrack)
		{
				sl::Pose zed_pose;
				zed.getPosition(zed_pose);
			
				cameraPos = glm::vec3( zed_pose.getTranslation().tx, 
									   zed_pose.getTranslation().ty, 
									   zed_pose.getTranslation().tz );
				cameraOri = glm::vec4( zed_pose.getOrientation().ox,
									   zed_pose.getOrientation().oy,
									   zed_pose.getOrientation().oz,
									   zed_pose.getOrientation().ow );
				cameraRot = glm::vec3( zed_pose.getRotationVector().x,
									   zed_pose.getRotationVector().y,
									   zed_pose.getRotationVector().z);


		}
		if(useSpatMap)
		{
			sl::SPATIAL_MAPPING_STATE mapping_state = zed.getSpatialMappingState();			

			//zed.extractWholeSpatialMap(spatialMeshsl);
			float duration = ofGetElapsedTimef() - ts_last;

			if (duration > 0.5) {
				zed.requestSpatialMapAsync();
				ts_last = ofGetElapsedTimef();
			}

			if (zed.getSpatialMapRequestStatusAsync() == ERROR_CODE::SUCCESS) {
				zed.retrieveSpatialMapAsync(spatialMeshsl);
			}

			//mesh.filter(sl::MeshFilterParameters::MESH_FILTER::LOW);

			spatialMesh.clear();
			spatialMesh.setMode(OF_PRIMITIVE_POINTS);

			for (auto v : spatialMeshsl.vertices)
			{
				spatialMesh.addVertex(ofVec3f(v.x, v.y, v.z));
			}

			for (auto id : spatialMeshsl.triangles)
			{
				//detPlaneMesh.addIndex(id[0]);
				//detPlaneMesh.addIndex(id[1]);
				//detPlaneMesh.addIndex(id[2]);
			}
		}

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
						
			if (zed.getPosition(pose) == POSITIONAL_TRACKING_STATE::OK)
			{ 
				sl::uint2 coord = sl::uint2( ofMap(ofGetMouseX(), 0, ofGetWidth(), 0, 1280, true),
											 ofMap(ofGetMouseY(), 0, ofGetHeight(), 0, 720, true));
				 
				find_plane_status = zed.findPlaneAtHit(coord, plane);
				
				if (find_plane_status == ERROR_CODE::SUCCESS) 
				{
					sl::Mesh mesh = plane.extractMesh();
					sl::float3 planeNorm = plane.getNormal();
					sl::float3 normNorm = planeNorm / planeNorm.norm();

					mesh.filter(sl::MeshFilterParameters::MESH_FILTER::LOW);
					//mesh.applyTexture();
					if (mesh.getNumberOfTriangles() > 0)
					{
						detPlaneMesh.clear();
						//detPlaneMesh.setMode(OF_PRIMITIVE_POINTS);
						detPlaneMesh.setMode(OF_PRIMITIVE_TRIANGLES);
						vector <sl::float3> boundVerts = plane.getBounds();
						
						for (auto v : mesh.vertices)
						{
							addPointToMesh(v);
						}

						// Now add verts
						for (auto id : mesh.triangles)
						{
							detPlaneMesh.addIndex(id[0]);
							detPlaneMesh.addIndex(id[1]);
							detPlaneMesh.addIndex(id[2]);
						}

						// Calc uv
						for (auto v : mesh.vertices)
						{
							glm::vec2 uv;
							uv.x = ofMap(v.x, 0, 10,0, 500);
							uv.y = ofMap(v.y, 0, 10, 0, 500);
							detPlaneMesh.addTexCoord(uv);
						}
					}


					//addPointToMesh(boundVerts.front(), &detPlaneMesh);
				}
			}
		}
	}
}

void ofxZedXRL::addPointToMesh(sl::float3 v)
{
	detPlaneMesh.addVertex( glm::vec3(v.x, v.y, v.z) );
	detPlaneMesh.addColor(  ofColor(0,  255, 0, 125)  );
}

ofMesh & ofxZedXRL::getMeshRGB(int meshStep)
{
	// make point cloud
	int w = matDepthMesRGB.getWidth();
	int h = matDepthMesRGB.getHeight();

	float *data = (float*)(matDepthMesRGB.getPtr<float>(MEM::CPU));

	pointCloudVerts.resize(w*h);

	pointMesh.clear();
	pointMesh.setMode(OF_PRIMITIVE_POINTS);

	int numChannels = 4;

	for (int y = 0; y < h; y += meshStep)
	{
		for (int x = 0; x < w; x += meshStep)
		{
			int indx = x * numChannels + y * w*numChannels;

			ofVec3f p(data[indx + 0],
					  data[indx + 1],
					  data[indx + 2]);

			// decompose 4th float to rgb values	
			ofColor c = getRGBFromFloat(data[indx + 3]);

			pointMesh.addVertex( p );
			pointMesh.addColor(  c );
		}
	}

	return pointMesh;
}
int ofxZedXRL::getMeshWidth()
{
	return matDepthMes.getWidth();
}

int ofxZedXRL::getMeshHeight()
{
	return matDepthMes.getHeight();
}

ofMesh & ofxZedXRL::getMesh(int meshStep)
{
	// make point cloud
	int w = matDepthMes.getWidth();
	int h = matDepthMes.getHeight();

	float *data = (float*)(matDepthMes.getPtr<sl::uchar1>(MEM::CPU));

	pointMesh.clear();
	pointCloudVerts.resize(w*h);

	pointMesh.setMode(OF_PRIMITIVE_POINTS);
	float scale = 1.0;

	bool invX = true;
	bool invY = true;

	int numChannels = 4;

	for (int y = 0; y < h; y += meshStep)
	{
		for (int x = 0; x < w; x += meshStep)
		{
			int indx = x * numChannels + y * w*numChannels;
			glm::vec3 p( data[indx + 0],
				         data[indx + 1],
				         data[indx + 2]);
			pointMesh.addVertex( p );
			pointMesh.addColor(  ofColor(0, 125, ofMap(p.x, -10000, 10000, 0, 255)) );
		}
	}	

	return pointMesh;
}
ofMesh & ofxZedXRL::getSpatialMesh()
{
	return spatialMesh;
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


ofPixels ofxZedXRL::getImageColor()
{

	ofImage img;

	ofxCv::toOf(cvMatLeft, img);

	return img.getPixels();
}

ofPixels ofxZedXRL::getImageCalibRGB(float depthThresh)
{
	ofPixels pixs; 

	// make point cloud
	int w = matDepthMesRGB.getWidth();
	int h = matDepthMesRGB.getHeight();

	pixs.allocate(w, h, OF_PIXELS_RGB);

	float *data = (float*)(matDepthMesRGB.getPtr<float>(MEM::CPU));
	unsigned char *pixels = new unsigned char[w * h * 3];

	int numChannels = 4;
	for (int y = 0; y < h; y += 1)
	{
		for (int x = 0; x < w; x += 1)
		{
			int indx = x * numChannels + y * w*numChannels;		
			int indxPix = x * 3 + y * 3 * w;

			
			float depth = data[indx + 2];
			if (isnan(depth) || abs(depth) < depthThresh || depth > 0.0)
			{
				pixs.setColor(x, y, ofColor(0));
			}
			else
			{
				// decompose 4th float to rgb values
				float col = data[indx + 3];
				unsigned char color[sizeof(float)];
				memcpy(color, &col, sizeof(float));

				ofColor c((int)color[0], (int)color[1], (int)color[2]);
				
				pixs.setColor(x, y, c);
			}
		}
	}
 
	return pixs;
}

glm::vec3 ofxZedXRL::getWorldCoordAt(glm::vec2 pos)
{
	int indx = (int)pos.x + (int)pos.y * getMeshWidth();
	return pointMesh.getVertices()[indx];
}

cv::Mat ofxZedXRL::getRGBDMat()
{
	return slMat2cvMat(matDepthMesRGB);
}

glm::vec3 ofxZedXRL::getTranslatedPos(int x, int y)
{
	float *data = (float*)(matDepthMes.getPtr<sl::uchar1>(MEM::CPU));
	
	int numChannels = 4;

	int w = matDepthMes.getWidth();
	int h = matDepthMes.getHeight();

	int indx = x * numChannels + y * w*numChannels;

	return glm::vec3( data[indx + 0],
					  data[indx + 1],
					  data[indx + 2]  );
}

cv::Mat ofxZedXRL::getDepthMat()
{
	return cvMatDepth;
}

ofColor ofxZedXRL::getRGBFromFloat(float v)
{
	// decompose 4th float to rgb values
	unsigned char color[sizeof(float)];
	memcpy(color, &v, sizeof(float));

	return ofColor((int)color[0], (int)color[1], (int)color[2]);
}