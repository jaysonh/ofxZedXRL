#include "ofxZedPosData.h"

void ofxZedPosData::update(sl::Pose & zedPose)
{
	cameraPos = glm::vec3( zedPose.getTranslation().tx,
						   zedPose.getTranslation().ty,
						   zedPose.getTranslation().tz  );

	cameraOri = glm::vec4( zedPose.getOrientation().ox,
						   zedPose.getOrientation().oy,
						   zedPose.getOrientation().oz,
						   zedPose.getOrientation().ow  );

	cameraRot = glm::vec3( zedPose.getRotationVector().x,
						   zedPose.getRotationVector().y,
						   zedPose.getRotationVector().z  );
}