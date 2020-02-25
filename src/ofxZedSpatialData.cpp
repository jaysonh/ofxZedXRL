#include "ofxZedSpatialData.h"

void ofxZedSpatialData::update( sl::Mesh slMesh )
{
	slMesh.filter(sl::MeshFilterParameters::MESH_FILTER::LOW);

	spatialMesh.clear();
	spatialMesh.setMode(OF_PRIMITIVE_POINTS);

	for (auto v : slMesh.vertices)
	{
		spatialMesh.addVertex( ofVec3f(v.x, v.y, v.z) );
	}

	for (auto id : slMesh.triangles)
	{
		spatialMesh.addIndex( id[0] );
		spatialMesh.addIndex( id[1] );
		spatialMesh.addIndex( id[2] );
	}
}