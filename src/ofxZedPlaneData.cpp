#include "ofxZedPlaneData.h"


void ofxZedPlaneData::update( sl::uint2 & coord, sl::Plane &plane )
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
			detPlaneMesh.addVertex(glm::vec3(v.x, v.y, v.z));
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
			uv.x = ofMap(v.x, 0, 10, 0, 500);
			uv.y = ofMap(v.y, 0, 10, 0, 500);
			detPlaneMesh.addTexCoord(uv);
		}
	}

}