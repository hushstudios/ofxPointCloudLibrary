#pragma once
#include "ofxPointCloudLibrary/Types.hpp"
#include "ofxPointCloudLibrary/Utils.hpp"

namespace ofxPointCloudLibrary {

/* iterative closest point alignment */
class Alignment
{
public:
	/* use iterative closest point to align sourceCloud to targetCloud */
	bool align( const std::vector<glm::vec3>& sourceCloud, const std::vector<glm::vec3>& targetCloud )
	{
		m_sourceCloud.reset( new PointCloud( toPcl( sourceCloud ) ) );
		m_targetCloud.reset( new PointCloud( toPcl( targetCloud ) ) );

		m_icp.setInputSource( m_sourceCloud );
		m_icp.setInputTarget( m_targetCloud );

		m_outputCloud.clear();
		m_icp.align( m_outputCloud );

		return hasConverged();
	}

	bool hasConverged() { return m_icp.hasConverged(); }
	glm::mat4 getAlignmentMatrix() { return toOf( m_icp.getFinalTransformation() ); }
	float getFitnessScore() { return m_icp.getFitnessScore(); }

	const PointCloud& getOutput() { return m_outputCloud; }

protected:
	pcl::IterativeClosestPoint<Point, Point> m_icp;
	PointCloud::Ptr m_sourceCloud;
	PointCloud::Ptr m_targetCloud;
	PointCloud m_outputCloud;
};

}  // namespace ofxPointCloudLibrary