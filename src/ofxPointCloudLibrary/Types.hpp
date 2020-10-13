#pragma once

#include "ofMain.h"
#include "ofxPointCloudLibrary/Common.hpp"

namespace ofxPointCloudLibrary {

// Point
using Point = pcl::PointXYZ;

// Point Cloud
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

struct PointCloudData
{
	PointCloudData() {}
	PointCloudData( const PointCloud& pc )
	    : nPoints( pc.size() ), width( pc.width ), height( pc.height ), isDense( pc.is_dense ) {}
	size_t nPoints = 0;
	size_t width   = 0;  // if height is 1, width == nPoints, otherwise number of columns in image-structured point cloud (rows + cols)
	size_t height  = 1;  // 1 means unstructured data, >1 means number of rows of structured data
	bool isDense   = false;

	// TODO - add support for:
	// pcl::PCLHeader header;                   // The point cloud header. It contains information about the acquisition time.
	// Eigen::Vector4f    sensor_origin_;       // Sensor acquisition pose (origin/translation).
	// Eigen::Quaternionf sensor_orientation_;  // Sensor acquisition pose (rotation).
};

// ------------------------
// oF <--> PCL conversions
// ------------------------

// Point
inline glm::vec3 toOf( const Point& point )
{
	return { point.x, point.y, point.z };
}

inline Point toPcl( const glm::vec3& point )
{
	return { point.x, point.y, point.z };
}

// PointCloud
inline std::vector<glm::vec3> toOf( const PointCloud& pointCloud )
{
	std::vector<glm::vec3> points;
	points.reserve( pointCloud.size() );
	for ( const auto& p : pointCloud ) {
		points.emplace_back( p.x, p.y, p.z );
	}
}

inline PointCloud toPcl( const std::vector<glm::vec3>& points )
{
	auto pc = PointCloud( points.size(), 1 );
	for ( int i = 0; i < points.size(); ++i ) pc[i] = toPcl( points[i] );
	return pc;
}


}  // namespace ofxPointCloudLibrary