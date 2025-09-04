#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <interfaces_pkg/msg/probability_point.hpp>
#include <interfaces_pkg/msg/probability_pcl.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <cmath>



#ifndef SERVER_TYPES_H
#define SERVER_TYPES_H
#define PCL_NO_PRECOMPILE
namespace bg = boost::geometry;

using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point>;

typedef pcl::PointXYZ point_t;
typedef interfaces_pkg::msg::ProbabilityPcl pcl_msg_t;
typedef interfaces_pkg::msg::ProbabilityPoint point_msg_t;
typedef std::map<std::tuple<float, float, float>, long unsigned int> point_2_count_map_t;
typedef std::map<std::tuple<float, float, float>, float> point_2_norm_cloud_map_t;
struct PointXYZProb
{
    PCL_ADD_POINT4D; // preferred way of adding a XYZ+padding
    float probability;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZProb, // here we assume a XYZ + "test" (as fields)
                                  (float, x, x)(float, y, y)(float, z, z)(float, probability, probability))
#endif