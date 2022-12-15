#ifndef CANNY_EVT_MINUS_SYSTEM_H
#define CANNY_EVT_MINUS_SYSTEM_H

#include <Eigen/Eigen>
#include <boost/make_shared.hpp>
#include <chrono>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <deque>
#include <esvo_core/container/CameraSystem.h>
#include <esvo_core/core/RegProblemLM.h>
#include <esvo_core/core/RegProblemSolverLM.h>
#include <canny_evt_minus/canny_evt_minus_optimization.h>
#include <canny_evt_minus/canny_evt_minus_types.h>
#include <image_transport/image_transport.h>
#include <omp.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration.h>
#include <pcl/search/impl/search.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <string>
#include <vector>

namespace canny_evt_minus
{
namespace offline
{
using namespace types;
/* load configuration */
void prelude( std::string &config_path );
/* register ros publisher and subscriber */
void init( ros::NodeHandle *nh, image_transport::ImageTransport *it );
/* laod data from rosbag, including raw event, depth and rgb */
void dataloading();
/* control main loop of the offline demo program */
void main();
/* estimate pose by ICP and TS alignment */
void tracking( RefFrame &ref, CurFrame &cur );
/* build point cloud map */
void mapping();
/* ICP with init pose */
void ICP( const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample );
/* get verbose */
bool getVerbose();
/* set verbose true */
void setVerbose();
/* set verbose false */
void unsetVerbose();
/* get visualize */
bool getVisualize();
/* set visualize true */
void setVisualize();
/* set visualize false */
void unsetVisualize();
/* publish result */
void publishPose( const ros::Time &t, Transformation &tr );
void publishPath( const ros::Time &t, Transformation &tr );
/* save result */
void saveTrajectory( const std::string &resultDir );

Eigen::Matrix<double, 3, 4> getP_ev();

int getHeight_ev();

int getWidth_ev();


class MyPointRepresentation : public pcl::PointRepresentation <pcl::PointNormal>
{
    using pcl::PointRepresentation<pcl::PointNormal>::nr_dimensions_;
public:
    MyPointRepresentation ()
    {
        nr_dimensions_ = 4;
    }
    virtual void copyToFloatArray (const pcl::PointNormal &p, float * out) const
    {
        // < x, y, z, curvature >
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        out[3] = p.curvature;
    }
};
 
} // namespace offline
} // namespace canny_evt_minus
#endif