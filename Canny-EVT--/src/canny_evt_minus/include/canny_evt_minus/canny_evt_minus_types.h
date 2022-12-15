#ifndef CANNY_EVT_MINUS_TYPES_H
#define CANNY_EVT_MINUS_TYPES_H

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <kindr/minimal/quat-transformation.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <prophesee_event_msgs/Event.h>
#include <prophesee_event_msgs/EventArray.h>
#include <ros/ros.h>

namespace canny_evt_minus
{
namespace types
{

using Event          = prophesee_event_msgs::Event;               //for prophess
using EventArray     = prophesee_event_msgs::EventArray;
//using Event          = dvs_msgs::Event;                              //for dvs
//using EventArray     = dvs_msgs::EventArray;
using EventQueue     = std::deque<Event>;
using Point          = pcl::PointXYZ;
using PointCloud     = pcl::PointCloud<Point>;
using Transformation = kindr::minimal::QuatTransformation;

/* control flags for canny_evt_minus offiine demo */
typedef struct Flags
{
  ros::Time ts;

  bool SD;   /* create semi dense point cloud */
  bool TS;   /* create time surface map */
  bool ICP;  /* estimate pose by ICP */
  bool ED;   /* estimate pose by event and semi dense point cloud */
  bool INIT; /* whether whole system is initialized */
  Flags() : SD( false ), TS( false ), ICP( false ), ED( false ), INIT( false ) {}
} Flags;

struct RefFrame
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PointCloud                  pc_;             /* semi dense point cloud in event camera framework*/
  Transformation              tr_;             /* event to world */
  std::vector<Transformation> tr_rel_history_; /* record relative pose estimated during two depth keyframe */
};

struct CurFrame
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  cv::Mat        ts_; /* time surface map */
  Transformation tr_; /* event to world */
  size_t         numEventsSinceLastObs_;
};

} // namespace types
} // namespace canny_evt_minus
#endif
