#ifndef ESVO_CORE_SEMIDENSE_H
#define ESVO_CORE_SEMIDENSE_H

#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/transform_broadcaster.h>

#include <esvo_core/container/CameraSystem.h>
#include <esvo_core/container/DepthMap.h>
#include <esvo_core/container/EventMatchPair.h>
#include <esvo_core/core/DepthFusion.h>
#include <esvo_core/core/DepthProblem.h>
#include <esvo_core/core/DepthProblemSolver.h>
#include <esvo_core/core/DepthRegularization.h>
#include <esvo_core/core/EventBM.h>
#include <esvo_core/tools/Visualization.h>
#include <esvo_core/tools/utils.h>

#include <kindr/minimal/position.h>
#include <kindr/minimal/quat-transformation.h>
#include <kindr/minimal/rotation-quaternion.h>

#include <dynamic_reconfigure/server.h>
#include <esvo_core/DVS_MappingStereoConfig.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <deque>
#include <future>
#include <map>
#include <mutex>

#include <cv_bridge/cv_bridge.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration.h>


#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
using pcl::visualization::PointCloudColorHandlerGenericField;

namespace esvo_core {
using namespace core;

// typedef <CurFrames> CurFrames;
// typedef <RefFrames> RefFrames;
// typedef <TempFrames> TempFrames;
struct CurFrames {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ros::Time t_;
  cv_bridge::CvImagePtr cur_depth_;
  cv_bridge::CvImagePtr cur_image_;
  cv_bridge::CvImagePtr cur_TS_;
  std::vector<pcl::PointNormal *> vPointXYZPtr_;
  Transformation tr_;
};

struct RefFrames {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ros::Time t_;
  cv_bridge::CvImagePtr ref_depth_;
  cv_bridge::CvImagePtr ref_image_;
  std::vector<pcl::PointNormal *> vPointXYZPtr_;
  Transformation tr_;
};

struct TempFrames {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ros::Time t_;
  cv_bridge::CvImagePtr temp_depth_;
  cv_bridge::CvImagePtr temp_image_;
  std::vector<pcl::PointNormal *> vPointXYZPtr_;
  Transformation tr_;
};

class MyPointRepresentation : public pcl::PointRepresentation <pcl::PointNormal>
{
    using pcl::PointRepresentation<pcl::PointNormal>::nr_dimensions_;
public:
    MyPointRepresentation ()
    {
        //定义尺寸值
        nr_dimensions_ = 4;
    }
    //覆盖copyToFloatArray方法来定义我们的特征矢量
    virtual void copyToFloatArray (const pcl::PointNormal &p, float * out) const
    {
        // < x, y, z, curvature >
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        out[3] = p.curvature;
    }
};
 

class semi_dense {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // typedef RotationQuaternionTemplate<Scalar> Rotation;

  semi_dense(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
  virtual ~semi_dense();

  bool MakeSDLoop(std::promise<void> prom_mapping, std::future<void> future_reset);

  void semidenseCallback(const sensor_msgs::ImageConstPtr &depth_map, const sensor_msgs::ImageConstPtr &image, const sensor_msgs::ImageConstPtr &TS_left);
  void semidenseirCallback(const sensor_msgs::ImageConstPtr &depth_map, const sensor_msgs::ImageConstPtr &image, const sensor_msgs::ImageConstPtr &TS_left);

  void stampedPoseCallback(const geometry_msgs::PoseStampedConstPtr &ps_msg);

  void make_semi_dense(cv_bridge::CvImagePtr &depthPtr, cv_bridge::CvImagePtr &imagePtr, cv_bridge::CvImagePtr &TSPtr);

  bool getPoseAt(const ros::Time &t, esvo_core::Transformation &Tr, const std::string &source_frame);

  void publishPose(const ros::Time &t, Transformation &tr);

  void publish_point_cloud(Eigen::MatrixXd &depth_eigen, int row, int col, Transformation tr, Transformation &pose_pub);

  void publish_mask(cv_bridge::CvImagePtr &depthPtr, int row, int col);

  void compute_pc_tr(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input, const pcl::PointCloud<pcl::PointXYZ>::Ptr &target, Eigen::Matrix4f &transformation, const double thresh);

  void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample);
  // Eigen::Quaterniond Quaternion_S_lerp(kindr::minimal::QuatTransformationTemplate.Rotation &start_q, kindr::minimal::QuatTransformationTemplate.Rotation &end_q, double t);
  Eigen::Quaterniond Quaternion_S_lerp(Eigen::Quaterniond &start_q, Eigen::Quaterniond &end_q, double t);

  void reset();

private:
  ros::NodeHandle nh_, pnh_;

  // Subscribers
  // ros::Subscriber depth_map_sub_, image_sub_;
  ros::Subscriber stampedPose_sub_;
  message_filters::Subscriber<sensor_msgs::Image> depth_map_sub_, depth_map_ir_sub_, image_sub_, image_ir_sub_, TS_left_sub_;

  // Publishers
  ros::Publisher pc_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher semi_dense_pub_;
  ros::Publisher canny_pub_;
  ros::Publisher mask_pub_;
  image_transport::ImageTransport it_;
  // double t_last_pub_pc_;

  // Depth-Image sync policy
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> ApproximateSyncPolicy;
  message_filters::Synchronizer<ApproximateSyncPolicy> DI_sync_;
  message_filters::Synchronizer<ApproximateSyncPolicy> DI_ir_sync_;


  // dynamic configuration (modify parameters online)
  // boost::shared_ptr<dynamic_reconfigure::Server<DVS_MappingStereoConfig> > server_;
  // dynamic_reconfigure::Server<DVS_MappingStereoConfig>::CallbackType dynamic_reconfigure_callback_;

  // offline data
  std::string dvs_frame_id_;
  std::string world_frame_id_;
  std::string depth_frame_id_;
  std::string calibInfoDir_;
  // CameraSystem::Ptr camSysPtr_;

  /* mapping parameters /
// range and visualization threshold
double invDepth_min_range_;
double invDepth_max_range_;
double cost_vis_threshold_;
size_t patch_area_;
double residual_vis_threshold_;
double stdVar_vis_threshold_;
size_t age_max_range_;
size_t age_vis_threshold_;
int fusion_radius_;
std::string FusionStrategy_;
int maxNumFusionFrames_;
int maxNumFusionPoints_;
size_t INIT_SGM_DP_NUM_Threshold_;
// module parameters
size_t PROCESS_EVENT_NUM_;
size_t TS_HISTORY_LENGTH_;
size_t mapping_rate_hz_;
// options
//bool changed_frame_rate_;
bool bRegularization_;
bool resetButton_;
bool bDenoising_;
bool bVisualizeGlobalPC_;
// visualization parameters
double visualizeGPC_interval_;
double visualize_range_;
size_t numAddedPC_threshold_;
// Event Block Matching (BM) parameters
double BM_half_slice_thickness_;
size_t BM_patch_size_X_;
size_t BM_patch_size_Y_;
size_t BM_min_disparity_;
size_t BM_max_disparity_;
size_t BM_step_;
double BM_ZNCC_Threshold_;
bool   BM_bUpDownConfiguration_;
*/
  // online data
  // EventQueue events_left_, events_right_;
  // TimeSurfaceHistory TS_history_;
  // StampedTimeSurfaceObs TS_obs_;
  // StampTransformationMap st_map_;
  std::shared_ptr<tf::Transformer> tf_;
  // size_t TS_id_;
  ros::Time tf_lastest_common_time_;

  // system
  std::string ESVO_System_Status_;
  // DepthProblemConfig::Ptr dpConfigPtr_;
  // DepthProblemSolver dpSolver_;
  // DepthFusion dFusor_;
  // DepthRegularization dRegularizor_;
  // Visualization visualizor_;
  // EventBM ebm_;

  // data transfer
  // std::vector<dvs_msgs::Event *> vALLEventsPtr_left_;// for BM
  // std::vector<dvs_msgs::Event *> vCloseEventsPtr_left_;// for BM
  // std::vector<dvs_msgs::Event *> vDenoisedEventsPtr_left_;// for BM
  // size_t totalNumCount_;// count the number of events involved
  // std::vector<dvs_msgs::Event *> vEventsPtr_left_SGM_;// for SGM

  // result
  // PointCloud::Ptr pc_, pc_near_, pc_global_;
  // DepthFrame::Ptr depthFramePtr_;
  // std::deque<std::vector<DepthPoint> > dqvDepthPoints_;

  // inter-thread management
  std::mutex data_mutex_;
  std::promise<void> mapping_thread_promise_, reset_promise_;
  std::future<void> mapping_thread_future_, reset_future_;

  cv::Mat semi_dense_;
  CurFrames curr_;
  RefFrames reff_;
  TempFrames temp1_;
  TempFrames temp2_;
  PointCloud::Ptr pc_world_;
  PointCloud::Ptr pc_local_;
  PointCloud::Ptr pc_global_;
  PointCloud::Ptr pc_temp_;
  PointCloud::Ptr pc_ref_;

  // std::shared_ptr <pcl::PointXYZ> ppoint_1;
  // std::shared_ptr <pcl::PointXYZ> ppoint_2;
  int global_pc_number;
  pcl::PointXYZ pc_global_temp_;
  // pc merge
  double thresh_pc_merge_ = 0.002;
  Eigen::Matrix4f pc_merge_tr_;
  Transformation pose_;

  ros::Time init_t_;

  std::vector<ros::Time> time_pose_;
  std::vector<ros::Time> time_depth_;
  std::map<ros::Time, cv_bridge::CvImagePtr> depth_mapPtr_;
  std::map<ros::Time, cv_bridge::CvImagePtr> image_mapPtr_;
  std::map<ros::Time, cv_bridge::CvImagePtr> TS_mapPtr_;
  std::map<ros::Time, string> id_map_;
  size_t TIME_DEPTH_LENGTH_;
  size_t POSE_LENGTH_;
  // size_t DEPTHZ_MAPPTR_LENGTH_;
  // size_t IMAGE_MAPPTR_LENGTH_;
  // size_t ID_MAP_LENGTH_;
  /**** mapping parameters ***/
  // range and visualization threshold
  // double invDepth_min_range_;
  // double invDepth_max_range_;
  // double cost_vis_threshold_;
  // size_t patch_area_;
  // double residual_vis_threshold_;
  // double stdVar_vis_threshold_;
  // size_t age_max_range_;
  // size_t age_vis_threshold_;
  // int fusion_radius_;
  // std::string FusionStrategy_;
  // int maxNumFusionFrames_;
  // int maxNumFusionPoints_;
  // size_t INIT_SGM_DP_NUM_Threshold_;
  // module parameters
  // size_t PROCESS_EVENT_NUM_;
  // size_t TS_HISTORY_LENGTH_;
  size_t makeSD_rate_hz_;
  // options
  bool changed_frame_rate_;
  // bool bRegularization_;
  // bool resetButton_;
  // bool bDenoising_;
  // bool bVisualizeGlobalPC_;
  // visualization parameters
  // double visualizeGPC_interval_;
  // double visualize_range_;
  // size_t numAddedPC_threshold_;
  // Event Block Matching (BM) parameters
  // double BM_half_slice_thickness_;
  // size_t BM_patch_size_X_;
  // size_t BM_patch_size_Y_;
  // size_t BM_min_disparity_;
  // size_t BM_max_disparity_;
  // size_t BM_step_;
  // double BM_ZNCC_Threshold_;
  // bool   BM_bUpDownConfiguration_;
  // cv::Mat semi_dense_;
  // CurFrames curr_;
  // RefFrames ref_;
  // TempFrames temp1_;
  // TempFrames temp2_;
  // PointCloud::Ptr pc_world_;
  // ros::Time ref_t_;
  // ros::Time cur_t_;
  // cv_bridge::CvImagePtr ref_detph_;
  // cv_bridge::CvImagePtr ref_image_;
  // cv_bridge::CvImagePtr cur_depth_;
  // cv_bridge::CvImagePtr cur_image_;
  // SGM parameters (Used by Initialization)
  // int num_disparities_;
  // int block_size_;
  // int P1_;
  // int P2_;
  // int uniqueness_ratio_;
  // cv::Ptr<cv::StereoSGBM> sgbm_;
  // Eigen::Quaterniond quat_s_;
  Transformation::Rotation::Implementation quat_s_;
  Transformation::Position temp_s_;
  // Transformation temp_p_;
  /**********************************************************/
  /******************** For test & debug ********************/
  /**********************************************************/
  // image_transport::Publisher invDepthMap_pub_, stdVarMap_pub_, ageMap_pub_, costMap_pub_;

  // For counting the total number of fusion
  // size_t TotalNumFusion_;
  // using namespace container;
  // using namespace tools;
  cv::Mat transfer;
  cv::Mat transfer_out;
  cv::Mat transfer_depth;
  cv::Mat cannyresult;
  cv::Mat rectification_matrix_;
  cv::Mat distortion_coefficients_;
  cv::Mat camera_matrix_;
  cv::Mat camera_matrix_new_;
  cv::Mat mask_;
  cv::Mat K_new_;
  cv::Mat rect_x_;
  cv::Mat rect_y_;
  bool rect_init_ = false;

  int semi_count_ = 0;
};

} // namespace esvo_core

#endif // ESVO_CORE_SEMIDENSE_H
