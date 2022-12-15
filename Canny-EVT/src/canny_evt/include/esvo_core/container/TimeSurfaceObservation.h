#ifndef ESVO_CORE_CONTAINER_TIMESURFACEOBSERVATION_H
#define ESVO_CORE_CONTAINER_TIMESURFACEOBSERVATION_H

#include <cv_bridge/cv_bridge.h>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

#include <kindr/minimal/quat-transformation.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <esvo_core/tools/TicToc.h>
#include <esvo_core/tools/utils.h>

//#define TIME_SURFACE_OBSERVATION_LOG
namespace esvo_core {
using namespace tools;
namespace container {
struct TimeSurfaceObservation {
  TimeSurfaceObservation(cv_bridge::CvImagePtr &left, cv_bridge::CvImagePtr &right, cv_bridge::CvImagePtr &all, Transformation &tr, size_t id, bool bCalcTsGradient = false) : tr_(tr), id_(id) {
    cv::cv2eigen(left->image, TS_left_);
    cv::cv2eigen(right->image, TS_right_);
    cv::cv2eigen(all->image, TS_all_);

    if (bCalcTsGradient) {
#ifdef TIME_SURFACE_OBSERVATION_LOG
      TicToc tt;
      tt.tic();
#endif
      cv::Mat cv_dTS_du_left, cv_dTS_dv_left;
      cv::Sobel(left->image, cv_dTS_du_left, CV_64F, 1, 0);
      cv::Sobel(left->image, cv_dTS_dv_left, CV_64F, 0, 1);
      cv::cv2eigen(cv_dTS_du_left, dTS_du_left_);
      cv::cv2eigen(cv_dTS_dv_left, dTS_dv_left_);
#ifdef TIME_SURFACE_OBSERVATION_LOG
      LOG(INFO) << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@Sobel computation (" << id_ << ") takes " << tt.toc() << " ms.";
#endif
    }
  }
TimeSurfaceObservation(cv_bridge::CvImagePtr &left, cv_bridge::CvImagePtr &right, Transformation &tr, size_t id, bool bCalcTsGradient = false) : tr_(tr), id_(id) {
    cv::cv2eigen(left->image, TS_left_);
    cv::cv2eigen(right->image, TS_right_);

    if (bCalcTsGradient) {
#ifdef TIME_SURFACE_OBSERVATION_LOG
      TicToc tt;
      tt.tic();
#endif
      cv::Mat cv_dTS_du_left, cv_dTS_dv_left;
      cv::Sobel(left->image, cv_dTS_du_left, CV_64F, 1, 0);
      cv::Sobel(left->image, cv_dTS_dv_left, CV_64F, 0, 1);
      cv::cv2eigen(cv_dTS_du_left, dTS_du_left_);
      cv::cv2eigen(cv_dTS_dv_left, dTS_dv_left_);
#ifdef TIME_SURFACE_OBSERVATION_LOG
      LOG(INFO) << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@Sobel computation (" << id_ << ") takes " << tt.toc() << " ms.";
#endif
    }
  }
  // override version without initializing the transformation in the constructor.
  TimeSurfaceObservation(cv_bridge::CvImagePtr &left, cv_bridge::CvImagePtr &right, cv_bridge::CvImagePtr &all, size_t id, bool bCalcTsGradient = false) : id_(id) {
   
    cvImagePtr_left_ = left;
    cvImagePtr_right_ = right;
    cvImagePtr_all_ = all;

    cv::cv2eigen(left->image, TS_left_);
    cv::cv2eigen(right->image, TS_right_);
    cv::cv2eigen(all->image, TS_all_);

    if (bCalcTsGradient) {
#ifdef TIME_SURFACE_OBSERVATION_LOG
      TicToc tt;
      tt.tic();
#endif
      cv::Mat cv_dTS_du_left, cv_dTS_dv_left;
      cv::Mat cv_dTS_du_right, cv_dTS_dv_right;
      cv::Mat cv_dTS_du_all, cv_dTS_dv_all;
      cv::Sobel(left->image, cv_dTS_du_left, CV_64F, 1, 0);
      cv::Sobel(left->image, cv_dTS_dv_left, CV_64F, 0, 1);

      cv::cv2eigen(cv_dTS_du_left, dTS_du_left_);
      cv::cv2eigen(cv_dTS_dv_left, dTS_dv_left_);

      cv::cv2eigen(cv_dTS_du_all, dTS_du_all_);
      cv::cv2eigen(cv_dTS_dv_all, dTS_dv_all_);
#ifdef TIME_SURFACE_OBSERVATION_LOG
      LOG(INFO) << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@Sobel computation (" << id_ << ") takes " << tt.toc() << " ms.";
#endif
    }
  }
TimeSurfaceObservation(cv_bridge::CvImagePtr &left, cv_bridge::CvImagePtr &right, size_t id, bool bCalcTsGradient = false) : id_(id) {
    cvImagePtr_left_ = left;
    cvImagePtr_right_ = right;
    cv::cv2eigen(left->image, TS_left_);
    cv::cv2eigen(right->image, TS_right_);

    if (bCalcTsGradient) {
#ifdef TIME_SURFACE_OBSERVATION_LOG
      TicToc tt;
      tt.tic();
#endif
      cv::Mat cv_dTS_du_left, cv_dTS_dv_left;
      cv::Mat cv_dTS_du_right, cv_dTS_dv_right;

      cv::Sobel(left->image, cv_dTS_du_left, CV_64F, 1, 0);
      cv::Sobel(left->image, cv_dTS_dv_left, CV_64F, 0, 1);

      cv::cv2eigen(cv_dTS_du_left, dTS_du_left_);
      cv::cv2eigen(cv_dTS_dv_left, dTS_dv_left_);
#ifdef TIME_SURFACE_OBSERVATION_LOG
      LOG(INFO) << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@Sobel computation (" << id_ << ") takes " << tt.toc() << " ms.";
#endif
    }
  }
  TimeSurfaceObservation(){};

  // added by zyf
  inline void add_mask(cv::Mat &mask) {
    cv::imwrite("/home/wp/zyf/mask_debug_0.png", mask);
    cv::imwrite("/home/wp/zyf/mask_debug_1.png", cvImagePtr_left_->image);
    cvImagePtr_left_->image = cvImagePtr_left_->image.mul(mask);
  }

  //added by zyf
  inline void ANNF(){
    //使用模糊TSM后得到的TS进行ANNF构建
    int r = TS_blurred_left_.rows();//positive 对应的TSM
    int c = TS_blurred_left_.cols();
    int radius = 5;
    Distance_Field_positive.resize(r,c);
    Col_ANNF_positive.resize(r,c);
    Row_ANNF_positive.resize(r,c);
    Distance_Field_positive.fill(-1);
    Col_ANNF_positive.fill(-1);
    Row_ANNF_positive.fill(-1);
    //cv::Mat TS_blurred_left_cv, TS_blurred_left_edge;
    Eigen::MatrixXd TS_blurred_left_edge;
    //cv::eigen2cv(TS_blurred_left_, TS_blurred_left_cv);
    TS_blurred_left_edge = TS_blurred_left_;
    //cv::Canny(TS_blurred_left_cv, TS_blurred_left_edge, 150, 100,3);
    for(int i=0; i<r; i++){
      for(int j=0; j<c; j++){
        if(TS_blurred_left_edge(i,j) > 15){
          BuildANNF_positive(i,j,radius);
        }
      }
    }
    //detect(Distance_Field_positive, TS_blurred_left_edge);

    int r1 = TS_blurred_right_.rows();//negative 对应的TSM
    int c1 = TS_blurred_right_.cols();
    //int radius = 8;
    Distance_Field_negative.resize(r1,c1);
    Col_ANNF_negative.resize(r1,c1);
    Row_ANNF_negative.resize(r1,c1);
    Distance_Field_negative.fill(-1);
    Col_ANNF_negative.fill(-1);
    Row_ANNF_negative.fill(-1);
    Eigen::MatrixXd TS_blurred_right_edge;
    //cv::Mat TS_blurred_right_cv, TS_blurred_right_edge;
    //cv::eigen2cv(TS_blurred_right_, TS_blurred_right_cv);
    TS_blurred_right_edge = TS_blurred_right_;
    //cv::Canny(TS_blurred_right_cv, TS_blurred_right_edge, 150, 100,3);
    for(int i=0; i<r1; i++){
      for(int j=0; j<c1; j++){
        if(TS_blurred_right_edge(i,j) > 15){
          BuildANNF_negative(i,j,radius);
        }
      }
    }
    //detect(Distance_Field_negative, TS_blurred_right_edge);

    int r2 = TS_blurred_all_.rows();//netural 对应的TSM
    int c2 = TS_blurred_all_.cols();
    int number_event = 0;
    //int radius = 8;
    Distance_Field_neutral.resize(r2,c2);
    Col_ANNF_neutral.resize(r2,c2);
    Row_ANNF_neutral.resize(r2,c2);
    Distance_Field_neutral.fill(-1);
    Col_ANNF_neutral.fill(-1);
    Row_ANNF_neutral.fill(-1);
    Eigen::MatrixXd TS_blurred_all_edge;
    //cv::Mat TS_blurred_all_cv, TS_blurred_all_edge;
    //cv::eigen2cv(TS_blurred_all_, TS_blurred_all_cv);
    TS_blurred_all_edge = TS_blurred_all_;
    // cv::Canny(TS_blurred_all_cv, TS_blurred_all_edge, 150, 100,3);
    for(int i=0; i<r2; i++){
      for(int j=0; j<c2; j++){
        if(TS_blurred_all_edge(i,j) > 15){
          BuildANNF_neutral(i,j,radius);
          number_event++;
        }
      }
    }
    //detect(Distance_Field_neutral, TS_blurred_all_edge);
    Distance_Field_neutral_show = (Distance_Field_neutral + Eigen::MatrixXd::Ones(Distance_Field_neutral.rows(), Distance_Field_neutral.cols()))*10;
  }

  inline void BuildANNF_positive(int row, int col, int radius){
    int r = TS_blurred_left_.rows();//positive 对应的TSM
    int c = TS_blurred_left_.cols();
    int start_r = row - radius;
    int start_c = col - radius;
    int end_r = row + radius;
    int end_c = col + radius;
    for(int i=start_r; i<=end_r; i++){
      for(int j=start_c; j<=end_c; j++){
        if(i<0 || j<0 || i>=r || j>=c){
          continue;
        }
        int d = sqrt((i-row)*(i-row) + (j-col)*(j-col));
        if(Distance_Field_positive(i,j) == -1){
          Distance_Field_positive(i,j) = d;
          Col_ANNF_positive(i,j) = col;
          Row_ANNF_positive(i,j) = row;
        }
        else if(d < Distance_Field_positive(i,j)){
          Distance_Field_positive(i,j) = d;
          Col_ANNF_positive(i,j) = col;
          Row_ANNF_positive(i,j) = row;
        }
      }
    }
  }

  inline void BuildANNF_negative(int row, int col, int radius){
    int r = TS_blurred_left_.rows();//positive 对应的TSM
    int c = TS_blurred_left_.cols();
    int start_r = row - radius;
    int start_c = col - radius;
    int end_r = row + radius;
    int end_c = col + radius;
    for(int i=start_r; i<=end_r; i++){
      for(int j=start_c; j<=end_c; j++){
        if(i<0 || j<0 || i>=r || j>=c){
          continue;
        }
        int d = sqrt((i-row)*(i-row) + (j-col)*(j-col));
        if(Distance_Field_negative(i,j) == -1){
          Distance_Field_negative(i,j) = d;
          Col_ANNF_negative(i,j) = col;
          Row_ANNF_negative(i,j) = row;
        }
        else if(d < Distance_Field_negative(i,j)){
          Distance_Field_negative(i,j) = d;
          Col_ANNF_negative(i,j) = col;
          Row_ANNF_negative(i,j) = row;
        }
      }
    }
  }

    inline void BuildANNF_neutral(int row, int col, int radius){
    int r = TS_blurred_left_.rows();//positive 对应的TSM
    int c = TS_blurred_left_.cols();
    int start_r = row - radius;
    int start_c = col - radius;
    int end_r = row + radius;
    int end_c = col + radius;
    for(int i=start_r; i<=end_r; i++){
      for(int j=start_c; j<=end_c; j++){
        if(i<0 || j<0 || i>=r || j>=c){
          continue;
        }
        int d = sqrt((i-row)*(i-row) + (j-col)*(j-col));
        if(Distance_Field_neutral(i,j) == -1){
          Distance_Field_neutral(i,j) = d;
          Col_ANNF_neutral(i,j) = col;
          Row_ANNF_neutral(i,j) = row;
        }
        else if(d < Distance_Field_neutral(i,j)){
          Distance_Field_neutral(i,j) = d;
          Col_ANNF_neutral(i,j) = col;
          Row_ANNF_neutral(i,j) = row;
        }
      }
    }
  }

  inline void detect(Eigen::MatrixXd &DF, Eigen::MatrixXd &TS){
     for(int i=0; i<DF.rows(); i++){
       for(int j=0; j<DF.cols(); j++){
         if(DF(i,j)==0 && TS(i,j) != 0){
           std::cout<<"right"<<std::endl;
         }
         else if(DF(i,j)!=0 && TS(i,j) != 0){
           std::cout<<"wrong"<<std::endl;
         }
       }
     }
  }

  inline bool isEmpty() {
    if (TS_left_.rows() == 0 || TS_left_.cols() == 0 || TS_right_.rows() == 0 || TS_right_.cols() == 0)
      return true;
    else
      return false;
  }

  inline void setTransformation(Transformation &tr) { tr_ = tr; }

  inline void GaussianBlurTS(size_t kernelSize) {
    cv::Mat mat_left_, mat_right_, mat_all_;
    cv::GaussianBlur(cvImagePtr_left_->image, mat_left_, cv::Size(kernelSize, kernelSize), 0.0);
    cv::GaussianBlur(cvImagePtr_right_->image, mat_right_, cv::Size(kernelSize, kernelSize), 0.0);
    cv::GaussianBlur(cvImagePtr_all_->image, mat_all_, cv::Size(kernelSize, kernelSize), 0.0);

    cv::cv2eigen(mat_left_, TS_left_);
    cv::cv2eigen(mat_right_, TS_right_);
    cv::cv2eigen(mat_all_, TS_all_);
  }

  inline void getTimeSurfaceNegative(size_t kernelSize) {
    Eigen::MatrixXd ceilMat(TS_left_.rows(), TS_left_.cols());
    ceilMat.setConstant(255.0);
    if (kernelSize > 0) {
      cv::Mat mat_left_;
      cv::GaussianBlur(cvImagePtr_left_->image, mat_left_, cv::Size(kernelSize, kernelSize), 0.0);
      cv::cv2eigen(mat_left_, TS_blurred_left_);
      TS_negative_left_ = ceilMat - TS_blurred_left_;
    } else {
      TS_negative_left_ = ceilMat - TS_left_;
    }
    
    ceilMat.setConstant(255.0);
    if (kernelSize > 0) {
      cv::Mat mat_right_;
      cv::GaussianBlur(cvImagePtr_right_->image, mat_right_, cv::Size(kernelSize, kernelSize), 0.0);
      cv::cv2eigen(mat_right_, TS_blurred_right_);
      TS_negative_right_ = ceilMat - TS_blurred_right_;
    } else {
      TS_negative_right_ = ceilMat - TS_right_;
    }

    ceilMat.setConstant(255.0);
    if (kernelSize > 0) {
      cv::Mat mat_all_;
      cv::GaussianBlur(cvImagePtr_all_->image, mat_all_, cv::Size(kernelSize, kernelSize), 0.0);
      cv::cv2eigen(mat_all_, TS_blurred_all_);
      TS_negative_all_ = ceilMat - TS_blurred_all_;
    } else {
      TS_negative_all_ = ceilMat - TS_all_;
    }
  }

  inline void computeTsNegativeGrad() {
    cv::Mat cv_TS_flipped_left, cv_TS_flipped_right, cv_TS_flipped_all;
    cv::eigen2cv(TS_negative_left_, cv_TS_flipped_left);
    cv::eigen2cv(TS_negative_right_, cv_TS_flipped_right);
    cv::eigen2cv(TS_negative_all_, cv_TS_flipped_all);

    cv::Mat cv_dFlippedTS_du_left, cv_dFlippedTS_dv_left, cv_dFlippedTS_du_right, cv_dFlippedTS_dv_right, cv_dFlippedTS_du_all, cv_dFlippedTS_dv_all;
    cv::Sobel(cv_TS_flipped_left, cv_dFlippedTS_du_left, CV_64F, 1, 0);
    cv::Sobel(cv_TS_flipped_left, cv_dFlippedTS_dv_left, CV_64F, 0, 1);

    cv::Sobel(cv_TS_flipped_right, cv_dFlippedTS_du_right, CV_64F, 1, 0);
    cv::Sobel(cv_TS_flipped_right, cv_dFlippedTS_dv_right, CV_64F, 0, 1);

    cv::Sobel(cv_TS_flipped_all, cv_dFlippedTS_du_all, CV_64F, 1, 0);
    cv::Sobel(cv_TS_flipped_all, cv_dFlippedTS_dv_all, CV_64F, 0, 1);
    
    //cv::imshow("cv_dFlippedTS_dv_left", cv_dFlippedTS_dv_left);
    //cv::waitKey(0);

    cv::cv2eigen(cv_dFlippedTS_du_left, dTS_negative_du_left_);
    cv::cv2eigen(cv_dFlippedTS_dv_left, dTS_negative_dv_left_);

    cv::cv2eigen(cv_dFlippedTS_du_right, dTS_negative_du_right_);
    cv::cv2eigen(cv_dFlippedTS_dv_right, dTS_negative_dv_right_);

    cv::cv2eigen(cv_dFlippedTS_du_all, dTS_negative_du_all_);
    cv::cv2eigen(cv_dFlippedTS_dv_all, dTS_negative_dv_all_);
  }
  
  //add by zzz
  Eigen::MatrixXd Distance_Field_positive, Col_ANNF_positive, Row_ANNF_positive,
                  Distance_Field_negative, Col_ANNF_negative, Row_ANNF_negative,
                  Distance_Field_neutral, Col_ANNF_neutral, Row_ANNF_neutral, Distance_Field_neutral_show;
  Eigen::MatrixXd TS_left_, TS_right_, TS_all_;
  Eigen::MatrixXd TS_blurred_left_, TS_blurred_right_, TS_blurred_all_;
  Eigen::MatrixXd TS_negative_left_, TS_negative_right_, TS_negative_all_;
  cv_bridge::CvImagePtr cvImagePtr_left_, cvImagePtr_right_, cvImagePtr_all_;
  Transformation tr_;
  Eigen::MatrixXd dTS_du_left_, dTS_dv_left_, dTS_du_right_, dTS_dv_right_, dTS_du_all_, dTS_dv_all_;
  Eigen::MatrixXd dTS_negative_du_left_, dTS_negative_dv_left_, dTS_negative_du_right_, dTS_negative_dv_right_, dTS_negative_du_all_, dTS_negative_dv_all_;
  size_t id_;
};

struct ROSTimeCmp {
  bool operator()(const ros::Time &a, const ros::Time &b) const { return a.toNSec() < b.toNSec(); }
};

using TimeSurfaceHistory = std::map<ros::Time, TimeSurfaceObservation, ROSTimeCmp>;
using StampedTimeSurfaceObs = std::pair<ros::Time, TimeSurfaceObservation>;

inline static TimeSurfaceHistory::iterator TSHistory_lower_bound(TimeSurfaceHistory &ts_history, ros::Time &t) {
  return std::lower_bound(ts_history.begin(), ts_history.end(), t, [](const std::pair<ros::Time, TimeSurfaceObservation> &tso, const ros::Time &t) { return tso.first.toSec() < t.toSec(); });
}

inline static TimeSurfaceHistory::iterator TSHistory_upper_bound(TimeSurfaceHistory &ts_history, ros::Time &t) {
  return std::upper_bound(ts_history.begin(), ts_history.end(), t, [](const ros::Time &t, const std::pair<ros::Time, TimeSurfaceObservation> &tso) { return t.toSec() < tso.first.toSec(); });
}
} // namespace container
} // namespace esvo_core

#endif // ESVO_CORE_CONTAINER_TIMESURFACEOBSERVATION_H