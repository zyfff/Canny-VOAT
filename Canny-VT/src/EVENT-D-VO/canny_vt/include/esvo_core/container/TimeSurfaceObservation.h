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
  TimeSurfaceObservation(cv_bridge::CvImagePtr &left, cv_bridge::CvImagePtr &right, Transformation &tr, size_t id, bool bCalcTsGradient = false) : tr_(tr), id_(id) {
    //cv::cv2eigen(left->image, TS_left_);
    //cv::cv2eigen(right->image, TS_right_);
    cv::Mat cv_dTS_du_left, cv_dTS_dv_left;
    cv::Sobel(left->image, cv_dTS_du_left, CV_64F, 1, 0);
    cv::Sobel(left->image, cv_dTS_dv_left, CV_64F, 0, 1);
    cv::cv2eigen(cv_dTS_du_left, dTS_du_left_);
    cv::cv2eigen(cv_dTS_dv_left, dTS_dv_left_);


    cv::Canny(left->image, left->image, 30, 120);
    cv::Canny(right->image, right->image, 30, 120);

    left->image.convertTo(left->image, CV_32F);
    left->image = 255 - left->image;
    right->image = 255 - right->image;
cv::Mat tmp = left->image;
    for(int i = 3; i<left->image.rows-3; i++){
      for(int j = 3; j<left->image.cols-3; j++){
        if(tmp.at<float>(i,j) == 255){
          left->image.at<float>(i-1,j) = 255;
          left->image.at<float>(i+1,j) = 255;
          left->image.at<float>(i,j-1) = 255;
          left->image.at<float>(i,j+1) = 255;

          left->image.at<float>(i-1,j-1) = 255;
          left->image.at<float>(i+1,j+1) = 255;
          left->image.at<float>(i+1,j-1) = 255;
          left->image.at<float>(i-1,j+1) = 255;

          left->image.at<float>(i-2,j) = 255;
          left->image.at<float>(i+2,j) = 255;
          left->image.at<float>(i,j-2) = 255;
          left->image.at<float>(i,j+2) = 255;

          left->image.at<float>(i-2,j-2) = 255;
          left->image.at<float>(i+2,j+2) = 255;
          left->image.at<float>(i+2,j-2) = 255;
          left->image.at<float>(i-2,j+2) = 255;

          left->image.at<float>(i-2,j-1) = 255;
          left->image.at<float>(i+2,j-1) = 255;
          left->image.at<float>(i+1,j-2) = 255;
          left->image.at<float>(i+1,j+2) = 255;

          left->image.at<float>(i-3,j) = 255;
          left->image.at<float>(i+3,j) = 255;
          left->image.at<float>(i,j-3) = 255;
          left->image.at<float>(i,j+3) = 255;

          left->image.at<float>(i-3,j-3) = 255;
          left->image.at<float>(i+3,j+3) = 255;
          left->image.at<float>(i+3,j-3) = 255;
          left->image.at<float>(i-3,j+3) = 255;

          left->image.at<float>(i-3,j-2) = 255;
          left->image.at<float>(i+3,j+2) = 255;
          left->image.at<float>(i+2,j-3) = 255;
          left->image.at<float>(i+2,j+3) = 255;

          left->image.at<float>(i-3,j-1) = 255;
          left->image.at<float>(i+3,j-1) = 255;
          left->image.at<float>(i-1,j-3) = 255;
          left->image.at<float>(i-1,j+3) = 255;
        }
      }
    }

    cvImagePtr_left_ = left;
    cvImagePtr_right_ = right;

    cv::cv2eigen(left->image, TS_left_);
    cv::cv2eigen(right->image, TS_right_);

    if (bCalcTsGradient) {
#ifdef TIME_SURFACE_OBSERVATION_LOG
      TicToc tt;
      tt.tic();
#endif
      // cv::Mat cv_dTS_du_left, cv_dTS_dv_left;
      // cv::Sobel(left->image, cv_dTS_du_left, CV_64F, 1, 0);
      // cv::Sobel(left->image, cv_dTS_dv_left, CV_64F, 0, 1);
      // cv::cv2eigen(cv_dTS_du_left, dTS_du_left_);
      // cv::cv2eigen(cv_dTS_dv_left, dTS_dv_left_);
#ifdef TIME_SURFACE_OBSERVATION_LOG
      LOG(INFO) << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@Sobel computation (" << id_ << ") takes " << tt.toc() << " ms.";
#endif
    }
  }

  // override version without initializing the transformation in the constructor.
  TimeSurfaceObservation(cv_bridge::CvImagePtr &left, cv_bridge::CvImagePtr &right, size_t id, bool bCalcTsGradient = false) : id_(id) {
    cv::Mat cv_dTS_du_left, cv_dTS_dv_left;
    cv::Mat cv_dTS_du_right, cv_dTS_dv_right;
    cv::Sobel(left->image, cv_dTS_du_left, CV_64F, 1, 0);
    cv::Sobel(left->image, cv_dTS_dv_left, CV_64F, 0, 1);

    cv::cv2eigen(cv_dTS_du_left, dTS_du_left_);
    cv::cv2eigen(cv_dTS_dv_left, dTS_dv_left_);
    
    cv::Canny(left->image, left->image, 40, 130);
    cv::Canny(right->image, right->image, 40, 130);

    // left->image.convertTo(left->image, CV_32F, 1.0/255.0);

    left->image = 255 - left->image;
    right->image = 255 - right->image;
    // std::cout<<left->image<<std::endl;
    // getchar();
    // cv::Mat tmp;
    // left->image.copyTo(tmp);
    // for(int i = 3; i<left->image.rows-3; i++){
    //   for(int j = 3; j<left->image.cols-3; j++){
    //     if(tmp.at<float>(i,j) == 1){
    //       left->image.at<float>(i,j) = 255;

    //       left->image.at<float>(i-1,j) = 255;
    //       left->image.at<float>(i+1,j) = 255;
    //       left->image.at<float>(i,j-1) = 255;
    //       left->image.at<float>(i,j+1) = 255;

    //       left->image.at<float>(i-1,j-1) = 255;
    //       left->image.at<float>(i+1,j+1) = 255;
    //       left->image.at<float>(i+1,j-1) = 255;
    //       left->image.at<float>(i-1,j+1) = 255;

    //       left->image.at<float>(i-2,j) = 255;
    //       left->image.at<float>(i+2,j) = 255;
    //       left->image.at<float>(i,j-2) = 255;
    //       left->image.at<float>(i,j+2) = 255;

    //       left->image.at<float>(i-2,j-2) = 255;
    //       left->image.at<float>(i+2,j+2) = 255;
    //       left->image.at<float>(i+2,j-2) = 255;
    //       left->image.at<float>(i-2,j+2) = 255;

    //       left->image.at<float>(i-2,j-1) = 255;
    //       left->image.at<float>(i+2,j-1) = 255;
    //       left->image.at<float>(i+1,j-2) = 255;
    //       left->image.at<float>(i+1,j+2) = 255;

    //       // left->image.at<float>(i-3,j) = 255;
    //       // left->image.at<float>(i+3,j) = 255;
    //       // left->image.at<float>(i,j-3) = 255;
    //       // left->image.at<float>(i,j+3) = 255;

    //       // left->image.at<float>(i-3,j-3) = 255;
    //       // left->image.at<float>(i+3,j+3) = 255;
    //       // left->image.at<float>(i+3,j-3) = 255;
    //       // left->image.at<float>(i-3,j+3) = 255;

    //       // left->image.at<float>(i-3,j-2) = 255;
    //       // left->image.at<float>(i+3,j+2) = 255;
    //       // left->image.at<float>(i+2,j-3) = 255;
    //       // left->image.at<float>(i+2,j+3) = 255;

    //       // left->image.at<float>(i-3,j-1) = 255;
    //       // left->image.at<float>(i+3,j-1) = 255;
    //       // left->image.at<float>(i-1,j-3) = 255;
    //       // left->image.at<float>(i-1,j+3) = 255;
    //       // // std::cout<<"Yes"<<std::endl;
    //     }
    //   }
    // }
//std::cout<<left->image<<std::endl;

    // for(int i = 1; i<right->image.rows-1; i++){
    //   for(int j = 1; j<right->image.cols-1; j++){
    //     if(right->image.at<float>(i,j) == 255){
    //       right->image.at<float>(i-1,j) = 255;
    //       right->image.at<float>(i+1,j) = 255;
    //       right->image.at<float>(i,j-1) = 255;
    //       right->image.at<float>(i,j+1) = 255;
    //     }
    //   }
    // }

    // left->image = 255 - left->image;
    // right->image = 255 - right->image;
    cvImagePtr_left_ = left;
    cvImagePtr_right_ = right;
    cv::cv2eigen(left->image, TS_left_);
    cv::cv2eigen(right->image, TS_right_);

    if (bCalcTsGradient) {
#ifdef TIME_SURFACE_OBSERVATION_LOG
      TicToc tt;
      tt.tic();
#endif
      // cv::Mat cv_dTS_du_left, cv_dTS_dv_left;
      // cv::Mat cv_dTS_du_right, cv_dTS_dv_right;
      // cv::Sobel(left->image, cv_dTS_du_left, CV_64F, 1, 0);
      // cv::Sobel(left->image, cv_dTS_dv_left, CV_64F, 0, 1);

      // cv::cv2eigen(cv_dTS_du_left, dTS_du_left_);
      // cv::cv2eigen(cv_dTS_dv_left, dTS_dv_left_);

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

  inline bool isEmpty() {
    if (TS_left_.rows() == 0 || TS_left_.cols() == 0 || TS_right_.rows() == 0 || TS_right_.cols() == 0)
      return true;
    else
      return false;
  }

  inline void setTransformation(Transformation &tr) { tr_ = tr; }

  inline void GaussianBlurTS(size_t kernelSize) {
    cv::Mat mat_left_, mat_right_;
    cv::GaussianBlur(cvImagePtr_left_->image, mat_left_, cv::Size(kernelSize, kernelSize), 0.0);
    cv::GaussianBlur(cvImagePtr_right_->image, mat_right_, cv::Size(kernelSize, kernelSize), 0.0);
    cv::cv2eigen(mat_left_, TS_left_);
    cv::cv2eigen(mat_right_, TS_right_);
  }

  inline void getTimeSurfaceNegative(size_t kernelSize) {
    // std::cout<<"000000000000000000000000000000000000"<<std::endl;
    Eigen::MatrixXd ceilMat(TS_left_.rows(), TS_left_.cols());
    ceilMat.setConstant(255.0);
    // if (kernelSize > 0) {
    //   cv::Mat mat_left_;
    //   cv::GaussianBlur(cvImagePtr_left_->image, mat_left_, cv::Size(kernelSize, kernelSize), 0.0);
    //   cv::cv2eigen(mat_left_, TS_blurred_left_);
    //   TS_negative_left_ = ceilMat - TS_blurred_left_;
    // } else {
    //   TS_negative_left_ = ceilMat - TS_left_;
    // }
    TS_negative_left_ = ceilMat - TS_left_;

    int radius = 20;
    // int radius1 = 30;
    int r = TS_negative_left_.rows();
    int c = TS_negative_left_.cols();
    Eigen::MatrixXd Distance_Field;
    Distance_Field.resize(r,c);
    Distance_Field.fill(255);
    #pragma omp parallel for
    for(int i=0; i<r; i++){
      for(int j=0; j<c; j++){
        if(TS_negative_left_(i,j) != 255){
          BuildDF(i,j,radius, Distance_Field);
          // BuildDF1(i,j,radius1, Distance_Field);
        }
      }
    }
    TS_blurred_left_ = Distance_Field;
    kernelSize = 0;
if (kernelSize > 0) {
      cv::Mat mat_left_;
      cv::Mat mat_df;
      cv::eigen2cv(Distance_Field, mat_df);
      cv::GaussianBlur(mat_df, mat_left_, cv::Size(kernelSize, kernelSize), 0.0);
      cv::cv2eigen(mat_left_, TS_blurred_left_);
    }

    TS_negative_left_ = TS_blurred_left_;
    Distance_Field_ = TS_negative_left_;

    // Eigen::MatrixXd tmp = TS_negative_left_;
    // // std::cout<<tmp<<std::endl;
    // for(int i = 5; i<tmp.rows()-5; i++){
    //   for(int j = 5; j<tmp.cols()-5; j++){
    //     if(tmp(i,j) != 255){
    //       for(int k=i-5; k<=i+5; k++){
    //         for(int g=j-5; g<=j+5; g++){
    //           TS_negative_left_(k,g) = tmp(i,j);
    //           // std::cout<<"11111111111111111111111111111"<<std::endl;
    //         }
    //       }
    //     }
    //   }
    // }
  }

  inline void BuildDF(int row, int col, int radius, Eigen::MatrixXd &Distance_Field){
    int r = TS_negative_left_.rows();
    int c = TS_negative_left_.cols();
    int start_r = row - radius;
    int start_c = col - radius;
    int end_r = row + radius;
    int end_c = col + radius;
    int n = 255/(radius * 1.5);
    #pragma omp parallel for
    for(int i=start_r; i<=end_r; i++){
      for(int j=start_c; j<=end_c; j++){
        if(i<0 || j<0 || i>=r || j>=c){
          continue;
        }
        double d = n * sqrt((i-row)*(i-row) + (j-col)*(j-col));
        // double d = (i-row)*(i-row) + (j-col)*(j-col);

        if(d < Distance_Field(i,j)){
          Distance_Field(i,j) = d;
        }
      }
    }
  }

  inline void BuildDF1(int row, int col, int radius, Eigen::MatrixXd &Distance_Field){
    int r = TS_negative_left_.rows();
    int c = TS_negative_left_.cols();
    int start_r = row - radius;
    int start_c = col - radius;
    int end_r = row + radius;
    int end_c = col + radius;
    for(int i=start_r; i<=end_r; i++){
      for(int j=start_c; j<=end_c; j++){
        if(i<0 || j<0 || i>=r || j>=c){
          continue;
        }
        double d = sqrt((i-row)*(i-row) + (j-col)*(j-col));
        // double d = (i-row)*(i-row) + (j-col)*(j-col);

        if(5*d < Distance_Field(i,j)){
          Distance_Field(i,j) = 5*d;
        }
      }
    }
  }

  inline void computeTsNegativeGrad() {
    cv::Mat cv_TS_flipped_left;
    cv::eigen2cv(TS_negative_left_, cv_TS_flipped_left);

    cv::Mat cv_dFlippedTS_du_left, cv_dFlippedTS_dv_left;
    cv::Sobel(cv_TS_flipped_left, cv_dFlippedTS_du_left, CV_64F, 1, 0);
    cv::Sobel(cv_TS_flipped_left, cv_dFlippedTS_dv_left, CV_64F, 0, 1);

    cv::cv2eigen(cv_dFlippedTS_du_left, dTS_negative_du_left_);
    cv::cv2eigen(cv_dFlippedTS_dv_left, dTS_negative_dv_left_);
  }

  Eigen::MatrixXd Distance_Field_;
  Eigen::MatrixXd TS_left_, TS_right_;
  Eigen::MatrixXd TS_blurred_left_;
  Eigen::MatrixXd TS_negative_left_;
  cv_bridge::CvImagePtr cvImagePtr_left_, cvImagePtr_right_;
  Transformation tr_;
  Eigen::MatrixXd dTS_du_left_, dTS_dv_left_;
  Eigen::MatrixXd dTS_negative_du_left_, dTS_negative_dv_left_;
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