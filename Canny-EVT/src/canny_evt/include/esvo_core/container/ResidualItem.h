#ifndef ESVO_CORE_CONTAINER_RESIDUALITEM_H
#define ESVO_CORE_CONTAINER_RESIDUALITEM_H

#include <Eigen/Eigen>
#include <memory>
#include <vector>

namespace esvo_core { namespace container {
struct ResidualItem {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p_;     // 3D coordinate in the reference frame
  Eigen::Vector2d p_img_; // 2D coordinate in the image plane
  //added by zzz
  Eigen::Vector3d pg_;     // 3D coordinate in the reference frame(for gradient)
  double weight_;
  Eigen::Vector2d grad_;
  Eigen::Vector2d optical_flow_;
  int polarity_;           // 1:positive -1:negative 0:neutral or unknown;
  //  double IRLS_weight_;
  //  double variance_;
  Eigen::VectorXd residual_;
  //  bool bOutlier_;

  ResidualItem();
  ResidualItem(const double x, const double y, const double z);
  void initialize(const double x, const double y, const double z);
  //added by zzz
  void initialize(const double x, const double y, const double z, const double xg, const double yg, const double zg);
};

using ResidualItems = std::vector<ResidualItem, Eigen::aligned_allocator<ResidualItem>>;
}} // namespace esvo_core::container

#endif // ESVO_CORE_CONTAINER_RESIDUALITEM_H
