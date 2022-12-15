#ifndef CANNY_EVT_MINUS_OPTIMIZATION_H
#define CANNY_EVT_MINUS_OPTIMIZATION_H

#include <Eigen/Eigen>
#include <canny_evt_minus/canny_evt_minus_offline.h>
#include <canny_evt_minus/canny_evt_minus_types.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/common/transforms.h>
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
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

namespace canny_evt_minus
{
namespace optimization
{
using namespace types;

class MyPointRepresentation : public pcl::PointRepresentation<pcl::PointNormal>
{
  using pcl::PointRepresentation<pcl::PointNormal>::nr_dimensions_;

public:
  MyPointRepresentation()
  {
    nr_dimensions_ = 4;
  }
  virtual void copyToFloatArray( const pcl::PointNormal &p, float *out ) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};

struct ResidualItem
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p_;     // 3D coordinate in the reference frame
  Eigen::Vector2d p_img_; // 2D coordinate in the image plane

  //  double IRLS_weight_;
  //  double variance_;
  Eigen::VectorXd residual_;
  //  bool bOutlier_;

  ResidualItem();
  ResidualItem( const double x, const double y, const double z );
  void initialize( const double x, const double y, const double z );
};

using ResidualItems = std::vector<ResidualItem, Eigen::aligned_allocator<ResidualItem>>;

struct LM_statics
{
  size_t nPoints_;
  size_t nfev_;
  size_t nIter_;
};
/**
 * Generic functor base for use with the Eigen-nonlinear optimization
 * toolbox. Please refer to the Eigen-documentation for further information.
 */
template <typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic> struct OptimizationFunctor
{
  /** undocumented */
  typedef _Scalar Scalar;
  /** undocumented */
  enum
  {
    InputsAtCompileTime = NX,
    ValuesAtCompileTime = NY
  };
  /** undocumented */
  typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
  /** undocumented */
  typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
  /** undocumented */
  typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

  /** undocumented */
  const int m_inputs;
  /** undocumented */
  int m_values;

  /** undocumented */
  OptimizationFunctor() : m_inputs( InputsAtCompileTime ), m_values( ValuesAtCompileTime ) {}
  /** undocumented */
  OptimizationFunctor( int inputs, int values ) : m_inputs( inputs ), m_values( values ) {}

  /** undocumented */
  int inputs() const { return m_inputs; }
  /** undocumented */
  int values() const { return m_values; }

  void resetNumberValues( int values ) { m_values = values; }
};

/* 2d-3d registration problem definition */
struct RegProblem : public OptimizationFunctor<double>
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /* RAII constructor */
  RegProblem();

  void setProblem( RefFrame *ref, CurFrame *cur );
  void setStochasticSampling( size_t offset, size_t N );
  // function that is inherited from optimization::OptimizationFunctor

  void getWarpingTransformation( Eigen::Matrix4d &warpingTransf, const Eigen::Matrix<double, 6, 1> &x ) const;
  void addMotionUpdate( const Eigen::Matrix<double, 6, 1> &dx );
  void setPose();
  int  operator()( const Eigen::Matrix<double, 6, 1> &x, Eigen::VectorXd &fvec ) const;
  bool patchInterpolation( const Eigen::MatrixXd &img, const Eigen::Vector2d &location, Eigen::MatrixXd &patch ) const;
  bool reprojection( const Eigen::Vector3d &p, const Eigen::Matrix4d &warpingTransf, Eigen::Vector2d &x1_s ) const;
  void world2Cam( const Eigen::Vector3d &p, Eigen::Vector2d &x ) const;
  bool isValidPatch( Eigen::Vector2d &patchCentreCoord, Eigen::MatrixXi &mask, size_t wx, size_t wy ) const;
  int  df( const Eigen::Matrix<double, 6, 1> &x, Eigen::MatrixXd &fjac ) const;
  void computeJ_G( const Eigen::Matrix<double, 6, 1> &x, Eigen::Matrix<double, 12, 6> &J_G );

  Eigen::Matrix3d cayley2rot( const Eigen::Vector3d &cayley ) const;
  // utils
  Eigen::Matrix4d T_ev_ref_;
  Eigen::Matrix4d T_world_ref_;
  Eigen::Matrix4d T_world_ev_;
  Eigen::Matrix3d R_; // R_ref_cur_;
  Eigen::Vector3d t_; // t_ref_cur
  cv::Mat         TS_;
  Eigen::MatrixXd TS_eigen_;
  size_t          numPoints_;
  size_t          numBatches_;
  int             BATCH_SIZE_ = 300;
  size_t          patchSize_  = 1;

  ResidualItems                ResItems_, ResItemsStochSampled_;
  Eigen::Matrix<double, 12, 6> J_G_0_;
  bool                         L2                       = true;
  bool                         Huber                    = !L2;
  int                          MAX_REGISTRATION_POINTS_ = 2000;

  RefFrame *ref_;
  CurFrame *cur_;
};

/* 2d-3d registration problem solver */
class RegProblemSolver
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RegProblemSolver();
  virtual ~RegProblemSolver();

  bool resetRegProblem( RefFrame *ref, CurFrame *cur );
  bool solve_numerical() = delete; /* currently unuesd */
  bool solve_analytical();

  LM_statics                  lmStatics_; // record LevenburgMarquardt log.
  std::shared_ptr<RegProblem> regProblemPtr_;
};

} // namespace optimization
} // namespace canny_evt_minus
#endif