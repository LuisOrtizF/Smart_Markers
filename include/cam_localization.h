#ifndef CAM_LOCALIZATION_H_
#define CAM_LOCALIZATION_H_

#include <Eigen/Geometry>
#include <aruco/aruco.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

//Solvers
#include <g2o/solvers/dense/linear_solver_dense.h>
#include "g2o/solvers/csparse/linear_solver_csparse.h"

//Utility includes
#include <g2o/stuff/sampler.h>
#include <g2o/stuff/command_args.h>

//Classes for used vertices and edges in the optimization
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>

//g2o
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/edge_se3_prior.h"

class  CamLocalization
{
    public:
    
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * Constructor 
         * @param markersMap: smart marker map 
         * @param poseTracker: aruco tracker
         * @param iterations: number of iterations for otimization
         */
        CamLocalization(const aruco::MarkerMap &markersMap, const aruco::MarkerMapPoseTracker &poseTracker, const int &iterations);
        cv::Affine3d process(const std::vector<aruco::Marker> &detectedMarkers);

    private:

        aruco::MarkerMap markersMap_;
        aruco::MarkerMapPoseTracker poseTracker_;
        int iterations_;
        std::vector<int> ids_;
        std::vector<cv::Affine3d> mm_poses_;

        // G2O nonlinear optimizer
        g2o::SparseOptimizer optimizer_;

        void setupOtimizer();

        void computeMMData ();

        cv::Affine3d computeMarkerPose (const std::vector<cv::Point3f> &points3D_f);

        cv::Affine3d optimizeCam ( const cv::Affine3d &T_cam_all_m, 
                                   const std::vector <cv::Affine3d> &local_poses_m, 
                                   const std::vector<cv::Affine3d> &local_m_cam);
};

#endif /* CAM_LOCALIZATION_H_ */