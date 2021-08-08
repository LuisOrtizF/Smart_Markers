#ifndef SM_MAPPING_H_
#define SM_MAPPING_H_

#include <vector>

#include <Eigen/Geometry>

#include <g2o/core/sparse_optimizer.h>

#include <opencv2/opencv.hpp>

#include <aruco/aruco.h>
#include "markermapper.h"

class  SMMapping
{
    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * Constructor 
         * @param mm: marker mapper map to be otimized
         * @param pms_file: txt file with pms data 
         */
        SMMapping(const aruco::MarkerMap &mm, const std::string &pms_file, const int &iterations);

        /**
         * Process and return a new marker map from initial aruco map 
         */
        aruco::MarkerMap process();

    private:

        aruco::MarkerMap mm_;
        std::string pms_file_;
        int iterations_,edge_id_;
        std::vector<int> ids_;
        std::vector<cv::Point3d> mm_contours_;
        std::vector<cv::Point3d> mm_centers_;
        std::vector<cv::Affine3d> mm_poses_;
        std::vector<Eigen::Isometry3d> mm_poses_iso;
        std::vector<cv::Point3d> pms_centers_;
        std::vector<cv::Affine3d> pms_poses_;
        std::vector<Eigen::Isometry3d> pms_poses_iso;
        std::vector<cv::Affine3d> mm_T_;
        std::vector<Eigen::Isometry3d> mm_T_iso;
        std::vector<cv::Affine3d> opt_poses_;
        std::vector<Eigen::Matrix<double, 6, 6>> mm_info_matrix_;
        std::vector<cv::Point3d> opt_contours_;
        aruco::MarkerMap opt_map_;

        // G2O nonlinear optimizer
        g2o::SparseOptimizer optimizer_;

        cv::Affine3d computeMarkerPose (const std::vector<cv::Point3f> &points3D_f);
        
        // Compute all POSES (translation wtr origin of the markermap), 3D contour and centers, for all markers in the markermap
        void computeMMData ();
        
        // Read (POSES IN QUATERNION FORMAT) PMS measurements (translation wtr origin of the markermap) for each marker in the markermap
        void computePMSData ();
        
        // Compute all transformations between all markers in the markermap
        void computeMMTrasformations ();

        std::vector<Eigen::Matrix<double, 6, 6>> computeMarkersInfoMatrix ();

        // Configures the nonlinear sparse optimizer
        void setupOptimizer ();
        void computeMMDataIsometry();
        void buildMarkersVertices ();
        void buildPMSVertices ();
        void otimize ();
        void optimization ();

        void computeOptMMContours ();

        void createOptMM ();
};

#endif /* SM_MAPPING_H_ */