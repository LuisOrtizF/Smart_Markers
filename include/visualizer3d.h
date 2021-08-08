#ifndef VISUALIZER3D_H_
#define VISUALIZER3D_H_

#include <cstring>

#include <Eigen/Geometry>

#include <opencv2/core.hpp>
#include <opencv2/viz.hpp>

#include <aruco/aruco.h>

#include "perspective_camera.h"

class Visualizer3D
{
    public:
        
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * Constructor with the window title as a string
         * @param title: window title 
         */
        Visualizer3D(const std::string& title);

        /**
         * Sets the intrinsic parameters of the virtual camera.
         * @param cam: object with the intrinsic parameters 
         */
        void setViewerIntrinsics(const cv::viz::Camera& cam);

        /**
         * Sets the pose of the virtual camera.
         * @param pose: desired camera pose
         */
        void setViewerPose(const cv::Affine3d& pose);

        /** 
        * Adds a ZX grid to the 3D scene.
        * @param nz: number of cells along z dir.
        * @param nx: number of cells along x dir.
        * @param sz: size of cell along z dir.
        * @param sx: size of cell along x dir.
        * @param color: color of the grid lines.
        */
        void addZXGrid(const size_t& nz, const size_t& nx, const double& sz, const double& sx, const cv::viz::Color& color);

        /**
         * Adds a ref. frame with the given pose to the 3D scene
         * @param pose: ref. frame pose
         * @param text: ref. frame info text
         */
        void addReferenceFrame(const cv::Affine3d& pose, const std::string& text);

        /**
         * Adds a camera with the parameters given
         * by the perspective cam to the 3D scene.
         * @param cam: perspective camera object
         * @param color: perspective camera color
         */
        void addCamera(PerspectiveCamera& cam, const cv::viz::Color& color);

        /** 
         * Adds a path+frame to the 3D scene.
         * @param from: init pose
         * @param to: final pose
         * @param name: path and line name
         * @param color: color of the line
         */
        void addTrajectory(const cv::Affine3d& from, const cv::Affine3d& to, const std::string& name, const cv::viz::Color& color);

        /**
         * Updates a camera with the parameters given
         * by the perspective in the 3D scene.
         * @param cam: perspective camera object
         * @param color: perspective camera color
         */
        void updateCamera(PerspectiveCamera& cam, const cv::viz::Color& color);

        /**
         * Add Aruco Marker Map
         * @param mm: Aruco Marker Map
         * @param name: name of map
         * @param color: color of markers
         */
        void addMarkerMap(const aruco::MarkerMap & mm, const std::string& name, const cv::viz::Color& color);
        
        /**
         * Remove widget from the 3d scene
         * @param name: name of widget to remove
         */
        void removeWidget(const std::string& name);

        /**
         * The window renders and starts the event loop.
         */
        void spin();

        /**
         * Starts the event loop for a given time.
         * @param time: Amount of time in milliseconds for the event loop to keep running.
         * @param force_redraw: If true, window renders.
         */
        void spinOnce(const int& time = 1, const bool& force_redraw = false);

        /**
         * Returns whether the event loop has been stopped.
         */
        bool wasStopped();

        // OpenCV::viz 3D visualizer
        cv::Ptr<cv::viz::Viz3d> viewer_;
};

#endif /* VISUALIZER3D_H_ */