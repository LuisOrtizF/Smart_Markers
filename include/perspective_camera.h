#ifndef PERSPECTIVE_CAMERA_H_
#define PERSPECTIVE_CAMERA_H_

#include <iostream>
#include <map>

#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>

/*
 * Represents a pinhole perspective camera.
 */
class PerspectiveCamera
{
    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /** 
         * Builds a perspective camera
         * with given intrinsic parameters and
         * image size.
         * column/row according to image size.
         * @param name: camera name
         * @param fx: focal distance in hor. dir.
         * @param fy: focal distance in ver. dir.
         * @param cx: projection center in hor. dir.
         * @param cy: projection center in ver. dir.
         * @param width: image width
         * @param height: image height
         */
        PerspectiveCamera(const std::string& name,
                        const double& fx, const double& fy,
                        const double& cx, const double& cy,
                        const int& width, const int& height):
        name_(name), fx_(fx), fy_(fy), cx_(cx), cy_(cy), width_(width), height_(height)
        {
            id_ = PerspectiveCamera::last_id_++;
            pose_ = Eigen::Isometry3d::Identity();
            gt_pose_ = Eigen::Isometry3d::Identity();
            opt_pose_ = Eigen::Isometry3d::Identity();
            has_gt_ = false;
            is_optimized_ = false;
        }

        /** 
         * Returns the camera name.
         */
        std::string getName() const;

        /**
         * Sets the camera name.
         * @param name: camera name
         */
        void setName(const std::string& name);

        /**
         * Returns the image size of the
         * camera in a Eigen::Vector2i
         * with width_ and height_ in this order.
         */
        Eigen::Vector2i getImageSize() const;

        /**
         * Returns the camera pose.
         */
        Eigen::Isometry3d getPose() const;

        /**
         * Returns the camera pose as Eigen::Affine3d.
         */
        Eigen::Affine3d getPoseAsAffine3d() const;

        /**
         * Sets the rotation/translation of the camera w.r.t
         * the world reference frame.
         * @param pose: camera pose
         */
        void setPose(const Eigen::Isometry3d& pose);

        /**
         * Returns the ground truth camera pose.
         */
        Eigen::Isometry3d getGTPose() const;

        /**
         * Returns the ground truth camera pose
         * as Eigen::Affine3d.
         */
        Eigen::Affine3d getGTPoseAsAffine3d() const;

        /**
         * Sets the ground truth camera pose.
         * @param gt_pose: ground truth pose
         */
        void setGTPose(const Eigen::Isometry3d& gt_pose);

        /**
         * Returns the optimized camera pose.
         */
        Eigen::Isometry3d getOptimizedPose() const;

        /**
         * Returns the optimized pose
         * as Eigen::Affine3d.
         */
        Eigen::Affine3d getOptimizedPoseAsAffine3d() const;

        /**
         * Sets the optimized camera pose.
         * @param opt_pose: optimized pose
         */
        void setOptimizedPose(const Eigen::Isometry3d& opt_pose);

        /**
         * Returns the camera position as a 3D vector.
         */
        Eigen::Vector3d getPosition() const;

        /**
         * Sets the position of the camera in the world.
         */
        void setPosition(const Eigen::Vector3d& pos);

        /**
         * Returns the camera orientation as a quaternion.
         */
        Eigen::Quaterniond getOrientation() const;

        /**
         * Sets the orientation of the camera in the world
         * from angles around the Z (roll), X (pitch) and Y (yaw)
         * axes.
         * @param r: roll angle (around Z axis) in degrees
         * @param p: pitch angle (around X axis) in degrees
         * @param y: yaw angle (around Y axis) in degrees
         */
        void setOrientation(const double& r, const double& p, const double& y);

        /**
         * Sets the orientation of the camera in the world
         * from given quaternion.
         * @param quat: quaternion with desired orientation
         */
        void setOrientation(const Eigen::Quaterniond& quat);

        /**
         * Returns the ground truth position as a 3D vector.
         */
        Eigen::Vector3d getGTPosition() const;

        /**
         * Returns the ground truth orientation as a quaternion.
         */
        Eigen::Quaterniond getGTOrientation() const;

        /**
         * Returns a virtual camera associated with this
         * perspective camera.
         */
        cv::viz::Camera getVizCamera() const;

        /**
         * Returns a 3x3 cv::Mat with the intrinsic parameters.
         */
        cv::Mat getIntrinsicsAsMat33() const;
    
        /**
         * Sets the intrinsic parameters of the camera.
         * @param fx: focal distance in hor. dir.
         * @param fy: focal distance in ver. dir.
         * @param cx: projection center coordinate in hor. dir.
         * @param cy: projection center coordinate in ver. dir.
         * @param width: image width
         * @param height: image height
         */
        void setIntrinsics(const double& fx, const double& fy,
                        const double& cx, const double& cy,
                        const int& width, const int& height);

        /**
         * Returns all 2D points that are projected
         * into the camera in the same format as stored.
         */
        std::map<size_t, Eigen::Vector2d> getProjections() const;

        /**
         * Returns the camera id.
         * NOTE: this is currently being used as timestep,
         * i.e. it is assumed that consecutive ids = consecutive timesteps.
         */
        size_t getId() const;

        /**
         * Transforms a 3D point from world coordinates
         * to camera coordinates according to camera pose.
         * @param pt_w: 3D point in world coordinates
         */
        Eigen::Vector3d transformPoint(const Eigen::Vector3d& pt_w) const;

        /**
         * Projects a 3D point in camera coordinates 
         * into image coordinates according to camera intrinsic parameters.
         * @param pt_c: 3D point in camera coordinates
         */
        Eigen::Vector2d projectCameraPoint(const Eigen::Vector3d& pt_c) const;

        /**
         * Projects a 3D point in world coordinates 
         * into image coordinates according to camera intrinsic parameters.
         * @param pt_w: 3D point in world coordinates
         * @param is_visible (output): true if the point is within the image limits
         */
        Eigen::Vector2d projectWorldPoint(const Eigen::Vector3d& pt_w, bool& is_visible) const;

        /**
         * Projects a 3D point in world coordinates 
         * into image coordinates according to camera intrinsic parameters.
         * The 2D point is stored into the container of all projections that are
         * visible in the camera.
         * @param pt_w: 3D point in world coordinates
         * @param pt_idx: index of the 3D point
         */
        void projectAndStoreWorldPoint(const Eigen::Vector3d& pt_w, const size_t& pt_idx);

        /**
         * Projects a set of 3D points in world ref. frame 
         * into image coordinates according to camera intrinsic parameters.
         * The projections are stored within the class instance.
         * @param world_points: vector of 3D points in world coordinates
         */
        void projectAndStoreWorldPoints(const std::vector<Eigen::Vector3d>& world_points);

        /**
         * Generates a .png image with all projected points
         */
        cv::Mat generateImage() const;

        /**
         * Returns true if the camera has ground truth and
         * false otherwise.
         */
        bool hasGT() const;

        /**
         * Returns true if the camera has an associated optimized pose and
         * false otherwise.
         */
        bool isOptimized() const;

        /**
         * Returns the squared error of the camera pose compared
         * to its ground truth.
         */
        double getSquaredError() const;

        /**
         * Returns the squared error of the optimized
         * camera compared to its ground truth.
         */
        double getOptimizedSquaredError() const;

        /**
         * Utility function used to check if a 2D point
         * is within image limits.
         * @param pt_im: 2D coordinates of a point
         */
        bool isVisible(const Eigen::Vector2d& pt_im) const;

    private:

        //Camera name
        std::string name_;

        //Last created ID
        static size_t last_id_;

        //Camera ID
        size_t id_;

        //2D projections of the 3D points related to the camera (in pixels)
        //as a map point_idx -> 2D projection
        std::map<size_t, Eigen::Vector2d> projections_;

        //Camera pose in world coordinates
        Eigen::Isometry3d pose_;

        //Ground truth of camera pose
        Eigen::Isometry3d gt_pose_;

        //Camera pose after optimization
        Eigen::Isometry3d opt_pose_;

        //Camera intrinsic parameters
        double fx_; //focal dist. along x dir.
        double fy_; //focal dist. along y dir.
        double cx_; //projection center x coord.
        double cy_; //projection center y coord.
        int width_; //image width
        int height_; //image height

        //Informs if the camera has an associated ground truth pose
        bool has_gt_;

        //Informs if the camera has an associated optimized pose
        bool is_optimized_;
};

#endif /*PERSPECTIVE_CAMERA_H_*/