#include "geometry.h"
#include "perspective_camera.h"

size_t PerspectiveCamera::last_id_ = 0;

std::string PerspectiveCamera::getName() const
{
    return name_;
}

void PerspectiveCamera::setName(const std::string& name)
{
    name_ = name;
}

Eigen::Vector2i PerspectiveCamera::getImageSize() const
{
    return Eigen::Vector2i(width_, height_);
}

Eigen::Isometry3d PerspectiveCamera::getPose() const
{
    return pose_;
}

Eigen::Affine3d PerspectiveCamera::getPoseAsAffine3d() const
{
    return egIsometry2egAffine(pose_);
}

void PerspectiveCamera::setPose(const Eigen::Isometry3d& pose)
{
    pose_ = pose;
}

Eigen::Isometry3d PerspectiveCamera::getGTPose() const
{
    return gt_pose_;
}

Eigen::Affine3d PerspectiveCamera::getGTPoseAsAffine3d() const
{
    return egIsometry2egAffine(gt_pose_);
}

void PerspectiveCamera::setGTPose(const Eigen::Isometry3d& gt_pose)
{
    has_gt_ = true;
    gt_pose_ = gt_pose;
}

Eigen::Isometry3d PerspectiveCamera::getOptimizedPose() const
{
    return opt_pose_;
}

Eigen::Affine3d PerspectiveCamera::getOptimizedPoseAsAffine3d() const
{
    return egIsometry2egAffine(opt_pose_);
}

void PerspectiveCamera::setOptimizedPose(const Eigen::Isometry3d& opt_pose)
{
    is_optimized_ = true;
    opt_pose_ = opt_pose;
}

Eigen::Vector3d PerspectiveCamera::getPosition() const
{
    return Eigen::Vector3d(pose_.translation()(0), pose_.translation()(1), pose_.translation()(2));
}

void PerspectiveCamera::setPosition(const Eigen::Vector3d& pos)
{
    pose_.translation() = pos;
}

Eigen::Quaterniond PerspectiveCamera::getOrientation() const
{
    Eigen::Quaterniond q;
    q = pose_.linear();
    return q;
}

void PerspectiveCamera::setOrientation(const double& r, const double& p, const double& y)
{
    Eigen::Isometry3d orien = egIsometryFromEuler(r, p, y);
    pose_.linear() = orien.linear();
}

void PerspectiveCamera::setOrientation(const Eigen::Quaterniond& quat)
{
    pose_.linear() = quat.matrix();
}

Eigen::Vector3d PerspectiveCamera::getGTPosition() const
{
    return Eigen::Vector3d(gt_pose_.translation()(0), gt_pose_.translation()(1), gt_pose_.translation()(2));
}

Eigen::Quaterniond PerspectiveCamera::getGTOrientation() const
{
    Eigen::Quaterniond q;
    q = gt_pose_.linear();
    return q;
}

cv::viz::Camera PerspectiveCamera::getVizCamera() const
{
    return cv::viz::Camera(fx_, fy_, cx_, cy_, cv::Size(width_, height_));
}

cv::Mat PerspectiveCamera::getIntrinsicsAsMat33() const
{
    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    K.at<double>(0,0) = fx_;
    K.at<double>(1,1) = fy_;
    K.at<double>(0,2) = cx_;
    K.at<double>(1,2) = cy_;

    return K;
}

void PerspectiveCamera::setIntrinsics(const double& fx, const double& fy,
                                      const double& cx, const double& cy,
                                      const int& width, const int& height)
{
    fx_ = fx;
    fy_ = fy;
    cx_ = cx;
    cy_ = cy;
    width_ = width;
    height_ = height;
}

std::map<size_t, Eigen::Vector2d> PerspectiveCamera::getProjections() const
{
    return projections_;
}

size_t PerspectiveCamera::getId() const
{
    return id_;
}

Eigen::Vector3d PerspectiveCamera::transformPoint(const Eigen::Vector3d& pt_w) const
{
    return gt_pose_.inverse() * pt_w;
}

Eigen::Vector2d PerspectiveCamera::projectCameraPoint(const Eigen::Vector3d& pt_c) const
{
    Eigen::Vector2d pt_im;
    double X = pt_c[0], Y = pt_c[1], Z = pt_c[2];

    pt_im[0] = fx_ * X/Z + cx_;
    pt_im[1] = fy_ * Y/Z + cy_;

    return pt_im;
}

Eigen::Vector2d PerspectiveCamera::projectWorldPoint(const Eigen::Vector3d& pt_w, bool& is_visible) const
{
    is_visible = false;

    Eigen::Vector3d pt_c = transformPoint(pt_w);
    Eigen::Vector2d pt_im = projectCameraPoint(pt_c);

    if(isVisible(pt_im))
        is_visible = true;

    return pt_im;
}

void PerspectiveCamera::projectAndStoreWorldPoint(const Eigen::Vector3d& pt_w, const size_t& pt_idx)
{
    bool is_visible;
    Eigen::Vector2d pt_im;

    pt_im = projectWorldPoint(pt_w, is_visible);

    //Store point projection in the map if it is visible
    if(is_visible)
    {
        projections_[pt_idx] = pt_im;
    }
}

void PerspectiveCamera::projectAndStoreWorldPoints(const std::vector<Eigen::Vector3d>& world_pts)
{
    for(size_t i = 0; i < world_pts.size(); i++)
    {
        projectAndStoreWorldPoint(world_pts[i], i);
    }
}

cv::Mat PerspectiveCamera::generateImage() const
{
    cv::Mat img = cv::Mat::zeros(height_, width_, CV_8UC3);

    std::map<size_t, Eigen::Vector2d>::const_iterator itr; 
    for(itr = projections_.begin(); itr != projections_.end(); itr++)
    {
        cv::Point pt(itr->second[0], itr->second[1]);
        circle(img, pt, 1, CV_RGB(255, 0, 0), -1);
    }

    return img;
}

bool PerspectiveCamera::hasGT() const
{
    return has_gt_;
}

bool PerspectiveCamera::isOptimized() const
{
    return is_optimized_;
}

double PerspectiveCamera::getSquaredError() const
{
    if(!has_gt_)
    {
        fprintf(stderr, "WARNING: attempt of computing error for a camera without ground truth.\n");
    }
    Eigen::Isometry3d rel_transf = relativeTransform(pose_, gt_pose_);
    return sqEuclidianNorm(rel_transf);
}

double PerspectiveCamera::getOptimizedSquaredError() const
{
    if(!has_gt_)
    {
        fprintf(stderr, "WARNING: attempt of computing error for a camera without ground truth.\n");
    }
    if(!is_optimized_)
    {
        fprintf(stderr, "WARNING: attempt of computing error for a camera without optimized coords.\n");
    }
    Eigen::Isometry3d rel_transf = relativeTransform(opt_pose_, gt_pose_);
    return sqEuclidianNorm(rel_transf);
}

bool PerspectiveCamera::isVisible(const Eigen::Vector2d& pt_im) const
{
    if(pt_im[0] >= 0 && pt_im[0] < width_ && pt_im[1] >= 0 && pt_im[1] < height_)
    {
        return true;
    }
    else
    {
        return false;
    }
}