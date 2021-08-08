#include "geometry.h"

double toDeg(const double& rad)
{
    return rad*180.0/M_PI;
}

double toRad(const double& deg)
{
    return deg*M_PI/180.0;
}

Eigen::Isometry3d relativeTransform(const Eigen::Isometry3d& from, const Eigen::Isometry3d& to)
{
    return from.inverse() * to;
}

double sqEuclidianNorm(const Eigen::Isometry3d& m)
{
    Eigen::Vector3d trans = m.translation();
    return trans.dot(trans);
}

cv::Affine3d egAffine2cvAffine(const Eigen::Affine3d& eigen_mat)
{
    //Create cv::Affine3d from Eigen::Affine3d raw data
    cv::Affine3d result(eigen_mat.data());
    //Transpose the result since cv::Affine3d is row major
    result.matrix = result.matrix.t();
    return result;
}

Eigen::Affine3d egIsometry2egAffine(const Eigen::Isometry3d& m)
{
    Eigen::Affine3d r;
    r.linear() = m.linear();
    r.translation() = m.translation();
    return r;
}

Eigen::Isometry3d egIsometryFromEuler(const double& r, const double& p, const double& y,
                                      const double& tx, const double& ty, const double& tz)
{
    Eigen::Isometry3d pose;
    Eigen::AngleAxisd rollTransf(toRad(r), Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitchTransf(toRad(p), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd yawTransf(toRad(y), Eigen::Vector3d::UnitY());
    pose = rollTransf*pitchTransf*yawTransf;
    pose.translation() = Eigen::Vector3d(tx, ty, tz);
    return pose;
}

Eigen::Isometry3d cvAffine2egIsometry(const cv::Affine3d& cv_mat)
{
    cv::Matx33d R33_cv = cv_mat.rotation();
    cv::Vec3d t_cv = cv_mat.translation();
    Eigen::Matrix3d R33_eg;
    for(int32_t i = 0; i < 3; i++)
        for(int32_t j = 0; j < 3; j++)
            R33_eg(i,j) = R33_cv(i,j);
    Eigen::Vector3d t_eg;
    for(int32_t k = 0; k < 3; k++)
        t_eg(k) = t_cv.val[k];
    Eigen::Isometry3d T_eg;
    T_eg.linear() = R33_eg;
    T_eg.translation() = t_eg;
    return T_eg;
}

cv::Affine3d egIsometry2cvAffine (const Eigen::Isometry3d& T_eg)
{
    cv::Affine3d T_cv;
    Eigen::Matrix3d R33_eg = T_eg.linear();
    cv::Matx33d R33_cv;
    for(int32_t i = 0; i < 3; i++)
        for(int32_t j = 0; j < 3; j++)
            R33_cv(i,j) = R33_eg(i,j);
    T_cv.rotation(R33_cv);
    Eigen::Vector3d t_eg = T_eg.translation();
    cv::Vec3d t_cv;

    for(int32_t k = 0; k < 3; k++)
        t_cv.val[k] = t_eg(k);
    T_cv.translation(t_cv);
    return T_cv;
}

Eigen::Quaterniond cvAffine2egQuat (const cv::Affine3d& cv_mat)
{
    cv::Matx33d R33_cv = cv_mat.rotation();
    Eigen::Matrix3d R33_eg;
    for(int32_t i = 0; i < 3; i++)
        for(int32_t j = 0; j < 3; j++) 
            R33_eg(i,j) = R33_cv(i,j);
    Eigen::Quaterniond q (R33_eg);
    return q;
}

double sqEuclidianDistance(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2)
{
   Eigen::Vector3d diff = v1 - v2;
   return diff.dot(diff);
}

double rotationAngle(const Eigen::Isometry3d& m)
{
   double trace = m.linear().trace();
   return acos(std::min(1.0, std::max(-1.0, (trace - 1)/2.0)));
}

cv::Affine3d slPose2cvAffine(sl::Pose &sl_pose)
{
    sl::Matrix3f R33_zed = sl_pose.getRotationMatrix();
    cv::Matx33d R33_zed_cv;
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            R33_zed_cv(i,j) = R33_zed.r[3*i+j];
    double tx_zed = sl_pose.getTranslation().tx;
    double ty_zed = sl_pose.getTranslation().ty;
    double tz_zed = sl_pose.getTranslation().tz;
    cv::Vec3d t_zed (tx_zed, ty_zed, tz_zed);
    cv::Affine3d T_zed(R33_zed_cv, t_zed);
    return T_zed;
}

cv::Affine3d cvMat2cvAffine (const cv::Mat &mat)
{
    assert(mat.total()==16);
    cv::Mat mat_inv;
    mat.convertTo(mat_inv, CV_64F);
    double tx = mat_inv.at<double>(0,3);
    double ty = mat_inv.at<double>(1,3);
    double tz = mat_inv.at<double>(2,3);
    // printf("\t\ttvec_Our: Tx: %.3f, Ty: %.3f, Tz: %.3f \n", tx_our, ty_our, tz_our);
    cv::Matx33d R33;
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            R33(i,j) = mat_inv.at<double>(i, j);
    cv::Vec3d t (tx, ty, tz);
    cv::Affine3d T;
    T.rotation(R33);
    T.translation(t);
    return T;
}