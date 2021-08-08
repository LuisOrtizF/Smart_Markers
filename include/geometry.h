#ifndef GEOMETRY_H_
#define GEOMETRY_H_

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>

/**
 * Converts a angle from radians to degrees.
 * @param rad: angle in radians
 */
double toDeg(const double& rad);

/**
 * Converts a angle from degrees to radians.
 * @param deg: angle in degrees
 */
double toRad(const double& deg);

/**
 * Computes the relative transformation between two ref. frames
 * referenced in the first ref. frame.
 * @param from: first ref. frame (origin of the transformation)
 * @param to: second ref. frame (target of the transformation)
 */
Eigen::Isometry3d relativeTransform(const Eigen::Isometry3d& from, const Eigen::Isometry3d& to);

/**
 * Computes the squared Euclidian norm of the translation block
 * in a pose matrix.
 * @param m: an isometry pose matrix
 */
double sqEuclidianNorm(const Eigen::Isometry3d& m);

/**
 * Converts an Eigen::Affine3d object into
 * an OpenCV::Affine3d one.
 * @param eigen_mat: Eigen::Affine3d object to be converted
 */
cv::Affine3d egAffine2cvAffine(const Eigen::Affine3d& eigen_mat);

/**
 * Converts an Eigen::Isometry3d object into
 * a Eigen::Affine3d one.
 * @param m: Eigen::Isometry3d object to be converted
 */
Eigen::Affine3d egIsometry2egAffine(const Eigen::Isometry3d& m);

/**
 * Creates an Isometry3D transform with given translation and orientation.
 * Orientation is informed by roll, pitch and yaw angles.
 * @param r: roll angle (in degress)
 * @param p: pitch angle (in degress)
 * @param y: yaw angle (in degress)
 */
Eigen::Isometry3d egIsometryFromEuler(const double& r, const double& p, const double& y,
                                      const double& tx = 0, const double& ty = 0,
                                      const double& tz = 0);

/**
 * Converts an OpenCV::Affine3d object into
 * a Eigen::Isometry3d one.
 * @param cv_mat: OpenCV::Affine3d object to be converted
 */
Eigen::Isometry3d cvAffine2egIsometry (const cv::Affine3d& cv_mat);

/**
 * Converts an Eigen::Isometry3d object into
 * an Opencv::Affine3d.
 * @param T_eg: Eigen::Isometry3d object to be converted
 */
cv::Affine3d egIsometry2cvAffine (const Eigen::Isometry3d& T_eg);

/**
 * Converts an Opencv::Affine3d object into
 * an Eigen::Quaternion.
 * @param cv_mat: Opencv::Affine3d object to be converted
 */
Eigen::Quaterniond cvAffine2egQuat (const cv::Affine3d& cv_mat);

/**
 * Computes the squared Euclidian distance between two 3D vectors
 * @param v1: first 3D vector
 * @param v2: second 3D vector
 */
double sqEuclidianDistance(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2);

/**
 * Computes the angle of the rotation block in a pose matrix.
 * @param m: an isometry pose matrix
 */
double rotationAngle(const Eigen::Isometry3d& m);

/**
 * Converts an sl::Pose object into
 * an cv::Affine3d.
 * @param sl_pose: ZED API pose object to be converted
 */
cv::Affine3d slPose2cvAffine(sl::Pose &sl_pose);

/**
 * Converts an cv::Mat object into
 * an cv::Affine3d.
 * @param mat: OpenCV cv::Mat pose object to be converted
 */
cv::Affine3d cvMat2cvAffine (const cv::Mat &mat);

#endif /* GEOMETRY_H_ */