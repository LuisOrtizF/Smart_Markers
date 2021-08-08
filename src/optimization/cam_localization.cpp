#include "geometry.h"
#include "cam_localization.h"

CamLocalization::CamLocalization (const aruco::MarkerMap &markersMap, const aruco::MarkerMapPoseTracker &poseTracker, const int &iterations)
{
    markersMap_ = markersMap;
    poseTracker_ = poseTracker;
    iterations_ = iterations;
    setupOtimizer();
    computeMMData();
}

void CamLocalization::setupOtimizer()
{
    // create the linear solver
    auto linearSolver = g2o::make_unique<g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>>();
    // create the block solver on top of the linear solver
    auto blockSolver = g2o::make_unique<g2o::BlockSolverX>(std::move(linearSolver));
    // create the algorithm to carry out the optimization
    g2o::OptimizationAlgorithmLevenberg* optimizationAlgorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));

    // Setup the optimizer_
    optimizer_.setVerbose(false);
    optimizer_.setAlgorithm(optimizationAlgorithm);

    // do this once after you created the optimizer_
    // add the camera parameters, caches are automatically resolved in the addEdge calls
    g2o::ParameterSE3Offset* cameraOffset = new g2o::ParameterSE3Offset;
    cameraOffset->setId(0);
    optimizer_.addParameter(cameraOffset);
}

cv::Affine3d CamLocalization::computeMarkerPose (const std::vector<cv::Point3f> &points3D_f)
{
    std::vector<cv::Point3d> points3D(points3D_f.begin(), points3D_f.end());

    //COMPUTE ROTATION
    cv::Point3d X_vec;
    X_vec.x = points3D[1].x - points3D[0].x;
    X_vec.y = points3D[1].y - points3D[0].y;
    X_vec.z = points3D[1].z - points3D[0].z;

    cv::Point3d Y_vec;
    Y_vec.x = points3D[0].x - points3D[3].x;
    Y_vec.y = points3D[0].y - points3D[3].y;
    Y_vec.z = points3D[0].z - points3D[3].z;

    cv::Point3d Z_vec;
    Z_vec.x = X_vec.y * Y_vec.z - X_vec.z * Y_vec.y;
    Z_vec.y = -(X_vec.x * Y_vec.z - X_vec.z * Y_vec.x);
    Z_vec.z = X_vec.x * Y_vec.y - X_vec.y * Y_vec.x;

    double X_mag = std::sqrt(X_vec.x * X_vec.x + X_vec.y * X_vec.y + X_vec.z * X_vec.z);
    double Y_mag = std::sqrt(Y_vec.x * Y_vec.x + Y_vec.y * Y_vec.y + Y_vec.z * Y_vec.z);
    double Z_mag = std::sqrt(Z_vec.x * Z_vec.x + Z_vec.y * Z_vec.y + Z_vec.z * Z_vec.z);

    cv::Point3d X_vec_norm, Y_vec_norm, Z_vec_norm;

    X_vec_norm.x = X_vec.x / X_mag;
    X_vec_norm.y = X_vec.y / X_mag;
    X_vec_norm.z = X_vec.z / X_mag;

    Y_vec_norm.x = Y_vec.x / Y_mag;
    Y_vec_norm.y = Y_vec.y / Y_mag;
    Y_vec_norm.z = Y_vec.z / Y_mag;

    Z_vec_norm.x = Z_vec.x / Z_mag;
    Z_vec_norm.y = Z_vec.y / Z_mag;
    Z_vec_norm.z = Z_vec.z / Z_mag;

    cv::Matx33d R33_norm;

    R33_norm(0,0) = X_vec_norm.x;
    R33_norm(1,0) = X_vec_norm.y;
    R33_norm(2,0) = X_vec_norm.z;

    R33_norm(0,1) = Y_vec_norm.x;
    R33_norm(1,1) = Y_vec_norm.y;
    R33_norm(2,1) = Y_vec_norm.z;
    
    R33_norm(0,2) = Z_vec_norm.x;
    R33_norm(1,2) = Z_vec_norm.y;
    R33_norm(2,2) = Z_vec_norm.z;

    cv::Affine3d pose;

    pose.rotation(R33_norm);

    //COMPUTE TRANSLATION WRT THE MM ORIGIN (MARKER CENTER)
    
    double base = ((points3D[1].x - points3D[0].x)/2.0);
    double height = ((points3D[3].y - points3D[0].y)/2.0);

    double t_x_center = points3D[0].x + base;
    double t_y_center = points3D[0].y + height;

    double sum_Z = 0;

    for (int i = 0; i < points3D.size(); i++)
        sum_Z += points3D[i].z;
    
    double t_z_center = sum_Z / points3D.size();
    cv::Vec3d t_vec (t_x_center, t_y_center, t_z_center);

    pose.translation(t_vec);

    return pose;
}

void CamLocalization::computeMMData ()
{
    markersMap_.getIdList(ids_, true);
    int aux = 0;
    for(auto idx : ids_)
    {
        aruco::Marker3DInfo info3D = markersMap_.getMarker3DInfo(idx);
        std::vector<cv::Point3f> points3D = info3D.points;
        cv::Affine3d pose;
        pose = computeMarkerPose(points3D);
        mm_poses_.push_back(pose);
        aux++;
    }
}

double computePoseError (const cv::Affine3d &pose, const std::vector <cv::Affine3d> &local_poses_m, const std::vector <cv::Affine3d> &local_m_cam)
{
    double rpe_total = 0;
    
    for(int i = 0; i < local_poses_m.size(); i++)
    {
        cv::Affine3d rel_transf = (local_poses_m[i].concatenate(pose.inv())).concatenate(local_m_cam[i].inv());
        //std::cout << rel_transf.translation() <<std::endl;
        double rpe = rel_transf.translation().dot(rel_transf.translation());
        rpe_total += rpe;
    }
    
    return rpe_total;
}

cv::Affine3d CamLocalization::process (const std::vector<aruco::Marker> &detectedMarkers)
{
    std::vector<cv::Affine3d> local_poses_m;
    std::vector<cv::Affine3d> local_m_cam;

    for (auto idx : markersMap_.getIndices(detectedMarkers))
    {
        //std::cout << detectedMarkers[idx].id << " ";
        std::vector<aruco::Marker> Mi;
        Mi.push_back(detectedMarkers[idx]);
        if (poseTracker_.isValid())
        {
            if (poseTracker_.estimatePose(Mi))
            {
                cv::Affine3d Pi = mm_poses_[detectedMarkers[idx].id];
                local_poses_m.push_back(Pi);
                //std::cout << "local_poses_m_" << detectedMarkers[idx].id << " = " << Pi.translation() << std::endl;
                cv::Mat C_Mi = poseTracker_.getRTMatrix().inv();
                cv::Affine3d C_Mi_cv = cvMat2cvAffine(C_Mi);
                //std::cout << "local_m_cam_" << detectedMarkers[idx].id << " = " << C_Mi_cv.translation() << std::endl;
                cv::Affine3d Mi_Ci = Pi.concatenate(C_Mi_cv.inv());
                //std::cout << "Mi_Ci: " << Mi_Ci.translation() << std::endl;
                local_m_cam.push_back(Mi_Ci);
                //std::cout << "local_m_cam_" << detectedMarkers[idx].id << " = " << Mi_Ci.inv().concatenate(Pi).translation() << std::endl;
            }
        }
    }

    cv::Affine3d opt_cam;

    if (poseTracker_.isValid())
    {
        if (poseTracker_.estimatePose(detectedMarkers)) 
        {
            if (local_poses_m.size() > 1)
            {
                cv::Mat C = poseTracker_.getRTMatrix().inv();
                cv::Affine3d T_cam_all_m = cvMat2cvAffine(C);
                //std::cout << "T_cam_all_m = " << T_cam_all_m.translation() << std::endl;
                // double rpe_before = computePoseError(T_cam_all_m,local_poses_m,local_m_cam);
                opt_cam = optimizeCam(T_cam_all_m, local_poses_m, local_m_cam);
                //std::cout << "T_cam_all_m_g2o = " << opt_cam.translation() << std::endl;
                // double rpe_after = computePoseError(opt_cam,local_poses_m,local_m_cam);
                // std::cout << "SM improves Cam Localization in: %" << rpe_after*100/rpe_before << std::endl;
            }                        
        }
    }

    return opt_cam;
}

cv::Affine3d CamLocalization::optimizeCam (const cv::Affine3d &T_cam_all_m, 
                                           const std::vector <cv::Affine3d> &local_poses_m, 
                                           const std::vector<cv::Affine3d> &local_m_cam)
{
    // Build the graph problem: 
    // -Add markers vertices
    int cam_vertex_id = 0;

    for(size_t i = 0; i < local_poses_m.size(); i++)
    {
        g2o::VertexSE3* vertex = new g2o::VertexSE3();
        vertex->setId(cam_vertex_id);
        vertex->setFixed(true);
        Eigen::Isometry3d pose_mi = cvAffine2egIsometry(local_poses_m[i]);
        vertex->setEstimate(pose_mi);
        optimizer_.addVertex(vertex);
        cam_vertex_id++;
    }

    // -Add camera vertex
    g2o::VertexSE3* vertex = new g2o::VertexSE3();
    vertex->setId(cam_vertex_id);
    vertex->setFixed(false);
    Eigen::Isometry3d T_cam_all_m_iso = cvAffine2egIsometry(T_cam_all_m);
    vertex->setEstimate(T_cam_all_m_iso);
    optimizer_.addVertex(vertex);

    //-Add num_obs edges for the observations
    Eigen::Matrix<double, 6, 6> info_matrix = Eigen::Matrix<double, 6, 6>::Identity();

    for (size_t i = 0; i < local_poses_m.size(); i++)
    {
        Eigen::Isometry3d Mi_Ci = cvAffine2egIsometry(local_m_cam[i]);
        g2o::EdgeSE3* edge = new g2o::EdgeSE3();
        edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer_.vertex(cam_vertex_id)));
        edge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer_.vertex(i)));
        edge->setMeasurement(Mi_Ci);
        edge->setInformation(info_matrix);
        edge->setId(i);
        optimizer_.addEdge(edge);
    }

    optimizer_.initializeOptimization();
    optimizer_.optimize(100);

    cv::Affine3d T_cam_all_m_g2o;
    T_cam_all_m_g2o = egIsometry2cvAffine(((g2o::VertexSE3*) optimizer_.vertex(cam_vertex_id))->estimate());

    optimizer_.clear();

    return T_cam_all_m_g2o;
}