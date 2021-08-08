#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/edge_se3_prior.h"

#include "geometry.h"
#include "sm_mapping.h"

SMMapping::SMMapping (const aruco::MarkerMap &mm, const std::string &pms_file, const int &iterations)
{
    mm_ = mm;
    pms_file_ = pms_file;
    iterations_ = iterations;
}

cv::Affine3d SMMapping::computeMarkerPose (const std::vector<cv::Point3f> &points3D_f)
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

void SMMapping::computeMMData ()
{
    mm_.getIdList(ids_, true);
    int aux = 0;
    for(auto idx : ids_)
    {
        aruco::Marker3DInfo info3D = mm_.getMarker3DInfo(idx);
        std::vector<cv::Point3f> points3D = info3D.points;
        cv::Affine3d pose;
        pose = computeMarkerPose(points3D);
        for(int j = 0; j < points3D.size(); j++)
            mm_contours_.push_back(points3D[j]);
        mm_poses_.push_back(pose);
        mm_centers_.push_back(mm_poses_[aux].translation());
        aux++;
    }
}

void SMMapping::computePMSData ()
{
    std::ifstream file;
    std::string row;
    file.open(pms_file_);
    
    double aux = 0;

    if (file.is_open())
    {
        while (getline(file, row))
        {
            std::istringstream row_stream (row);
            double item_stream;
            std::vector<double> row_vector;
            
            while(row_stream>>item_stream)
                row_vector.push_back(item_stream);

            Eigen::Quaterniond q;
            cv::Vec3d t;
            cv::Matx33d R33;
            cv::Affine3d RT;

            q.x() = row_vector[3];
            q.y() = row_vector[4];
            q.z() = row_vector[5];
            q.w() = row_vector[6];
            Eigen::Matrix3d  Q = q.toRotationMatrix();

            t << row_vector[0], row_vector[1], row_vector[2];
            RT.translation(t);
            
            for (int v = 0; v < 3; v++) 
                for (int u = 0; u < 3; u++) 
                    R33(u,v) = Q(u,v);
            
            RT.rotation(R33);
            pms_poses_.push_back(RT);
            pms_centers_.push_back(RT.translation());
        }
        file.close(); 
    }
    else
        std::cout << "\t\tUnable to open PMS Data file!\n" << std::endl;
}

void SMMapping::computeMMTrasformations ()
{
    int c = 1;
    // int d = 0;
    for (int a = 0; a < mm_poses_.size()-1; a++)
    {
        for(int b = c; b < mm_poses_.size(); b++)
        {
            mm_T_.push_back(mm_poses_[b].concatenate(mm_poses_[a].inv()));
            // std::cout << "{" << a << "}" << "{"<< b << "}" << "=" << all_T[d].translation() << std::endl;
            // d++;
        }
        c++;
    }
    // std::cout << "\n" << std::endl;
}

std::vector<Eigen::Matrix<double, 6, 6>> SMMapping::computeMarkersInfoMatrix ()
{
    int c = 1;
    int d = 0;

    int num_markers = mm_contours_.size()/4;
    
    for (int a = 0; a < (num_markers-1); a++)
    {
        for(int b = c; b < num_markers; b++)
        {
            std::vector<double> Errors;
            double sum = 0;
            int aux = 0;

            Eigen::Matrix<double, 6, 6> info_matrix = Eigen::Matrix<double, 6, 6>::Identity();

            for(int i = (a*4); i < ((a+1)*4); i++)
            {
                Eigen::Vector3d p3D_o, p3D_t, p3D_ot;

                p3D_o.x() = mm_contours_[i].x;
                p3D_o.y() = mm_contours_[i].y;
                p3D_o.z() = mm_contours_[i].z;

                p3D_t.x() = mm_contours_[4*b+aux].x;
                p3D_t.y() = mm_contours_[4*b+aux].y;
                p3D_t.z() = mm_contours_[4*b+aux].z;

                p3D_ot = mm_T_iso[d].linear() * p3D_o + mm_T_iso[d].translation();

                double error =  std::sqrt(std::pow( p3D_ot.x() - p3D_t.x(), 2) +
                                          std::pow( p3D_ot.y() - p3D_t.y(), 2) +
                                          std::pow( p3D_ot.z() - p3D_t.z(), 2));
                
                sum+=error;
                Errors.push_back(error);

                aux++;
            }

            double mean = sum/4;
            double sum2 = 0;

            for(int i = 0; i < 4; i++)
                sum2 += (Errors[i] - mean)*(Errors[i] - mean);   

            double variance = sum2/4;
            mm_info_matrix_.push_back(info_matrix * 1/mean);
            d++;
        }
        c++;
    }

    return mm_info_matrix_;
}

void SMMapping::computeMMDataIsometry ()
{
    for (int i = 0; i < mm_poses_.size(); i++)
        mm_poses_iso.push_back(cvAffine2egIsometry(mm_poses_[i]));
    for (int i = 0; i < pms_poses_.size(); i++)
        pms_poses_iso.push_back(cvAffine2egIsometry(pms_poses_[i]));
    for (int i = 0; i < mm_T_.size(); i++)
        mm_T_iso.push_back(cvAffine2egIsometry(mm_T_[i]));
}

void SMMapping::setupOptimizer ()
{
    // create the linear solver
    auto linearSolver = g2o::make_unique<g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>>();
    // create the block solver on top of the linear solver
    auto blockSolver = g2o::make_unique<g2o::BlockSolverX>(std::move(linearSolver));
    // create the algorithm to carry out the optimization
    g2o::OptimizationAlgorithmLevenberg* optimizationAlgorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));

    // Setup the optimizer_
    // optimizer_.setVerbose(true);
    optimizer_.setAlgorithm(optimizationAlgorithm);

    // do this once after you created the optimizer_
    // add the camera parameters, caches are automatically resolved in the addEdge calls
    g2o::ParameterSE3Offset* cameraOffset = new g2o::ParameterSE3Offset;
    cameraOffset->setId(0);
    optimizer_.addParameter(cameraOffset);
}

void SMMapping::buildMarkersVertices ()
{
    // Build the graph problem: 
    // -Add vertices
    for(size_t i = 0; i < mm_poses_iso.size(); i++)
    {
        g2o::VertexSE3* vertex = new g2o::VertexSE3();
        vertex->setId(i);

        if (i == 0)
            vertex->setFixed(true);
        else
            vertex->setFixed(false);

        vertex->setEstimate(mm_poses_iso[i]);
        optimizer_.addVertex(vertex);
    }

    computeMarkersInfoMatrix();
    
    //-Add num_obs edges for the observations
    int c = 1;
    edge_id_ = 0;
    
    for (size_t a = 0; a < mm_poses_iso.size()-1; a++)
    {
        for(size_t b = c; b < mm_poses_iso.size(); b++)
        {
            g2o::EdgeSE3* edge = new g2o::EdgeSE3();
            edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer_.vertex(a)));
            edge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer_.vertex(b)));
            edge->setMeasurement(mm_T_iso[edge_id_]);
            edge->setInformation(mm_info_matrix_[edge_id_]);
            edge->setId(edge_id_);
            optimizer_.addEdge(edge);
            edge_id_++;
        }
        c++;
    }
}

void SMMapping::buildPMSVertices ()
{
    Eigen::Matrix<double, 6, 6> pms_information = Eigen::Matrix<double, 6, 6>::Identity();

    pms_information(0,0) = 1/std::pow(0.0023,2);
    pms_information(1,1) = 1/std::pow(0.0023,2);
    pms_information(2,2) = 1/std::pow(0.0023,2);
    
    pms_information(3,3) = 1/std::pow(0.0015,2);
    pms_information(4,4) = 1/std::pow(0.0015,2);
    pms_information(5,5) = 1/std::pow(0.0015,2);

    for (size_t e = 0; e < pms_poses_iso.size(); e++)
    {
        g2o::EdgeSE3Prior* edgeimu = new g2o::EdgeSE3Prior();
        edgeimu->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer_.vertex(e)));
        edgeimu->setMeasurement(pms_poses_iso[e]);
        edgeimu->setInformation(pms_information);
        edgeimu->setId(edge_id_+e+1);
        edgeimu->setParameterId(0, 0);
        optimizer_.addEdge(edgeimu);
    }
}

void SMMapping::otimize ()
{
    //Get sum of errors before optimization
    optimizer_.computeActiveErrors();
    //printf("Initial chi2: %f\n", optimizer_.activeChi2());
    
    //Perform the optimization
    printf("Optimizing\n");
    optimizer_.initializeOptimization();
    optimizer_.optimize(iterations_);
    //printf("Final chi2: %f\n", optimizer_.activeChi2());

    //Show optimized estimate: for each vertex, show its state (estimate() member function)
    std::cerr << "number of vertices:" << optimizer_.vertices().size() << std::endl;
    std::cerr << "number of edges:" << optimizer_.edges().size() << std::endl;  

    for (size_t i = 0; i < mm_poses_iso.size(); i++)
    {
        opt_poses_.push_back(egIsometry2cvAffine(((g2o::VertexSE3*) optimizer_.vertex(i))->estimate()));
        //std::cout << ((g2o::VertexSE3*) optimizer_.vertex(i))->estimate().translation().transpose() << std::endl;
    }
}

void SMMapping::optimization ()
{
    computeMMDataIsometry();
    setupOptimizer();
    buildMarkersVertices();
    buildPMSVertices();
    otimize();
}

void SMMapping::computeOptMMContours ()
{  
    for (int i = 0; i < mm_poses_.size(); i++)
    {
        cv::Affine3d mmT = mm_poses_[0].concatenate(mm_poses_[i].inv());

        for(int j = (i*4); j < ((i+1)*4); j++)
        {
            cv::Vec3d p3d, p3d_mmT, p3d_mm2g2o;

            p3d.val[0] = mm_contours_[j].x;
            p3d.val[1] = mm_contours_[j].y;
            p3d.val[2] = mm_contours_[j].z;

            p3d_mmT = mmT.rotation() * p3d + mmT.translation();

            p3d_mm2g2o = opt_poses_[i].rotation() * p3d_mmT + opt_poses_[i].translation(); 

            cv::Point3d p3d_cv;

            p3d_cv.x = p3d_mm2g2o.val[0];
            p3d_cv.y = p3d_mm2g2o.val[1];
            p3d_cv.z = p3d_mm2g2o.val[2];

            opt_contours_.push_back(p3d_cv);
       }           
    }
}

void SMMapping::createOptMM ()
{
    std::vector<cv::Point3f> ourpoints3D(opt_contours_.begin(), opt_contours_.end());
    opt_map_.setDictionary(mm_.getDictionary());
    opt_map_.mInfoType = aruco::MarkerMap::METERS;

    for (int i = 0; i < ids_.size(); i++)
    {
        aruco::Marker3DInfo m3di;

        for(int j = (i*4); j < ((i+1)*4); j++)
            m3di.push_back(ourpoints3D[j]);     

        m3di.id = ids_[i];
        opt_map_.push_back(m3di);
    }
}

aruco::MarkerMap SMMapping::process()
{
    computeMMData();
    computePMSData();
    computeMMTrasformations();
    optimization();
    computeOptMMContours();
    createOptMM();
    return opt_map_;
}