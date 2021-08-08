#include "zed.h"
#include <Eigen/Geometry>
#include "geometry.h"

void ZED::initFromSVO(std::string &svo_file) 
{
    sl::String file = svo_file.c_str();
    param_.input.setFromSVOFile(file);
    param_.depth_mode = sl::DEPTH_MODE::QUALITY;
    param_.camera_disable_self_calib = true;
    param_.coordinate_units = sl::UNIT::METER;
    param_.coordinate_system = sl::COORDINATE_SYSTEM::IMAGE;
    runtime_parameters_.sensing_mode = sl::SENSING_MODE::FILL;
}

void ZED::openCam() 
{
    sl::ERROR_CODE state = cam_->open(param_);

    if (state != sl::ERROR_CODE::SUCCESS) 
    {
        std::cout << "\n\n\tOpen ZED Camera: " << state << ". Exit program.\n" << std::endl;
        cam_->close();
        exit(1);
    }

    sl::CalibrationParameters cam_params = getCalibrationParams();
    sl::Resolution resolution = getResolution();
    fx_ = cam_params.left_cam.fx;
    fy_ = cam_params.left_cam.fy;
    cx_ = cam_params.left_cam.cx;
    cy_ = cam_params.left_cam.cy;
    width_ = resolution.width;
    height_ = resolution.height;
    //Distortion factor : [ k1, k2, p1, p2, k3 ]. Radial (k1,k2,k3) and Tangential (p1,p2) distortion.
    k1_ = cam_params.left_cam.disto[0];
    k2_ = cam_params.left_cam.disto[1];
    p1_ = cam_params.left_cam.disto[2];
    p2_ = cam_params.left_cam.disto[3];

}

void ZED::enableTracking()
{
    sl::ERROR_CODE state = cam_->enablePositionalTracking();
    if(state != sl::ERROR_CODE::SUCCESS)
    {
        std::cout << "\n\n\tEnabling positionnal tracking failed: " << state << ". Exit program.\n" << std::endl;
        cam_->close();
        exit(1);
    }
}

sl::Resolution ZED::getResolution()
{
    sl::CameraInformation cam_info = cam_->getCameraInformation();
    return cam_info.camera_configuration.resolution;
}

sl::CalibrationParameters ZED::getCalibrationParams()
{
    sl::CameraInformation cam_info = cam_->getCameraInformation();
    return cam_info.camera_configuration.calibration_parameters;
}

sl::CameraInformation ZED::getCameraInformation()
{
    return cam_->getCameraInformation();
}

void ZED::setSVOPosition(int &frame_number)
{
    cam_->setSVOPosition(frame_number);
}

int ZED::getSVOPosition()
{
    return cam_->getSVOPosition();
}

int ZED::getSVONumberOfFrames()
{
    return cam_->getSVONumberOfFrames();
}

bool ZED::grab()
{
    sl::ERROR_CODE returned_state = cam_->grab(runtime_parameters_);
    bool status;
    if(returned_state == sl::ERROR_CODE::SUCCESS){
        status = true;
    }
    else{
        std::cout<< "\n\n\tError during capture: " << sl::toString(returned_state) << ".\n" << std::endl;
        status = false;
    }
    return status;
}

// Mapping between MAT_TYPE and CV_TYPE
int ZED::getOCVtype(sl::MAT_TYPE type) 
{
    int cv_type = -1;
    switch (type) {
        case sl::MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
        case sl::MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
        case sl::MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
        case sl::MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
        case sl::MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
        case sl::MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
        case sl::MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
        case sl::MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
        default: break;
    }
    return cv_type;
}

cv::Mat ZED::slMat2cvMat(sl::Mat &sl_img)
{
    cv::Mat cvImage = cv::Mat(sl_img.getHeight(), sl_img.getWidth(), getOCVtype(sl_img.getDataType()), sl_img.getPtr<sl::uchar1>(sl::MEM::CPU));
    return cvImage;
}

cv::Mat ZED::getCvLeftImage()
{
    sl::Mat left_image(sl::Resolution(width_, height_), sl::MAT_TYPE::U8_C4);
    cam_->retrieveImage(left_image, sl::VIEW::LEFT);
    cv::Mat left_image_cv;
    cv::cvtColor(slMat2cvMat(left_image), left_image_cv, CV_RGBA2RGB);
    return left_image_cv;
}

cv::Mat ZED::getCvRightImage()
{
    sl::Mat right_image(sl::Resolution(width_, height_), sl::MAT_TYPE::U8_C4);
    cam_->retrieveImage(right_image, sl::VIEW::RIGHT);
    cv::Mat right_image_cv;
    cv::cvtColor(slMat2cvMat(right_image), right_image_cv, CV_RGBA2RGB);
    return right_image_cv;
}

cv::Mat ZED::getCvDephtMap()
{
    sl::Mat depth_map(sl::Resolution(width_, height_), sl::MAT_TYPE::F32_C1);
    cam_->retrieveMeasure(depth_map, sl::MEASURE::DEPTH);
    sl::convertUnit(depth_map, cam_->getInitParameters().coordinate_units, sl::UNIT::MILLIMETER);
    cv::Mat depth16_;
    slMat2cvMat(depth_map).convertTo(depth16_, CV_16UC1);
    return depth16_;
}

cv::Mat ZED::getDisplayingDepth()
{
    sl::Mat depth_image(sl::Resolution(width_, height_), sl::MAT_TYPE::U8_C4);
    cam_->retrieveImage(depth_image, sl::VIEW::DEPTH);
    return slMat2cvMat(depth_image);
}

std::string ZED::getTimestampMilli()
{   
    uint64_t time_ms = cam_->getTimestamp(sl::TIME_REFERENCE::IMAGE).getMilliseconds();
    std::string time_ms_str = std::to_string(time_ms);
    std::string tum_time;  

    for(int i = 0; i < time_ms_str.length(); i++) 
    {
        if(i == 10)
            tum_time+=".";
        tum_time+=time_ms_str[i];
    }
    return tum_time;
}

void ZED::saveLeftImage() 
{
    if (outDir_ != "/home")
    {
        cv::Mat left_image_cv = getCvLeftImage();
        std::string filename = (outDir_+"/rgb/"+getTimestampMilli()+".png").c_str();
        rgbFile_ << getTimestampMilli() << " " << "rgb/" << getTimestampMilli() << ".png" << "\n";
        cv::imwrite(filename, left_image_cv);
    }
    else   
        std::cout<< "\n\n\tInit ZED Class with 'outDir' parameter!\n" << std::endl;
}

void ZED::saveRightImage() 
{
    if (outDir_ != "/home")
    {
        cv::Mat right_image_cv = getCvRightImage();
        std::string filename = (outDir_+"/rgb/r_"+getTimestampMilli()+".png").c_str();
        cv::imwrite(filename, right_image_cv);
    }
    else   
        std::cout<< "\n\n\tInit ZED Class with 'outDir' parameter!\n" << std::endl;
}

void ZED::saveDepthMap()
{
    if (outDir_ != "/home")
    {
        cv::Mat depth16 = getCvDephtMap();
        std::string filename = (outDir_+"/depth/"+getTimestampMilli()+".png").c_str();
        depthFile_ << getTimestampMilli() << " " << "depth/" << getTimestampMilli() << ".png" << "\n";
        cv::imwrite(filename, depth16);
    }
    else
        std::cout<< "\n\n\tInit ZED Class with 'outDir' parameter!\n" << std::endl;
}

cv::Affine3d ZED::getPositionRefCam()
{
    //std::cout << "Camera Pose OK!" << std::endl;
    cv::Affine3d T = getPoseAsCvAffine3d(sl::REFERENCE_FRAME::CAMERA);
    return T;
}

cv::Affine3d ZED::getPositionRefWorld()
{
    //std::cout << "Trajectory OK!" << std::endl;
    cv::Affine3d T = getPoseAsCvAffine3d(sl::REFERENCE_FRAME::WORLD);
    return T;
}

cv::Affine3d ZED::getPoseAsCvAffine3d(const sl::REFERENCE_FRAME &ref)
{
    // Get the position of the camera in a fixed reference frame
    sl::POSITIONAL_TRACKING_STATE tracking_state;
    sl::Pose pose;
    tracking_state = cam_->getPosition(pose, ref);
    if(tracking_state == sl::POSITIONAL_TRACKING_STATE::OK)
    {
        cv::Affine3d pose_cv = slPose2cvAffine(pose);
        return pose_cv;
    }
}

cv::Affine3d ZED::savePositionRefSMap(cv::Affine3d & map_pose)
{
    if (outDir_ != "/home")
    {
        cv::Affine3d T = getPositionRefSMap(map_pose);
        Eigen::Quaterniond q = cvAffine2egQuat(T);
        double tx = T.translation().val[0];
        double ty = T.translation().val[1];
        double tz = T.translation().val[2];
        poseFile_ << getTimestampMilli() << " " << tx << " " << ty << " " << tz << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
    }
    else
        std::cout<< "\n\n\tInit ZED Class with 'outDir' parameter!\n" << std::endl;
}

cv::Affine3d ZED::getPositionRefSMap(cv::Affine3d & map_pose)
{
    cv::Affine3d T = getPositionRefWorld().concatenate(map_pose);
    return  T;
}

void ZED::saveCamParams(std::string &file_name)
{   
    cv::Mat CameraMatrix;
    cv::Mat Distorsion;

    CameraMatrix = (cv::Mat_<float>(3,3) << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1);
    Distorsion = (cv::Mat_<float>(1,4) << k1_, k2_, p1_, p2_);
    cv::FileStorage fs;
    fs.open(file_name, cv::FileStorage::WRITE);
    fs << "image_width" << width_;
    fs << "image_height" << height_;
    fs << "camera_matrix" << CameraMatrix;
    fs << "distortion_coefficients" << Distorsion;
    std::cout<< "\n\n\tCamera Parameters Saved in: "+file_name+".\n" << std::endl;
}

void ZED::disableTracking()
{
    cam_->disablePositionalTracking();
}

void ZED::closeCam()
{
    cam_->close();
    depthFile_.close();
    rgbFile_.close();
    cloudFile_.close();
    poseFile_.close();
}

sl::Mat ZED::getPointCloud()
{
    sl::Mat point_cloud;
    cam_->retrieveMeasure(point_cloud, sl::MEASURE::XYZRGBA);
    return point_cloud;
}

void ZED::savePointCloud() 
{
    if (outDir_ != "/home")
    {
        sl::Mat point_cloud = getPointCloud();
        std::string filename = (outDir_+"/cloud/"+getTimestampMilli()+".pcd").c_str();
        cloudFile_ << getTimestampMilli() << " " << "cloud/" << getTimestampMilli() << ".pcd" << "\n";
        auto state = point_cloud.write(filename.c_str());
        if (state != sl::ERROR_CODE::SUCCESS)
            std::cout << "\n\n\tFailed to save point cloud... Please check that you have permissions to write at this location ("<< filename<<"). Re-run the sample with administrator rights under windows" << std::endl;
    }
    else
        std::cout<< "\n\n\tInit ZED Class with 'outDir' parameter!\n" << std::endl;
}

void ZED::savePointCloud(std::vector<cv::Point3f> &points3d) 
{
    if (outDir_ != "/home")
    {
        sl::Resolution cloud_res(8, 1);
        sl::Mat point_cloud(cloud_res, sl::MAT_TYPE::F32_C4, sl::MEM::CPU);

        for(size_t i = 0; i < points3d.size(); i++)
        {
            sl::float4 point3D;
            point3D.x = points3d[i].x;
            point3D.y = points3d[i].y;
            point3D.z = points3d[i].z;
            point_cloud.setValue(i,0,point3D);
        }
        std::string filename = (outDir_+"/cloud/"+getTimestampMilli()+"_markers_corners_zed.pcd").c_str();
        auto state = point_cloud.write(filename.c_str());
        if (state != sl::ERROR_CODE::SUCCESS)
            std::cout << "\n\n\tFailed to save point cloud... Please check that you have permissions to write at this location ("<< filename<<"). Re-run the sample with administrator rights under windows" << std::endl;
    }
    else
        std::cout<< "\n\n\tInit ZED Class with 'outDir' parameter!\n" << std::endl;
}

cv::Point3f ZED::get3dPoint(int i, int j)
{
    sl::Mat point_cloud = getPointCloud();
    sl::float4 point3D;
    cv::Point3f point3D_cv;
    // Get the 3D point cloud values for pixel (i,j)
    point_cloud.getValue(i,j,&point3D);
    point3D_cv.x = point3D.x;
    point3D_cv.y = point3D.y;
    point3D_cv.z = point3D.z;
    return point3D_cv;
}