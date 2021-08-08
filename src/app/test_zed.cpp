
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>

#include "geometry.h"
#include "visualizer2d.h"
#include "visualizer3d.h"
#include "perspective_camera.h"
#include "zed.h"

int main(int argc, char **argv) 
{
    if (argc != 5 )
    {
        printf("\n\n\tSyntax is: %s <video.svo> <firstFrame> <lastFrame> <dir_to_save>", argv[0]);
        std::cout << "\n\n\tPress [Enter] to continue.\n" << std::endl;
        std::cin.ignore();
        return 1;
    }

    int firstFrame = atoi(argv[2]);
    int lastFrame  = atoi(argv[3]);
    std::string main_dir = argv[4];

    /** ZED CAMERA */
    // Create camera
    std::string video_file = argv[1];
    ZED cam(main_dir);
    cam.initFromSVO(video_file);
    // Open camera
    cam.openCam();
    // Enable positional tracking
    cam.enableTracking();
    // Set initial frame
    cam.setSVOPosition(firstFrame);
    // Get ZED parameters
    sl::CalibrationParameters camInt = cam.getCalibrationParams();
    float fx_l = camInt.left_cam.fx;
    float fy_l = camInt.left_cam.fy;
    float cx_l = camInt.left_cam.cx;
    float cy_l = camInt.left_cam.cy;
    // Get ZED resolution
    sl::Resolution resolution = cam.getResolution();

    /*** Set up the visualization 2D ***/
    Visualizer2D scene2D("Test ZED");

    /*** Set up the visualization and scene3D elements ***/
    // Create 3D window
    Visualizer3D scene3D("Test ZED");
    // Set viwer pose
    cv::Affine3d scene3D_pose;
    cv::Matx33d scene3D_rot;
    scene3D_rot(0,0) = 1;
    scene3D_rot(1,1) = -1;
    scene3D_rot(2,2) = -1;
    scene3D_pose.rotation() = scene3D_rot;
    scene3D.setViewerPose(scene3D_pose);
    scene3D.addReferenceFrame(cv::Affine3d::Identity(), "origin");
    scene3D.addZXGrid(100,15,0.5,0.5, cv::viz::Color::silver());
    cv::viz::Color color = cv::viz::Color::black();
    // Create cam for visualization 
    PerspectiveCamera zedCam("zedCam", fx_l, fy_l, cx_l, cy_l, resolution.width, resolution.height);
    scene3D.addCamera(zedCam, color);
    // Define color for trajectory
    cv::viz::Color zedColor = cv::viz::Color::red();

    /*** Create map to save the trajectory - init with identity***/ 
    std::map<int, cv::Affine3d> T_total;
    T_total.insert(std::make_pair(firstFrame, cv::Affine3d::Identity())); 

    // Start SVO playback
    for (int frame = firstFrame; frame <= lastFrame; frame++)
    {
        if (cam.grab()) 
        {
            int svo_position = cam.getSVOPosition();
            std::cout << "Current Frame: " << svo_position << std::endl;            
            std::string time_ms = cam.getTimestampMilli();   
            //std::cout<< time_ms <<std::endl;

            // Get ZED data
            cv::Mat left = cam.getCvLeftImage();
            cv::Mat right = cam.getCvRightImage();
            cv::Mat depth = cam.getCvDephtMap();

            // Visualization 2D
            std::vector<cv::Mat> images;
            images.push_back(left);
            images.push_back(right);
            scene2D.showMultiImages(images);
            scene2D.showDepthMap(depth);
            cv::waitKey(1);

            // Get ZED position tracking
            cv::Affine3d T_cam = cam.getPositionRefCam();
            cv::Affine3d T_world = cam.getPositionRefWorld();

            /**  Visualization 3D **/
            // Update frustum
            zedCam.setPose(cvAffine2egIsometry(T_world));
            scene3D.updateCamera(zedCam, color);
            T_total.insert(std::make_pair(frame, T_world));
            // Plot trajectory
            if(frame>firstFrame)
                scene3D.addTrajectory(T_total[frame-1], T_total[frame], "cam_traj_"+std::to_string(frame), zedColor); 
            scene3D.spinOnce(1, true);

            // Save ZED data in TUM format
            cam.saveDepthMap();
            cam.saveLeftImage();

            if(svo_position == lastFrame+1)
                break;
        }
        else
            break;        
    }

    cam.disableTracking();
    cam.closeCam();

    while(!scene3D.wasStopped())
    {
        cv::waitKey(1);
        scene3D.spinOnce(1, true);
    }

    cv::destroyAllWindows();
    return 0;
}