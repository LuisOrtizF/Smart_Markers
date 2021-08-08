
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <aruco/aruco.h>
#include "markermapper.h"

#include "geometry.h"
#include "visualizer2d.h"
#include "visualizer3d.h"
#include "zed.h"
 
int main(int argc, char **argv) 
{
    if (argc != 6 )
    {
        printf("\n\n\tSyntax is: %s <video.svo> <aruco_config> <outDir> <marker_size> <marker_ref>", argv[0]);
        std::cout << "\n\n\tPress [Enter] to continue.\n" << std::endl;
        std::cin.ignore();
        return 1;
    }

    std::string videoFile   = argv[1];
    std::string arucoConfig = argv[2];
    std::string outDir      = argv[3];
    float markerSize        = atof(argv[4]);
    int refMarkerId         = atoi(argv[5]);

    /*** ZED camera ***/
    // Create camera
    ZED zed;
    zed.initFromSVO(videoFile);
    // Open camera
    zed.openCam();
    // Get ZED intrinsic params 
    sl::CalibrationParameters camInt = zed.getCalibrationParams();
    float fx_l = camInt.left_cam.fx;
    float fy_l = camInt.left_cam.fy;
    float cx_l = camInt.left_cam.cx;
    float cy_l = camInt.left_cam.cy;
    // Get ZED resolution
    sl::Resolution resolution = zed.getResolution();

    /** ARUCO Marker Map Creation */
    aruco::CameraParameters CamParam;
    CamParam.CameraMatrix = (cv::Mat_<float>(3,3) << fx_l, 0, cx_l, 0, fy_l, cy_l, 0, 0, 1);
    CamParam.Distorsion = (cv::Mat_<float>(1,5) << 0, 0, 0, 0, 0);
    CamParam.CamSize = cv::Size(zed.getResolution().width, zed.getResolution().height);
    // Create Marker Mapper
    std::shared_ptr<aruco_mm::MarkerMapper> AMM;
    AMM = aruco_mm::MarkerMapper::create();
    AMM->setParams(CamParam,markerSize,refMarkerId);
    AMM->getMarkerDetector().loadParamsFromFile(arucoConfig);

    /*** Set up the visualization 2D ***/
    Visualizer2D scene2D("Test MM");

    // Start SVO playback
    char key = ' ';
    while (key != 'q') 
    {
        if (zed.grab()) 
        {
            int svoPosition = zed.getSVOPosition();
            std::cout << "Current Frame: " << svoPosition << std::endl;               

            // Retrive left image from ZED zed in format: CV_RGBA2RGB
            cv::Mat left = zed.getCvLeftImage();

            // Process image to crete aruco marker mapper map
            AMM->process(left, svoPosition);

            // Visualization 2D
            cv::Mat left_draw;
            left.copyTo(left_draw);
            AMM->drawDetectedMarkers(left_draw);
            scene2D.showMarkers(left_draw);
            key = cv::waitKey(1);
            
            if(svoPosition == zed.getSVONumberOfFrames())
                break;
        }
        else
            break;
    }

    zed.closeCam();

    AMM->optimize();
    AMM->getMarkerMap().saveToFile(outDir+"/mm.yml");
    AMM->getCameraParams().saveToFile(outDir+"/mm_cam.yml");

    /*** Set up the visualization and scene3D elements ***/
    // Create 3D window
    Visualizer3D scene3D("Test MM");
    scene3D.addReferenceFrame(cv::Affine3d::Identity(), "origin");
    scene3D.addZXGrid(100,15,0.5,0.5, cv::viz::Color::silver());
    // Define markers colors
    cv::viz::Color mmColor = cv::viz::Color::black();
    // Visualization 3D of the aruco marker map
    scene3D.addMarkerMap(AMM->getMarkerMap(), "Mapper_Map", mmColor);
    scene3D.spinOnce(1, true);

    while(!scene3D.wasStopped())
    {
        cv::waitKey(1);
        scene3D.spinOnce(1, true);
    }

    cv::destroyAllWindows();
    return 0;
}