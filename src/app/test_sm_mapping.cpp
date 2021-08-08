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
#include "sm_mapping.h"

int main(int argc, char **argv) 
{
    if (argc != 7 )
    {
        printf("\n\n\tSyntax is: %s <video.svo> <pms_data_file> <aruco_config> <outDir> <marker_size> <marker_ref>", argv[0]);
        std::cout << "\n\n\tPress [Enter] to continue.\n" << std::endl;
        std::cin.ignore();
        return 1;
    }

    std::string videoFile   = argv[1];
    std::string pmsDataFile = argv[2];
    std::string arucoConfig = argv[3];
    std::string outDir      = argv[4];
    float markerSize        = atof(argv[5]);
    int refMarkerId         = atoi(argv[6]);

    /*** ZED camera ***/
    // Create camera
    ZED zed;
    zed.initFromSVO(videoFile);
    // Open camera
    zed.openCam();
    // Save Cam Params
    std::string camParamsFile = outDir+"/sm_cam.yml";
    zed.saveCamParams(camParamsFile);
    // Get ZED intrinsic params 
    sl::CalibrationParameters camInt = zed.getCalibrationParams();
    float fx_l = camInt.left_cam.fx;
    float fy_l = camInt.left_cam.fy;
    float cx_l = camInt.left_cam.cx;
    float cy_l = camInt.left_cam.cy;

    /** ARUCO Marker Map Creation */
    aruco::CameraParameters CamParam;
    CamParam.CameraMatrix = (cv::Mat_<float>(3,3) << fx_l, 0, cx_l, 0, fy_l, cy_l, 0, 0, 1);
    CamParam.Distorsion = (cv::Mat_<float>(1,5) << 0, 0, 0, 0, 0);
    CamParam.CamSize = cv::Size(zed.getResolution().width, zed.getResolution().height);
    // Create Marker Mapper
    std::shared_ptr<aruco_mm::MarkerMapper> AMM;
    AMM = aruco_mm::MarkerMapper::create();
    AMM->setParams(CamParam, markerSize, refMarkerId);
    AMM->getMarkerDetector().loadParamsFromFile(arucoConfig);

    /*** Set up the visualization 2D ***/
    Visualizer2D scene2D("Test Mapping");

    // Start SVO playback
    char key = ' ';
    while (key != 'q') 
    {
        if (zed.grab()) 
        {
            int svoPosition = zed.getSVOPosition();
            std::cout << "Current Frame: " << svoPosition << std::endl;

            // Get ZED data
            cv::Mat left = zed.getCvLeftImage();

            // Process left_draw to crete aruco marker map
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

    // Get Initial marker map using marker_mapper
    AMM->optimize();
    aruco::MarkerMap mmMap = AMM->getMarkerMap();

    // Get Optimized marker map using Smart Markers data
    SMMapping sm (mmMap, pmsDataFile, 50);
    aruco::MarkerMap smMap = sm.process();
    
    // Save final markers map 
    smMap.saveToFile(outDir+"/sm_map.yml");

    /*** Set up the visualization and scene3D elements ***/
    // Create 3D window
    Visualizer3D scene3D("Test Mapping");
    scene3D.addReferenceFrame(cv::Affine3d::Identity(), "origin");
    scene3D.addZXGrid(100,15,0.5,0.5, cv::viz::Color::silver());
    // Define markers colors
    cv::viz::Color mmColor = cv::viz::Color::red();
    cv::viz::Color smColor = cv::viz::Color::green();
    // Visulize aruco map and smart markers map to compare results
    scene3D.addMarkerMap(mmMap, "Mappper_Map", mmColor);
    scene3D.addMarkerMap(smMap, "Smart_Marker", smColor);
    scene3D.spinOnce(1, true);

    while(!scene3D.wasStopped())
    {
        cv::waitKey(1);
        scene3D.spinOnce(1, true);
    }

    cv::destroyAllWindows();
    return 0;
}