
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>

#include "geometry.h"
#include "visualizer2d.h"
#include "visualizer3d.h"
#include "zed.h"
#include "cam_localization.h"

int main(int argc, char **argv) 
{
    if (argc != 5 )
    {
        printf("\n\n\tSyntax is: %s <video.svo> <sm_map> <aruco_config> <outDir>", argv[0]);
        std::cout << "\n\n\tPress [Enter] to continue.\n" << std::endl;
        std::cin.ignore();
        return 1;
    }

    std::string videoFile = argv[1];
    std::string smMapFile = argv[2];
    std::string arucoConfig = argv[3];
    std::string outDir = argv[4];

    /** ZED CAMERA **/
    // Create camera
    ZED zed(outDir);
    zed.initFromSVO(videoFile);
    // Open camera
    zed.openCam();
    // Get ZED parameters
    sl::CalibrationParameters camInt = zed.getCalibrationParams();
    float fx_l = camInt.left_cam.fx;
    float fy_l = camInt.left_cam.fy;
    float cx_l = camInt.left_cam.cx;
    float cy_l = camInt.left_cam.cy;
    // Get ZED resolution
    sl::Resolution resolution = zed.getResolution();
    // Enable positional tracking
    zed.enableTracking();

    /** ARUCO */
    // Set up aruco params
    aruco::CameraParameters camParam;
    camParam.CameraMatrix = (cv::Mat_<float>(3,3) << fx_l, 0, cx_l, 0, fy_l, cy_l, 0, 0, 1);
    camParam.Distorsion = (cv::Mat_<float>(1,5) << 0, 0, 0, 0, 0);
    camParam.CamSize = cv::Size(resolution.width, resolution.height);
    // Create markers detector
    aruco::MarkerDetector mDetector;
    mDetector.loadParamsFromFile(arucoConfig);
    // Create marker map
    aruco::MarkerMap smMap;
    smMap.readFromFile(smMapFile);
    // Create pose tracker
    aruco::MarkerMapPoseTracker poseTracker;
    float markerSize = cv::norm(smMap[0][0] - smMap[0][1]);
    if (smMap.isExpressedInMeters() && camParam.isValid())
        poseTracker.setParams(camParam, smMap, markerSize);

    /** Set up visualization 2D **/
    Visualizer2D scene2D("Test Localization");

    /** Set up  visualization and scene3D elements **/
    // Create 3D window
    Visualizer3D scene3D("Test Localization");
    // Add reference frame
    scene3D.addReferenceFrame(cv::Affine3d::Identity(), "origin");
    // Add xz grid
    scene3D.addZXGrid(100, 15, 0.5, 0.5, cv::viz::Color::silver());
    // Define color for zed
    cv::viz::Color zedColor = cv::viz::Color::red();
    // Define color for smCam
    cv::viz::Color smColor = cv::viz::Color::green();
    // Create cameras fro visulization
    PerspectiveCamera zedCam("zedCam", fx_l, fy_l, cx_l, cy_l, resolution.width, resolution.height);
    PerspectiveCamera smCam("smCam", fx_l, fy_l, cx_l, cy_l, resolution.width, resolution.height);
    // Add cameras
    scene3D.addCamera(zedCam, zedColor);
    scene3D.addCamera(smCam, smColor);

    // Create vectors to save the trajectories
    std::vector<cv::Affine3d> smPoses;
    std::vector<cv::Affine3d> zedPoses;

    /** Camera Localization using Optimized Smart Marker Map*/
    CamLocalization localization (smMap, poseTracker, 50);

    // Start SVO playback
    char key = ' ';
    cv::Affine3d T_zed;
    int countPoses = 0;
    int firstFrame = 0;
    int lastFrame = zed.getSVONumberOfFrames();
    //int lastFrame = 630;
    //zed.setSVOPosition(firstFrame);

    for (int frame = firstFrame; frame <= lastFrame; frame++)
    {
        if (key != 'q' && !scene3D.wasStopped() && zed.grab() && frame != lastFrame+1) 
        {         
            // Retrive depth and left images from zed cam
            cv::Mat left = zed.getCvLeftImage();
            cv::Mat depth = zed.getCvDephtMap(); 

            // zed.saveLeftImage();
            // zed.saveDepthMap();          
            
            // Markers detection
            std::vector<aruco::Marker> detectedMarkers = mDetector.detect(left, camParam, markerSize);
            
            // Visualization markers
            cv::Mat left_draw;
            left.copyTo(left_draw);
            for (auto idx : smMap.getIndices(detectedMarkers))
            {
                detectedMarkers[idx].draw(left_draw, cv::Scalar(0, 0, 255), 2);
                aruco::CvDrawingUtils::draw3dCube(left_draw, detectedMarkers[idx], camParam);
                aruco::CvDrawingUtils::draw3dAxis(left_draw, detectedMarkers[idx], camParam);
            }
            scene2D.showMarkers(left_draw);
            scene2D.showDepthMap(zed.getDisplayingDepth());
            key = cv::waitKey(1);

            // Compute Camera Pose using Smart Markers
            cv::Affine3d T_sm = localization.process(detectedMarkers);

            if(T_sm.translation().val[0] == 0 && T_sm.translation().val[1] == 0 && T_sm.translation().val[2] == 0)
                continue;
            else
                countPoses++;

            std::cout << "Frame: " << frame << ", Pose: " << countPoses << std::endl;

            // Push back cam path obtained SM
            smPoses.push_back(T_sm); 

            cv::Affine3d T_zed = zed.getPositionRefSMap(smPoses[0]);
            zedPoses.push_back(T_zed);
            
            std::cout << smPoses[countPoses-1].translation() << std::endl;
            std::cout << zedPoses[countPoses-1].translation() << std::endl;

            // Visualization 3D of the cameras frustum
            zedCam.setPose(cvAffine2egIsometry(zedPoses[countPoses-1]));
            scene3D.updateCamera(zedCam, zedColor);
            smCam.setPose(cvAffine2egIsometry(smPoses[countPoses-1]));
            scene3D.updateCamera(smCam, smColor);

            if (countPoses > 1){
                // Visualization of the cameras paths
                scene3D.addTrajectory(zedPoses[countPoses-2], zedPoses[countPoses-1], "zed_traj_"+std::to_string(countPoses-1), zedColor);
                scene3D.addTrajectory(smPoses[countPoses-2], smPoses[countPoses-1], "sm_traj_"+std::to_string(countPoses-1), smColor); 
            }
            scene3D.spinOnce(1, true);
        }
        else{
            break;
        }
    }

    zed.disableTracking();
    zed.closeCam();

    while(!scene3D.wasStopped())
    {
        cv::waitKey(1);
        scene3D.spinOnce(1, true);
    }

    cv::destroyAllWindows();
    return 0;
}