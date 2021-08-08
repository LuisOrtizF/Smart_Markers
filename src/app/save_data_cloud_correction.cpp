#include <iostream>
#include <string>
#include "zed.h"
#include <aruco/aruco.h>
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
    // Load SVO video
    zed.initFromSVO(videoFile);
    // Open camera
    zed.openCam();
    // Enable positional tracking
    zed.enableTracking();
    // Get ZED resolution
    sl::Resolution resolution = zed.getResolution();

    // Get ZED parameters
    sl::CalibrationParameters camInt = zed.getCalibrationParams();
    float fx_l = camInt.left_cam.fx;
    float fy_l = camInt.left_cam.fy;
    float cx_l = camInt.left_cam.cx;
    float cy_l = camInt.left_cam.cy;

    int firstFrame = 0;
    zed.setSVOPosition(firstFrame);
    //std::cout << zed.getSVOPosition() << std::endl;
    int lastFrame = zed.getSVONumberOfFrames();

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

    // Create vectors to save the trajectories
    cv::Affine3d smPose;

    /** Camera Localization using Optimized Smart Marker Map*/
    CamLocalization localization (smMap, poseTracker, 50);

    int countPoses = 0;

    // Start SVO playback
    char key = ' ';
    for (int frame = firstFrame; frame <= lastFrame; frame++)
    {
        if (key != 'q' && zed.grab() && frame != lastFrame+1) 
        {       
            // Retrive depth and left images from zed cam
            cv::Mat left = zed.getCvLeftImage();

            // Markers detection
            std::vector<aruco::Marker> detectedMarkers = mDetector.detect(left, camParam, markerSize);
            
            // Compute Camera Pose using Smart Markers
            cv::Affine3d T_sm = localization.process(detectedMarkers);

            if(T_sm.translation().val[0] == 0 && T_sm.translation().val[1] == 0 && T_sm.translation().val[2] == 0)
                continue;
            else
                countPoses++;

            if(countPoses==1)
            {
                smPose = T_sm;

                cv::Affine3d T_zed = zed.getPositionRefSMap(smPose);
                
                std::cout << "\tFrame: " << frame << "\n\tCam Pose: " << T_zed.translation() << "\n\tTimestamp: " << zed.getTimestampMilli() <<std::endl;

                zed.saveLeftImage();
                zed.saveDepthMap(); 
                zed.savePositionRefSMap(smPose);
                zed.savePointCloud(); 
                
                std::vector<cv::Point3f> markers_p3d;
                for(size_t i = 0; i < detectedMarkers.size(); i++)
                {
                    for(size_t j = 0; j < detectedMarkers[i].size(); j++)
                    {
                        int x = detectedMarkers[i][j].x;
                        int y = detectedMarkers[i][j].y;
                        cv::Point3f p3d = zed.get3dPoint(x, y);
                        markers_p3d.push_back(p3d);
                    }
                }
                zed.savePointCloud(markers_p3d);
                break;
            }
        }
    }

    zed.disableTracking();
    zed.closeCam();
    return 0;
}