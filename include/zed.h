#ifndef ZED_H_
#define ZED_H_

#include <iostream>
#include <string>

#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>

class ZED
{
    public:
        /* 
         * Builds a ZED camera
        */
        ZED() : outDir_("/home"){
            cam_ = new sl::Camera;
        }

        /** 
         * Builds a ZED camera
         * @param outDir: directory to save data
        **/
        ZED(std::string &outDir) : outDir_(outDir) 
        {
            cam_ = new sl::Camera;
            rgbFile_.open (outDir_+"/rgb.txt");
            depthFile_.open (outDir_+"/depth.txt");
            poseFile_.open (outDir_+"/groundtruth.txt");
            cloudFile_.open (outDir_+"/cloud.txt");
            if (!depthFile_.is_open() || !rgbFile_.is_open() || !poseFile_.is_open() || !cloudFile_.is_open())
            {
                printf( "\n\tUnable to open depth.txt or rgb.txt or groundtruth.txt or cloud.txt to save data!\n\n" );
                exit(1);
            }
        }

        // deconstructor
        ~ZED(){
            delete cam_;
        };

        /** 
         * Init ZED camera from svo file
         * @param svo_file: svo file full path
         */
        void initFromSVO(std::string &svo_file);

        /** 
         * Open ZED camera with specyfic parameters, call after initParameters()
         */
        void openCam();
        /** 
         * Open ZED camera positional tracking, call after openCam()
         */
        void enableTracking();

        /** Returns the resolution of the ZED camera (width, height)
         */
        sl::Resolution getResolution();

        /** Returns the calibration parameters, serial number and other 
         * information about the camera being used.
         * When reading an SVO file, the parameters will correspond to the camera used for recording.
         */
        sl::CalibrationParameters getCalibrationParams();

        /** 
         * Returns the calibration parameters, serial number and other 
         * information about the camera being used.
         */
        sl::CameraInformation getCameraInformation();

        /** 
         * Sets the playback cursor to the desired frame number in the SVO file.
         * The function allows you to move around within a played-back SVO 
         * file. After calling, the next call to grab() will read the 
         * provided frame number.
         * @param frame_number: the number of the desired frame to be decoded.
         */
        void setSVOPosition(int &frame_number);
        
        /** 
         * Returns the current playback position in the SVO file.
         * The position corresponds to the number of frames already read from the SVO file, starting from 0 to n.
         * Each grab() call increases this value by one (except when using InitParameters::svo_real_time_mode).
         * The current frame position in the SVO file. Returns -1 if the SDK is not reading an SVO.
         */
        int getSVOPosition();

        /** 
         * Returns the number of frames in the SVO file. 
         * @note: Works only if the camera is open in SVO reading mode.
         */
        int getSVONumberOfFrames();

        /** 
         * This function will grab the latest images from the camera, 
         * rectify them, and compute the measurements based on the 
         * RuntimeParameters provided (depth, point cloud, tracking, etc.)
         */
        bool grab();
        
        /** 
         * Convert sl::Mat to cv::Mat (share buffer)
         */
        cv::Mat slMat2cvMat (sl::Mat &sl_img);

        /** 
         * Retrieves OpenCV left image from the camera (or SVO file).
         */
        cv::Mat getCvLeftImage();

        /** 
         * Retrieves OpenCV right image from the camera (or SVO file).
         */
        cv::Mat getCvRightImage();

        /** 
         * Retrieves OpenCV depth image from the camera (or SVO file).
         */
        cv::Mat getCvDephtMap();

        /** 
         * Retrieves OpenCV depth image (only for visualiztion purposes) from the camera (or SVO file).
         */
        cv::Mat getDisplayingDepth();

        /** 
         * Retrieves point cloud from the left camera (or SVO file).
         */
        sl::Mat getPointCloud();

        /** 
         * Returns the timestamp in milliseconds.
         * This function can also be used when playing back an SVO file.
         */
        std::string getTimestampMilli(); 

        /** 
        * Writes the left image (in color) into a file defined by its extension.
        */
        void saveLeftImage();
        
        /** 
        * Writes the right image (in color) into a file defined by its extension.
        */
        void saveRightImage();
       
        /** 
        * Writes the depth map (in milimeters) into a file defined by its extension.
        */
        void saveDepthMap();

        /** 
        * Writes the point cloud into a .pcd files
        */
        void savePointCloud();

        /** 
        * Retrieves the estimated position and orientation of the camera in CAMERA reference frame.
        */
        cv::Affine3d getPositionRefCam();

        /** 
        * Retrieves the estimated position and orientation of the camera in WORLD reference frame.
        */
        cv::Affine3d getPositionRefWorld();

        /** 
        * Retrieves the estimated position and orientation of the camera in SM MAP reference frame.
        */
        cv::Affine3d getPositionRefSMap(cv::Affine3d & map_pose);

        /** 
        * Save the estimated position and orientation of the camera in SM MAP reference frame.
        */
        cv::Affine3d savePositionRefSMap(cv::Affine3d & map_pose);

        /**
         * Save ZED intrinsic params and distortion params in yml format
        * @param file_name: file to save cam params.
        */
        void saveCamParams(std::string &file_name);

        /**
         * Get 3D point form point cloud using 2d coodinates 
         * @param i: pixel coordinate in x.
         * @param j: pixel coordinate in y.
        */
        cv::Point3f get3dPoint(int i, int j);

        void savePointCloud(std::vector<cv::Point3f> &points3d); 
        
        /** 
         * Disables the positional tracking.
         */
        void disableTracking();

        /**
         * Close ZED camera
         * If open() has been called, this function will close the connection to the camera 
         * (or the SVO file) and free the corresponding memory.
         * If open() wasn't called or failed, this function won't have any effects.
         */
        void closeCam();

    private:
        std::string outDir_;
        sl::Camera* cam_;
        sl::InitParameters param_;
        // Set runtime parameters after opening the camera
        sl::RuntimeParameters runtime_parameters_;

        int getOCVtype(sl::MAT_TYPE type);
        std::ofstream rgbFile_, depthFile_, poseFile_, cloudFile_;
        double fx_,fy_,cx_,cy_,k1_,k2_,p1_,p2_;
        int width_,height_;

        // Compute sl::Pose like cv::Affine3d in the specific reference frame (CAMERA OR WORLD) 
        cv::Affine3d getPoseAsCvAffine3d(const sl::REFERENCE_FRAME &ref);
};

#endif /* ZED_H_ */