#ifndef VISUALIZER2D_H_
#define VISUALIZER2D_H_

#include <vector>
#include <string>

#include <opencv2/opencv.hpp>

class Visualizer2D
{
    public:
        
        /**
         * Constructor
         */
        Visualizer2D(const std::string &win_name);

        /**
         * Add new 2D OpenCV window
         * @param win_name: window name
         * @param width: window width
         * @param height: window height
         * @param x: x-coordinate of the window
         * @param y: y-coordinate of the window
         */
        void addWindow(const std::string &win_name,
                       const int &width, const int &height, const int &x, const int &y);

        /**
         * Joint and visualize multiple images in the same OpenCV window
         * @param images: vector of images
         */
        void showMultiImages(const std::vector<cv::Mat> &images);
        
        /**
         * Show One image
         * @param image: Opencv image
         */
        void showSingleImage(const cv::Mat &image);

        /**
         * Show Depth Map
         * @param depth_map: depth map image
         */
        void showDepthMap(const cv::Mat &depth_map);

        /**
         * Show detected markers by aruco
         * @param image: image with markers
         */
        void showMarkers(const cv::Mat &image);

        private:
            std::string win_name_;

};
#endif /* VISUALIZER2D_H_ */