#include "visualizer2d.h"

Visualizer2D::Visualizer2D(const std::string &win_name)
{
    win_name_ = win_name;
}

void Visualizer2D::addWindow(const std::string &name, const int &width, const int &height, 
                             const int &x, const int &y)
{
    cv::namedWindow(name, CV_WINDOW_NORMAL);
    cv::resizeWindow(name, width, height);
    cv::moveWindow(name, x, y);
    cv::waitKey(1);
}

void Visualizer2D::showMultiImages(const std::vector<cv::Mat> &images)
{
    cv::Size sz = images[0].size();
    addWindow(win_name_ + ": Images", 1920, 300, 0, 0);
    // Add Images
    int32_t dst_height = sz.height;
    int32_t dst_width = 0;
    for (int i = 0; i < images.size(); i++)
        dst_width += images[i].cols;
    cv::Mat dst(dst_height, dst_width, CV_8UC4);
    for (int i = 0; i < images.size(); i++)
    {
        cv::Mat aux(dst, cv::Rect(i*sz.width, 0, sz.width, sz.height));
        images[i].copyTo(aux);
    }
    // Add Cam number
    for (int i = 0; i < images.size(); i++)
        cv::putText( dst, "cam_" + std::to_string(i), cv::Point2d((sz.width+10)*i, sz.height-10),
                     CV_FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 255, 0), 3);
    cv::imshow(win_name_ + ": Images", dst);
}

void Visualizer2D::showSingleImage(const cv::Mat &image)
{
    addWindow(win_name_ + ": Image", 1920, 300, 0, 0);
    cv::imshow(win_name_ + ": Image", image);
}

void Visualizer2D::showDepthMap(const cv::Mat &depth_map)
{
    addWindow(win_name_ + ": Depth", 880, 300, 1040, 375);
    cv::imshow(win_name_ + ": Depth", depth_map);
}

void Visualizer2D::showMarkers(const cv::Mat &image)
{
    addWindow(win_name_ + ": Markers", 880, 300, 1040, 725);
    cv::imshow(win_name_ + ": Markers", image);
}