#include <sstream>

#include "geometry.h"
#include "visualizer3d.h"

Visualizer3D::Visualizer3D(const std::string& title)
{
    viewer_ = new cv::viz::Viz3d(title);
    viewer_->spinOnce(1, true);
    viewer_->setBackgroundColor(cv::viz::Color::white());
    viewer_->setWindowPosition(cv::Point(0, 375));
    viewer_->setWindowSize(cv::Size(960, 675));
}

void Visualizer3D::setViewerIntrinsics(const cv::viz::Camera& cam)
{
    viewer_->setCamera(cam);
}

void Visualizer3D::setViewerPose(const cv::Affine3d& pose)
{
    viewer_->setViewerPose(pose);
}

void Visualizer3D::addZXGrid(const size_t& nz, const size_t& nx, const double& sz, const double& sx, const cv::viz::Color& color)
{
    cv::Point3d center(0.0, 0.0, 0.0);
    cv::Vec3d normal(0.0000000001, 1.0, 0.0);
    cv::Vec3d new_yaxis(0.0, 1.0, 0.0);
    cv::Vec2i cells(nz, nx);
    cv::Vec2d size(sz, sx);
    //Draw a grid
    cv::viz::WGrid grid (center, normal, new_yaxis, cells, size, color);
    viewer_->showWidget("grid_xz", grid);
}

void Visualizer3D::addReferenceFrame(const cv::Affine3d& pose, const std::string& text)
{
    //Create a 3D axis
    cv::viz::WCameraPosition cam_widget(0.1);
    viewer_->showWidget(text, cam_widget, pose);
    //Create a name
    cv::Point3d pos(pose.translation()(0) - 0.01, pose.translation()(1) - 0.01, pose.translation()(2) - 0.01);
    double scale = 0.05;
    cv::viz::WText3D text_widget(text, pos, scale, true, cv::viz::Color::black());
    viewer_->showWidget("text_" + text, text_widget);
}

void Visualizer3D::addTrajectory(const cv::Affine3d& from, const cv::Affine3d& to,
                                 const std::string& name, const cv::viz::Color& color)
{
    cv::viz::WLine line = cv::viz::WLine(from.translation(), to.translation(), color);	
    viewer_->showWidget("line_" + name, line);
    cv::viz::WCameraPosition axis = cv::viz::WCameraPosition ( 0.09 );
    viewer_->showWidget("pose_" + name, axis, to);
}

void Visualizer3D::updateCamera(PerspectiveCamera& cam, const cv::viz::Color& color)
{
    //Remove camera widgets if they exist
    try
    {
        viewer_->removeWidget(cam.getName());
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        printf("WARNING: Widget name does not exist. Ignoring...\n");
    }
    //Add updated camera widget
    addCamera(cam, color);
}

void Visualizer3D::addCamera(PerspectiveCamera& cam, const cv::viz::Color& color)
{
    //Create a 3D frustrum for the camera pose
    cv::viz::WCameraPosition cam_widget(cam.getVizCamera().getFov(), 0.1, color);
    viewer_->showWidget(cam.getName(), cam_widget, egAffine2cvAffine(cam.getPoseAsAffine3d()));
    //Create a text
    cv::Point3d pos(cam.getPoseAsAffine3d().translation()(0) - 0.01,
        cam.getPoseAsAffine3d().translation()(1) - 0.01,
        cam.getPoseAsAffine3d().translation()(2) - 0.01);
    double scale = 0.01;
    cv::viz::WText3D text_widget(cam.getName(), pos, scale, true, color);
    viewer_->showWidget("text_" + cam.getName(), text_widget);
}

void Visualizer3D::addMarkerMap(const aruco::MarkerMap & mm, const std::string& name, const cv::viz::Color& color)
{
    std::vector<cv::Point3f> p3d;
    std::vector<int> ids;
    mm.getIdList(ids, true);

    for(auto idx : ids)
    {
        aruco::Marker3DInfo info3d = mm.getMarker3DInfo(idx);
        std::vector<cv::Point3f> point3d = (info3d.points);

        cv::viz::WText3D text_widget(name+": "+std::to_string(info3d.id), point3d[0], 0.025, true, color);
        viewer_->showWidget("id_" + name + std::to_string(info3d.id), text_widget);
        viewer_->spinOnce(1, true);
        
        for(int j = 0; j <  4; j++)
        {
            p3d.push_back(point3d[j]); 
            int id = std::rand() % 100000; 

            if(j==3)
            {
                cv::viz::WLine line = cv::viz::WLine(point3d[j], point3d[0], color);	
                viewer_->showWidget("line_" + std::to_string(id), line);
            }
            else
            {
                cv::viz::WLine line = cv::viz::WLine(point3d[j], point3d[j+1], color);	
                viewer_->showWidget("line_" + std::to_string(id), line);
            }
            viewer_->spinOnce(1, true);
        }
    }

    cv::viz::WCloud markers = cv::viz::WCloud(p3d, color);
    markers.setRenderingProperty( cv::viz::POINT_SIZE, 4 );
    viewer_->showWidget("mm_" + name, markers);
}

void Visualizer3D::removeWidget(const std::string& name)
{
    //Remove sphere widgets if they exist
    try
    {
        viewer_->removeWidget(name);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        printf("WARNING: Widget name does not exist. Ignoring...\n");
    }
}

void Visualizer3D::spin()
{
    viewer_->spin();
}

void Visualizer3D::spinOnce(const int& time, const bool& force_redraw)
{
    viewer_->spinOnce(time, force_redraw);
}

bool Visualizer3D::wasStopped()
{
    return viewer_->wasStopped();
}