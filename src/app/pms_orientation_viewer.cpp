#include <iostream>
#include <sstream>

#include <opencv2/viz.hpp>
#include <opencv2/opencv.hpp>
// #include <opencv2/core/eigen.hpp>

#include <Eigen/Dense>

#include <libserial/SerialPort.h>
#include <cstdlib>
#include <cstring>
#include <unistd.h>

void PMS_Rt (LibSerial::DataBuffer buffer, std::vector<cv::Affine3d> & Rt)
{
    std::string row(buffer.begin(), buffer.end());

    if (!row.empty() && row.length() >= 19 && row.length() <= 23)
    {
        std::istringstream row_stream(row);
        std::string item_stream;
        std::vector<double> row_vector;

        while(std::getline(row_stream, item_stream, ' '))
        {
            double item = std::stod(item_stream);
            row_vector.push_back(item);
        }
            
        //std::cout <<  row_vector[0] << " " << row_vector[1] << " " << row_vector[2] << " " << row_vector[3] << std::endl;

        Eigen::Quaterniond q;
        Eigen::Matrix3d R33_eigen;
        Eigen::Matrix3d R33_eigen_T;

        cv::Vec3d t_cv;
        cv::Matx33d R33_cv;
        cv::Affine3d Rt_cv;

        t_cv << 0.00, 0.00, 0.00;

        q.w() = row_vector[0];
        q.x() = row_vector[1];
        q.y() = row_vector[2];
        q.z() = row_vector[3];

        // std::cout <<  q.w() << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;
        // R33_eigen = q.toRotationMatrix();

        //Conjugate quaternion is used here so the change of signs: q.w(), -q.x(), -q.y(), -q.z(), 
        //This quaternion describes the orientation of the earth frame relative to the sensor frame
        //The resulting rotation matrix represents the rotation of the global frame relative to the sensor frame

        R33_eigen(0,0) = 2*q.w()*q.w() - 1 + 2*q.x()*q.x();
        R33_eigen(0,1) = 2*(q.x()*q.y() + q.w()*q.z());
        R33_eigen(0,2) = 2*(q.x()*q.z() - q.w()*q.y());
        R33_eigen(1,0) = 2*(q.x()*q.y() - q.w()*q.z());
        R33_eigen(1,1) = 2*q.w()*q.w() - 1 + 2*q.y()*q.y();
        R33_eigen(1,2) = 2*(q.y()*q.z() + q.w()*q.x());
        R33_eigen(2,0) = 2*(q.x()*q.z() + q.w()*q.y());
        R33_eigen(2,1) = 2*(q.y()*q.z() - q.w()*q.x());
        R33_eigen(2,2) = 2*q.w()*q.w() - 1 + 2*q.z()*q.z();

        //The resulting rotation matrix represents the rotation of sensor frame relative to the global frame.
        R33_eigen_T = R33_eigen.transpose();

        for (int v = 0; v < 3; v++) 
            for (int u = 0; u < 3; u++) 
                R33_cv(u,v) = R33_eigen_T(u,v);

        //cv::eigen2cv(R33_eigen,R33_cv);
                
        Rt_cv.rotation(R33_cv);

        Rt.push_back(Rt_cv);
    }
}

void init_viz (std::string viz_name, cv::viz::Viz3d & viz)
{
    viz = cv::viz::getWindowByName(viz_name);
    viz.setBackgroundColor(cv::viz::Color::white());
    viz.setWindowPosition(cv::Point(0, 0));
    viz.setWindowSize(cv::Size(800, 400));

    // Plot the MAP ORIGIN
    cv::viz::WCoordinateSystem origin = cv::viz::WCoordinateSystem(0.25);
    viz.showWidget("origin", origin);
}

int main (int argc, char** argv)
{
    printf("\n%s\n\n", argv[0]);
    
    // Instantiate a SerialPort object.
    LibSerial::SerialPort serial_port;

    try
    {
        // Open the Serial Port at the desired hardware port.
        serial_port.Open("/dev/ttyUSB0");
    }
    catch(const LibSerial::OpenFailed& e1)
    {
        std::cerr << "Exception:" << e1.what() << std::endl;
        return 1; 
    }

    // Set the baud rate of the serial port.
    serial_port.SetBaudRate(LibSerial::BaudRate::BAUD_9600);

    // Wait for data to be available at the serial port.
    while(!serial_port.IsDataAvailable()) 
    {
        usleep(1000) ;
    }

    // Specify a timeout value (in milliseconds).
    size_t ms_timeout = 500;

    cv::viz::Viz3d pms_viz;
    init_viz ("PMS_Viewer", pms_viz);

    while(1) 
    {
        usleep(1000);
        LibSerial::DataBuffer read_buffer;

        try
        {
            // Read as many bytes as are available during the timeout period.
            serial_port.Read(read_buffer, 0, ms_timeout);
        }
        catch (const LibSerial::ReadTimeout&)
        {
            std::vector<cv::Affine3d> Rt_affine;
            PMS_Rt(read_buffer, Rt_affine);

            if(Rt_affine.size() == 1)
            {
                // std::cout << Rt_affine[0].rotation() << "\n" <<std::endl;

                pms_viz.showWidget("pms_rt", cv::viz::WTrajectory(Rt_affine, cv::viz::WTrajectory::FRAMES, 0.1, cv::viz::Color::black()));
                
                if (!pms_viz.wasStopped())
                {
                    pms_viz.spinOnce(1,true);
                }
                else
                {
                    return 0;
                }
            }
        }
    }
}