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

void PMS_Rt (std::string row, std::vector<cv::Affine3d> & Rt, Eigen::Quaterniond & q)
{
    std::istringstream row_stream(row);
    std::string item_stream;
    std::vector<std::string> row_vector;

    while(std::getline(row_stream, item_stream, ' '))
    {
        std::string item = item_stream;
        row_vector.push_back(item);
    }
        
    // std::cout <<  row_vector[0] << " " << row_vector[1] << " " << row_vector[2] << " " << row_vector[3] << " " << row_vector[4] << std::endl;

    Eigen::Matrix3d R33_eigen;
    Eigen::Matrix3d R33_eigen_T;

    cv::Matx33d R33_cv;
    cv::Affine3d Rt_cv;

    q.w() = std::stod(row_vector[0]);
    q.x() = std::stod(row_vector[1]);
    q.y() = std::stod(row_vector[2]);
    q.z() = std::stod(row_vector[3]);

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

    cv::Vec3d t_cv;

    double z = (std::stod(row_vector[4])); // in meters

    t_cv << 0.00, 0.00, z;

    // std::cout << t_cv <<std::endl;

    Rt_cv.translation(t_cv);

    Rt.push_back(Rt_cv);
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
        usleep(1000) ;

    // Specify a timeout value (in milliseconds).
    size_t ms_timeout = 550;

    cv::viz::Viz3d pms_viz;
    init_viz ("PMS_Viewer", pms_viz);

    int aux = 0, N = 100;

    double sum_tz = 0, sum_qx = 0, sum_qy = 0, sum_qz = 0;

    std::vector<double> errors_t, errors_qx, errors_qy, errors_qz;

    while(aux<N) 
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
            std::string row(read_buffer.begin(), read_buffer.end());

            // -0.00_-0.00_-0.00_-0.00_0.00
            if (!row.empty() && row.size() >= 24 && row.size() <= 28)
            {
                std::vector<cv::Affine3d> Rt_affine;
                Eigen::Quaterniond q1 = Eigen::Quaterniond::Identity();
                Eigen::Quaterniond q2;
                Eigen::Quaterniond error_q;

                PMS_Rt(row, Rt_affine, q2);

                if(Rt_affine[0].translation().val[2] >= 0.49 && Rt_affine[0].translation().val[2] <= 0.51)
                {
                    pms_viz.showWidget("pms_rt", cv::viz::WTrajectory(Rt_affine, cv::viz::WTrajectory::BOTH, 0.1, cv::viz::Color::blue()));
                    
                    //error in the tralation components (only in z))
                    double error_tz = std::fabs(Rt_affine[0].translation().val[2] - 0.5); // meters
                    errors_t.push_back(error_tz);

                    //error in the quaternion components (x,y,z)
                    error_q = q2 * q1.inverse();
                    double error_qx = std::fabs(error_q.x());
                    double error_qy = std::fabs(error_q.y());
                    double error_qz = std::fabs(error_q.z());

                    errors_qx.push_back(error_qx);
                    errors_qy.push_back(error_qy);
                    errors_qz.push_back(error_qz);

                    sum_tz+=error_tz;
                    sum_qx+=error_qx;
                    sum_qy+=error_qy;
                    sum_qz+=error_qz;

                    // the angle (in radian) between two rotations
                    // double angularDistance = q2.angularDistance(q1);
                    // std::cout << Rt_affine[0].translation().val[2] << std::endl; 
                    // std::cout <<  q1.w() << " " << q1.x() << " " << q1.y() << " " << q1.z() << std::endl;
                    // std::cout <<  q2.w() << " " << q2.x() << " " << q2.y() << " " << q2.z() << std::endl;
                    // std::cout <<  diff.w() << " " << diff.x() << " " << diff.y() << " " << diff.z() << std::endl;
                    // std::cout << angularDistance << std::endl;

                    aux++;
                }
            }

            if (!pms_viz.wasStopped())
                pms_viz.spinOnce(1,true);
            else
                return 0;

        }
    }

    double sum_tz2 = 0;
    double sum_qx2 = 0;
    double sum_qy2 = 0;
    double sum_qz2 = 0;

    double mean_tz = sum_tz/N;
    double mean_qx = sum_qx/N;
    double mean_qy = sum_qy/N;
    double mean_qz = sum_qz/N;

    for(int i = 0; i < N; i++)
    {
        sum_tz2 += (errors_t[i] - mean_tz)*(errors_t[i] - mean_tz);
        sum_qx2 += (errors_qx[i] - mean_qx)*(errors_qx[i] - mean_qx);
        sum_qy2 += (errors_qy[i] - mean_qy)*(errors_qy[i] - mean_qy);
        sum_qz2 += (errors_qz[i] - mean_qz)*(errors_qz[i] - mean_qz);
    }

    double var_tz = sum_tz2/N;
    double var_qx = sum_qx2/N;
    double var_qy = sum_qy2/N;
    double var_qz = sum_qz2/N;

    std::cout << "Variance in tz: " << var_tz << std::endl; 
    std::cout << "Variance in qx: " << var_qx << std::endl;
    std::cout << "Variance in qy: " << var_qy << std::endl;
    std::cout << "Variance in qz: " << var_qz << std::endl;
}