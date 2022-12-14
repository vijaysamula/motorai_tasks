#include "camera_lidar_fusion.hpp"


CameraLidarFusion::CameraLidarFusion(const std::string& image_path,const std::string& cloud_path, const std::string& calibrated_file):inputCloud_(new pcl::PointCloud<PointT>)
{
    readingImage(image_path);
    readingPointCloud(cloud_path);
    process( calibrated_file);
}

CameraLidarFusion::~CameraLidarFusion(){}

void CameraLidarFusion::readingImage(const std::string& file){
    try {
        inputImage_ = cv::imread(file, cv::IMREAD_COLOR);
    }
    catch (const cv::Exception& ex){
        std::cerr<<"Couldn't read image from "<< file << std::endl;
    }
}

void CameraLidarFusion::readingPointCloud(const std::string& file){

    if (pcl::io::loadPCDFile<PointT> (file, *inputCloud_) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << inputCloud_->points.size () << " data points from "+file << std::endl;

    
}

void CameraLidarFusion::readingCalibratedData(const std::string& file, cv::Mat& intrinsic, cv::Mat& base_to_cam, cv::Mat& base_to_lidar){
  
    intrinsic.at<double>(0, 0) = 750.03420145;
    intrinsic.at<double>(0, 1) = 0.;
    intrinsic.at<double>(0, 2) = 960.;
    intrinsic.at<double>(0, 3) = 0.;
    intrinsic.at<double>(1, 0) = 0.;
    intrinsic.at<double>(1, 1) = 750.03420145;
    intrinsic.at<double>(1, 2) = 600.;
    intrinsic.at<double>(1, 3) = 0.;
    intrinsic.at<double>(2, 0) = 0.;
    intrinsic.at<double>(2, 1) = 0.;
    intrinsic.at<double>(2, 2) = 1.;
    intrinsic.at<double>(2, 3) = 0.;
   
   
   
    base_to_cam.at<double>(0, 0) = 1. ;
    base_to_cam.at<double>(0, 1) = 0. ;
    base_to_cam.at<double>(0, 2) = 0. ;
    base_to_cam.at<double>(0, 3) = 2.29 ;
    base_to_cam.at<double>(1, 0) = 0. ;
    base_to_cam.at<double>(1, 1) = 1. ;
    base_to_cam.at<double>(1, 2) = 0. ;
    base_to_cam.at<double>(1, 3) = 0.033 ;
    base_to_cam.at<double>(2, 0) = 0. ;
    base_to_cam.at<double>(2, 1) = 0. ;
    base_to_cam.at<double>(2, 2) = 1. ;
    base_to_cam.at<double>(2, 3) = 1.97 ;
    base_to_cam.at<double>(3, 0) = 0. ;
    base_to_cam.at<double>(3, 1) = 0. ;
    base_to_cam.at<double>(3, 2) = 0. ;
    base_to_cam.at<double>(3, 3) = 1. ;

    base_to_lidar.at<double>(0, 0) = 1. ;
    base_to_lidar.at<double>(0, 1) = 0. ;
    base_to_lidar.at<double>(0, 2) = 0. ;
    base_to_lidar.at<double>(0, 3) = 2.2 ;
    base_to_lidar.at<double>(1, 0) = 0. ;
    base_to_lidar.at<double>(1, 1) = 1. ;
    base_to_lidar.at<double>(1, 2) = 0. ;
    base_to_lidar.at<double>(1, 3) = 0.  ;
    base_to_lidar.at<double>(2, 0) = 0. ;
    base_to_lidar.at<double>(2, 1) = 0. ;
    base_to_lidar.at<double>(2, 2) = 1. ;
    base_to_lidar.at<double>(2, 3) = 2.2 ;
    base_to_lidar.at<double>(3, 0) = 0. ;
    base_to_lidar.at<double>(3, 1) = 0. ;
    base_to_lidar.at<double>(3, 2) = 0. ;
    base_to_lidar.at<double>(3, 3) = 1.  ;
}

void CameraLidarFusion::process(const std::string& calibrated_file){
    cv::Mat cloud_image = inputImage_.clone();
    // Initialing the matrices 
    cv::Mat  intrinsic(3,4 , cv::DataType<double>::type);
    cv::Mat  base_to_cam (4,4 , cv::DataType<double>::type);
    cv::Mat  base_to_lidar (4,4 , cv::DataType<double>::type);
    readingCalibratedData(calibrated_file, intrinsic, base_to_cam, base_to_lidar);

    cv::Mat homogeneous_xyz(4, 1, cv::DataType<double>::type);
    cv::Mat cartesian_uv(3, 1, cv::DataType<double>::type);

    // Determining the cam to lidar transformation matrix
    cv::Mat lidar_to_camera = base_to_cam.inv() * base_to_lidar;
    std::cout<<"The lidar to camera transformation matrx: "<< lidar_to_camera<<std::endl;
    for (auto pt : *inputCloud_){
        
        if (pt.z < -2.35 ) continue;   // Removed the road points
        /* As the given data is from UE4 Carla simulator. We must change from Carla's coordinate system to a standard camera coordinate system.

          z                       z
            |    x                 /
            |   /                 / 
            |  /           to    /___________x
            | /                 |
            |/__________y       |y
        
        (x,y,z)  to (y, -z, x)
        */
        homogeneous_xyz.at<double>(0,0) = pt.y;
        homogeneous_xyz.at<double>(1,0) = -pt.z;
        homogeneous_xyz.at<double>(2,0) = pt.x;
        homogeneous_xyz.at<double>(3,0) = 1;

        // Projection from homogeneous coordinates to catesian co-ordinates
        cartesian_uv = intrinsic * lidar_to_camera * homogeneous_xyz;

        cv::Point pt_lidar;
        pt_lidar.x = cartesian_uv.at<double>(0,0)/cartesian_uv.at<double>(2,0);
        pt_lidar.y = cartesian_uv.at<double>(1,0)/cartesian_uv.at<double>(2,0);

        // Coloring the pixels based on the x value 
        float val = pt.x;
        float maxVal = 10.0;
        int red = std::min(255, (int) (255 * std::abs((val - maxVal) / maxVal)));
        int green = std::min(255, (int) (255 * (1 - std::abs((val - maxVal) / maxVal))));
        cv::circle(cloud_image, pt_lidar, 3, cv::Scalar(0,green, red), -1);
        //std::cout<<pt_lidar<<std::endl;;
    }
    cv::imwrite("/home/vijay/Documents/CV/cv_2022/motorai/sensor_data_fusion/camera_lidar_data/camera_lidar.jpg", cloud_image);
    
}


int main(){
    std::string calibrated_file = "/home/vijay/Documents/CV/cv_2022/motorai/sensor_data_fusion/camera_lidar_data/outfile.txt";
    std::string image_path = "/home/vijay/Documents/CV/cv_2022/motorai/sensor_data_fusion/camera_lidar_data/camera_front_left_frame_10460.jpg";
    std::string cloud_path = "/home/vijay/Documents/CV/cv_2022/motorai/sensor_data_fusion/camera_lidar_data/lidar.pcd";
    CameraLidarFusion camera_lidar(image_path, cloud_path,calibrated_file);
    
    



    return 0;
}
