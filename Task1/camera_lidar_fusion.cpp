#include "camera_lidar_fusion.hpp"


CameraLidarFusion::CameraLidarFusion(){
    
}

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
    intrinsic.at<double>(1, 0) = 0.;
    intrinsic.at<double>(1, 1) = 750.03420145;
    intrinsic.at<double>(1, 2) = 600.;
    intrinsic.at<double>(2, 0) = 0.;
    intrinsic.at<double>(2, 1) = 0.;
    intrinsic.at<double>(2, 2) = 1.;
   
   
   
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

cv::Mat CameraLidarFusion::process(const PCPtr& cloud, cv::Mat& image,const std::string& calibrated_file){

    cv::Mat  intrinsic(3,3 , cv::DataType<double>::type);
    cv::Mat  base_to_cam (4,4 , cv::DataType<double>::type);
    cv::Mat  base_to_lidar (4,4 , cv::DataType<double>::type);
    readingCalibratedData(calibrated_file, intrinsic, base_to_cam, base_to_lidar);
    return image;
}

int main(){
    std::string calibrated_file = "/home/vijay/Documents/CV/cv_2022/motorai/sensor_data_fusion/camera_lidar_data/outfile.txt";
    CameraLidarFusion camera_lidar;
    //camera_lidar.readingCalibratedData(calibrated_file);


    return 0;
}
