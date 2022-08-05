#pragma once

#include <pcl/io/pcd_io.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include <pcl/common/transforms.h>

#include "opencv2/core/core.hpp"
#include <opencv2/imgcodecs.hpp> 

#include <yaml-cpp/yaml.h>

class CameraLidarFusion {
    public:
        using PointT = pcl::PointXYZI;
        using PointCloudT = pcl::PointCloud<PointT>;
        using PCPtr = PointCloudT::Ptr;
        using PCConstPtr = PointCloudT::ConstPtr;

        CameraLidarFusion();
        cv::Mat process(const PCPtr& cloud, cv::Mat& image, const std::string& calibrated_file);
        void readingPointCloud(const std::string& file);
        void readingImage(const std::string& file);
        void readingCalibratedData(const std::string& file, cv::Mat& intrinsic, cv::Mat& base_to_cam, cv::Mat& base_to_lidar);
        
    
    private:
        cv::Mat inputImage_;
        PCPtr inputCloud_;
        YAML::Node cfg_;
       
};