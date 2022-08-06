#pragma once

#include <pcl/io/pcd_io.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include <pcl/common/transforms.h>

#include "opencv2/core/core.hpp"
#include <opencv2/imgcodecs.hpp> 
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "opencv2/calib3d/calib3d.hpp"
#include <pcl/common/common.h>
#include <yaml-cpp/yaml.h>

class CameraLidarFusion {
    public:
        using PointT = pcl::PointXYZ;
        using PointCloudT = pcl::PointCloud<PointT>;
        using PCPtr = PointCloudT::Ptr;
        using PCConstPtr = PointCloudT::ConstPtr;

        CameraLidarFusion(const std::string& image_path, const std::string& cloud_path, const std::string& calibrated_file);
        void process( const std::string& calibrated_file);
        void readingPointCloud(const std::string& file);
        void readingImage(const std::string& file);
        void readingCalibratedData(const std::string& file, cv::Mat& intrinsic, cv::Mat& base_to_cam, cv::Mat& base_to_lidar);
        void showLidarTopview();
    
    private:
        cv::Mat inputImage_;
        PCPtr inputCloud_;
        YAML::Node cfg_;
       
};