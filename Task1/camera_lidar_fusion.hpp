#pragma once

#include <pcl/io/pcd_io.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>

#include "opencv2/core/core.hpp"
#include <opencv2/imgcodecs.hpp> 
#include <opencv2/imgproc.hpp>



class CameraLidarFusion {
    public:
        using PointT = pcl::PointXYZ;
        using PointCloudT = pcl::PointCloud<PointT>;
        using PCPtr = PointCloudT::Ptr;
        using PCConstPtr = PointCloudT::ConstPtr;

        /*!
        * Constructor.
        * @param image path, point cloud path, and calibrated file.
        */
        CameraLidarFusion(const std::string& image_path, const std::string& cloud_path, const std::string& calibrated_file);

        /*!
        * Destructor.
        */
        ~CameraLidarFusion();

        /*!
        * Fused lidar and camera data.
        * @param  calibrated file.
        */
        void process( const std::string& calibrated_file);

        /*!
        * Reads lidar file.
        * @param  lidar file path.
        */
        void readingPointCloud(const std::string& file);

        /*!
        * Reads image file.
        * @param  image file path.
        */
        void readingImage(const std::string& file);

        /*!
        * Reads calibrated file and updates the matrices.
        * @param  calibrated file, intrinsic, base_to_cam, base_to_lidar matrices.
        */
        void readingCalibratedData(const std::string& file, cv::Mat& intrinsic, cv::Mat& base_to_cam, cv::Mat& base_to_lidar);
    
    private:
        cv::Mat inputImage_;

        PCPtr inputCloud_;

       
};