#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/CameraInfo.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <chrono>
class LidarToDepthConverter {
public:
    LidarToDepthConverter(ros::NodeHandle& nh) : nh_(nh) {        
        is_cam_ready = false;
        max_depth = 50.0f;                
        DIAMOND_KERNEL_5 = (cv::Mat_<uchar>(5, 5) <<
    0, 0, 1, 0, 0,
    0, 1, 1, 1, 0,
    1, 1, 1, 1, 1,
    0, 1, 1, 1, 0,
    0, 0, 1, 0, 0);
       
        in_fill = true; // Default value is true
        ros::param::get("in_fill", in_fill); // Try to get value from parameter, if available
        camera_info_sub_ = nh_.subscribe<sensor_msgs::CameraInfo>("/hikrobot_camera/camera_info", 1, &LidarToDepthConverter::cameraInfoCallback, this);        
        point_cloud_sub = nh_.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 1, &LidarToDepthConverter::pointCloudCallback, this);
        depth_image_pub = nh_.advertise<sensor_msgs::Image>("/lidar_depth", 1);
    }

private:
    bool in_fill;
    ros::NodeHandle nh_;
    ros::Subscriber point_cloud_sub, camera_info_sub_;         
    ros::Publisher depth_image_pub;
    sensor_msgs::CameraInfo cam_info;
    bool is_cam_ready;
    float centre_x, centre_y, focal_x, focal_y;
    float max_depth;
    
    cv::Mat DIAMOND_KERNEL_5;
    

    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {        
        cam_info = *msg;
        centre_x = cam_info.K[2];
        centre_y = cam_info.K[5];
        focal_x = cam_info.K[0];
        focal_y = cam_info.K[4];
        is_cam_ready = true;
    }



    cv::Mat fill_in_fast(cv::Mat depthMap, bool extrapolate=true, std::string blur_type="bilateral") {    
        // Invert
        depthMap = max_depth - depthMap;                
        cv::dilate(depthMap, depthMap, DIAMOND_KERNEL_5);

        // Hole closing
        cv::morphologyEx(depthMap, depthMap, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

        // Fill empty spaces with dilated values
        cv::Mat dilated;
        cv::dilate(depthMap, dilated, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7)));
        cv::Mat emptyPixels = (depthMap < 0.1f);
        dilated.copyTo(depthMap, emptyPixels);

        // Large Fill
        // cv::dilate(depthMap, depthMap, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(31, 31)));     
        // // Median blur
        cv::medianBlur(depthMap, depthMap, 5);

        // // Bilateral or Gaussian blur
        // if (blurType == "bilateral") {
        //     // Bilateral blur
        //     cv::bilateralFilter(depthMap, depthMap, 5, 1.5, 2.0);
        // } else if (blurType == "gaussian") {
        //     // Gaussian blur
            cv::Mat blurred;
            cv::GaussianBlur(depthMap, blurred, cv::Size(5, 5), 0);
            cv::Mat validPixels = (depthMap > 0.1f);
            blurred.copyTo(depthMap, validPixels);
        // }
        // // Invert
        depthMap = max_depth - depthMap;

        return depthMap;
    }


                
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        
        if (is_cam_ready){
            //  auto start = std::chrono::high_resolution_clock::now();

            pcl::PointCloud<pcl::PointXYZ> cloud;
            pcl::fromROSMsg(*msg, cloud);
            
            cv::Mat cv_image = cv::Mat(cam_info.height, cam_info.width, CV_32FC1,cv::Scalar(max_depth));

            for (int i = 0; i < cloud.points.size(); i++) {
                float x_in_cam = -cloud.points[i].y;
                float y_in_cam = -cloud.points[i].z;
                float z_in_cam = cloud.points[i].x;
                float z = z_in_cam;;
                float u = (x_in_cam  * focal_x) / z;
                float v = (y_in_cam  * focal_y) / z;
                int pixel_pos_x = (int)(u + centre_x);
                int pixel_pos_y = (int)(v + centre_y);

                if (pixel_pos_x > (cam_info.width - 1)) {
                    pixel_pos_x = cam_info.width - 1;
                }
                if (pixel_pos_y > (cam_info.height - 1)) {
                    pixel_pos_y = cam_info.height - 1;
                }

                if (z < max_depth){
                    cv_image.at<float>(pixel_pos_y, pixel_pos_x) = z;                    
                }
                
          
                
            }
            if(in_fill){ cv_image = fill_in_fast(cv_image); }
            
            cv_image.convertTo(cv_image, CV_16UC1,1000.0f);
            

            sensor_msgs::ImagePtr output_image = cv_bridge::CvImage(std_msgs::Header(), "16UC1", cv_image).toImageMsg();
          
            output_image->header = msg->header;
            output_image->header.stamp = msg->header.stamp;
            depth_image_pub.publish(output_image);

            //  auto end = std::chrono::high_resolution_clock::now();
            // // Calculate the duration in milliseconds
            // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            // // Output the duration
            // std::cout << "Time taken: " << duration.count() << " milliseconds" << std::endl;
        }
    }



};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_to_depth");
    ros::NodeHandle nh;
    LidarToDepthConverter converter(nh);

    ros::spin();

    return 0;
}












    
