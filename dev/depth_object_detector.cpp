#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

#include "utility_ros_msgs/ObjectArray.h"

#include <Eigen/Core>
#include <Eigen/Dense>

#include "visualization_msgs/MarkerArray.h"

bool image_received = false;
bool depth_is_subscribed = false;
pcl::PointCloud<pcl::PointXYZ> depth;
int cols, rows;
cv::Mat cloudImage;
void pcsCB(const sensor_msgs::PointCloud2ConstPtr& input)
{
    if (image_received)
    {
        pcl::fromROSMsg( *input, depth);
        cloudImage.create(rows,cols,CV_32FC3);
        for(size_t i=0;i<cols;i++){
          for(size_t j=0;j<rows;j++){
            cloudImage.at<cv::Vec3f>(j,i)[0] = depth.at(i,j).x;
            cloudImage.at<cv::Vec3f>(j,i)[1] = depth.at(i,j).y;
            cloudImage.at<cv::Vec3f>(j,i)[2] = depth.at(i,j).z;
          }
        }
        depth_is_subscribed = true;
    }
}

utility_ros_msgs::ObjectArray msg;
int sampling_pixel_step = 3;
int sampling_num = pow(3.0, 2.0);//n^2をいれること
void yoloCB(const darknet_ros_msgs::BoundingBoxes &boxes)
{
	// ROS_INFO("boundingbox_callback");
    msg.objects.clear();
    int count;
	Eigen::Vector3f object_sampling_position;
    utility_ros_msgs::Object object;
    for (int i = 0; i < boxes.bounding_boxes.size(); i++)
    {
        object.bbPix.x = boxes.bounding_boxes[i].xmin;
        object.bbPix.y = boxes.bounding_boxes[i].ymin;
        object.bbPix.w = boxes.bounding_boxes[i].xmax - boxes.bounding_boxes[i].xmin;
        object.bbPix.h = boxes.bounding_boxes[i].ymax - boxes.bounding_boxes[i].ymin;

        object.Class = boxes.bounding_boxes[i].Class;

        if(depth_is_subscribed)
        {
            count = 0;
            object_sampling_position = Eigen::Vector3f::Zero();
            //複数の点のpointcloudの平均を取る
            cv::Point center((boxes.bounding_boxes[i].xmin + boxes.bounding_boxes[i].xmax)/2, (boxes.bounding_boxes[i].ymin + boxes.bounding_boxes[i].ymax)/2);
            for (int t = 0; t < sqrt(sampling_num); t++)
            {
                for(int s=0; s<sqrt(sampling_num); s++)
                {
                    if(0<(center.x + sampling_pixel_step*(t-1))
                    && (center.x + sampling_pixel_step*(t-1))<cols
                    && 0<(center.y + sampling_pixel_step*(s-1))
                    && (center.y + sampling_pixel_step*(s-1))<rows)
                    {
                        if(!std::isnan(cloudImage.at<cv::Vec3f>(center.y + sampling_pixel_step*(s-1), center.x + sampling_pixel_step*(t-1))[2]))
                        {
                          object_sampling_position(0) = object_sampling_position(0) + cloudImage.at<cv::Vec3f>(center.y + sampling_pixel_step*(s-1), center.x + sampling_pixel_step*(t-1))[1];//カメラ座標系x軸 //入れ替え x,y inoue
                          object_sampling_position(1) = object_sampling_position(1) - cloudImage.at<cv::Vec3f>(center.y + sampling_pixel_step*(s-1), center.x + sampling_pixel_step*(t-1))[0];//カメラ座標系x軸
                          object_sampling_position(2) = object_sampling_position(2) + cloudImage.at<cv::Vec3f>(center.y + sampling_pixel_step*(s-1), center.x + sampling_pixel_step*(t-1))[2];//カメラ座標系x軸
                          count = count + 1;
                        }
                    }
                }
            }
            if (count > 0)
            {
            
                object.position3d.x = object_sampling_position(0)/count;//カメラ座標系x軸
                object.position3d.y = object_sampling_position(1)/count;//カメラ座標系x軸
                object.position3d.z = object_sampling_position(2)/count;//カメラ座標系x軸
                ROS_INFO("object[%s]: (%lf, %lf, %lf)", object.Class.c_str(), object.position3d.x, object.position3d.y, object.position3d.z);
                if (!std::isnan(object.position3d.x) && !std::isnan(object.position3d.y) && !std::isnan(object.position3d.z))
                    msg.objects.push_back(object);
            }
        }

    }
}

int main (int argc, char **argv)
{
    ros::init (argc,argv,"depth_position_detector");
    ros::NodeHandle nh;

    std::string yolo_topic = "/darknet_ros/bounding_boxes";
    std::string pcs_topic = "/camera/depth/points";
    std::string objectArray_topic = "/bounding_boxes_pix3d";
    // std::string rviz_topic = "/orb_slam2_mono/rviz_objectArray";

    ros::Subscriber yolo_sub = nh.subscribe(yolo_topic, 1, &yoloCB);
	ros::Subscriber pcs_sub = nh.subscribe(pcs_topic, 1, &pcsCB);
    ros::Publisher object_pub = nh.advertise<utility_ros_msgs::ObjectArray>(objectArray_topic, 1);
    // ros::Publisher object_pub_rviz = nh.advertise<visualization_msgs::MarkerArray>(rviz_topic, 1);

    ros::Rate loop_rate(20);
	while (ros::ok())
    {
        ros::spinOnce();
        object_pub.publish(msg);
        loop_rate.sleep();
    }

  return 0;
}
