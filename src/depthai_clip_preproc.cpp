#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "tracking_msgs/msg/detections3_d.hpp"
#include "tracking_msgs/msg/detection3_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp>

using std::placeholders::_1;

class DepthAIPreProc : public rclcpp::Node
{
  public:
    DepthAIPreProc()
    : Node("depthai_preproc_node")
    {
      this->subscription_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
      "depthai_detections", 10, std::bind(&DepthAIPreProc::topic_callback, this, _1));
      this->img_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "depthai_image", 10, std::bind(&DepthAIPreProc::image_callback, this, _1));
      this->publisher_ = this->create_publisher<tracking_msgs::msg::Detections3D>("converted_detections", 10);
    //   this->img_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("test_image", 10);

      this->declare_parameter("tracker_frame",rclcpp::ParameterType::PARAMETER_STRING);
      this->declare_parameter("labels",rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
      
      this->tracker_frame_ = this->get_parameter("tracker_frame").as_string();
      this->labels_ = this->get_parameter("labels").as_string_array();

      this->tf_buffer_ =std::make_unique<tf2_ros::Buffer>(this->get_clock());
      this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      this->det_msg_ = tracking_msgs::msg::Detection3D();
      this->dets_msg_ = tracking_msgs::msg::Detections3D();
      this->dets_msg_.detections.reserve(this->max_dets_);
    }

  private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        this->last_img_ = msg;   
        this->cv_ptr_bgr_ = cv_bridge::toCvCopy(this->last_img_, sensor_msgs::image_encodings::BGR8);
    }
    
    void topic_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
    {
      rclcpp::Time time_det_rcvd = this->get_clock()->now();
      diagnostic_msgs::msg::KeyValue kv;     

      this->dets_msg_ = tracking_msgs::msg::Detections3D();
      this->dets_msg_.header.stamp = msg->header.stamp;     
      this->dets_msg_.header.frame_id = this->tracker_frame_;

      // Add metadata for later analysis
      kv.key = "time_det_rcvd";
      kv.value = std::to_string(time_det_rcvd.nanoseconds());
      this->dets_msg_.metadata.emplace_back(kv);
      kv.key = "num_dets_rcvd";
      kv.value = std::to_string(msg->detections.size());
      this->dets_msg_.metadata.emplace_back(kv);


      cv::Mat cv_mat_out = cv::Mat::zeros(this->cv_ptr_bgr_->image.size(), this->cv_ptr_bgr_->image.type());

      for (auto it = msg->detections.begin(); it != msg->detections.end(); it++)
      {
            this->det_msg_ = tracking_msgs::msg::Detection3D();

            // Convert spatial information
            this->obj_pose_det_frame_ = geometry_msgs::msg::PoseStamped();
            this->obj_pose_det_frame_.header = msg->header;
            this->obj_pose_det_frame_.header.stamp = rclcpp::Time(0);
            this->obj_pose_det_frame_.pose = it->results[0].pose.pose;
            this->obj_pose_trk_frame_ = this->tf_buffer_->transform(this->obj_pose_det_frame_,this->tracker_frame_);
            this->det_msg_.pose = obj_pose_trk_frame_.pose;
            this->det_msg_.bbox.center = obj_pose_trk_frame_.pose;
            this->det_msg_.bbox.size.x = 0;
            this->det_msg_.bbox.size.y = 0;
            this->det_msg_.bbox.size.z = 0;

            // Add semantic information
            this->det_msg_.class_string = this->labels_[std::stoi(it->results[0].hypothesis.class_id)];
            this->det_msg_.class_confidence = it->results[0].hypothesis.score;

            this->dets_msg_.detections.emplace_back(this->det_msg_);



            // Draw BB around opencv mat
            cv::Mat mask = cv::Mat::zeros(this->cv_ptr_bgr_->image.size(), CV_8U);

            int xmax = std::min((int)(it->bbox.center.position.x + it->bbox.size.x/(2)), cv_ptr_bgr_->image.cols);
            int xmin = std::max((int)(it->bbox.center.position.x - it->bbox.size.x/(2)), 0);
            int ymax = std::min((int)(it->bbox.center.position.y + it->bbox.size.y/(2)), cv_ptr_bgr_->image.rows);
            int ymin = std::max((int)(it->bbox.center.position.y - it->bbox.size.y/(2)), 0);


            // cv::Rect maskRect(xmin, ymin, xmax-xmin, ymax-ymin);
            // mask(maskRect) = cv::Scalar(255);
            this->cv_ptr_bgr_->image.copyTo(cv_mat_out, mask);
            
      }

      
      this->publisher_->publish(this->dets_msg_);

    //   cv::imwrite("./test_img.jpg", cv_mat_out);
    //   // Declare what you need
    //   cv::FileStorage file("some_name.ext", cv::FileStorage::WRITE);

    //   // Write to file!
    //   file << "matName" << this->cv_ptr_bgr_->image;

    //   // Close the file and release all the memory buffers
    //   file.release();
           
    }

    std::string tracker_frame_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    geometry_msgs::msg::PoseStamped obj_pose_det_frame_;
    geometry_msgs::msg::PoseStamped obj_pose_trk_frame_;
    sensor_msgs::msg::Image::SharedPtr last_img_;
    cv_bridge::CvImagePtr cv_ptr_bgr_{std_msgs::msg::Header(),"bgr8",cv::Mat::zeros(1280,720)};


    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_subscription_;
    rclcpp::Publisher<tracking_msgs::msg::Detections3D>::SharedPtr publisher_;
    tracking_msgs::msg::Detections3D dets_msg_;
    tracking_msgs::msg::Detection3D det_msg_;
    int max_dets_{250}; 

    std::vector<std::string> labels_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthAIPreProc>());
  rclcpp::shutdown();
  return 0;
}