#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "tracking_msgs/msg/detections3_d.hpp"
#include "tracking_msgs/msg/detection3_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class TfPreProc : public rclcpp::Node
{
  public:
    TfPreProc()
    : Node("tf_preproc_node")
    {
      this->timer_ = this->create_wall_timer(100ms, std::bind(&TfPreProc::callback, this));
      this->publisher_ = this->create_publisher<tracking_msgs::msg::Detections3D>("converted_detections_headset", 10);

      this->declare_parameter("tracker_frame",rclcpp::ParameterType::PARAMETER_STRING);
      this->declare_parameter("source_frame",rclcpp::ParameterType::PARAMETER_STRING);
      this->declare_parameter("labels",rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
      
      this->tracker_frame_ = this->get_parameter("tracker_frame").as_string();
      this->source_frame_ = this->get_parameter("source_frame").as_string();
      this->labels_ = this->get_parameter("labels").as_string_array();

      this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock(),500ms);
      this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      // Block until transform becomes available
      this->tf_buffer_->canTransform(this->source_frame_,this->tracker_frame_,tf2::TimePointZero,10s);
      this->det_msg_ = tracking_msgs::msg::Detection3D();
      this->dets_msg_ = tracking_msgs::msg::Detections3D();
      this->dets_msg_.detections.reserve(this->max_dets_);
    }

  private:
    void callback()
    {
        this->dets_msg_ = tracking_msgs::msg::Detections3D();
        this->dets_msg_.header.stamp = this->get_clock()->now();   
        this->dets_msg_.header.frame_id = this->tracker_frame_;

        pose_in.header.stamp = this->dets_msg_.header.stamp;
        pose_in.header.frame_id = this->source_frame_;

        this->det_msg_ = tracking_msgs::msg::Detection3D();
        

        // Convert spatial information
        tf_buffer_->transform(pose_in,pose_out,this->tracker_frame_,tf2::TimePointZero,this->source_frame_,200ms);
        this->det_msg_.pose = pose_out.pose;
        this->det_msg_.bbox.center.position = pose_out.pose.position;
        this->det_msg_.bbox.center.orientation = pose_out.pose.orientation;
        this->det_msg_.bbox.size.x = 0;
        this->det_msg_.bbox.size.y = 0;
        this->det_msg_.bbox.size.z = 0;

        // Add semantic information
        this->det_msg_.class_string = this->labels_[0];
        this->det_msg_.class_confidence = .99;
        this->dets_msg_.detections.emplace_back(this->det_msg_);

        this->publisher_->publish(this->dets_msg_);
    }

    std::string tracker_frame_;
    std::string source_frame_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    geometry_msgs::msg::PoseStamped pose_in;
    geometry_msgs::msg::PoseStamped pose_out;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<tracking_msgs::msg::Detections3D>::SharedPtr publisher_;
    tracking_msgs::msg::Detections3D dets_msg_;
    tracking_msgs::msg::Detection3D det_msg_;
    int max_dets_{5}; 

    std::vector<std::string> labels_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfPreProc>());
  rclcpp::shutdown();
  return 0;
}