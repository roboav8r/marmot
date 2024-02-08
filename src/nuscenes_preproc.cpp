#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "tracking_msgs/msg/detections3_d.hpp"
#include "tracking_msgs/msg/detection3_d.hpp"
#include "foxglove_msgs/msg/scene_update.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

using std::placeholders::_1;

class DetConverter : public rclcpp::Node
{
  public:
    DetConverter()
    : Node("detection_converter")
    {

      subscription_ = this->create_subscription<foxglove_msgs::msg::SceneUpdate>(
      "nuscenes_detections", 10, std::bind(&DetConverter::topic_callback, this, _1));

      publisher_ = this->create_publisher<tracking_msgs::msg::Detections3D>("converted_detections", 10);

      det_msg_ = tracking_msgs::msg::Detection3D();
      dets_msg_ = tracking_msgs::msg::Detections3D();
      dets_msg_.detections.reserve(this->max_dets_);
    }

  private:
    void topic_callback(const foxglove_msgs::msg::SceneUpdate::SharedPtr msg)
    {
     
      if (msg->entities.size() > 0) // If there are messages to be published
      {
        this->dets_msg_ = tracking_msgs::msg::Detections3D();
        this->dets_msg_.header.stamp = msg->entities[0].timestamp;
        this->dets_msg_.header.frame_id = msg->entities[0].frame_id;
        this->dets_msg_.detections.reserve(this->max_dets_);

        for (auto it = msg->entities.begin(); it != msg->entities.end(); it++)
        {
              this->det_msg_ = tracking_msgs::msg::Detection3D();

              // Convert entity to detection3d
              this->det_msg_.pose = it->cubes[0].pose;
              this->det_msg_.class_string = it->metadata[0].value;
              this->det_msg_.class_confidence = std::stof(it->metadata[1].value);
              this->det_msg_.attribute = it->metadata[2].value;
              this->det_msg_.bbox.center = it->cubes[0].pose;
              this->det_msg_.bbox.size = it->cubes[0].size;

              diagnostic_msgs::msg::KeyValue kv;
              kv.key = it->metadata[3].key;
              kv.value = it->metadata[3].value;
              this->det_msg_.metadata.emplace_back(kv);

              // Add detection to Detections3d
              this->dets_msg_.detections.emplace_back(det_msg_);
        }

        this->publisher_->publish(dets_msg_);

      } else { // Don't publish a message
        return;

      }
           
    }
    
    rclcpp::Subscription<foxglove_msgs::msg::SceneUpdate>::SharedPtr subscription_;
    rclcpp::Publisher<tracking_msgs::msg::Detections3D>::SharedPtr publisher_;
    tracking_msgs::msg::Detections3D dets_msg_;
    tracking_msgs::msg::Detection3D det_msg_;
    int max_dets_{250}; 

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DetConverter>());
  rclcpp::shutdown();
  return 0;
}