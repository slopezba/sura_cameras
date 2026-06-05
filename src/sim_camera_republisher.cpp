#include <memory>
#include <string>

#include <camera_info_manager/camera_info_manager.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

class SimCameraRepublisher : public rclcpp::Node
{
public:
  SimCameraRepublisher()
  : Node("sim_camera_republisher"),
    camera_info_manager_(this)
  {
    const std::string input_image_topic =
      declare_parameter<std::string>("input_image_topic", "");
    const std::string input_compressed_topic =
      declare_parameter<std::string>("input_compressed_topic", "");
    const std::string camera_name = declare_parameter<std::string>("camera_name", "");
    const std::string camera_info_url =
      declare_parameter<std::string>("camera_info_url", "");
    frame_id_ = declare_parameter<std::string>("frame_id", "");

    if (input_image_topic.empty()) {
      throw std::runtime_error("Parameter 'input_image_topic' cannot be empty.");
    }

    if (!camera_name.empty()) {
      camera_info_manager_.setCameraName(camera_name);
    }
    if (!camera_info_url.empty() && !camera_info_manager_.loadCameraInfo(camera_info_url)) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to load camera calibration from '%s'. Publishing basic CameraInfo.",
        camera_info_url.c_str());
    }

    auto sensor_qos = rclcpp::SensorDataQoS();
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", sensor_qos);
    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", rclcpp::QoS(10));

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      input_image_topic, sensor_qos,
      std::bind(&SimCameraRepublisher::on_image, this, std::placeholders::_1));

    const std::string compressed_topic =
      input_compressed_topic.empty() ? input_image_topic + "/compressed" : input_compressed_topic;
    compressed_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "image_raw/compressed", sensor_qos);
    compressed_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      compressed_topic, sensor_qos,
      std::bind(&SimCameraRepublisher::on_compressed_image, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Republishing simulated camera image '%s' as 'image_raw'",
      input_image_topic.c_str());
  }

private:
  void on_image(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    sensor_msgs::msg::Image image = *msg;
    if (!frame_id_.empty()) {
      image.header.frame_id = frame_id_;
    }
    image_pub_->publish(image);

    sensor_msgs::msg::CameraInfo info = camera_info_manager_.getCameraInfo();
    info.header = image.header;
    if (info.width == 0) {
      info.width = image.width;
    }
    if (info.height == 0) {
      info.height = image.height;
    }
    camera_info_pub_->publish(info);
  }

  void on_compressed_image(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    sensor_msgs::msg::CompressedImage image = *msg;
    if (!frame_id_.empty()) {
      image.header.frame_id = frame_id_;
    }
    compressed_pub_->publish(image);
  }

  std::string frame_id_;
  camera_info_manager::CameraInfoManager camera_info_manager_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimCameraRepublisher>());
  rclcpp::shutdown();
  return 0;
}
