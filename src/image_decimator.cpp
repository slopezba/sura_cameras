#include <algorithm>
#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

class ImageDecimator : public rclcpp::Node
{
public:
  ImageDecimator()
  : Node("image_decimator")
  {
    decimation_x_ = declare_parameter<int>("decimation_x", 2);
    decimation_y_ = declare_parameter<int>("decimation_y", 2);
    offset_x_ = declare_parameter<int>("offset_x", 0);
    offset_y_ = declare_parameter<int>("offset_y", 0);
    width_ = declare_parameter<int>("width", 0);
    height_ = declare_parameter<int>("height", 0);
    interpolation_ = declare_parameter<std::string>("interpolation", "area");

    auto sensor_qos = rclcpp::SensorDataQoS();
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("decimated/image_raw", sensor_qos);
    camera_info_pub_ =
      create_publisher<sensor_msgs::msg::CameraInfo>("decimated/camera_info", rclcpp::QoS(10));

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "image_raw", sensor_qos,
      std::bind(&ImageDecimator::on_image, this, std::placeholders::_1));
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "camera_info", rclcpp::QoS(10),
      std::bind(&ImageDecimator::on_camera_info, this, std::placeholders::_1));
  }

private:
  void on_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    camera_info_ = msg;
  }

  void on_image(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      const cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvShare(msg, msg->encoding);
      const cv::Mat cropped = crop(cv_image->image);
      const cv::Mat output = resize(cropped);

      cv_bridge::CvImage output_image;
      output_image.header = msg->header;
      output_image.encoding = msg->encoding;
      output_image.image = output;
      image_pub_->publish(*output_image.toImageMsg());

      if (camera_info_) {
        camera_info_pub_->publish(decimated_camera_info(msg->header, output.cols, output.rows));
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Failed to decimate image: %s", ex.what());
    }
  }

  cv::Mat crop(const cv::Mat & image) const
  {
    const int x0 = std::clamp(offset_x_, 0, image.cols);
    const int y0 = std::clamp(offset_y_, 0, image.rows);
    const int crop_width = width_ > 0 ? width_ : image.cols - x0;
    const int crop_height = height_ > 0 ? height_ : image.rows - y0;
    const int x1 = std::min(x0 + crop_width, image.cols);
    const int y1 = std::min(y0 + crop_height, image.rows);
    return image(cv::Rect(x0, y0, x1 - x0, y1 - y0));
  }

  cv::Mat resize(const cv::Mat & image) const
  {
    const int safe_decimation_x = std::max(decimation_x_, 1);
    const int safe_decimation_y = std::max(decimation_y_, 1);
    const int output_width = std::max(image.cols / safe_decimation_x, 1);
    const int output_height = std::max(image.rows / safe_decimation_y, 1);

    cv::Mat output;
    const int interpolation =
      interpolation_ == "nearest" ? cv::INTER_NEAREST : cv::INTER_AREA;
    cv::resize(image, output, cv::Size(output_width, output_height), 0.0, 0.0, interpolation);
    return output;
  }

  sensor_msgs::msg::CameraInfo decimated_camera_info(
    const std_msgs::msg::Header & header, int width, int height) const
  {
    sensor_msgs::msg::CameraInfo info = *camera_info_;
    info.header = header;
    info.width = width;
    info.height = height;

    const double dx = static_cast<double>(std::max(decimation_x_, 1));
    const double dy = static_cast<double>(std::max(decimation_y_, 1));
    const double ox = static_cast<double>(offset_x_);
    const double oy = static_cast<double>(offset_y_);

    info.k[0] /= dx;
    info.k[2] = (info.k[2] - ox) / dx;
    info.k[4] /= dy;
    info.k[5] = (info.k[5] - oy) / dy;

    info.p[0] /= dx;
    info.p[2] = (info.p[2] - ox) / dx;
    info.p[5] /= dy;
    info.p[6] = (info.p[6] - oy) / dy;

    info.binning_x = std::max(info.binning_x, 1u) * static_cast<uint32_t>(std::max(decimation_x_, 1));
    info.binning_y = std::max(info.binning_y, 1u) * static_cast<uint32_t>(std::max(decimation_y_, 1));
    info.roi.x_offset = 0;
    info.roi.y_offset = 0;
    info.roi.width = width;
    info.roi.height = height;
    return info;
  }

  int decimation_x_;
  int decimation_y_;
  int offset_x_;
  int offset_y_;
  int width_;
  int height_;
  std::string interpolation_;

  sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageDecimator>());
  rclcpp::shutdown();
  return 0;
}
