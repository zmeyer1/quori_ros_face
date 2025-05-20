def main():
    print('Hi from quori_face.')


if __name__ == '__main__':
    main()

"""
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImageDisplayNode : public rclcpp::Node {
public:
  ImageDisplayNode() : Node("image_display_node") {
    // 1. Create Subscriber
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_topic", 10,
        std::bind(&ImageDisplayNode::imageCallback, this, std::placeholders::_1));
    
    // 3. Create OpenCV Window
    cv::namedWindow("Image Display", cv::WINDOW_NORMAL); 
    
    // 4. Set to Fullscreen
    cv::setWindowProperty("Image Display", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN); 
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        // 2. Convert ROS Image to cv::Mat
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        // 5. Display the Image
        cv::imshow("Image Display", cv_ptr->image);
        
        // 6. Handle User Input (Optional)
        cv::waitKey(1); 

    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Could not convert to 'bgr8': %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageDisplayNode>());
  rclcpp::shutdown();
  return 0;
}

"""