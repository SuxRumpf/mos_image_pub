#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std::chrono_literals;

class ImagePublisherNode : public rclcpp::Node {
public:
    ImagePublisherNode() : Node("image_publisher") {
        this->declare_parameter<std::vector<long>>("cam_list", {0});
        std::vector<long> cam_list = this->get_parameter("cam_list").as_integer_array();


        for(long li : cam_list)
        {
            capArray.emplace_back(cv::VideoCapture(li,cv::CAP_V4L2));
            image_pub_array.emplace_back(
                image_transport::create_publisher(this, "image" + std::to_string(li)));
        }

        timer_ = this->create_wall_timer(30ms, [this](){publishImage();});
    }

    void publishImage() {
        for(int cnt = 0; cnt < capArray.size(); cnt++)
        {
            auto img_msg = std::make_shared<sensor_msgs::msg::Image>();
            cv::Mat img;
            capArray[cnt] >> img;

            try {
                img_msg = cv_bridge::CvImage(
                    std_msgs::msg::Header(),   // Header, z.B. Zeitstempel und Frame ID
                    "bgr8",                    // Encoding (BGR8 für farbige Bilder)
                    img                      // Das OpenCV-Mat-Bild
                ).toImageMsg();
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Conversion failed: %s", e.what());
                return;
            }

            image_pub_array[cnt].publish(img_msg);
        }
    }

private:
    std::vector<cv::VideoCapture> capArray;
    std::vector<image_transport::Publisher> image_pub_array;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImagePublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
