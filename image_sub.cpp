#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>


class ImageSubscriberNode : public rclcpp::Node
{
public:
    ImageSubscriberNode() : Node("image_subscriber_node")
    {

        // Open the serial port
        serial_port_fd_ = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_port_fd_ == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");
            return;
        }
        
        struct termios options;
        tcgetattr(serial_port_fd_, &options);
        cfsetispeed(&options, B9600);
        cfsetospeed(&options, B9600);
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        tcsetattr(serial_port_fd_, TCSANOW, &options);
        


        // Subscribe to /image topic
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                processImage(msg);
            });
    }

 ~ ImageSubscriberNode()
       {
        // Close the serial port
        if (serial_port_fd_ != -1)
        {
            std::string cmd = "C 0 0\n";
            write(serial_port_fd_, cmd.c_str(), cmd.length());

            close(serial_port_fd_);
        }    


    }

private:
    void processImage(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        
        if (count%60==0)
        {
        // Print some information about the received image message
        RCLCPP_INFO(this->get_logger(), "Received image: width = %d, height = %d, encoding = %s",
                    msg->width, msg->height, msg->encoding.c_str());
                    RCLCPP_INFO(this->get_logger(), "First Pixel Values: Red = %d, Green = %d, Blue = %d",
                    msg->data[0], msg->data[1], msg->data[2]);


        // You can add further processing here based on your application's needs

        std::string cmd = "C 1 1\n";
        write(serial_port_fd_, cmd.c_str(), cmd.length());
        RCLCPP_INFO(this->get_logger(), "%s",cmd.c_str());
        }

        count++;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
     int serial_port_fd_;
     unsigned long int count=0; 
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSubscriberNode>());
    rclcpp::shutdown();
    return 0;
}
