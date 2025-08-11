// Pure pursuit 1 lookahead 100% passes tests

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>
#include <string>
#include <vector>
#include <numeric>
#include <algorithm>

class LineTrackerNode : public rclcpp::Node
{
public:
    LineTrackerNode() : Node("line_tracker_node")
    {
        serial_port_fd_ = open("/dev/serial0", O_RDWR | O_NOCTTY);
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

        // Stop motors at start
        std::string stop_cmd = "C 0 0\n";
        write(serial_port_fd_, stop_cmd.c_str(), stop_cmd.length());

        // Image subscriber
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10,
            std::bind(&LineTrackerNode::processImage, this, std::placeholders::_1));
    }

    ~LineTrackerNode()
    {
        if (serial_port_fd_ != -1)
        {
            std::string stop_cmd = "C 0 0\n";
            write(serial_port_fd_, stop_cmd.c_str(), stop_cmd.length());
            close(serial_port_fd_);
        }
    }

private:
    void processImage(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (msg->encoding != "rgb8")
        {
            RCLCPP_ERROR(this->get_logger(), "Unsupported encoding: %s", msg->encoding.c_str());
            return;
        }

        int width = msg->width;
        int height = msg->height;
        int scan_row = height * 7 / 8;  // Scan closer to bottom
        std::vector<int> gray_row(width);

        for (int x = 0; x < width; x++)
        {
            int idx = (scan_row * width + x) * 3;
            if (idx + 2 >= static_cast<int>(msg->data.size()))
                return;

            float r = static_cast<float>(msg->data[idx]);
            float g = static_cast<float>(msg->data[idx + 1]);
            float b = static_cast<float>(msg->data[idx + 2]);
            gray_row[x] = static_cast<int>(0.299f * r + 0.587f * g + 0.114f * b);
        }

        double row_mean = std::accumulate(gray_row.begin(), gray_row.end(), 0.0) / width;
        int threshold = static_cast<int>(row_mean * 0.6);

        int sum_x = 0, count_black = 0;
        for (int x = 0; x < width; x++)
        {
            if (gray_row[x] < threshold)
            {
                sum_x += x;
                count_black++;
            }
        }

        std::string cmd;

        if (count_black > 0)
        {
            int tape_center_x = sum_x / count_black;
            double dx = tape_center_x - (width / 2.0);
            double dy = 55.0;  // shorter lookahead for sharper turns
            double curvature = (2.0 * dx) / (dx * dx + dy * dy);

            double base_speed = 0.01;  // very slow
            double left_speed, right_speed;

            // 🚨 HARD TURN MODE
            if (std::abs(dx) > 80)
            {
                if (dx < 0)
                {
                    // Line far left → turn left (left motor off)
                    left_speed = 0.0;
                    right_speed = base_speed;
                }
                else
                {
                    // Line far right → turn right (right motor off)
                    left_speed = base_speed;
                    right_speed = 0.0;
                }
            }
            else
            {
                // Normal pure pursuit
                left_speed = base_speed * (1.0 - curvature);
                right_speed = base_speed * (1.0 + curvature);
            }

            // Clamp: prevent reverse & cap speed
            int left = std::clamp(static_cast<int>(left_speed * 100), 0, 10);
            int right = std::clamp(static_cast<int>(right_speed * 100), 0, 10);

            cmd = "C " + std::to_string(left) + " " + std::to_string(right) + "\n";
            write(serial_port_fd_, cmd.c_str(), cmd.length());

            if (++frame_count_ % 30 == 0)
            {
                RCLCPP_INFO(this->get_logger(),
                            "Line X=%d dx=%.1f curv=%.3f => %s",
                            tape_center_x, dx, curvature, cmd.c_str());
            }
        }
        else
        {
            // Line lost → stop
            cmd = "C 0 0\n";
            write(serial_port_fd_, cmd.c_str(), cmd.length());
            RCLCPP_WARN(this->get_logger(), "Line lost. Stopping.");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    int serial_port_fd_;
    unsigned long int frame_count_ = 0;
};
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineTrackerNode>());
    rclcpp::shutdown();
    return 0;
}