#include <memory>
#include <string>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/float32.hpp"

class ArmCommander : public rclcpp::Node
{
public:
    ArmCommander()
    : Node("arm_commander")
    {
        pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/target_pose", 10);
        grip_pub_ = this->create_publisher<std_msgs::msg::Float32>("/target_grip", 10);
        RCLCPP_INFO(this->get_logger(), "ArmCommander node started. Enter commands interactively.");
        run_loop();
    }

private:
    void run_loop()
    {
        std::string line;
        while (rclcpp::ok())
        {
            std::cout << "Enter command (pose x y z roll pitch yaw | grip value | exit): ";
            std::getline(std::cin, line);
            if (line.empty()) continue;

            if (line.rfind("pose", 0) == 0) {
                double x, y, z, roll, pitch, yaw;
                if (sscanf(line.c_str(), "pose %lf %lf %lf %lf %lf %lf", &x, &y, &z, &roll, &pitch, &yaw) == 6) {
                    geometry_msgs::msg::Pose msg;
                    msg.position.x = x;
                    msg.position.y = y;
                    msg.position.z = z;
                    msg.orientation.x = roll;
                    msg.orientation.y = pitch;
                    msg.orientation.z = yaw;
                    pose_pub_->publish(msg);
                    RCLCPP_INFO(this->get_logger(), "Pose sent: (%.2f, %.2f, %.2f, roll=%.2f pitch=%.2f yaw=%.2f)", x, y, z, roll, pitch, yaw);
                } else {
                    std::cout << "Invalid pose command. Usage: pose x y z roll pitch yaw\n";
                }
            } 
            else if (line.rfind("grip", 0) == 0) {
                float value;
                if (sscanf(line.c_str(), "grip %f", &value) == 1) {
                    std_msgs::msg::Float32 msg;
                    msg.data = value;
                    grip_pub_->publish(msg);
                    RCLCPP_INFO(this->get_logger(), "Gripper command sent: %.2f", value);
                } else {
                    std::cout << "Invalid grip command. Usage: grip value\n";
                }
            } 
            else if (line == "exit") {
                break;
            } 
            else {
                std::cout << "Unknown command\n";
            }

            rclcpp::spin_some(this->get_node_base_interface());
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr grip_pub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmCommander>();
    rclcpp::shutdown();
    return 0;
}
