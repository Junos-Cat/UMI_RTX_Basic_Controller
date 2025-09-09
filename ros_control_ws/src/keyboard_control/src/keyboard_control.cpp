#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/float32.hpp"

#include <iostream>
#include <termios.h>  // For non-blocking keyboard input
#include <unistd.h>
#include <map>


char getch() {
    char buf = 0;
    struct termios old = {};
    if (tcgetattr(STDIN_FILENO, &old) < 0) perror("tcsetattr()");
    struct termios newt = old;
    newt.c_lflag &= ~ICANON;
    newt.c_lflag &= ~ECHO;
    newt.c_cc[VMIN] = 0;
    newt.c_cc[VTIME] = 1;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &newt) < 0) perror("tcsetattr ICANON");
    if (read(STDIN_FILENO, &buf, 1) < 0) perror ("read()");
    tcsetattr(STDIN_FILENO, TCSANOW, &old);
    return buf;
}

class KeyboardArmControl : public rclcpp::Node {
public:
    KeyboardArmControl() : Node("keyboard_arm_control") {
        joint_pub = this->create_publisher<sensor_msgs::msg::JointState>("motor_commands", 10);
        pose_pub = this->create_publisher<geometry_msgs::msg::Pose>("target_pose", 10);
        grip_pub = this->create_publisher<std_msgs::msg::Float32>("target_grip", 10);

        // Initialize joint angles
        /*joint_state.position = {0, 0, 0, 0, 0, 0, 0, 0};
        joint_state.name = {"zed", "shoulder", "elbow", "wrist1", "wrist2", "wrist3", "gripper_left", "gripper_right"};*/

        x = y = z = 0.0;
        yaw = pitch = roll = 0.0;
        grip = 0.0;
    }

    void run() {
        while (rclcpp::ok()) {
            char c = getch();
            if (c != 0) handle_key(c);
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr grip_pub;

    sensor_msgs::msg::JointState joint_state;
    geometry_msgs::msg::Pose target_pose;
    std_msgs::msg::Float32 target_grip;

    double x, y, z;
    double yaw, pitch, roll;
    double grip;

    double position_speed = 0.1;
    double angle_speed = 5;

    void handle_key(char c) {
        switch(c) {
            case 'w': z += position_speed; break;
            case 's': z -= position_speed; break;
            case 'a': y += position_speed; break;
            case 'd': y -= position_speed; break;
            case 'q': x += position_speed; break;
            case 'e': x -= position_speed; break;
            case 'i': pitch += angle_speed; break;
            case 'k': pitch -= angle_speed; break;
            case 'j': yaw += angle_speed; break;
            case 'l': yaw -= angle_speed; break;
            case 'u': roll += angle_speed; break;
            case 'o': roll -= angle_speed; break;
            case '+': grip += 0.05; if (grip>1.0) grip=1.0; break;
            case '-': grip -= 0.05; if (grip<0.0) grip=0.0; break;
            case 'x': rclcpp::shutdown(); return;
        }

        // Publish joint commands (optional if you want direct joint control)
        /*joint_state.header.stamp = this->get_clock()->now();
        joint_pub->publish(joint_state);*/

        // Publish pose
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;
        target_pose.orientation.x = yaw;
        target_pose.orientation.y = pitch;
        target_pose.orientation.z = roll;
        pose_pub->publish(target_pose);

        // Publish grip
        target_grip.data = grip;
        grip_pub->publish(target_grip);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardArmControl>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
