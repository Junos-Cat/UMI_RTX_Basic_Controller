#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <algorithm> 
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace std::chrono_literals;

/**
 * @brief Node that listens to keyboard input and publishes
 *        gripper and pose commands on the target_grip and
 *        target_pose topics used by the nodeArm and 
 *        nodeInverseKinematics
 */
class nodeKeyboardCommand : public rclcpp::Node
{
public:
    nodeKeyboardCommand() : Node("node_Keyboard_Command")
    {
        // Create publishers for gripper and pose commands
        gripPublisherObject = this->create_publisher<std_msgs::msg::Float32>("target_grip", 5);
        posePublisherObject = this->create_publisher<geometry_msgs::msg::Pose>("target_pose", 5);

        // Timer to periodically check and publish updates
        //timer_ = this->create_wall_timer(20ms, std::bind(&nodeKeyboardCommand::timer_publish, this));
    }

    /**
     * @brief Main loop for capturing keyboard input
     *        and updating pose / grip commands accordingly.
     */
    void key_loop()
    {
        while (rclcpp::ok())
        {
            char c = getKey();

            // Check for arrow keys (ESC + [ + code)
            if (c == 0x1B)  // ESC
            {          
                char c2 = getKey();
                char c3 = getKey();

                if (c2 == 0x5B) 
                {
                    switch (c3) 
                    {
                        case 0x41: // Up arrow
                            grip_pose += 0.02f;
                            dirty_grip = true;
                            printf("up arrow\n");
                            break;

                        case 0x42: // Down arrow
                            grip_pose -= 0.02f;
                            dirty_grip = true;
                            printf("down arrow\n");
                            break;
                    }
                }
            }
            else
            {
                // Handle regular keys
                switch (c)
                {
                    // Incremental XYZ position
                    case 0x65: x_pose -= euclidean_delta; dirty_pose = true; printf("e\n"); break; // e
                    case 0x74: x_pose += euclidean_delta; dirty_pose = true; printf("t\n"); break; // t
                    case 0x64: y_pose -= euclidean_delta; dirty_pose = true; printf("d\n"); break; // d
                    case 0x67: y_pose += euclidean_delta; dirty_pose = true; printf("g\n"); break; // g
                    case 0x63: z_pose -= euclidean_delta; dirty_pose = true; printf("c\n"); break; // c
                    case 0x62: z_pose += euclidean_delta; dirty_pose = true; printf("b\n"); break; // b

                    // Teleport position reset
                    case 0x72: x_pose = x_pose_origin; dirty_pose = true; printf("r\n"); break; // r
                    case 0x66: y_pose = y_pose_origin; dirty_pose = true; printf("f\n"); break; // f
                    case 0x76: z_pose = z_pose_origin; dirty_pose = true; printf("v\n"); break; // v

                    // Incremental quaternion orientation
                    case 0x79: x_angle -= quaternion_delta; dirty_pose = true; printf("y\n"); break; // y
                    case 0x69: x_angle += quaternion_delta; dirty_pose = true; printf("i\n"); break; // i
                    case 0x68: y_angle -= quaternion_delta; dirty_pose = true; printf("h\n"); break; // h
                    case 0x6B: y_angle += quaternion_delta; dirty_pose = true; printf("k\n"); break; // k
                    case 0x6E: z_angle -= quaternion_delta; dirty_pose = true; printf("n\n"); break; // n
                    case 0x2C: z_angle += quaternion_delta; dirty_pose = true; printf(",\n"); break; // ,
                    case 0x2F: w_angle -= quaternion_delta; dirty_pose = true; printf("/\n"); break; // /
                    case 0x70: w_angle += quaternion_delta; dirty_pose = true; printf("p\n"); break; // p

                    // Teleport quaternion reset
                    case 0x75: x_angle = x_angle_origin; dirty_pose = true; printf("u\n"); break; // u
                    case 0x6A: y_angle = y_angle_origin; dirty_pose = true; printf("j\n"); break; // j
                    case 0x6D: z_angle = z_angle_origin; dirty_pose = true; printf("m\n"); break; // m
                    case 0x3B: w_angle = w_angle_origin; dirty_pose = true; printf(";\n"); break; // ;
                }
            }

            // Clamp gripper values between [0.0, 0.1]
            grip_pose = std::max(0.0f, std::min(grip_pose, 0.1f));

            // Publish if state changed
            if (dirty_grip)
            {
                grip_message.data = grip_pose;
                gripPublisherObject->publish(grip_message);
                dirty_grip = false;
            }

            if (dirty_pose)
            {
                pose_message.position.x = x_pose;
                pose_message.position.y = y_pose;
                pose_message.position.z = z_pose;

                pose_message.orientation.x = x_angle;
                pose_message.orientation.y = y_angle;
                pose_message.orientation.z = z_angle;
                pose_message.orientation.w = w_angle;

                posePublisherObject->publish(pose_message);
                dirty_pose = false;
            }
        }
    }

private:
    /**
     * @brief Timer-based publisher that republishes dirty state changes.
     */
    void timer_publish()
    {
        if (dirty_grip)
        {
            auto grip_msg = std_msgs::msg::Float32();
            grip_msg.data = grip_pose;
            gripPublisherObject->publish(grip_msg);
            dirty_grip = false;
        }

        if (dirty_pose)
        {
            auto pose_msg = geometry_msgs::msg::Pose();
            pose_msg.position.x = x_pose;
            pose_msg.position.y = y_pose;
            pose_msg.position.z = z_pose;

            pose_msg.orientation.x = x_angle;
            pose_msg.orientation.y = y_angle;
            pose_msg.orientation.z = z_angle;
            pose_msg.orientation.w = w_angle;

            posePublisherObject->publish(pose_msg);
            dirty_pose = false;
        }
    }

    /**
     * @brief Read a single keypress without requiring ENTER.
     * @return char pressed key
     */
    char getKey()
    {
        char c;
        struct termios oldt, newt;

        // Save current terminal settings
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;

        // Disable canonical mode and echo
        newt.c_lflag &= ~(ICANON | ECHO);

        // Apply new settings
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        c = getchar(); // Read one char

        // Restore terminal settings
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

        return c;
    }
    
    // ROS handles
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gripPublisherObject;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr posePublisherObject;

    // Messages
    std_msgs::msg::Float32 grip_message;
    geometry_msgs::msg::Pose pose_message;

    // State variables
    float counter = 0;
    bool dirty_grip = false;
    bool dirty_pose = false;

    // Position state
    float x_pose = 0, y_pose = 0, z_pose = 0;

    // Orientation state
    float x_angle = 0, y_angle = 0, z_angle = 0, w_angle = 0;

    // Increments
    float quaternion_delta = 1;
    float euclidean_delta = 0.1;

    // Origin reset points
    float x_pose_origin = 0, y_pose_origin = 0, z_pose_origin = 0.5;
    float x_angle_origin = 0, y_angle_origin = 0, z_angle_origin = 0, w_angle_origin = 0;

    // Gripper
    bool increase = true;
    float grip_pose = 0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto keyboard_control = std::make_shared<nodeKeyboardCommand>();

    // Run the key loop in a separate thread so the timer can still fire
    std::thread key_thread([&]() { keyboard_control->key_loop(); });

    rclcpp::spin(keyboard_control);

    key_thread.join();
    rclcpp::shutdown();
    return 0;
}
