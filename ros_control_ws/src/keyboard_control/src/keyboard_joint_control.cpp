#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <string>
#include <thread>

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

class KeyboardJointControl : public rclcpp::Node {
public:
    KeyboardJointControl() : Node("keyboard_joint_control") {
        joint_pub = this->create_publisher<sensor_msgs::msg::JointState>("motor_commands", 10);

        joint_state.name = {"zed", "shoulder", "elbow", "wrist1", "wrist2", "wrist3", "gripper_left", "gripper_right"};
        joint_state.position = std::vector<double>(joint_state.name.size(), 0.0);

        current_joint = 0;
        increment = 0.05; // 0.05 rad per key press
        std::cout << "Keyboard control initialized!\n";
        print_instructions();
    }

    void run() {
        while (rclcpp::ok()) {
            char c = getch();
            if (c != 0) handle_key(c);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub;
    sensor_msgs::msg::JointState joint_state;

    size_t current_joint;
    double increment;

    void handle_key(char c) {
        switch(c) {
            case 'w': joint_state.position[current_joint] += increment; break;
            case 's': joint_state.position[current_joint] -= increment; break;
            case 'a': current_joint = (current_joint == 0) ? joint_state.position.size()-1 : current_joint-1; break;
            case 'd': current_joint = (current_joint+1) % joint_state.position.size(); break;
            case 'x': rclcpp::shutdown(); return;
            default: return;
        }

        joint_state.header.stamp = this->get_clock()->now();
        joint_pub->publish(joint_state);

        print_status();
    }

    void print_instructions() {
        std::cout << "Use keys to control joints:\n";
        std::cout << "  w/s : increase/decrease current joint\n";
        std::cout << "  a/d : select previous/next joint\n";
        std::cout << "  x   : exit\n";
        std::cout << "Current joint highlighted in []\n";
    }

    void print_status() {
        std::cout << "\rJoint positions: ";
        for (size_t i = 0; i < joint_state.position.size(); i++) {
            if (i == current_joint) std::cout << "[" << joint_state.position[i] << "] ";
            else std::cout << joint_state.position[i] << " ";
        }
        std::cout << std::flush;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardJointControl>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
