/*******************************************************************************
 * Twist Commander for MoveIt Servo Demo
 * 
 * This node publishes twist commands to test the MoveIt Servo demo.
 * It can be used to send predefined motion sequences or interactive commands.
 *******************************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <cmath>

class TwistCommander : public rclcpp::Node
{
public:
    TwistCommander() : Node("twist_commander"), sequence_step_(0)
    {
        // Create publisher for twist commands
        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/servo_twist_cmds", 10);
        
        // Create timer to send periodic commands
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TwistCommander::publishTwistCommand, this));
        
        RCLCPP_INFO(this->get_logger(), "Twist Commander started - sending test sequence");
        RCLCPP_INFO(this->get_logger(), "Publishing twist commands to: /servo_twist_cmds");
    }

private:
    void publishTwistCommand()
    {
        auto twist_msg = geometry_msgs::msg::Twist();
        
        // Create a simple motion sequence
        double time_sec = sequence_step_ * 0.1;  // 100ms steps
        
        if (time_sec < 2.0)
        {
            // Move forward in X direction
            twist_msg.linear.x = 0.05;
            twist_msg.linear.y = 0.0;
            twist_msg.linear.z = 0.0;
            twist_msg.angular.x = 0.0;
            twist_msg.angular.y = 0.0;
            twist_msg.angular.z = 0.0;
        }
        else if (time_sec < 4.0)
        {
            // Move up in Z direction
            twist_msg.linear.x = 0.0;
            twist_msg.linear.y = 0.0;
            twist_msg.linear.z = 0.05;
            twist_msg.angular.x = 0.0;
            twist_msg.angular.y = 0.0;
            twist_msg.angular.z = 0.0;
        }
        else if (time_sec < 6.0)
        {
            // Rotate around Z axis
            twist_msg.linear.x = 0.0;
            twist_msg.linear.y = 0.0;
            twist_msg.linear.z = 0.0;
            twist_msg.angular.x = 0.0;
            twist_msg.angular.y = 0.0;
            twist_msg.angular.z = 0.2;
        }
        else if (time_sec < 8.0)
        {
            // Move back to origin
            twist_msg.linear.x = -0.05;
            twist_msg.linear.y = 0.0;
            twist_msg.linear.z = -0.05;
            twist_msg.angular.x = 0.0;
            twist_msg.angular.y = 0.0;
            twist_msg.angular.z = -0.2;
        }
        else
        {
            // Stop motion
            twist_msg.linear.x = 0.0;
            twist_msg.linear.y = 0.0;
            twist_msg.linear.z = 0.0;
            twist_msg.angular.x = 0.0;
            twist_msg.angular.y = 0.0;
            twist_msg.angular.z = 0.0;
            
            // Reset sequence after 10 seconds
            if (time_sec > 10.0)
            {
                sequence_step_ = 0;
                RCLCPP_INFO(this->get_logger(), "Restarting motion sequence");
            }
        }
        
        twist_pub_->publish(twist_msg);
        
        // Log current command every 1 second
        if (sequence_step_ % 10 == 0)
        {
            RCLCPP_INFO(this->get_logger(), 
                       "Step %d (%.1fs): lin=[%.3f, %.3f, %.3f], ang=[%.3f, %.3f, %.3f]",
                       sequence_step_, time_sec,
                       twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z,
                       twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z);
        }
        
        sequence_step_++;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int sequence_step_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<TwistCommander>();
    
    RCLCPP_INFO(node->get_logger(), "Starting Twist Commander...");
    
    rclcpp::spin(node);
    
    RCLCPP_INFO(node->get_logger(), "Shutting down Twist Commander");
    rclcpp::shutdown();
    return 0;
}
