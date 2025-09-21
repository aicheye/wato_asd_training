#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <optional>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class ControlNode : public rclcpp::Node {
  public:
    ControlNode():Node("control"){

        lookahead_distance_ = 1.0;  // Lookahead distance
        goal_tolerance_ = 0.1;     // Distance to consider the goal reached
        linear_speed_ = 0.5;       // Constant forward speed

        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });
 
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        // Timer
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), [this]() { controlLoop(); });
    }

    private:
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    
        // Timer
        rclcpp::TimerBase::SharedPtr control_timer_;
    
        // Data
        nav_msgs::msg::Path::SharedPtr current_path_;
        nav_msgs::msg::Odometry::SharedPtr robot_odom_;
    
        // Parameters
        double lookahead_distance_;
        double goal_tolerance_;
        double linear_speed_;

      void controlLoop(){
        if (!current_path_ || !robot_odom_) return;

        auto lookahead_point = findLookaheadPoint();

        if(!lookahead_point) return;

        auto cmd_vel = computeVelocity(*lookahead_point);

        cmd_vel_pub_->publish(cmd_vel);
      }

      std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint(){

        const auto robot_pose = robot_odom_->pose.pose.position;
        
        for(const auto& pose_stamped : current_path_->poses){
          const auto& path_point = pose_stamped.pose.position;

          double distance = computeDistance(path_point, robot_pose);

          if (distance >= lookahead_distance_) return pose_stamped;
        }

        return current_path_->poses.back();
      }

    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
        // TODO: Implement logic to compute velocity commands
        geometry_msgs::msg::Twist cmd_vel;

        const auto& robot_pose = robot_odom_->pose.pose;
        double rx = robot_pose.position.x;
        double ry = robot_pose.position.y;

        tf2::Quaternion q(
            robot_pose.orientation.x,
            robot_pose.orientation.y,
            robot_pose.orientation.z,
            robot_pose.orientation.w
        );

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        double tx = target.pose.position.x;
        double ty = target.pose.position.y;

        double dx = tx - rx;
        double dy = ty - ry;
        double local_x =  cos(yaw) * dx + sin(yaw) * dy;
        double local_y = -sin(yaw) * dx + cos(yaw) * dy;

        double L = std::sqrt(local_x*local_x + local_y*local_y);

        if (L < 0.000001) {
          cmd_vel.linear.x = 0.0;
          cmd_vel.linear.y = 0.0;
          cmd_vel.linear.z = 0.0;
          cmd_vel.angular.x = 0.0;
          cmd_vel.angular.y = 0.0;
          cmd_vel.angular.z = 0.0;
          return cmd_vel;  // Already at target
        }

        double curvature = 2.0 * local_y / (L * L);
        double v = 0.5; 
        double omega = v * curvature;

        cmd_vel.linear.x = v;
        cmd_vel.angular.z = omega;

        return cmd_vel;
    }

    double computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
        return std::sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
    }

};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
