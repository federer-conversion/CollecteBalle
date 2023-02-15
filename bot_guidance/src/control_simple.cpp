#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

double yaw = 0;               // cap a suivre
vector<double> X = {0, 0, 0}; // position robot
vector<double> Y = {0, 0, 0}; // position cible

class Subscriber_publisher : public rclcpp::Node
{
public:
  Subscriber_publisher()
      : Node("simple_controler")
  {
    commande_pub = this->create_publisher<geometry_msgs::msg::Twist>("/demo/cmd_vel", 1000);
    timer_ = this->create_wall_timer(500ms, std::bind(&Subscriber_publisher::timer_callback, this));
    subscriptionBotPose_ = this->create_subscription<geometry_msgs::msg::Pose>("/robot_position", 1000, bind(&Subscriber_publisher::PositionCallBack, this, placeholders::_1));
    subscriptionGoalPose_ = this->create_subscription<geometry_msgs::msg::Pose>("/target", 1000, bind(&Subscriber_publisher::PositionCibleCallBack, this, placeholders::_1));
  }

private:
  void PositionCallBack(const geometry_msgs::msg::Pose::SharedPtr message)
  {
    // position du robot dans le repere du terrain
    X[0] = message->position.x;
    X[1] = message->position.y;
    X[2] = 0.;
    yaw = tf2::getYaw(message->orientation);
    cout << yaw << endl;
  }

  void PositionCibleCallBack(const geometry_msgs::msg::Pose::SharedPtr message)
  {
    // position de la balle de tennis
    Y[0] = message->position.x;
    Y[1] = message->position.y;
    Y[2] = 0.;
  }

  void timer_callback()
  {
    // declaration des variables
    double e;
    double delta_theta;
    double theta_voulu;
    geometry_msgs::msg::Twist msg;
    double k = 8;

    theta_voulu = atan2(Y[1] - X[1], Y[0] - X[0]);

    delta_theta = theta_voulu - yaw;
    e = 2 * atan(tan(delta_theta / 2));
    msg.angular.z = k * e;

    // publication du message
    double dx, dy;
    double k_linear = 1;
    dx = X[0] - Y[0];
    dy = X[1] - Y[1];
    double distance = sqrt(dx * dx + dy * dy);
    msg.linear.x = k_linear * distance;
    commande_pub->publish(msg);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriptionBotPose_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriptionGoalPose_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr commande_pub;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber_publisher>());
  rclcpp::shutdown();
  return 0;
}