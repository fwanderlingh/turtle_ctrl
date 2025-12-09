#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cmath>

#include "turtlesim_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

// Topic dal quale leggere la posizione: /turtle1/pose
// Tipo messaggio posizione: turtlesim_msgs/msg/Pose

class TurtleCtrl : public rclcpp::Node
{
    // Subscriber topic della Pose
    rclcpp::Subscription<turtlesim_msgs::msg::Pose>::SharedPtr poseSubscriber_;

    // Publisher topic Velocità di Riferimento
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPublisher_;

    // Variable in cui salviamo la posizione letta dal Subscriber
    turtlesim_msgs::msg::Pose poseMsg_;

    // Timer periodico (Loop principale)
    rclcpp::TimerBase::SharedPtr loopTimer_;

    double lambda_; // Guadagno errore di posizione
    double gamma_;  // Guadagno errore angolare

    double goalX_, goalY_; // Posizione Goal

public:
  // Costruttore della classe
  TurtleCtrl() : Node("turtle_ctrl_node")
  {

    lambda_ = 0.5;
    gamma_ = 1.0;

    goalX_ = 7.0; //2.5;
    goalY_ = 7.0; //2.5;

    poseSubscriber_ = this->create_subscription<turtlesim_msgs::msg::Pose>
    ("/turtle1/pose", 10, std::bind(&TurtleCtrl::PoseCallback, this, _1));

    cmdVelPublisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

    loopTimer_ = this->create_wall_timer(100ms,
                  std::bind(&TurtleCtrl::ControlLoop, this));

  }

  void PoseCallback(const turtlesim_msgs::msg::Pose::SharedPtr msg)
  {
    // Stampa la posizione di turtle1
    poseMsg_ = *msg;
    //std::cout << "Pose (x,y,θ): " << poseMsg_.x << ", " << poseMsg_.y
    //            << ", " << poseMsg_.theta << std::endl;
  }

  void ControlLoop()
  {
    // Errori

    double err_lin = 0.0;
    double err_ang = 0.0;

    err_lin = std::sqrt( std::pow((goalX_ - poseMsg_.x),2) +
                         std::pow((goalY_ - poseMsg_.y),2) );

    double goalHeading = std::atan2( (goalY_ - poseMsg_.y), (goalX_ - poseMsg_.x) );
    err_ang = wrap_around_pi(poseMsg_.theta - goalHeading);

    geometry_msgs::msg::Twist cmd;

    /// Velocità di controllo
    // Comando di surge "u"
    cmd.linear.x = lambda_ * err_lin;
    // Comando di yaw rate "r"
    cmd.angular.z = - gamma_ * err_ang;

    if(std::abs(err_lin) > 0.1){
      std::cout << "Pos Error: " << err_lin << std::endl; 
      std::cout << "Ang Error: " << err_ang << std::endl; 
      
      // Pubblica sul topic il messagio "comando di velocità"
      cmdVelPublisher_->publish(cmd);
    } else {
      std::cout << "Goal Reached!" << std::endl; 
      rclcpp::shutdown();
    }
  }

  double wrap_around_pi(double angle){
    if (angle > M_PI){
      angle = angle - 2*M_PI;
    } else if(angle < -M_PI){
      angle = angle + 2*M_PI;
    }
    return angle;
  }

};



int main(int argc, char *argv[])
{
    // Inizializzo ROS
    rclcpp::init(argc, argv);
    
    // Definisco e lancio il nodo
    std::shared_ptr ctrlNode = std::make_shared<TurtleCtrl>();
    rclcpp::spin(ctrlNode);

    // Al termine (Ctrl+C) chiudo ROS
    rclcpp::shutdown();
    return 0;
}

