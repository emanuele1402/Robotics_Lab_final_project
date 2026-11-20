#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/bool.hpp"
#include <ros_gz_interfaces/srv/set_entity_pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>

using namespace std::chrono_literals;

class Nav2Sequencer : public rclcpp::Node {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  Nav2Sequencer() : Node("nav2_sequencer") {
    this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    client_pose = this->create_client<ros_gz_interfaces::srv::SetEntityPose>("/world/conveyor_world/set_pose");

    start_seq=this->create_subscription<std_msgs::msg::Bool>("/start_nav_seq",10,std::bind(&Nav2Sequencer::startseq_callback, 
                                                                              this, std::placeholders::_1));
    
    // Definiamo le due posizioni (x, y)
    goals_ = {{8.63, -4.17}, {6.05, 5.25}}; 
    current_goal_index_ = 0;

    
  }

private:
  void startseq_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if(msg->data)
    {
        execute_current_step();

    }
  }

  void execute_current_step() {
    if (current_goal_index_ >= goals_.size()) {
      RCLCPP_INFO(this->get_logger(), "Tutte le missioni completate!");
      return;
    }

    auto target = goals_[current_goal_index_];
    send_nav_goal(target.first, target.second);
  }

  void send_nav_goal(double x, double y) {
    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Action server non disponibile!");
      return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.orientation.w = 1.0;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    send_goal_options.result_callback = [this](const GoalHandleNav::WrappedResult & result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "Goal %ld raggiunto!", current_goal_index_ + 1);
        
        // Se abbiamo raggiunto il primo goal, facciamo l'azione speciale
        if (current_goal_index_ == 0) {
          teleport_box("box1_painted", 7.0, -4.17, 0.1, 0.0, 0.0, 0.0);
          teleport_box("box2_painted", 8.63, -4.17, 0.40, 0.0, 0.0, 0.0);
          current_goal_index_++;
          execute_current_step();
        }
        else if (current_goal_index_ == 1) {
          teleport_box("box2_painted", 9.0, 4.71, 0.1, 0.0, 0.0, 0.0);
          current_goal_index_++;
          
        }

        
      }
    };

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  void teleport_box(std::string name, double x, double y, double z, double roll, double pitch, double yaw) {
        if (!client_pose->wait_for_service(1s)) {
            RCLCPP_ERROR(this->get_logger(), "Servizio set_pose non disponibile!");
            return;
        }

        auto request = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
        request->entity.name = name;
        request->entity.type = ros_gz_interfaces::msg::Entity::MODEL;
        
        request->pose.position.x = x;
        request->pose.position.y = y;
        request->pose.position.z = z;

        // Conversione da angoli di Eulero (Radianti) a Quaternione
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw); // Roll, Pitch, Yaw

        request->pose.orientation.x = q.x();
        request->pose.orientation.y = q.y();
        request->pose.orientation.z = q.z();
        request->pose.orientation.w = q.w();

        client_pose->async_send_request(request);
    }

  rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_seq;
  rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr client_pose;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::pair<double, double>> goals_;
  size_t current_goal_index_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nav2Sequencer>());
  rclcpp::shutdown();
  return 0;
}
