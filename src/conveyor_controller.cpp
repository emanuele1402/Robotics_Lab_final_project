#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ros_gz_interfaces/srv/set_entity_pose.hpp>
#include "std_msgs/msg/bool.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class ConveyorController : public rclcpp::Node {
public:
    ConveyorController() : Node("conveyor_controller") {
        pub_box1 = this->create_publisher<geometry_msgs::msg::Twist>("/model/box1/cmd_vel", 10);
        pub_box2 = this->create_publisher<geometry_msgs::msg::Twist>("/model/box2/cmd_vel", 10);
        
        sub_lidar = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/lidar_scan", 10, std::bind(&ConveyorController::lidar_callback, this, std::placeholders::_1));
        
        sub_painted_ = this->create_subscription<std_msgs::msg::Bool>("/painted_box", 10, std::bind(&ConveyorController::painted_callback, 
                                                                              this, std::placeholders::_1));
        pub_obj_detected = this->create_publisher<std_msgs::msg::Bool> ("/object_detected",10);
        nav_start_seq = this->create_publisher<std_msgs::msg::Bool> ("/start_nav_seq",10);
        client_pose = this->create_client<ros_gz_interfaces::srv::SetEntityPose>("/world/conveyor_world/set_pose");

        pub_piston = this->create_publisher<std_msgs::msg::Float64MultiArray>("/piston_ns/piston_controller/commands", 10);

        timer = this->create_wall_timer(100ms, std::bind(&ConveyorController::control_loop, this));
        state = MOVING_BOTH;
        obj_detected.data=false;
    }

private:
    // Aggiunti stati per gestire la seconda box fino al teletrasporto
    enum State { MOVING_BOTH, WAITING_BOX1, TELEPORTING_BOX1,PISTON_RETRACTING, MOVING_BOX2, WAITING_BOX2, TELEPORTING_BOX2, FINISHED };
    State state;
    bool object_detected = false;

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Rileva se un oggetto interrompe il raggio (distanza < 0.55m)
        bool current_reading = (msg->ranges[0] < 0.55);
    
        
        if (state == MOVING_BOTH || state == MOVING_BOX2) {
            object_detected = current_reading;
        }

        obj_detected.data = current_reading;
        pub_obj_detected->publish(obj_detected);
    }

    void control_loop() {
        geometry_msgs::msg::Twist stop, move;
        move.linear.x = 0.5;

        switch (state) {
            case MOVING_BOTH:
                pub_box1->publish(move);
                pub_box2->publish(move);
                if (object_detected) {
                    state = WAITING_BOX1;
                }
                break;

            case WAITING_BOX1:
                pub_box1->publish(stop);
                pub_box2->publish(stop);
                if(painted_){
                     painted_ = false; 
                     state = TELEPORTING_BOX1;
                } 
                break;

            case TELEPORTING_BOX1:
                // AGGIUNTE GRAFFE QUI
                {
                    teleport_box("box1_painted", 2.3, 0.0, 1.3, 0.0, 0.0, 1.57); 
                    teleport_box("box1", -50.0, 0.0, -50.0, 0.0, 0.0, 0.0);
                    
                    auto piston_msg = std_msgs::msg::Float64MultiArray();
                    piston_msg.data = {20.0}; 
                    pub_piston->publish(piston_msg);
                    rclcpp::sleep_for(500ms);
                    piston_start_time_ = this->now();
                    state = PISTON_RETRACTING;
                }
                break;
            
            case PISTON_RETRACTING:
                {
                    
                    

                    
                    auto piston_msg = std_msgs::msg::Float64MultiArray();
                    piston_msg.data = {-5.0}; // Comando di ritorno
                    pub_piston->publish(piston_msg);
                    
                    RCLCPP_INFO(this->get_logger(), "COMANDO RITRAZIONE INVIATO");
                    object_detected = false;
                    state = MOVING_BOX2;
                    
                }
                break;

            case MOVING_BOX2:
                pub_box2->publish(move);
                if (object_detected) {
                    state = WAITING_BOX2;
                }
                break;

            case WAITING_BOX2:
                pub_box2->publish(stop);
                if(painted_){
                    painted_ = false;
                    state = TELEPORTING_BOX2;
                }
                break;

            case TELEPORTING_BOX2:
              {
                teleport_box("box2_painted", 2.3, 0.0, 1.3, 0.0, 0.0, 0.0);
                teleport_box("box2", -50.0, 1.0, -50.0, 0.0, 0.0, 0.0);
                auto piston_msg = std_msgs::msg::Float64MultiArray();
                piston_msg.data = {13.0}; 
                pub_piston->publish(piston_msg);
                rclcpp::sleep_for(500ms);
                state = FINISHED;
              }
              break;

            case FINISHED:
                {
                    auto piston_msg = std_msgs::msg::Float64MultiArray();
                    piston_msg.data = {-8.0}; 
                    pub_piston->publish(piston_msg);
                    rclcpp::sleep_for(500ms);
                    std_msgs::msg::Bool navstart_msg;
		    navstart_msg.data = true;
		    nav_start_seq->publish(navstart_msg);
                }
                break;
        }
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
    void painted_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "Ricevuto painted: %s", msg->data ? "TRUE" : "FALSE");
      if (msg->data == true) {
        painted_ = true;
       }
    }
    

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_box1, pub_box2;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_obj_detected;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr nav_start_seq;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_lidar;
    rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr client_pose;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_painted_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_piston;
    rclcpp::Time piston_start_time_;
    bool painted_ = false;
    
    std_msgs::msg::Bool obj_detected;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConveyorController>());
    rclcpp::shutdown();
    return 0;
}
