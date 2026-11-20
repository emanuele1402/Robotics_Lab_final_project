#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <vector>
#include <array>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/bool.hpp"
#include "aruco_msgs/msg/marker_array.hpp"

#include "final_project/action/linear_trajectory.hpp"
#include "final_project/msg/position_error.hpp"

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2_kdl/tf2_kdl.hpp>
#include <Eigen/Dense>

using LinearTrajectory = final_project::action::LinearTrajectory;
using namespace std::chrono_literals;

class KukaActionClient : public rclcpp::Node
{
public:
    using LinearTrajectory = final_project::action::LinearTrajectory;
    using GoalHandleLinTraj = rclcpp_action::ClientGoalHandle<LinearTrajectory>;

    explicit KukaActionClient(const rclcpp::NodeOptions & options)
    : Node("kuka_action_client",options)
    {
      this->client_ptr_ = rclcpp_action::create_client<LinearTrajectory>(
      this,
      "linear_trajectory");  

      pub_painted_ = this->create_publisher<std_msgs::msg::Bool>("/painted_box",10);

      sub_obj_detected = this->create_subscription<std_msgs::msg::Bool>("/object_detected",10,std::bind(&KukaActionClient::detected_callback, 
                                                                              this, std::placeholders::_1));
      aruco_sub_ = this->create_subscription<aruco_msgs::msg::MarkerArray>("/marker_publisher/markers",100,
                                          std::bind(&KukaActionClient::marker_callback, this, std::placeholders::_1));
     

      state = IDLE;

    }
    void send_goal(const LinearTrajectory::Goal & goal_msg)
    {
        using namespace std::placeholders;

        if (!this->client_ptr_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
        }

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<LinearTrajectory>::SendGoalOptions();
        send_goal_options.goal_response_callback =
        std::bind(&KukaActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
        std::bind(&KukaActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
        std::bind(&KukaActionClient::result_callback, this, _1);
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
    
    using Waypoint = std::array<double, 3>;
    using WaypointVector = std::vector<Waypoint>;
    enum State {IDLE, MOVE_TO_ARUCO,WAITING_DETECTION, PAINTING};
    State state;
    rclcpp_action::Client<LinearTrajectory>::SharedPtr client_ptr_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_painted_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_obj_detected;
    rclcpp::Subscription<aruco_msgs::msg::MarkerArray>::SharedPtr aruco_sub_;
    geometry_msgs::msg::Point marker_pos_;
    bool emitter_actived_ = false;
    int detected_id_=-1;
    double total_mission_time_ = 0.0;
    bool active_varnish=false;
    
    WaypointVector path_painting;
    double dim_face_box=0.3; //face box has dimension 0.3x0.3
    double height_box=0.3; //height box is 0.3
    unsigned int partition_step =2; // number of partition for horizontal face box
    unsigned int number_waypoints=(partition_step+1)*2;
    double step=0.3/partition_step; //horizontal progress step of face box

  void detected_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if(msg->data && state == IDLE)
    {
        std_msgs::msg::Bool painted_msg;
		painted_msg.data = false;
	    pub_painted_->publish(painted_msg);
        state = MOVE_TO_ARUCO;
        auto goal_msg = LinearTrajectory::Goal();
        goal_msg.s_type = "trapezoidal";
        goal_msg.traj_duration = 10.0;
        goal_msg.acc_duration = 2.0;
        goal_msg.total_time= 10.1;
        goal_msg.trajectory_len= 1000;
        goal_msg.end_pos[0]= -1.4;
        goal_msg.end_pos[1]= -0.1;
        goal_msg.end_pos[2]= 1.65;
        goal_msg.v_init = 0.0;
        goal_msg.v_end = 0.0;
        goal_msg.rot_x = 0.0;
        goal_msg.rot_y = 0.0;
        send_goal(goal_msg);

    }
  }
  void marker_callback(const aruco_msgs::msg::MarkerArray::SharedPtr msg) {
    
    if (state != WAITING_DETECTION) {
        return; 
    }

    if (!msg->markers.empty()) {
        for (const auto & marker : msg->markers) {
            if (marker.id == 1 || marker.id == 2) {
                this->detected_id_ = marker.id;
                this->marker_pos_ = marker.pose.pose.position;
                
                
                RCLCPP_INFO(this->get_logger(), "Rilevamento stabile ottenuto! Inizio painting...");
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Marker rilevato a destinazione! ID: %d | Pos: [%.2f, %.2f, %.2f]", detected_id_, marker_pos_.x, marker_pos_.y, marker_pos_.z); 
                path_painting = generate_path_painting(marker_pos_, detected_id_);

                if (path_painting.empty()) {
                    RCLCPP_ERROR(this->get_logger(), "ERRORE: Generazione path fallita.");
                    state = IDLE;
                    return;
                }

                

                auto goal_msg = LinearTrajectory::Goal();
                goal_msg.s_type = "trapezoidal";
                goal_msg.traj_duration = 2.0;
                goal_msg.acc_duration = 0.2;
                goal_msg.total_time= 2.1;
                goal_msg.trajectory_len= 1000;
                goal_msg.end_pos[0]= path_painting[0][0];
                goal_msg.end_pos[1]= path_painting[0][1];
                goal_msg.end_pos[2]= path_painting[0][2];
                goal_msg.v_init = 0.0;
                goal_msg.v_end = 0.0;
                goal_msg.rot_x = (detected_id_ == 1)? 1.57 : -1.57;
                goal_msg.rot_y = 0.0;
                send_goal(goal_msg);
                if(!emitter_actived_){
                    std::string cmd_particles = "ign topic -t /model/kuka_robot/link/end_effector_nozzle/particle_emitter/emitter/cmd "
                                "-m ignition.msgs.ParticleEmitter -p 'emitting: {data: true}' &";
                    std::system(cmd_particles.c_str());
                    emitter_actived_=true;
                    
                }

                state = PAINTING; 
                
                break;
            }
        }
    }
  }
  void goal_response_callback(const GoalHandleLinTraj::SharedPtr & goal_handle)
    {
     if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
     } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
     }
  }

  void feedback_callback(
    GoalHandleLinTraj::SharedPtr,
    const std::shared_ptr<const LinearTrajectory::Feedback> feedback)
   { 
        std::stringstream ss;
    
        
        
        ss << "Time: " << feedback->elapsed_time << "s | ";
          
        
        /*ss << "Errors: ";
        for (auto const & err : feedback->err_pos) {
            ss << "[" << err.x << ", " << err.y << ", " << err.z << "] ";
        }*/
        
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());

        
  }

  bool validate_result(const GoalHandleLinTraj::WrappedResult & result) {
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            return true;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(),  "Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(),  "Unknown result code");
            break;
    }
    return false;
  }

  void result_callback(const GoalHandleLinTraj::WrappedResult & result)
  {
    static unsigned int idx_waypoints = 1;
    if (!validate_result(result)) {
        state = IDLE;
        detected_id_=-1;
        return;
    }

    if(state == MOVE_TO_ARUCO)
    {
        state = WAITING_DETECTION;
        detected_id_ = -1;
        total_mission_time_ = 0.0;
        std::stringstream ss;
        ss << "Time: " << std::fixed << std::setprecision(2) << result.result->total_execution_time << "s | ";
        
        /*ss << "Errors: ";
        auto final_error = result.result->err_pos_final;
        ss << "[" << final_error.x << ", " << final_error.y << ", " << final_error.z << "] ";*/
        
        
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        RCLCPP_INFO(this->get_logger(), " wating to read id marker");
        return;
        
        
    }

    if(state == PAINTING){
       total_mission_time_ += result.result->total_execution_time;
       
       
        if(idx_waypoints < number_waypoints){
         
         
            
         RCLCPP_INFO(this->get_logger(), "painting started");
         auto goal_msg = LinearTrajectory::Goal();
         goal_msg.s_type = "trapezoidal";
         goal_msg.traj_duration = 1.0;
         goal_msg.acc_duration = 0.2;
         goal_msg.total_time = 1.0;
         goal_msg.trajectory_len = 100;
         goal_msg.v_init = 0.0;
         goal_msg.v_end = 0.0;   
         goal_msg.end_pos[0] = path_painting[idx_waypoints][0];
         goal_msg.end_pos[1] = path_painting[idx_waypoints][1];
         goal_msg.end_pos[2] = path_painting[idx_waypoints][2];
         goal_msg.rot_x = (detected_id_ == 1) ? 1.57 : -1.57;
         goal_msg.rot_y = 0.0;
         send_goal(goal_msg);
         
         
         
         idx_waypoints++;
        }else{
           std::string cmd_off = "ign topic -t /model/kuka_robot/link/end_effector_nozzle/particle_emitter/emitter/cmd "
					          "-m ignition.msgs.ParticleEmitter -p 'emitting: {data: false}' &";
		   std::system(cmd_off.c_str());
           emitter_actived_ = false;
           std_msgs::msg::Bool painted_msg;
		   painted_msg.data = true;
		   pub_painted_->publish(painted_msg);
           RCLCPP_INFO(this->get_logger(), "painting finished");
           state = IDLE;
           active_varnish=false;
           idx_waypoints = 1; // Reset per il prossimo marker
           RCLCPP_INFO(this->get_logger(), "Painting completato. Tempo totale: %.2f", total_mission_time_);
        }
         
    
      }
          
  }
    
    
  
  WaypointVector generate_path_painting(geometry_msgs::msg::Point marker_pos, int id) {
    WaypointVector path;
    unsigned int j=0;
    RCLCPP_INFO(this->get_logger(), 
        "\033[1;35m[MARKER FISSU]\033[0m Target acquisito! ID: %d | Pos: x: %.3f, y: %.3f, z: %.3f", 
        id, marker_pos.x, marker_pos.y, marker_pos.z);
    
    if(id == 1 || id == 2) {
        
        for (unsigned int i = 0; i < number_waypoints; ++i) {
            Waypoint wp; 

            /*if(i == 0){
                wp[0] = marker_pos.x - (dim_face_box/2.0);
                wp[1] = (id ==1? (marker_pos.y - (dim_face_box/2.0)-0.1):(marker_pos.y + (dim_face_box/2.0)+0.1));
                wp[2] = marker_pos.z;//1.2;

            }else{

                if(i/2 ==j){
                    wp[0] = marker_pos.x;
                    wp[1] = (id ==1? marker_pos.y:marker_pos.y+0.2);
                    wp[2] = marker_pos.z+0.2;
                }
            }*/

            
            unsigned int r=(i+1)/2;
            wp[0] = marker_pos.x - (dim_face_box/2.0)+ j*step;
            wp[1] = (id ==1? (marker_pos.y - (dim_face_box/2.0)-0.1):(marker_pos.y + (dim_face_box/2.0)+0.1));
            wp[2] = marker_pos.z - (((i==0)||((i!=0)&&(r %2 == 0)))? 0 : (height_box/2.0));

            if(i%2 !=0){
                j++;
            }

            path.push_back(wp);

            RCLCPP_INFO(this->get_logger(), 
                "  -> Waypoint [%u]: x: %.3f, y: %.3f, z: %.3f", 
                i, wp[0], wp[1], wp[2]);
        }
    }
    RCLCPP_INFO(this->get_logger(), 
        "\033[1;32m[PATH GENERATED]\033[0m ID: %d | Numero Waypoints nel path: %zu", 
        id, path.size());
    return path;
  }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KukaActionClient>(rclcpp::NodeOptions{}));
    rclcpp::shutdown();
    return 0;
    
}

