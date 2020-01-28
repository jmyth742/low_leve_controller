// ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

// ros_control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>

// NaN
#include <limits>

// ostringstream
#include <sstream>


class fourwd_robot : public hardware_interface::RobotHW
{
public:
  fourwd_robot()
 {
   ROS_INFO("robot interface begins...");
  }

  void init(ros::NodeHandle& nh){
    // connect and register the joint state interface
    //-- single rear wheel joint
    std::string rear_wheel_name = "rear_wheel_joint";
    nh.param("rear_wheel", rear_wheel_name, rear_wheel_name);

    //-- single front steer joint
    std::string front_steer_name = "front_steer_joint";
    nh.param("front_steer", front_steer_name, front_steer_name);

    hardware_interface::JointStateHandle state_handle_pos(front_steer_name, &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_pos);

    // c./ackermann_steering_controller/src/ackermann_steering_controller.cpponnect and register the joint position interface
    hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(front_steer_name), &cmd[0]);
    jnt_pos_interface.registerHandle(pos_handle);

    //vel
    hardware_interface::JointStateHandle state_handle_vel(rear_wheel_name, &pos[2], &vel[2], &eff[2]);
    jnt_state_interface.registerHandle(state_handle_vel);

    hardware_interface::JointHandle vel_handle(jnt_state_interface.getHandle(rear_wheel_name), &cmd[2]);
    jnt_vel_interface.registerHandle(vel_handle);

    //hardware_interface::JointHandle vel_handle_a(jnt_state_interface.getHandle("B"), &cmd[1]);
    registerInterface(&jnt_state_interface);

    registerInterface(&jnt_pos_interface);
    registerInterface(&jnt_vel_interface);

  //   // Velocity and acceleration limits:
  //   nh.param("linear/x/has_velocity_limits"    , limiter_lin_.has_velocity_limits    , limiter_lin_.has_velocity_limits    );
  //   nh.param("linear/x/has_acceleration_limits", limiter_lin_.has_acceleration_limits, limiter_lin_.has_acceleration_limits);
  //   nh.param("linear/x/has_jerk_limits"        , limiter_lin_.has_jerk_limits        , limiter_lin_.has_jerk_limits        );
  //   nh.param("linear/x/max_velocity"           , limiter_lin_.max_velocity           ,  limiter_lin_.max_velocity          );
  //   nh.param("linear/x/min_velocity"           , limiter_lin_.min_velocity           , -limiter_lin_.max_velocity          );
  //   nh.param("linear/x/max_acceleration"       , limiter_lin_.max_acceleration       ,  limiter_lin_.max_acceleration      );
  //   nh.param("linear/x/min_acceleration"       , limiter_lin_.min_acceleration       , -limiter_lin_.max_acceleration      );
  //   nh.param("linear/x/max_jerk"               , limiter_lin_.max_jerk               ,  limiter_lin_.max_jerk              );
  //   nh.param("linear/x/min_jerk"               , limiter_lin_.min_jerk               , -limiter_lin_.max_jerk              );
  //
  //   nh.param("angular/z/has_velocity_limits"    , limiter_ang_.has_velocity_limits    , limiter_ang_.has_velocity_limits    );
  //   nh.param("angular/z/has_acceleration_limits", limiter_ang_.has_acceleration_limits, limiter_ang_.has_acceleration_limits);
  //   nh.param("angular/z/has_jerk_limits"        , limiter_ang_.has_jerk_limits        , limiter_ang_.has_jerk_limits        );
  //   nh.param("angular/z/max_velocity"           , limiter_ang_.max_velocity           ,  limiter_ang_.max_velocity          );
  //   nh.param("angular/z/min_velocity"           , limiter_ang_.min_velocity           , -limiter_ang_.max_velocity          );
  //   nh.param("angular/z/max_acceleration"       , limiter_ang_.max_acceleration       ,  limiter_ang_.max_acceleration      );
  //   nh.param("angular/z/min_acceleration"       , limiter_ang_.min_acceleration       , -limiter_ang_.max_acceleration      );
  //   nh.param("angular/z/max_jerk"               , limiter_ang_.max_jerk               ,  limiter_ang_.max_jerk              );
  //   nh.param("angular/z/min_jerk"               , limiter_ang_.min_jerk               , -limiter_ang_.max_jerk              );
  //
  // // If either parameter is not available, we need to look up the value in the URDF
  // bool lookup_wheel_separation_h = !nh.getParam("wheel_separation_h", wheel_separation_h_);
  // bool lookup_wheel_radius = !nh.getParam("wheel_radius", wheel_radius_);


  }


  ros::Time getTime() const {return ros::Time::now();}
  ros::Duration getPeriod() const {return ros::Duration(0.01);}
  void read(){
  }
  void write(){

  }


private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  double cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];
};
