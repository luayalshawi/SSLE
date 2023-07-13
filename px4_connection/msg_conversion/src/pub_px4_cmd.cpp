#include <ros/ros.h>
#include <iostream>
#include <prometheus_msgs/ControlCommand.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Path.h>
#include "state_from_mavros.h"
#include "message_utils.h"
#include "command_to_mavros.h"
#include "prometheus_control_utils.h" 
#include "KeyboardEvent.h"
#include "quadrotor_msgs/PositionCommand.h"


#define NODE_NAME "fuel_control"
#define VEL_XY_STEP_SIZE 0.1
#define VEL_Z_STEP_SIZE 0.1
#define YAW_STEP_SIZE 0.08
#define TRA_WINDOW 2000

using namespace std;


prometheus_msgs::ControlCommand Command_to_pub;//即将发布的command

ros::Publisher move_pub;
ros::Subscriber fuel_cmd_sub;

//callback function
void cmd_transfer_cb(const quadrotor_msgs::PositionCommand cmd)
{
   
    Command_to_pub.header.stamp = ros::Time::now();
    Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
    Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1; 
    Command_to_pub.source = NODE_NAME;
    Command_to_pub.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_ALL;
    Command_to_pub.Reference_State.Move_frame      = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_to_pub.Reference_State.position_ref[0] = cmd.position.x;
    Command_to_pub.Reference_State.position_ref[1] = cmd.position.y;
    Command_to_pub.Reference_State.position_ref[2] = cmd.position.z;
    Command_to_pub.Reference_State.velocity_ref[0] = cmd.velocity.x;
    Command_to_pub.Reference_State.velocity_ref[1] = cmd.velocity.y;
    Command_to_pub.Reference_State.velocity_ref[2] = cmd.velocity.z;
    Command_to_pub.Reference_State.acceleration_ref[0] = cmd.acceleration.x;
    Command_to_pub.Reference_State.acceleration_ref[1] = cmd.acceleration.y;
    Command_to_pub.Reference_State.acceleration_ref[2] = cmd.acceleration.z;
    Command_to_pub.Reference_State.yaw_ref  = cmd.yaw;

    move_pub.publish(Command_to_pub);
    
}

//mian function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pub_px4_cmd");
    ros::NodeHandle nh("~");
    KeyboardEvent keyboardcontrol;
    char key_now, key_wait;
    char key_last;
    string uav_name;
    nh.param<string>("uav_name", uav_name, "/uav0");
    if (uav_name == "/uav0")
        uav_name = "";

    //　【发布】　控制指令
    move_pub = nh.advertise<prometheus_msgs::ControlCommand>(uav_name + "/prometheus/control_command", 10);

    fuel_cmd_sub = nh.subscribe<quadrotor_msgs::PositionCommand>(uav_name + "/planning/pos_cmd", 10, cmd_transfer_cb);


    // 初始化命令 - Idle模式 电机怠速旋转 等待来自上层的控制指令
    Command_to_pub.Mode                                = prometheus_msgs::ControlCommand::Idle;
    Command_to_pub.Command_ID                          = 0;
    Command_to_pub.source = NODE_NAME;
    Command_to_pub.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
    Command_to_pub.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_to_pub.Reference_State.position_ref[0]     = 0;
    Command_to_pub.Reference_State.position_ref[1]     = 0;
    Command_to_pub.Reference_State.position_ref[2]     = 0;
    Command_to_pub.Reference_State.velocity_ref[0]     = 0;
    Command_to_pub.Reference_State.velocity_ref[1]     = 0;
    Command_to_pub.Reference_State.velocity_ref[2]     = 0;
    Command_to_pub.Reference_State.acceleration_ref[0] = 0;
    Command_to_pub.Reference_State.acceleration_ref[1] = 0;
    Command_to_pub.Reference_State.acceleration_ref[2] = 0;
    Command_to_pub.Reference_State.yaw_ref             = 0;


    while (ros::ok())
    {
      nh.getParam("uav_name", uav_name);

      sleep(5.0);
      // ARM
      cout << " " <<endl;
      cout << "Arm and Switch to OFFBOARD." <<endl;
  
      Command_to_pub.header.stamp = ros::Time::now();
      Command_to_pub.Mode = prometheus_msgs::ControlCommand::Idle;
      Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
      Command_to_pub.source = NODE_NAME;
      Command_to_pub.Reference_State.yaw_ref = 999;
      move_pub.publish(Command_to_pub);
      sleep(1.0);

      // Takeoff
      cout << " " <<endl;
      cout << "Switch to Takeoff Mode." <<endl;
      Command_to_pub.header.stamp = ros::Time::now();
      Command_to_pub.Mode = prometheus_msgs::ControlCommand::Takeoff;
      Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
      Command_to_pub.Reference_State.yaw_ref = 0.0;
      Command_to_pub.source = NODE_NAME;
      move_pub.publish(Command_to_pub);
      sleep(1.0);


      cout << " " <<endl;
      cout << "FUEL exploration starts." <<endl;

      sleep(0.5);

      keyboardcontrol.RosWhileLoopRun();
      key_wait = keyboardcontrol.GetPressedKey();
      while(key_wait != U_KEY_H){
        keyboardcontrol.RosWhileLoopRun();
        key_wait = keyboardcontrol.GetPressedKey();
        ros::spinOnce();
      }

      cout << " " <<endl;
      cout << "FUEL exploration ends." <<endl;


      keyboardcontrol.RosWhileLoopRun();
      key_now = keyboardcontrol.GetPressedKey();
    }

    return 0;
}