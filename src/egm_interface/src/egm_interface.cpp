/***********************************************************************************************************************
 *
 * Copyright (c) 2018, ABB Schweiz AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with
 * or without modification, are permitted provided that
 * the following conditions are met:
 *
 *    * Redistributions of source code must retain the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer.
 *    * Redistributions in binary form must reproduce the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer in the documentation
 *      and/or other materials provided with the
 *      distribution.
 *    * Neither the name of ABB nor the names of its
 *      contributors may be used to endorse or promote
 *      products derived from this software without
 *      specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***********************************************************************************************************************
 */


#include <assert.h>
#include <sys/time.h>
#include <unistd.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>

#include <abb_libegm/egm_controller_interface.h>

# define M_PI       3.14159265358979323846  /* pi */

class EGMInterfaceNode{
  private:
  ros::Publisher * joint_pub;       // Robot joint values publisher for ros_control position controller (in Gazebo) 
  ros::Publisher * pose_pub;
  ros::Subscriber des_pos_sub;      // Robot reference cartesian position subscriber from egm_path

  //tf2_ros::Buffer tfBuffer;
  //tf2_ros::TransformListener tfListener;
  tf2_ros::TransformBroadcaster tfRobPose;  // TF broadcaster for the current robot cartesian position (from EGM)

  ros::NodeHandle nh;

  // Boost components for managing asynchronous UDP socket(s).
  boost::asio::io_service io_service;
  boost::thread_group thread_group;

  int port_number = 6510;                   // EGM sensor server port number

  abb::egm::wrapper::Input input;           // EGM Incoming message from robot controller (real or RobotStudio)
  abb::egm::wrapper::Output output;         // EGM Outcoming message to robot controller (real or RobotStudio)

  abb::egm::wrapper::CartesianPose initial_pose;    // Robot cartesian position at the beginning of the EGM communication
  abb::egm::wrapper::CartesianPose command_pose;    // Robot cartesian position to send through the EGM Outcoming message

  geometry_msgs::TransformStamped input_pose;       // Robot current cartesian position, from EGM incoming message
  geometry_msgs::TransformStamped reference_pose;   // Desired robot cartesian position, from egm_path
  geometry_msgs::Transform output_pose;             // Robot command cartesian position, sent in the EGM outcoming message
  

  const int egm_rate = 250.0; // [Hz] (EGM communication rate, specified by the EGMActPose RAPID instruction).
  int sequence_number = 0;    // [-] (sequence number of a received EGM message).
  double time = 0.0;          // [seconds] (elapsed time during an EGM communication session).

  double incr_max_trans = 0.015;          // Max allowed position increment (between input and output) [m]
  double incr_max_rot = 1.2*M_PI/180.;    // Max allowed rotation increment (between input and output) [rad]

  void callbackEGMPath(const geometry_msgs::TransformStampedPtr & msg);   // Callback to manage incoming message from egm_path
  geometry_msgs::Transform SaturateOutput();    // Function to saturate the command_pose considering the max allowed increments

  void PrintMsgTransform(const geometry_msgs::Vector3 trans, const geometry_msgs::Quaternion quat, const std::string name);
  void PrintTFTransform(const tf2::Vector3 trans, const tf2::Quaternion quat, const std::string name);
  void PrintTFQuatRPY(const tf2::Quaternion quat);
  void PrintTFQuatAngleAxis(const tf2::Quaternion quat);
  tf2::Vector3 SaturateVector3(const tf2::Vector3 input, const double max_length);
  tf2::Quaternion SaturateQuaternion(const tf2::Quaternion input, const double max_angle);
  tf2::Quaternion Quat_MsgToTF(const geometry_msgs::Quaternion q_msg);
  geometry_msgs::Quaternion Quat_TFToMsg(const tf2::Quaternion q_tf2);
  tf2::Vector3 Trans_MsgToTF(const geometry_msgs::Vector3 t_msg);
  geometry_msgs::Vector3 Trans_TFToMsg(const tf2::Vector3 t_tf2);

  sensor_msgs::JointState EGMJointsToJointState(const abb::egm::wrapper::Joints input_joints);
  sensor_msgs::JointState InitialJointState();
  std_msgs::Float64MultiArray EGMJointsToArmController(const abb::egm::wrapper::Joints input_joints);
  std_msgs::Float64MultiArray InitialArmController();
  geometry_msgs::TransformStamped EGMCartesianToTransformStamped(const abb::egm::wrapper::CartesianPose input_pose);
  geometry_msgs::TransformStamped InitialEGMTransformStamped();

  public:
  abb::egm::EGMControllerInterface egm_interface;
  abb::egm::wrapper::CartesianPose TransformToEGMCartesian(const geometry_msgs::Transform input_pose);

  EGMInterfaceNode();                   // Initialization of the node

  bool Initialize();                    // Initialize server for the EGM communication
  bool checkRAPIDExecutionState();      // Function to check if RAPID state has started correctly
  void CommandRobot();                  // Communication loop receiving and sending messages to the EGM
  void Shutdown();                      // Terminate processes before the shutdown

};

EGMInterfaceNode::EGMInterfaceNode() : nh(), egm_interface(io_service, port_number)
{
  joint_pub = new ros::Publisher(nh.advertise<std_msgs::Float64MultiArray>("abb_tfm/arm_controller/command",1));
  pose_pub = new ros::Publisher(nh.advertise<geometry_msgs::TransformStamped>("robot_egm",1));
  des_pos_sub = nh.subscribe("egm_ref", 1, &EGMInterfaceNode::callbackEGMPath,this);
}

sensor_msgs::JointState EGMInterfaceNode::InitialJointState()
{
  sensor_msgs::JointState output;
  int num = 6;
  double angle = 0;

  for (int idx = 0; idx < num; ++idx)
  {
    output.name.push_back("joint_" + std::to_string(idx+1));
    output.position.push_back(angle * M_PI/180.);
  } 
  return output;
}

sensor_msgs::JointState EGMInterfaceNode::EGMJointsToJointState(const abb::egm::wrapper::Joints input_joints)
{
  sensor_msgs::JointState output;
  //ROS_INFO("Test");

  for (int idx = 0; idx < input_joints.values_size(); ++idx)
  {
    output.name.push_back("joint_" + std::to_string(idx+1));
    output.position.push_back(input_joints.values(idx)*M_PI/180.);
  } 
  return output;
}

std_msgs::Float64MultiArray EGMInterfaceNode::InitialArmController()
{
  std_msgs::Float64MultiArray output;
  int num = 6;
  double angle = 0;

  output.layout.dim.push_back(std_msgs::MultiArrayDimension());
  output.layout.dim[0].size = num;
  output.layout.dim[0].stride = 1;
  output.layout.dim[0].label = "joints";

  output.data.clear();
  for (int idx = 0; idx < 6; ++idx)
  {
    output.data.push_back(angle*M_PI/180.);
  } 
  return output;
}

std_msgs::Float64MultiArray EGMInterfaceNode::EGMJointsToArmController(const abb::egm::wrapper::Joints input_joints)
{
  std_msgs::Float64MultiArray output;

  output.layout.dim.push_back(std_msgs::MultiArrayDimension());
  output.layout.dim[0].size = input_joints.values_size();
  output.layout.dim[0].stride = 1;
  output.layout.dim[0].label = "joints";

  output.data.clear();
  for (int idx = 0; idx < input_joints.values_size(); ++idx)
  {
    output.data.push_back(input_joints.values(idx)*M_PI/180.);
  } 
  return output;
}

geometry_msgs::TransformStamped EGMInterfaceNode::InitialEGMTransformStamped()
{
  geometry_msgs::TransformStamped output;

  output.header.stamp = ros::Time::now();
  output.header.frame_id = "base_link";
  output.child_frame_id = "robot_egm";
  output.transform.translation.x = 1445./1000.;
  output.transform.translation.y = 0.0/1000.;
  output.transform.translation.z = 760./1000.;
  //tf2::Quaternion quat;
  //quat.setRPY(0.0, 0.0, 0.0);
  output.transform.rotation.x = 0.;
  output.transform.rotation.y = -1.0;
  output.transform.rotation.z = 0.;
  output.transform.rotation.w = 0.;

  return output;
}

geometry_msgs::TransformStamped EGMInterfaceNode::EGMCartesianToTransformStamped(const abb::egm::wrapper::CartesianPose input_pose)
{
  geometry_msgs::TransformStamped output;

  output.header.stamp = ros::Time::now();
  output.header.frame_id = "base_link";
  output.child_frame_id = "robot_egm";
  output.transform.translation.x = input_pose.position().x()/1000.;
  output.transform.translation.y = input_pose.position().y()/1000.;
  output.transform.translation.z = input_pose.position().z()/1000.;
  //tf2::Quaternion quat;
  //quat.setRPY(0.0, 0.0, 0.0);
  output.transform.rotation.x = input_pose.quaternion().u1();
  output.transform.rotation.y = input_pose.quaternion().u2();
  output.transform.rotation.z = input_pose.quaternion().u3();
  output.transform.rotation.w = input_pose.quaternion().u0();

  return output;
}

abb::egm::wrapper::CartesianPose EGMInterfaceNode::TransformToEGMCartesian(const geometry_msgs::Transform input_pose)
{
  abb::egm::wrapper::CartesianPose output;
  output.mutable_position()->set_x(input_pose.translation.x*1000.);
  output.mutable_position()->set_y(input_pose.translation.y*1000.);
  output.mutable_position()->set_z(input_pose.translation.z*1000.);
  output.mutable_quaternion()->set_u1(input_pose.rotation.x);
  output.mutable_quaternion()->set_u2(input_pose.rotation.y);
  output.mutable_quaternion()->set_u3(input_pose.rotation.z);
  output.mutable_quaternion()->set_u0(input_pose.rotation.w);
  
  return output;
}

void EGMInterfaceNode::callbackEGMPath(const geometry_msgs::TransformStampedPtr & msg)
{
  reference_pose = * msg;

  output_pose = SaturateOutput();
  command_pose = TransformToEGMCartesian(output_pose);
}

geometry_msgs::Transform EGMInterfaceNode::SaturateOutput()
{
  bool saturation_verbose = true;

  geometry_msgs::Transform current = input_pose.transform;
  geometry_msgs::Transform reference = reference_pose.transform;

  tf2::Vector3 current_trans = Trans_MsgToTF(current.translation);
  tf2::Vector3 reference_trans = Trans_MsgToTF(reference.translation);
  tf2::Vector3 raw_correction_trans = reference_trans - current_trans;

  tf2::Vector3 sat_correction_trans = SaturateVector3(raw_correction_trans,incr_max_trans);
  tf2::Vector3 sat_reference_trans = current_trans + sat_correction_trans;

  tf2::Quaternion current_quat = Quat_MsgToTF(current.rotation);
  tf2::Quaternion reference_quat = Quat_MsgToTF(reference.rotation);
  tf2::Quaternion raw_correction_quat = current_quat.inverse() * reference_quat;

  tf2::Quaternion sat_correction_quat = SaturateQuaternion(raw_correction_quat,incr_max_rot);
  tf2::Quaternion sat_reference_quat = current_quat * sat_correction_quat;
  
  geometry_msgs::Transform sat_reference;
  sat_reference.translation = Trans_TFToMsg(sat_reference_trans);
  sat_reference.rotation = Quat_TFToMsg(sat_reference_quat);

  if (saturation_verbose && sequence_number%egm_rate == 0)
  {
    ROS_INFO("Saturation summary");
    //PrintTFTransform(current_trans,current_quat,"Current");
    //PrintTFTransform(reference_trans,reference_quat,"Reference");
    //PrintTFTransform(raw_correction_trans,raw_correction_quat,"Raw correction");
    PrintTFTransform(sat_correction_trans,sat_correction_quat,"Saturated correction");
    //PrintTFTransform(sat_reference_trans,sat_reference_quat,"Saturated reference");
  }
  return sat_reference;
}

void EGMInterfaceNode::PrintMsgTransform(const geometry_msgs::Vector3 trans, const geometry_msgs::Quaternion quat, const std::string name)
{
  PrintTFTransform(Trans_MsgToTF(trans),Quat_MsgToTF(quat),name);
}

void EGMInterfaceNode::PrintTFTransform(const tf2::Vector3 trans, const tf2::Quaternion quat, const std::string name)
{
  tf2::Vector3 T = trans * 1000.; //In millimetres
  ROS_INFO("%s frame", name.c_str());
  ROS_INFO("T [mm] %.1f %.1f %.1f, Q %.4f %.4f %.4f %.4f",T.getX(),T.getY(),T.getZ(),quat.getX(),quat.getY(),quat.getZ(),quat.getW());
  //PrintTFQuatRPY(quat);
  PrintTFQuatAngleAxis(quat);
}

void EGMInterfaceNode::PrintTFQuatRPY(const tf2::Quaternion quat)
{
  double R,P,Y;
  tf2::Matrix3x3(quat).getRPY(R,P,Y);
  ROS_INFO("RPY [deg] %.3f %.3f %.3f",R*180./M_PI,P*180./M_PI,Y*180./M_PI);
}

void EGMInterfaceNode::PrintTFQuatAngleAxis(const tf2::Quaternion quat)
{
  tf2::Vector3 axis = quat.getAxis();
  ROS_INFO("Angle [deg] %.3f, axis %.3f %.3f %.3f",quat.getAngleShortestPath()*180./M_PI,axis.getX(),axis.getY(),axis.getZ());
}

tf2::Vector3 EGMInterfaceNode::SaturateVector3(const tf2::Vector3 input, const double max_length)
{
  if (input.length() > max_length)
  {
    return input * max_length / input.length();
  }
  else
  {
    return input;
  }
}

tf2::Quaternion EGMInterfaceNode::SaturateQuaternion(const tf2::Quaternion input, const double max_angle)
{
  double angle = input.getAngleShortestPath();
  tf2::Vector3 axis = input.getAxis();
  if (angle > max_angle)
  {
    angle = max_angle;
  }
  tf2::Quaternion output(axis,angle);
  return output;
}

tf2::Quaternion EGMInterfaceNode::Quat_MsgToTF(const geometry_msgs::Quaternion q_msg)
{
  tf2::Quaternion q_tf2(q_msg.x,q_msg.y,q_msg.z,q_msg.w);
  return q_tf2;
}

geometry_msgs::Quaternion EGMInterfaceNode::Quat_TFToMsg(const tf2::Quaternion q_tf2)
{
  geometry_msgs::Quaternion q_msg;
  tf2::convert(q_tf2,q_msg);
  return q_msg;
}

tf2::Vector3 EGMInterfaceNode::Trans_MsgToTF(const geometry_msgs::Vector3 t_msg)
{
  tf2::Vector3 t_tf2(t_msg.x,t_msg.y,t_msg.z);
  return t_tf2;  
}

geometry_msgs::Vector3 EGMInterfaceNode::Trans_TFToMsg(const tf2::Vector3 t_tf2)
{
  geometry_msgs::Vector3 t_msg;
  t_msg.x = t_tf2.getX();
  t_msg.y = t_tf2.getY();
  t_msg.z = t_tf2.getZ();
  return t_msg;
}

bool EGMInterfaceNode::Initialize()
{
  if(!egm_interface.isInitialized())
  {
    ROS_ERROR("EGM interface failed to initialize (e.g. due to port already bound)");
    return false;
  }
  // Spin up a thread to run the io_service.
  thread_group.create_thread(boost::bind(&boost::asio::io_service::run, &io_service));

  joint_pub->publish(InitialArmController());
  
  return true;
}

bool EGMInterfaceNode::checkRAPIDExecutionState()
{
  bool wait = true;
  bool connection_established = false;
  while(ros::ok() && wait)
  {
    if(egm_interface.isConnected())
    {
      connection_established = true;
      if(egm_interface.getStatus().rapid_execution_state() == abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_UNDEFINED)
      {
        ROS_WARN("RAPID execution state is UNDEFINED (might happen first time after controller start/restart). Try to restart the RAPID program.");
      }
      else
      {
        wait = egm_interface.getStatus().rapid_execution_state() != abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_RUNNING;
        if (!wait) 
        {
          ROS_INFO("RAPID execution state is RUNNING");
          return true;
        }
      }
    }
    else 
    {
      if (connection_established) return false;
    }
    
    tfRobPose.sendTransform(InitialEGMTransformStamped());
    ros::Duration(0.5).sleep();
  }
}

void EGMInterfaceNode::Shutdown()
{
  io_service.stop();
  thread_group.join_all();
}

void EGMInterfaceNode::CommandRobot()
{
  bool wait = true;
  bool first_message = true;

  ROS_INFO("EGM command started");
  while(ros::ok() && wait)
  {
    // Wait for a new EGM message from the EGM client (with a timeout of 1000 ms).
    if(egm_interface.waitForMessage(1000))
    {
      // Read the message received from the EGM client.
      egm_interface.read(&input);
      
      // Publish robot joints and tf pose
      joint_pub->publish(EGMJointsToArmController(input.feedback().robot().joints().position()));

      input_pose = EGMCartesianToTransformStamped(input.feedback().robot().cartesian().pose());
      tfRobPose.sendTransform(input_pose);
      pose_pub->publish(input_pose);
      
      sequence_number = input.header().sequence_number();
      time = sequence_number/((double) egm_rate);

      if(first_message)//(sequence_number == 0)
      {
        // Reset all references, if it is the first message.
        output.Clear();
        initial_pose.CopyFrom(input.feedback().robot().cartesian().pose());
        command_pose.CopyFrom(initial_pose);
        command_pose.clear_euler();
        output_pose = input_pose.transform;
      }
      else
      {
        // Allow callback to get called
        ros::spinOnce();
      }
      // Pendiente: Elinimar angulos de Euler de output, tienen más prioridad
      if (output.mutable_robot()->mutable_cartesian()->mutable_pose()->has_euler())
      {
        ROS_INFO("Output has Euler");
        output.mutable_robot()->mutable_cartesian()->mutable_pose()->clear_euler();
        ROS_INFO("Deleted Euler angles");
      }
      output.mutable_robot()->mutable_cartesian()->mutable_pose()->CopyFrom(command_pose);
      // Write references back to the EGM client.
      egm_interface.write(output);

      if (sequence_number%egm_rate == 0 || first_message)
      {
        if (first_message) 
        {
          ROS_INFO("First Command message");
          first_message = false;
        }
        PrintMsgTransform(input_pose.transform.translation,input_pose.transform.rotation,"Input");
        PrintMsgTransform(output_pose.translation,output_pose.rotation,"Output");
      }
    }
    else
    {
      ROS_INFO("EGM communication finished, exiting...");
      wait = false;
    }
  }
}




int main(int argc, char** argv)
{
  //----------------------------------------------------------
  // Preparations
  //----------------------------------------------------------
  // Initialize the node.
  ros::init(argc, argv, "egm_interface");
  EGMInterfaceNode* node = new EGMInterfaceNode();

  // Check Server initialization
  if(!node->Initialize()) return 0;

  //----------------------------------------------------------
  // Execute a pose controller loop.
  //
  // Note: The EGM communication session is started by the
  //       EGMRunPose RAPID instruction.
  //----------------------------------------------------------
  ROS_INFO("Starting EGM Pose Controller");
  ROS_INFO("Waiting for an EGM communication session to start...");

  // Wait for correct RAPID Execution State and Start commanding the robot
  if (node->checkRAPIDExecutionState()) node->CommandRobot();

  // Run a smooth shutdown
  node->Shutdown();

  return 0;
}
