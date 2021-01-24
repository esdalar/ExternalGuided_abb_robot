/*
 * Copyright (c) 2017-20, Asea Brown Boveri S.A, Spain
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the FreeBSD Project.
 *
 */

#include <assert.h>
#include <sys/time.h>
#include <unistd.h>
#include <math.h>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_msgs/String.h>

#include "fiducial_msgs/Fiducial.h"
#include "fiducial_msgs/FiducialArray.h"
#include "fiducial_msgs/FiducialTransform.h"
#include "fiducial_msgs/FiducialTransformArray.h"

class EGMPathNode {
    private:
    ros::Publisher * egm_pub;       //Topic publisher for EGM_interface node
    ros::Subscriber aruco_sub;      //Topic subscriber for aruco_detect node

    tf2_ros::Buffer tfBuffer_target;                    //Buffer for the TF listener
    tf2_ros::TransformListener tfListener_target;       //TF listener for "robot_egm", "robot_target" -> Position error
    tf2_ros::Buffer tfBuffer_reference;                 //Buffer for the TF listener
    tf2_ros::TransformListener tfListener_reference;    //TF listener for "base_link", "robot_egm_ref" -> To be send to EGM
    tf2_ros::TransformBroadcaster wobj_broadcaster;     //TF broadcaster for workobject = aruco marker
    tf2_ros::TransformBroadcaster egm_broadcaster;      //TF broadcaster for "robot_egm_ref" : reference por EGM
    tf2_ros::StaticTransformBroadcaster target_broadcaster;     //TF broadcaster for "robot_target" : waypoints
    
    ros::NodeHandle nh;         // Ros node handle

    // Multiple targets variables
    
    std::vector<geometry_msgs::Transform> robot_targets;    //List of robot targets (waypoints)
    geometry_msgs::TransformStamped current_target;         //Transform from "work_object" to "robot_target" (current waypoint)
    geometry_msgs::TransformStamped current_to_target;      //Transform from "robot_egm" (current robot position) to "robot_target"
    geometry_msgs::TransformStamped current_to_egm;         //Transform from "robot_egm" to "robot_egm_ref", equivalent to position increment
    int step = 0;                       // Current step, used to change between targets (waypoints)
    double threshold_trans = 0.003;     // Convergence threshold for the targets (translation) in [m]
    double threshold_rot = 0.05;        // Convergence threshold for the targets (rotation) in [rad]

    ros::WallTime convergence_timer;    // Stores the latest moment a target was reached [s]
    double convergence_min_time = 0.1;  // Minimum required time to pass after a target was reached to converge again [s]
    bool last_target_reached = false;   // Indicates if last target has been reached

    // Error storage variables

    tf2::Vector3 Er_trans, Er_trans_accum; //Error vectors (x,y,z) for translation and accumulated translation
    tf2::Vector3 Er_rot_axis;              //Rotation error axis
    double er_trans = 0.;       // Magnitude of the translation error (Er_trans) [m]
    double er_rot = 0.;         // Magnitude of the rotation error (Er_rot) [rad]
    double er_trans_accum = 0.; // Magnitude of the translation accumulated error (Er_trans_accum) [m]

    // Control law variables

    tf2::Vector3 Incr_trans, Incr_trans_accum; //Increment vectors (x,y,z) for translation, and accumulated translation
    tf2::Quaternion Incr_rot;                   //Increment quaternion for rotation
    double Incr_rot_angle;
    double incr_max_trans = 0.005;      // Max allowed translation increment [m]
    double incr_max_trans_i = 0.002;    // Max allowed accumulated (integral) increment [m]
    double incr_max_rot = 0.5 * M_PI / 180.;         // Max allowed rotation increment [rad]

    double gain_trans_p = 0.8;          // Proportional translation gain [-]
    double gain_trans_i = 0.0; //0.01;         // Integral translation gain [-]
    double gain_rot_p = 0.5;            // Proportional rotation gain [-]

    // Additional variables

    int seqnum = 0;                     // Control law loop number
    bool print_info = false;            // Indicates if info about the loop is going to be printed
    bool marker_detected = false;       // Indicates if aruco marker is being detected

    void arucoCallback(const fiducial_msgs::FiducialTransformArrayPtr & msg);
    tf2::Vector3 Vector3Saturate(const tf2::Vector3 input, const double max_magnitude);
    double ScalarSaturate(const double input, const double max_value);

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

    public:   
    EGMPathNode();

    void AddTarget(const double x, const double y, const double z, const double q_x, const double q_y, const double q_z, const double q_w);
    bool UpdateData();
    void CalculateErrors();
    void ControlLaw();
    void Broadcast();

    void InitTargets();
    void NextRobotTarget();
    bool isTargetReached();
    void Test();
    void PrintQuat(const tf2::Quaternion, const std::string name);
};

EGMPathNode::EGMPathNode() : nh(), tfListener_target(tfBuffer_target), tfListener_reference(tfBuffer_reference)
{
    egm_pub = new ros::Publisher(nh.advertise<geometry_msgs::TransformStamped>("egm_ref",1));
    aruco_sub = nh.subscribe("fiducial_transforms",1, &EGMPathNode::arucoCallback,this);
    
}


void EGMPathNode::PrintMsgTransform(const geometry_msgs::Vector3 trans, const geometry_msgs::Quaternion quat, const std::string name)
{
  PrintTFTransform(Trans_MsgToTF(trans),Quat_MsgToTF(quat),name);
}

void EGMPathNode::PrintTFTransform(const tf2::Vector3 trans, const tf2::Quaternion quat, const std::string name)
{
  tf2::Vector3 T = trans * 1000.; //In millimetres
  ROS_INFO("%s frame", name.c_str());
  ROS_INFO("T [mm] %.1f %.1f %.1f, Q %.4f %.4f %.4f %.4f",T.getX(),T.getY(),T.getZ(),quat.getX(),quat.getY(),quat.getZ(),quat.getW());
  //PrintTFQuatRPY(quat);
  PrintTFQuatAngleAxis(quat);
}

void EGMPathNode::PrintTFQuatRPY(const tf2::Quaternion quat)
{
  double R,P,Y;
  tf2::Matrix3x3(quat).getRPY(R,P,Y);
  ROS_INFO("RPY [deg] %.3f %.3f %.3f",R*180./M_PI,P*180./M_PI,Y*180./M_PI);
}

void EGMPathNode::PrintTFQuatAngleAxis(const tf2::Quaternion quat)
{
  tf2::Vector3 axis = quat.getAxis();
  ROS_INFO("Angle [deg] %.3f, axis %.3f %.3f %.3f",quat.getAngleShortestPath()*180./M_PI,axis.getX(),axis.getY(),axis.getZ());
}

tf2::Quaternion EGMPathNode::Quat_MsgToTF(const geometry_msgs::Quaternion q_msg)
{
  tf2::Quaternion q_tf2(q_msg.x,q_msg.y,q_msg.z,q_msg.w);
  return q_tf2;
}

geometry_msgs::Quaternion EGMPathNode::Quat_TFToMsg(const tf2::Quaternion q_tf2)
{
  geometry_msgs::Quaternion q_msg;
  tf2::convert(q_tf2,q_msg);
  return q_msg;
}

tf2::Vector3 EGMPathNode::Trans_MsgToTF(const geometry_msgs::Vector3 t_msg)
{
  tf2::Vector3 t_tf2(t_msg.x,t_msg.y,t_msg.z);
  return t_tf2;  
}

geometry_msgs::Vector3 EGMPathNode::Trans_TFToMsg(const tf2::Vector3 t_tf2)
{
  geometry_msgs::Vector3 t_msg;
  t_msg.x = t_tf2.getX();
  t_msg.y = t_tf2.getY();
  t_msg.z = t_tf2.getZ();
  return t_msg;
}

tf2::Vector3 EGMPathNode::Vector3Saturate(const tf2::Vector3 input, const double max_magnitude)
{
    if (input.length() > max_magnitude)
    {
        return (max_magnitude/input.length()) * input ;
    }
    else
    {
        return input;
    }
}

double EGMPathNode::ScalarSaturate(const double input, const double max_value)
{
    if (abs(input) > max_value)
    {
        return max_value * input / abs(input);
    }
    else
    {
        return input;
    }
}

void EGMPathNode::AddTarget(const double x, const double y, const double z, const double q_x, const double q_y, const double q_z, const double q_w)
{
    geometry_msgs::Transform target;

    target.translation.x = x;
    target.translation.y = y;
    target.translation.z = z;
    target.rotation.x = q_x;
    target.rotation.y = q_y;
    target.rotation.z = q_z;
    target.rotation.w = q_w;
    robot_targets.push_back(target);
}

void EGMPathNode::InitTargets()
{
    int idx = 0;

    current_target.header.stamp = ros::Time::now();
    current_target.header.frame_id = "work_object";
    current_target.child_frame_id = "robot_target";

    ROS_INFO("Robot targets");
    for(geometry_msgs::Transform target_print : robot_targets)
    {
        ROS_INFO("Point %d: T %0.2f %0.2f %0.2f, Qxyzw %0.4f %0.4f %0.4f %0.4f",++idx,target_print.translation.x,target_print.translation.y,target_print.translation.z,target_print.rotation.x,target_print.rotation.y,target_print.rotation.z,target_print.rotation.w);
    }

    step = 0;
    convergence_timer = ros::WallTime::now();
    current_target.transform = robot_targets[step];
    target_broadcaster.sendTransform(current_target);
    ROS_INFO("Next target: T %.2f %.2f %.2f", current_target.transform.translation.x,current_target.transform.translation.y,current_target.transform.translation.z);
}

bool EGMPathNode::isTargetReached()
{
    double elapsed_time = (ros::WallTime::now()-convergence_timer).toSec();

    if ((elapsed_time > convergence_min_time) && !last_target_reached) //Pendiente: ver si es correcto
    {
        if ((er_trans<threshold_trans) && (er_rot<threshold_rot)) //Pendiente: ver las rotaciones
        {
            ROS_INFO("Step %d reached", step);
            return true;
        }
        else {
            if (print_info) ROS_INFO("Step not reached, Err T [mm] %.02f > %.02f, Err RPY %.02f > %.02f",er_trans*1000.,threshold_trans*1000.,er_rot, threshold_rot);
            return false;
        }
    }
    else{
        //if (print_info) ROS_INFO("Elapsed time = %.3f s, convergence_min_time = %.3f s", elapsed_time, convergence_min_time);
        return false;
    }
}

void EGMPathNode::NextRobotTarget()
{
    if (++step < robot_targets.size())    // Pendiente: Se está excediendo la longitud del vector (fixed, comprobar!)
    {
        current_target.header.stamp = ros::Time::now();
        current_target.transform = robot_targets[step];
        target_broadcaster.sendTransform(current_target);
        convergence_timer = ros::WallTime::now();
        ROS_INFO("Next target: T %.2f %.2f %.2f", current_target.transform.translation.x,current_target.transform.translation.y,current_target.transform.translation.z);
    }
    else 
    {
        ROS_INFO("No more targets defined");
        last_target_reached = true;
    }
}

bool EGMPathNode::UpdateData()
{
    try 
    {
        current_to_target = tfBuffer_target.lookupTransform("robot_egm", "robot_target", ros::Time(0));
        CalculateErrors();
    }
    catch (tf2::TransformException &ex) 
    {
        ROS_WARN("%s",ex.what());
        ros::Duration(0.2).sleep();
        return false;
    }
    return true;
}

void EGMPathNode::CalculateErrors()
{
    

    Er_trans.setX(current_to_target.transform.translation.x);
    Er_trans.setY(current_to_target.transform.translation.y);
    Er_trans.setZ(current_to_target.transform.translation.z);
    er_trans = Er_trans.length();
    
    //tf2::convert(quat,current_to_target.transform.rotation);
    tf2::Quaternion quat(current_to_target.transform.rotation.x,current_to_target.transform.rotation.y,current_to_target.transform.rotation.z,current_to_target.transform.rotation.w);
    er_rot = quat.getAngleShortestPath();
    Er_rot_axis = quat.getAxis();

    Er_trans_accum += Er_trans;
    Er_trans_accum = Vector3Saturate(Er_trans_accum, 0.050);
    er_trans_accum = Er_trans_accum.length();
    
    
    if (print_info)
    {
        ROS_INFO("SeqNum = %d, waypoint (%d/%d)", seqnum++, step+1,(int)robot_targets.size());
        ROS_INFO("Error T [mm] = %.2f : %.2f %.2f %.2f",er_trans*1000.,Er_trans.getX()*1000.,Er_trans.getY()*1000.,Er_trans.getZ()*1000.);
        //ROS_INFO("Error Taccum [mm] = %.2f : %.2f %.2f %.2f",er_trans_accum*1000.,Er_trans_accum.getX()*1000.,Er_trans_accum.getY()*1000.,Er_trans_accum.getZ()*1000.);
        ROS_INFO("Error R [deg] = %.3f, axis (%.2f,%.2f,%.2f)",er_rot*180./M_PI,Er_rot_axis.getX(),Er_rot_axis.getY(),Er_rot_axis.getZ());
    }
}

void EGMPathNode::ControlLaw()
{
    tf2::Quaternion quat;

    Incr_trans = gain_trans_p * Er_trans;               // Translation Proportional increment = Proportional gain * Translation error
    Incr_trans_accum = gain_trans_i * Er_trans_accum;   // Translation Integral increment = Integral gain * Translation accumulated error
    Incr_rot_angle = gain_rot_p * er_rot;                     // Rotation increment = Proportional gain * Rotation error

    //Incr_trans_accum = Vector3Saturate(Incr_trans_accum,incr_max_trans_i);      // Saturate Integral translation increment
    //Incr_trans = Vector3Saturate(Incr_trans+Incr_trans_accum,incr_max_trans);   // Saturate Proportional + Integral translation increment
    //Incr_rot_angle = ScalarSaturate(Incr_rot_angle,incr_max_rot);                         // Saturate Proportial rotation increment

    Incr_rot = tf2::Quaternion(Er_rot_axis,Incr_rot_angle);

    
    if (!marker_detected || last_target_reached) // If marker is not detected, robot increment is set to zero (=robot waits standstill)
    {
        current_to_egm.transform.translation.x = 0.;
        current_to_egm.transform.translation.y = 0.;
        current_to_egm.transform.translation.z = 0.;
        Incr_rot.setRPY(0.,0.,0.);
        current_to_egm.transform.rotation = tf2::toMsg(Incr_rot);
    }
    else // Normal increment control
    {
        if (print_info)
        {
            ROS_INFO("EGM Incr T [mm] %.2f %.2f %.2f, R [deg] %.3f (%.2f,%.2f,%.2f)",Incr_trans.getX()*1000.,Incr_trans.getY()*1000.,Incr_trans.getZ()*1000.,Incr_rot_angle*180./M_PI,Er_rot_axis.getX(),Er_rot_axis.getY(),Er_rot_axis.getZ());
            print_info = false;
        }
        current_to_egm.transform.translation.x = Incr_trans.getX();
        current_to_egm.transform.translation.y = Incr_trans.getY();
        current_to_egm.transform.translation.z = Incr_trans.getZ();
        
        current_to_egm.transform.rotation = tf2::toMsg(Incr_rot);
    }
    current_to_egm.header.stamp = ros::Time::now();
    current_to_egm.header.frame_id = "robot_egm";
    current_to_egm.child_frame_id = "robot_egm_ref";
}

void EGMPathNode::Broadcast()
{
    egm_broadcaster.sendTransform(current_to_egm);
    try {
        egm_pub->publish(tfBuffer_reference.lookupTransform("base_link", "robot_egm_ref", ros::Time(0)));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(0.2).sleep();
    }
}

void EGMPathNode::arucoCallback(const fiducial_msgs::FiducialTransformArrayPtr & msg)
{
    int num_markers_detected = msg->transforms.size();
    if (num_markers_detected > 0)
    {
        marker_detected = true;
        fiducial_msgs::FiducialTransform ft = msg->transforms[0];
        int fiducial_id = ft.fiducial_id;
        int seqno = msg->header.seq;
        
        ROS_INFO("Marker %d detected in image %d",fiducial_id,seqno);

        geometry_msgs::TransformStamped input;
        input.header = msg->header;
        input.transform = ft.transform;
        input.child_frame_id = "work_object";
        wobj_broadcaster.sendTransform(input);

    } else    
    {
        ROS_INFO("No marker detected");
        marker_detected = false;
    }
    print_info = true;
}

void EGMPathNode::Test()
{
    tf2::Quaternion q_origin(robot_targets[0].rotation.x,robot_targets[0].rotation.y,robot_targets[0].rotation.z,robot_targets[0].rotation.w);
    tf2::Quaternion q_middle;
    tf2::Quaternion q_end(robot_targets[1].rotation.x,robot_targets[1].rotation.y,robot_targets[1].rotation.z,robot_targets[1].rotation.w);
    tf2::Quaternion q_path;
    tf2::Quaternion q_scaled;

    q_path = q_origin.inverse()*q_end;
    q_middle = q_path * 0.5; //no funciona así
    q_middle = q_origin.slerp(q_end,0.5);
    q_scaled = tf2::Quaternion(q_path.getAxis(),2.0);

    PrintQuat(q_origin,"origin");
    PrintQuat(q_end,"end");
    PrintQuat(q_path,"origin to end");
    PrintQuat(q_middle,"middle");
    PrintQuat(q_scaled,"scaled");

    //quat.setRPY(Incr_rot.getX(),Incr_rot.getY(),Incr_rot.getZ());
    //current_to_egm.transform.rotation = tf2::toMsg(quat);
}

void EGMPathNode::PrintQuat(const tf2::Quaternion input, const std::string name)
{
    double roll,pitch,yaw;
    tf2::Quaternion q_reconstruct(input.getAxis(),input.getAngleShortestPath());

    ROS_INFO("Quaternion %s",name.c_str());
    ROS_INFO("\tValues: %.4f %.4f %.4f %.4f",input.getX(),input.getY(),input.getZ(),input.getW());
    ROS_INFO("\tLength: %.4f",input.length());
    ROS_INFO("\tShortest angle: %.4f %.4f",input.getAngleShortestPath(),input.getAngleShortestPath());
    tf2::Matrix3x3(input).getRPY(roll,pitch,yaw);
    ROS_INFO("\tRPY: %.3f %.3f %.3f",roll,pitch,yaw);
    ROS_INFO("Reconstruction: %.4f %.4f %.4f %.4f",q_reconstruct.getX(),q_reconstruct.getY(),q_reconstruct.getZ(),q_reconstruct.getW());
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "egm_path");
  EGMPathNode* node = new EGMPathNode();
  
  // Targets definition
  // tool0 facing aruco:  -0.7071068,0.7071068,0.,0.
  // tool0 facing floor:   0.5,-0.5,0.5,0.5

  // Target 1
  node->AddTarget(-0.43,0.4,1.0, -0.7071068,0.7071068,0.,0.);
  // Target 2
  //node->AddTarget(-0.43,0.4,0.63, 0.5,-0.5,0.5,0.5);
  node->AddTarget(-0.43,0.4,0.63, -0.7071068,0.7071068,0.,0.);
  // Target 3
  node->AddTarget(-0.43,0.4,1.0, 0.5,-0.5,0.5,0.5);

  // Initialize first target
  node->InitTargets();

  //node->Test();
  //return 0;

  ROS_INFO("Control started");
  while (ros::ok())
  {
    ros::spinOnce();            // Allows the execution of the aruco callback
    if (node->UpdateData())     // Updates position TF and calculates current robot position error, returns false if TF update was not succesful
    {
        if(node->isTargetReached()) node->NextRobotTarget();    // Checks if target has been reached and, if so, changes to the next target
        node->ControlLaw();     // Calculates next robot egm reference position
        node->Broadcast();      // Publishes and broadcast next robot egm reference
    }
  }
  return 0;
}
