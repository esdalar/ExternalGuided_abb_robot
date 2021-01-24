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
    ros::Publisher * pose_pub;
    ros::Subscriber fta_sub;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    tf2_ros::TransformBroadcaster wobj_broadcaster;
    
    //tf2_ros::TransformListener tfListener(tfBuffer);

    ros::NodeHandle nh;

    void ftaCallback(const fiducial_msgs::FiducialTransformArrayPtr & msg);

    public:
    //geometry_msgs::TransformStamped desired_oMe;
    tf2_ros::StaticTransformBroadcaster oMe_broadcaster;
    EGMPathNode();
};


void EGMPathNode::ftaCallback(const fiducial_msgs::FiducialTransformArrayPtr & msg)
{
    int num_markers_detected = msg->transforms.size();
    if (num_markers_detected > 0)
    {
        fiducial_msgs::FiducialTransform ft = msg->transforms[0];
        int fiducial_id = ft.fiducial_id;
        int seqno = msg->header.seq;
        
        //ROS_INFO("Marker %d detected in position: %.2f %.2f %.2f",fiducial_id,ft.transform.translation.x,ft.transform.translation.y,ft.transform.translation.z);
        ROS_INFO("Marker %d detected in image %d",fiducial_id,seqno);

        geometry_msgs::TransformStamped input;
        input.header = msg->header;
        input.transform = ft.transform;
        input.child_frame_id = "work_object";
        wobj_broadcaster.sendTransform(input);
        
        geometry_msgs::TransformStamped output;
        try {
            //output = tfBuffer.lookupTransform("robot_desired_pose", "world", ros::Time(0));//ros::Time(0),msg->header.stamp);
            //output = tfBuffer.lookupTransform("robot_desired_pose", "base_link", ros::Time(0));//ros::Time(0),msg->header.stamp);
            output = tfBuffer.lookupTransform("base_link", "robot_desired_pose", ros::Time(0));//ros::Time(0),msg->header.stamp);
            pose_pub->publish(output);
            
            geometry_msgs::Vector3 T = output.transform.translation;
            /*tf2::Quaternion q(output.transform.rotation.x,output.transform.rotation.y,output.transform.rotation.z,output.transform.rotation.w);
            tf2::Matrix3x3 R(q);
            double roll, pitch, yaw;
            R.getRPY(roll,pitch,yaw);
            ROS_INFO("Robot desdired position: T %.2f %.2f %.2f, RPY %.2f %.2f %.2f",T.x,T.y,T.z,roll,pitch,yaw);*/
            ROS_INFO("Robot desdired position: T %.2f %.2f %.2f",T.x,T.y,T.z);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(0.2).sleep();
        }
    } else    
    {
        ROS_INFO("No marker detected");
    }
    
}

EGMPathNode::EGMPathNode() : nh(), tfListener(tfBuffer)
{
    pose_pub = new ros::Publisher(nh.advertise<geometry_msgs::TransformStamped>("egm_desired_pose",1));
    fta_sub = nh.subscribe("fiducial_transforms",1, &EGMPathNode::ftaCallback,this);
    
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "egm_path");

  //ros::NodeHandle n;
  EGMPathNode* node = new EGMPathNode();
  
  geometry_msgs::TransformStamped desired_oMe;

  desired_oMe.header.stamp = ros::Time::now();
  //desired_oMe.header.frame_id = "fiducial_101";
  desired_oMe.header.frame_id = "work_object";
  desired_oMe.child_frame_id = "robot_desired_pose";
  desired_oMe.transform.translation.x = 0.;
  desired_oMe.transform.translation.y = 0.;
  desired_oMe.transform.translation.z = 0.5;
  //tf2::Quaternion quat;
  //quat.setRPY(0.0, 0.0, 0.0); 0.5, -0.5, 0.5, 0.5
  desired_oMe.transform.rotation.x = -0.7071068;
  desired_oMe.transform.rotation.y = 0.7071068;
  desired_oMe.transform.rotation.z = 0.;
  desired_oMe.transform.rotation.w = 0.;
  /*desired_oMe.transform.rotation.x = 0.5;
  desired_oMe.transform.rotation.y = -0.5;
  desired_oMe.transform.rotation.z = 0.5;
  desired_oMe.transform.rotation.w = 0.5;*/

  node->oMe_broadcaster.sendTransform(desired_oMe);
  ROS_INFO("Broadcasting desired relative position for robot");
  //tf2_ros::Buffer tfBuffer;
  
  ros::spin();

  return 0;
}
