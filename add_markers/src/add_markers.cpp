/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <tf/transform_listener.h>

float delta = 0.4f;
bool pick = false;
bool drop = false;
bool wait = false;
float x1 = 0.5;
float y1 = -4.0;
float x2 = 0.5;
float y2 = -5.5;
float x1L = x1 - delta;
float y1L = y1 - delta;
float x1U = x1 + delta;
float y1U = y1 + delta;
float x2L = x2 - delta;
float y2L = y2 - delta;
float x2U = x2 + delta;
float y2U = y2 + delta;

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{

    float x_pos = msg->pose.pose.position.x;
    float y_pos = msg->pose.pose.position.y;


    // position need to be corrected with tf odom to map

    if(!pick) {
        ROS_INFO("ODOM POS ([%f],[%g])", x_pos, y_pos);
    }

    if (x_pos > x1L && x_pos < x1U &&
        y_pos > y1L && y_pos < y1U)
		{
		ROS_INFO("Pick point reached ([%f],[%g])", x_pos, y_pos);
        pick = true;
        drop = true;
        }
		
    if (x_pos > x2L && x_pos < x2U &&
        y_pos > y2L && y_pos < y2U + delta)
        {
        ROS_INFO("Drop point reached ([%f],[%g])", x_pos,y_pos);
        pick = true;
        drop = false;
        wait = false;
        } 

}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "basic_shapes");
    ros::NodeHandle n;
    ros::Rate r(20);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Subscriber odom_subscriber = n.subscribe("/odom", 1000, odom_callback);;

    // Set our marker type to be a bottle
    uint32_t shape = visualization_msgs::Marker::CUBE;
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    // INITALI THE MARKER IS IN THE START POSITIION
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;
    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0.5;
    marker.pose.position.y = -4.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    while (ros::ok())
    {
    marker.lifetime = ros::Duration();
        // Publish the marker
        while (marker_pub.getNumSubscribers() < 1)
        {
            if (!ros::ok())
            {
                return 0;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }

        if (wait) {
            marker.header.frame_id = "/plate_top_link";
            marker.pose.position.x = 0.0;
            marker.pose.position.y = 0.0;
        } 
		else {
            if (pick) {
                marker.header.frame_id = "/map";
                marker.pose.position.x = -4.0;
                marker.pose.position.y = 0.5;
            }
            if (drop) {
                marker.header.frame_id = "/map";
                marker.pose.position.x = -5.5;
                marker.pose.position.y = 0.5;
            }
        }
        
        marker_pub.publish(marker);
        ros::spinOnce();
    }
    return 0;
}


