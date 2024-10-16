/**
* This file is part of DSO.
* 
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/


#pragma once
#include "boost/thread.hpp"
#include "util/MinimalImage.h"
#include "IOWrapper/Output3DWrapper.h"


#include "FullSystem/HessianBlocks.h"
#include "util/FrameShell.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace dso
{

class FrameHessian;
class CalibHessian;
class FrameShell;


namespace IOWrap
{

class ROSOutputWrapper : public Output3DWrapper
{
public:
        inline ROSOutputWrapper(std::shared_ptr<rclcpp::Node> node)
        {
            _pub_campose = node->create_publisher<geometry_msgs::msg::PoseStamped>("campose", 2);
            printf("OUT: Created ROSOutputWrapper\n");
        }

        virtual ~ROSOutputWrapper()
        {
            printf("OUT: Destroyed ROSOutputWrapper\n");
        }

        virtual void publishCamPose(FrameShell* frame, CalibHessian* HCalib) override
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = rclcpp::Time(static_cast<int64_t>(frame->timestamp * 1e9));
            pose.header.frame_id = "world";

            // Extract the translation part
            pose.pose.position.x = frame->camToWorld.translation().x();
            pose.pose.position.y = frame->camToWorld.translation().y();
            pose.pose.position.z = frame->camToWorld.translation().z();

            // Extract the rotation part (as a quaternion)
            Eigen::Quaterniond quaternion = frame->camToWorld.unit_quaternion();
            pose.pose.orientation.x = quaternion.x();
            pose.pose.orientation.y = quaternion.y();
            pose.pose.orientation.z = quaternion.z();
            pose.pose.orientation.w = quaternion.w();

            _pub_campose->publish(pose);
        }
private:
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _pub_campose;

};



}



}