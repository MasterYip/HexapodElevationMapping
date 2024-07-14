/**
 * @file tf_manager2.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief tf_manager for optitrack
 * @version 0.1
 * @date 2024-07-14
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <Eigen/Core>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

class TFManager
{
  private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub;
    ros::Publisher base_odom_pub;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;

    tf2_ros::TransformBroadcaster br;

    std::string odomTopicName_;
    std::string baseOdomTopicName_;

    std::string worldFrameName_;
    std::string baseFrameName_;
    std::string odomFrameName_;

  public:
    void odomCallback(const geometry_msgs::PoseStamped &msg)
    {
        static ros::Time last_timestamp;
        if (msg.header.stamp == last_timestamp)
        {
            ROS_WARN("Duplicate timestamp detected. Ignoring the data.");
            return;
        }
        last_timestamp = msg.header.stamp;
        geometry_msgs::TransformStamped world2base_tf_;
        // geometry_msgs::Pose pose_base;
        // geometry_msgs::Twist twist_base;
        // try
        // {
        //     geometry_msgs::TransformStamped transformStamped;
        //     transformStamped =
        //         tfBuffer_.lookupTransform(odomFrameName_, baseFrameName_, ros::Time::now());   // odomFrameName_: "HexPercept3_odom", baseFrameName_: "base"
        //     // apply transform to the pose
        //     tf2::doTransform(msg.pose.pose, pose_base, transformStamped);
        //     // apply transform rotation to the twist (from odom ref to world ref)
        //     tf2::Transform tf_transform;
        //     tf2::fromMsg(transformStamped.transform, tf_transform);
        //     tf2::Vector3 linear_vel_base(msg.twist.twist.linear.x, msg.twist.twist.linear.y,
        //                             msg.twist.twist.linear.z);
        //     tf2::Vector3 angular_vel_base(msg.twist.twist.angular.x, msg.twist.twist.angular.y,
        //                              msg.twist.twist.angular.z);
        //     linear_vel_base = tf_transform.getBasis() * linear_vel_base;  // 获得tf_transform的目的就是 获得 basis matrix for the rotation 3*3的旋转矩阵
        //     angular_vel_base = tf_transform.getBasis() * angular_vel_base;
        //     tf2::Transform tf_pose_base;
        //     tf2::fromMsg(pose_base, tf_pose_base);
        //     tf2::Vector3 linear_vel = tf_pose_base.getBasis() * linear_vel_base;
        //     tf2::Vector3 angular_vel = tf_pose_base.getBasis() * angular_vel_base;
        //     twist_base.linear.x = linear_vel.getX();
        //     twist_base.linear.y = linear_vel.getY();
        //     twist_base.linear.z = linear_vel.getZ();
        //     twist_base.angular.x = angular_vel.getX();
        //     twist_base.angular.y = angular_vel.getY();
        //     twist_base.angular.z = angular_vel.getZ();
        // }
        // catch (tf2::TransformException &ex)
        // {
        //     ROS_WARN("%s", ex.what());
        // }
        // world2base_tf_.header.stamp = msg.header.stamp;
        // world2base_tf_.header.frame_id = worldFrameName_; // worldFrameName_: "odom"
        // world2base_tf_.child_frame_id = baseFrameName_;   // baseFrameName_: "base"
        // world2base_tf_.transform.translation.x = msg.pose.position.x;
        // world2base_tf_.transform.translation.y = msg.pose.position.y;
        // world2base_tf_.transform.translation.z = msg.pose.position.z;
        // world2base_tf_.transform.rotation = msg.pose.orientation;
        // br.sendTransform(world2base_tf_); // 建立odom和base之间的tf关系

        geometry_msgs::Twist twist_base;
        twist_base.linear.x = 0;
        twist_base.linear.y = 0;
        twist_base.linear.z = 0;
        twist_base.angular.x = 0;
        twist_base.angular.y = 0;
        twist_base.angular.z = 0;

        nav_msgs::Odometry base_odom;
        base_odom.header.stamp = msg.header.stamp;
        base_odom.header.frame_id = worldFrameName_;
        base_odom.pose.pose = msg.pose;     //Pose in WORLD frame
        base_odom.twist.twist = twist_base; //FIXME: Twist in BASE frame
        base_odom_pub.publish(base_odom);   // 这应该是base在odom坐标系下的位姿和速度
    }

    TFManager() : tfListener_(tfBuffer_)
    {
        ros::NodeHandle nh_("~");

        ros::param::get("~odom_topic_name", odomTopicName_);
        ros::param::get("~base_odom_topic_name",
                        baseOdomTopicName_); // baseOdomTopicName_: "/base_odom"

        ros::param::get("~world_frame_name", worldFrameName_); // worldFrameName_: "odom"
        ros::param::get("~base_frame_name", baseFrameName_);
        ros::param::get("~odom_frame_name", odomFrameName_);

        odom_sub = nh_.subscribe(odomTopicName_, 10, &TFManager::odomCallback, this);
        base_odom_pub = nh_.advertise<nav_msgs::Odometry>(baseOdomTopicName_, 1);
        ros::spin();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_manager");
    TFManager();
    return 0;
}
