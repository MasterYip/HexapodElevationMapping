/**
 * @file tf_manager.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief tf_manager for T265
 * @version 0.1
 * @date 2024-09-23
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
    double odomTransFactor_ = 1.21;

  public:
    void odomCallback(const nav_msgs::Odometry &msg)
    {
        static ros::Time last_timestamp;
        if (msg.header.stamp == last_timestamp)
        {
            ROS_WARN("Duplicate timestamp detected. Ignoring the data.");
            return;
        }
        last_timestamp = msg.header.stamp;
        geometry_msgs::TransformStamped world2base_tf_;
        geometry_msgs::Pose pose_base;
        geometry_msgs::Pose odom_pose = msg.pose.pose;
        odom_pose.position.x *= odomTransFactor_;
        odom_pose.position.y *= odomTransFactor_;
        odom_pose.position.z *= odomTransFactor_;
        geometry_msgs::Twist twist_base;
        try
        { // transform from odom to base
            geometry_msgs::TransformStamped transformStamped;
            transformStamped =
                tfBuffer_.lookupTransform(odomFrameName_, baseFrameName_, ros::Time::now());
            // apply transform to the pose
            tf2::doTransform(odom_pose, pose_base, transformStamped);
            // apply transform rotation to the twist (from odom ref to world ref)
            tf2::Transform tf_transform;
            tf2::fromMsg(transformStamped.transform, tf_transform);
            tf2::Vector3 linear_vel_base(msg.twist.twist.linear.x * odomTransFactor_,
                                         msg.twist.twist.linear.y * odomTransFactor_,
                                         msg.twist.twist.linear.z * odomTransFactor_);
            tf2::Vector3 angular_vel_base(msg.twist.twist.angular.x, msg.twist.twist.angular.y,
                                          msg.twist.twist.angular.z);
            linear_vel_base = tf_transform.getBasis() * linear_vel_base;
            angular_vel_base = tf_transform.getBasis() * angular_vel_base;
            tf2::Transform tf_pose_base;
            tf2::fromMsg(pose_base, tf_pose_base);
            tf2::Vector3 linear_vel = tf_pose_base.getBasis() * linear_vel_base;
            tf2::Vector3 angular_vel = tf_pose_base.getBasis() * angular_vel_base;
            twist_base.linear.x = linear_vel.getX();
            twist_base.linear.y = linear_vel.getY();
            twist_base.linear.z = linear_vel.getZ();
            twist_base.angular.x = angular_vel.getX();
            twist_base.angular.y = angular_vel.getY();
            twist_base.angular.z = angular_vel.getZ();
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
        }
        world2base_tf_.header.stamp = msg.header.stamp;
        world2base_tf_.header.frame_id = worldFrameName_;
        world2base_tf_.child_frame_id = baseFrameName_;
        world2base_tf_.transform.translation.x = pose_base.position.x;
        world2base_tf_.transform.translation.y = pose_base.position.y;
        world2base_tf_.transform.translation.z = pose_base.position.z;
        world2base_tf_.transform.rotation = pose_base.orientation;
        br.sendTransform(world2base_tf_);

        nav_msgs::Odometry base_odom;
        base_odom.header.stamp = msg.header.stamp;
        base_odom.header.frame_id = worldFrameName_;
        base_odom.pose.pose = pose_base;    //Pose in WORLD frame
        base_odom.twist.twist = twist_base; //FIXME: Twist in BASE frame
        base_odom_pub.publish(base_odom);
    }

    TFManager() : tfListener_(tfBuffer_)
    {
        ros::NodeHandle nh_("~");

        ros::param::get("~odom_topic_name", odomTopicName_);
        ros::param::get("~base_odom_topic_name", baseOdomTopicName_);

        ros::param::get("~world_frame_name", worldFrameName_);
        ros::param::get("~base_frame_name", baseFrameName_);
        ros::param::get("~odom_frame_name", odomFrameName_);

        ros::param::get("~odom_trans_factor", odomTransFactor_);

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
