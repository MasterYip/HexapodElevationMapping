#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

void odomCallback(const nav_msgs::Odometry &msg);

// 设置坐标转换关系
tf::Transform transform;  // World 2 odom
tf::Transform transform2; // odom 2 base_link
tf::Quaternion q;
tf::Quaternion q2;
void timerCallback(const ros::TimerEvent &event);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "world2odom_pub");
    ros::NodeHandle nh("~");

    std::string odomTopicName;
    std::string odomFrameName;
    std::string worldFrameName;

    ros::param::get("~odom_topic_name", odomTopicName);
    ros::param::get("~odom_frame_name", odomFrameName);
    ros::param::get("~world_frame_name", worldFrameName);

    ros::Subscriber odom_sub = nh.subscribe(odomTopicName, 10, odomCallback);
    ros::spin();
}
/**
 * @brief 接收机器人gazebo插件的torso Odometry的消息，回调函数中发布tf坐标关系，父坐标系为world，子坐标系为base_link
 * @param[in] msg           My Param doc
 * @author Tipriest (a1503741059@163.com)
 */
void odomCallback(const nav_msgs::Odometry &msg)
{
    static ros::Time last_timestamp;
    if (msg.header.stamp == last_timestamp)
    {
        ROS_WARN("Duplicate timestamp detected. Ignoring the data.");
        return;
    }
    last_timestamp = msg.header.stamp;
    static tf::TransformBroadcaster br;
    const geometry_msgs::Pose &pose = msg.pose.pose;
    const geometry_msgs::Point &pos = pose.position;
    const geometry_msgs::Quaternion &ori = pose.orientation;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "odom";
    transformStamped.transform.translation.x = pos.x;
    transformStamped.transform.translation.y = pos.y;
    transformStamped.transform.translation.z = pos.z;
    transformStamped.transform.rotation.x = ori.x;
    transformStamped.transform.rotation.y = ori.y;
    transformStamped.transform.rotation.z = ori.z;
    transformStamped.transform.rotation.w = ori.w;
    br.sendTransform(transformStamped);
}
