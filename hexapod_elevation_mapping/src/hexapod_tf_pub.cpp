#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

void odomCallback(const nav_msgs::Odometry& msg);

// 设置坐标转换关系
tf::Transform transform;   // World 2 odom
tf::Transform transform2;  // odom 2 base_link
tf::Quaternion q;
tf::Quaternion q2;
void timerCallback(const ros::TimerEvent& event);

int main(int argc, char** argv) {
  ros::init(argc, argv, "poseLinkPub");
  ros::NodeHandle nh("~");

  std::string odomTopicName;
  ros::param::get("~odom_topic_name", odomTopicName);
  ROS_INFO("odomTopicName is %s", odomTopicName.c_str());
  // nh.param<std::string>("/elspider_air/pose_link_pub_node/odomTopicName_", odomTopicName, std::string("/torso_odom"));
  ros::Subscriber odom_sub = nh.subscribe(odomTopicName, 10, odomCallback);

  double var[6];
  nh.param<double>("base_link2velodyne_lidar_x_trans_offset", var[0], double(0.0));
  nh.param<double>("base_link2velodyne_lidar_y_trans_offset", var[1], double(0.0));
  nh.param<double>("base_link2velodyne_lidar_z_trans_offset", var[2], double(0.0));
  nh.param<double>("base_link2velodyne_lidar_pitch_revolute_offset", var[3], double(0.0));
  nh.param<double>("base_link2velodyne_lidar_roll_revolute_offset", var[4], double(0.0));
  nh.param<double>("base_link2velodyne_lidar_yaw_revolute_offset", var[5], double(0.0));
  transform.setOrigin(tf::Vector3(var[0], var[1], var[2]));
  q.setRPY(var[3], var[4], var[5]);
  transform.setRotation(q);

  double var2[6];
  nh.param<double>("odom_link2base_link_x_trans_offset", var2[0], double(0.0));
  nh.param<double>("odom_link2base_link_y_trans_offset", var2[1], double(0.0));
  nh.param<double>("odom_link2base_link_z_trans_offset", var2[2], double(0.0));
  nh.param<double>("odom_link2base_link_pitch_revolute_offset", var2[3], double(0.0));
  nh.param<double>("odom_link2base_link_roll_revolute_offset", var2[4], double(0.0));
  nh.param<double>("odom_link2base_link_yaw_revolute_offset", var2[5], double(0.0));
  transform2.setOrigin(tf::Vector3(var2[0], var2[1], var2[2]));
  q2.setRPY(var2[3], var2[4], var2[5]);
  transform2.setRotation(q2);

  ros::Timer timer = nh.createTimer(ros::Duration(0.05), timerCallback);
  // nh.param("fsm/thresh_replan1",       fp_->replan_thresh1_, -1.0);
  // nh.getParam("poseLinkPub");
  ros::spin();
}
/**
 * @brief 接收机器人gazebo插件的torso Odometry的消息，回调函数中发布tf坐标关系，父坐标系为world，子坐标系为base_link
 * @param[in] msg           My Param doc
 * @author Tipriest (a1503741059@163.com)
 */
void odomCallback(const nav_msgs::Odometry& msg) {
  static ros::Time last_timestamp;
  if (msg.header.stamp == last_timestamp) {
    ROS_WARN("Duplicate timestamp detected. Ignoring the data.");
    return;
  }

  // 更新上一个时间戳
  last_timestamp = msg.header.stamp;
  static tf::TransformBroadcaster br;
  // 提取Odometry消息中的位置和姿态信息
  const geometry_msgs::Pose& pose = msg.pose.pose;
  const geometry_msgs::Point& pos = pose.position;
  const geometry_msgs::Quaternion& ori = pose.orientation;
  // 创建TransformStamped消息
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
  // send the transform
  br.sendTransform(transformStamped);
}

void timerCallback(const ros::TimerEvent& event) {
  // 在定时器回调函数中处理逻辑
  static tf::TransformBroadcaster static_broadcaster;
  static_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/base_link", "/velodyneLidar"));
  static_broadcaster.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "/odom", "/base_link"));
}