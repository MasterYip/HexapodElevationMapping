#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

class World2OdomPub
{
  private:
    ros::NodeHandle nh_;
    std::string odomTopicName_;
    std::string odomFrameName_;
    std::string worldFrameName_;
    ros::Subscriber odom_sub;

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
        static tf::TransformBroadcaster br;
        const geometry_msgs::Pose &pose = msg.pose.pose;
        const geometry_msgs::Point &pos = pose.position;
        const geometry_msgs::Quaternion &ori = pose.orientation;
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = worldFrameName_;
        transformStamped.child_frame_id = odomFrameName_;
        transformStamped.transform.translation.x = pos.x;
        transformStamped.transform.translation.y = pos.y;
        transformStamped.transform.translation.z = pos.z;
        transformStamped.transform.rotation.x = ori.x;
        transformStamped.transform.rotation.y = ori.y;
        transformStamped.transform.rotation.z = ori.z;
        transformStamped.transform.rotation.w = ori.w;
        br.sendTransform(transformStamped);
    }

    World2OdomPub()
    {
        ros::NodeHandle nh_("~");

        ros::param::get("~odom_topic_name", odomTopicName_);
        ros::param::get("~odom_frame_name", odomFrameName_);
        ros::param::get("~world_frame_name", worldFrameName_);

        odom_sub = nh_.subscribe(odomTopicName_, 10, &World2OdomPub::odomCallback, this);

        ros::spin();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "world2odom_pub");
    World2OdomPub();
    return 0;
}
