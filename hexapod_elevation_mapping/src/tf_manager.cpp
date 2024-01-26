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
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;

    tf2_ros::TransformBroadcaster br;

    std::string odomTopicName_;

    std::string worldFrameName_;
    std::string baseFrameName_;
    std::string odomFrameName_;

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
        try
        {
            geometry_msgs::TransformStamped transformStamped;
            transformStamped =
                tfBuffer_.lookupTransform(odomFrameName_, baseFrameName_, ros::Time::now());
            // apply transform to the pose
            tf2::doTransform(msg.pose.pose, pose_base, transformStamped);
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
    }

    TFManager() : tfListener_(tfBuffer_)
    {
        ros::NodeHandle nh_("~");

        ros::param::get("~odom_topic_name", odomTopicName_);

        ros::param::get("~world_frame_name", worldFrameName_);
        ros::param::get("~base_frame_name", baseFrameName_);
        ros::param::get("~odom_frame_name", odomFrameName_);

        odom_sub = nh_.subscribe(odomTopicName_, 10, &TFManager::odomCallback, this);

        ros::spin();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_manager");
    TFManager();
    return 0;
}
