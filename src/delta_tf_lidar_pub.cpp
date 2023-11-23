#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <queue>

// Rosparam parameters
const char* topic_default = "/delta/pose";
const char* global_frame_default = "map";
const char* pose_frame_default = "odom";
std::string global_frame, pose_frame;

void deltaMsgCallback(const geometry_msgs::PoseStamped::ConstPtr &m)
{
    // Static Transform Map -> IMU
    static tf::TransformBroadcaster br;
	tf::Transform transform; 

    transform.setOrigin( 
        tf::Vector3(
            m->pose.position.x,
            m->pose.position.y,
            m->pose.position.z
        ) 
    );
    tf::Quaternion quat;
    tf::quaternionMsgToTF(m->pose.orientation, quat);
    transform.setRotation(quat);        
    br.sendTransform(tf::StampedTransform(transform, m->header.stamp, global_frame, pose_frame));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "delta_filter_node");
    ros::NodeHandle nh;

    int spinrate = 200; // Hz
    // Topic params 
    std::string topic_listen;
    nh.param<std::string>("topic_listen", topic_listen, std::string(topic_default)); 
    nh.param<std::string>("global_frame", global_frame, std::string(global_frame_default));
    nh.param<std::string>("pose_frame", pose_frame, std::string(pose_frame_default));
    nh.param<int>("cam_rate", spinrate, 200);
    // Publishers and subscribers
    ros::Subscriber delta_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(topic_listen, 1000, deltaMsgCallback);

    // Main processing loop, wait for callbacks to happen
    ros::Rate rate(spinrate*2);
    while(ros::ok()) {
        ros::spinOnce();
	    rate.sleep();
    }

    return 0;
}


