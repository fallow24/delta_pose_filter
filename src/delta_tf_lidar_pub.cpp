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
const char* pose_frame_default = "imu";
std::string global_frame, pose_frame;

ros::Publisher lidar_pub;
sensor_msgs::PointCloud2 msg;

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
    tf::Quaternion quat; // here changed order: x y z w !!
    tf::quaternionMsgToTF(m->pose.orientation, quat);
    transform.setRotation(quat);        
    br.sendTransform(tf::StampedTransform(transform, m->header.stamp, global_frame, pose_frame));
}

void lidarMsgCallback(const sensor_msgs::PointCloud2::ConstPtr& m)
{
    std_msgs::Header header;
    header.seq =  m->header.seq;
    header.frame_id = m->header.frame_id;

    // important part right here
    header.stamp = ros::Time::now(); 

    msg.header = header;
    msg.height = m->height;
    msg.width = m->width;
    msg.data = m->data;
    msg.fields = m->fields;
    msg.is_bigendian = m->is_bigendian;
    msg.is_dense = m->is_dense;
    msg.point_step = m->point_step;
    msg.row_step = m->row_step; 

    lidar_pub.publish(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "delta_filter_node");
    ros::NodeHandle nh;

    // Topic params 
    std::string topic_listen;
    nh.param<std::string>("topic_listen", topic_listen, std::string(topic_default)); 
    nh.param<std::string>("global_frame", global_frame, std::string(global_frame_default));
    nh.param<std::string>("pose_frame", pose_frame, std::string(pose_frame_default));

    // Publishers and subscribers
    ros::Subscriber delta_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(topic_listen, 1000, deltaMsgCallback);

    // Subscriber to update Livox Time
    ros::Subscriber lidar_pose_sub = nh.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 1000, lidarMsgCallback);
    lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("/livox/lidar_t", 1000);

    // Main processing loop, wait for callbacks to happen
    while(ros::ok()) {
        // TODO: add loop rate based on publish frequency
        ros::spinOnce();
    }

    return 0;
}


