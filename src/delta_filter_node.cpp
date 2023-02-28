#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <queue>

// Rosparam parameters
const char* topic_publish_default = "/delta/pose";
const char* topic_pose_imu_default = "/posePub_merged";
const char* topic_pose_cam_default = "/camera/pose"; 

const char* frame_id_imu_default = "map";//"imu"; // SHOULD BE IMU!!!
const char* frame_id_cam_default = "camera_frame";

int imu_rate, cam_rate; // In Hz
int n_acc; // Accumulation window of slower pose stream 

// Which pose stream to use for interpolation (the faster one)
enum INTERPOLATE_STREAM {
    CAM, IMU, NONE
} interpolate;

// Flags to initialize pose streams
bool initialized_cam, initialized_imu;

// geometry_msgs Pose objects for IMU, CAM, and filtered pose
geometry_msgs::PoseStamped last_imu_pose, last_cam_pose, filtered_pose_msg;

// tf Pose objects for calculating interpolated delta
tf::Pose pose_interpolated, last_pose_interpolated;

// Pose accumulator for the faster stream
std::queue<geometry_msgs::PoseStamped> accumulator; 

inline void waitForAccumulator(double t) 
{
    while( !(accumulator.front().header.stamp.toSec() < t 
      && t < accumulator.back().header.stamp.toSec()) 
      && ros::ok() ) 
        ros::spinOnce();
}

inline void pushToAccumulator(const geometry_msgs::PoseStamped::ConstPtr &m)
{
    accumulator.push(*m);
    if (accumulator.size() > n_acc)
        accumulator.pop();
}

inline tf::Transform pose_diff(const geometry_msgs::PoseStamped::ConstPtr &m, geometry_msgs::PoseStamped &last_pose_msg)
{
    tf::Stamped<tf::Pose> current_pose, last_pose;
    tf::poseStampedMsgToTF(*m, current_pose);
    tf::poseStampedMsgToTF(last_imu_pose, last_pose);
    return current_pose.inverseTimes(last_pose);
} 

void imuMsgCallback(const geometry_msgs::PoseStamped::ConstPtr &m)
{
    // Initialization on first run
    if (!initialized_imu) {
        last_imu_pose = *m;
        initialized_imu = true;
        return;
    }
    
    // If cam interpolation is active, wait for cam timestamps to go past current time and do interpolation
    if (interpolate == CAM) {

        // Sanity check, can only interpolate if buffer is fully accumulated
        if (accumulator.size() == n_acc) {

            // Get timestamp from current and wait for accumulator to go past that  
            tfScalar t_current = m->header.stamp.toSec();
            waitForAccumulator(t_current);

            // Convert geometry_msgs to Quaternion format
            tf::Quaternion q1, q2, q_res;
            tf::quaternionMsgToTF(accumulator.front().pose.orientation, q1);
            tf::quaternionMsgToTF(accumulator.back().pose.orientation, q2);
            // Convert geometry_msgs Points to tf Vectors
            tf::Vector3 v1, v2, v_res;
            tf::pointMsgToTF(accumulator.front().pose.position, v1);
            tf::pointMsgToTF(accumulator.back().pose.position, v2);
            // Get time parameter for slerp
            double t_acc_last = accumulator.front().header.stamp.toSec();
            double t_acc_latest = accumulator.back().header.stamp.toSec();
            double t = (t_current - t_acc_last) / (t_acc_latest - t_acc_last);
            // Interpolate rotation
            q_res = tf::slerp(q1, q2, t);
            // Interpolate position
            v_res = tf::lerp(v1, v2, t);
            // Construct interpolated result
            pose_interpolated = tf::Pose(q_res, v_res);

            // Calculate interpolated delta
            tf::Pose diff_interpolated = pose_interpolated.inverseTimes(last_pose_interpolated);

            // Calculate measured delta
            tf::Pose diff_measured = pose_diff(m, last_imu_pose);

            // tf::poseTFToMsg(pose_interpolated, filtered_pose_msg.pose);
            // filtered_pose_msg.header = m->header;

          
            // For Debugging:
            // if ( t_acc_last < t_current 
            // && t_current < t_acc_latest) {
            //     ROS_INFO("OK! IMU_CURRENT: %f, CAM_LAST: %f, CAM_CURRENT: %f", t_current, accumulator.front().header.stamp.toSec(), accumulator.back().header.stamp.toSec());  

            
            // } else {
            //     ROS_INFO("WARN: IMU_CURRENT: %f, CAM_LAST: %f, CAM_CURRENT: %f", t_current, accumulator.front().header.stamp.toSec(), accumulator.back().header.stamp.toSec());
            // }
            last_pose_interpolated = pose_interpolated;
        }

    // Otherwise, if IMU interpolation is active, push current pose to queue
    } else if (interpolate == IMU) {
        pushToAccumulator(m);
    }

    // Save current pose to last pose for next iteration
    last_imu_pose = *m;
}

void camMsgCallback(const geometry_msgs::PoseStamped::ConstPtr &m)
{
    // Initialization on first run
    if (!initialized_cam) {
        last_cam_pose = *m;
        initialized_cam = true;
        return;
    }

    // If cam interpolation is active, push cam poses to queue
    if (interpolate == CAM) {
        pushToAccumulator(m);

    // Otherwise, if IMU interpolation is active, wait for the IMU poses in the queue
    } else if (interpolate == IMU) {
        
        // Sanity check, can only interpolate if buffer is fully accumulated
        if (accumulator.size() == n_acc) {

            // Get timestamp from current and wait for accumulator to go past that 
            tfScalar t_current = m->header.stamp.toSec();
            waitForAccumulator(t_current);
        }
    }
    
    // Save current pose to last pose for next iteration
    last_cam_pose = *m;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "delta_filter_node");
    ros::NodeHandle nh;

    // Topic params 
    std::string topic_publish, topic_pose_imu, topic_pose_cam;
    nh.param<std::string>("topic_publish", topic_publish, std::string(topic_publish_default)); 
    nh.param<std::string>("topic_pose_imu", topic_pose_imu, std::string(topic_pose_imu_default)); 
    nh.param<std::string>("topic_pose_cam", topic_pose_cam, std::string(topic_pose_cam_default)); 
    nh.param<int>("imu_rate", imu_rate, 125); // Jaspers Code uses 125 Hz by default
    nh.param<int>("cam_rate", cam_rate, 200); // Intels T265 uses 200 Hz by default

    // Determine accumulation window size of faster stream
    double imu_t = 1.0 / imu_rate, cam_t = 1.0 / cam_rate; 
    if (imu_rate < cam_rate) {
        interpolate = CAM;
        n_acc = std::ceil( imu_t / cam_t ) + 1;
        ROS_INFO("Interpolating Camera Pose measurements using %d samples", n_acc);
    } else if (cam_rate < imu_rate) {
        interpolate = IMU;
        n_acc = std::ceil( cam_t / imu_t ) + 1;
        ROS_INFO("Interpolating Imu Pose measurements using %d samples", n_acc);
    } else {
        interpolate = NONE;
        n_acc = 1;
        ROS_INFO("Interpolation disabled.");
    }
    
    // Initialize variables to false
    initialized_cam = false;
    initialized_imu = false;

    // Publishers and subscribers
    ros::Subscriber cam_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(topic_pose_cam, 1000, camMsgCallback);
    ros::Subscriber imu_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(topic_pose_imu, 1000, imuMsgCallback);
    ros::Publisher filtered_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(topic_publish, 1000);
    
    while(ros::ok()) {
        ros::spinOnce();
    }

    return 0;
}


