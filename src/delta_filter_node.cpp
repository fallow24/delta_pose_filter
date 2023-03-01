#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

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

// Transformation between IMU and Camera pose frame (FROM Imu TO Cam)
tf::StampedTransform tf_imu2cam, axes_imu2cam;
ros::Publisher filtered_pose_pub;

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
            tf::Pose rotated_diff_interpolated;
            rotated_diff_interpolated.mult(tf_imu2cam, diff_interpolated);

            tf::Vector3 orig(0,0,0);

            // Calculate measured delta
            tf::Pose diff_measured = pose_diff(m, last_imu_pose);

            tf::Vector3 ti = diff_interpolated.getOrigin();
            tf::Vector3 rti = rotated_diff_interpolated.getOrigin();
            tf::Vector3 tm = diff_measured.getOrigin();

            ROS_INFO("Interpolated diff translation:\t\tx=%f, y=%f, z=%f", ti.x(), ti.y(), ti.z());
            ROS_INFO("Rotated interpolated diff translation:\tx=%f, y=%f, z=%f", rti.x(), rti.y(), rti.z());
            ROS_INFO("Measured diff translation:\t\tx=%f, y=%f, z=%f\n", tm.x(), tm.y(), tm.z());
            // ROS_INFO("Interpolated r_diff: %f, Measured r_diff: %f", diff_interpolated.getRotation().angle( tf::Quaternion::getIdentity() ), diff_measured.getRotation().angle( tf::Quaternion::getIdentity() ));
            // ROS_INFO("Interpolated t_diff: %f, Measured t_diff: %f", diff_interpolated.getOrigin().distance( orig ), diff_measured.getOrigin().distance( orig ));

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
    
    // Testing, apply correct rotation
    tf::Stamped<tf::Pose> this_pose;
    tf::poseStampedMsgToTF(*m, this_pose);
    tf::Pose rotated_pose;

    rotated_pose.mult(this_pose, axes_imu2cam);
    rotated_pose.mult(tf_imu2cam, rotated_pose);
    
    filtered_pose_msg.header = m->header;
    filtered_pose_msg.header.frame_id = "map";
    tf::poseTFToMsg(rotated_pose, filtered_pose_msg.pose);
    filtered_pose_pub.publish( filtered_pose_msg );
    // Save current pose to last pose for next iteration
    last_cam_pose = *m;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "delta_filter_node");
    ros::NodeHandle nh;

    // Topic params 
    std::string topic_publish, topic_pose_imu, topic_pose_cam;
    std::string frame_id_imu, frame_id_cam;
    nh.param<std::string>("topic_publish", topic_publish, std::string(topic_publish_default)); 
    nh.param<std::string>("topic_pose_imu", topic_pose_imu, std::string(topic_pose_imu_default)); 
    nh.param<std::string>("topic_pose_cam", topic_pose_cam, std::string(topic_pose_cam_default)); 
    nh.param<std::string>("frame_id_imu", frame_id_imu, std::string(frame_id_imu_default));
    nh.param<std::string>("frame_id_cam", frame_id_cam, std::string(frame_id_cam_default));
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

    // Get static tf between imu and camera frame
    tf::TransformListener tf_listener;
    tf_listener.waitForTransform(frame_id_imu, frame_id_cam, ros::Time(0), ros::Duration(1.0)); // wait 5 seconds for tf
    tf_listener.waitForTransform(std::string("axes_cam"), std::string("axes_imu"), ros::Time(0), ros::Duration(1.0)); // wait 5 seconds for tf
    tf_listener.lookupTransform(frame_id_imu, frame_id_cam, ros::Time(0), tf_imu2cam);
    tf_listener.lookupTransform(std::string("axes_cam"), std::string("axes_imu"), ros::Time(0), axes_imu2cam);
    ROS_INFO("Found transform cam -> imu:\n[x,y,z]=%f, %f, %f\n[x,y,z,w]=%f, %f, %f, %f", 
        tf_imu2cam.getOrigin().x(),
        tf_imu2cam.getOrigin().y(),
        tf_imu2cam.getOrigin().z(),
        tf_imu2cam.getRotation().x(),
        tf_imu2cam.getRotation().y(),
        tf_imu2cam.getRotation().z(),
        tf_imu2cam.getRotation().w()
    );

    // Publishers and subscribers
    ros::Subscriber cam_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(topic_pose_cam, 1000, camMsgCallback);
    ros::Subscriber imu_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(topic_pose_imu, 1000, imuMsgCallback);
    filtered_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(topic_publish, 1000);
    
    // Main processing loop, wait for callbacks to happen
    while(ros::ok()) {
        // TODO: add loop rate based on publish frequency
        ros::spinOnce();
    }

    return 0;
}


