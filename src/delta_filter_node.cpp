#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <geometry_msgs/PoseStamped.h>
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
int fast_rate, slow_rate; // Redefinition, same as above
int n_acc; // Accumulation window of slower pose stream 

double r_sphere; // Radius of sphere in m

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
// Callback queue for the faster stream. For more than 2 estimators this should be a vector of CallbackQueues.
ros::CallbackQueue callbacks_fast;

// Transformation between IMU and Camera pose frame (FROM Imu TO Cam)
tf::StampedTransform tf_map_imu2cam, tf_axes_imu2cam;
ros::Publisher filtered_pose_pub; 

// Debug stuff
ros::Publisher debug_pose_pub;
geometry_msgs::PoseStamped debug_pose_msg;
bool publish_debug_topic;

// Global defines as objects on heap 
tf::Vector3 origin(0,0,0);
tf::Quaternion rot_zero(0, 0, 0, 1);
const double small_number = 0.00001;

uint32_t sequence = 0; // Sequence number for publish msg

inline void waitForAccumulator(double t) 
{   
    static ros::Rate rate(fast_rate); 
    // Wait for the queue to be not empty
    while (accumulator.size() < 1 && ros::ok()) {
        callbacks_fast.callOne( ros::WallDuration() );
	rate.sleep();
    }
	
    // Wait for the query time to be between the accumulator times
    while( !(accumulator.front().header.stamp.toSec() < t 
      && t < accumulator.back().header.stamp.toSec()) 
      && ros::ok() ) {
        // If interpolation is disabled, all msgs have the same queue
        if (interpolate == NONE) ros::spinOnce();
        else callbacks_fast.callOne( ros::WallDuration() );
        rate.sleep();
    }   

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
    tf::poseStampedMsgToTF(last_pose_msg, last_pose);
    return current_pose.inverseTimes(last_pose);
} 

inline double max(double a, double b, double c)
{
    return std::max(std::max(a,b), c);
}

inline double min(double a, double b, double c)
{
    return std::min(std::min(a,b), c);
}

inline double mid(double a, double b, double c)
{
    return std::min( std::max(a,b), std::max(b,c) );
}

void apply_delta_filter_and_publish(const geometry_msgs::PoseStamped::ConstPtr &m) 
{
    // Get timestamp from current and wait for accumulator to go past that  
    ros::Time stamp_current = m->header.stamp;
    tfScalar t_current = stamp_current.toSec();

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

    // rotate interpolated pose via basis change
    pose_interpolated.mult(pose_interpolated, tf_axes_imu2cam);
    pose_interpolated.mult(tf_map_imu2cam, pose_interpolated);

    // Calculate measured and interpolated deltas
    tf::Pose diff_measured;
    if (interpolate == CAM)
        diff_measured = pose_diff(m, last_imu_pose);
    else 
        diff_measured = pose_diff(m, last_cam_pose);
    
    tf::Pose diff_interpolated;
    diff_interpolated = pose_interpolated.inverseTimes(last_pose_interpolated);

    // Get rotational distance (angle) and translational distance of deltas
    double dist_r_int = diff_interpolated.getRotation().angle( rot_zero ); //*2
    double dist_r_mea = diff_measured.getRotation().angle( rot_zero ); //*2
    double dist_t_int = diff_interpolated.getOrigin().distance(origin); // these will return positive!
    double dist_t_mea = diff_measured.getOrigin().distance(origin); // these will return positive!

    // Plausibility check, which transform to use
    tf::Pose tf_delta;
    double dist_t_model = std::max(dist_r_mea , dist_r_int) * r_sphere;
    if (fabs(dist_t_mea - dist_t_model) < fabs(dist_t_int - dist_t_model) ) {
        tf_delta = diff_measured; // Use measurement if its closer to model
    } else { // Otherwise, use mean of both transforms:
        tf::Quaternion delta_rot = tf::slerp(diff_measured.getRotation(), diff_interpolated.getRotation(), 0.5);
        tf::Vector3 delta_trans = diff_measured.getOrigin(); // tf::lerp(diff_measured.getOrigin(), diff_interpolated.getOrigin(), 0.5);
        tf_delta = tf::Pose(delta_rot, delta_trans);
    }

    // ESTIMATE "REAL" ARC LENGTH USING GEOMETRIC MEAN WITH SIMILARITY SCORES
    double alpha = mid(dist_t_int, dist_t_mea, dist_t_model) - min(dist_t_int, dist_t_mea, dist_t_model);
    double beta = max(dist_t_int, dist_t_mea, dist_t_model) - mid(dist_t_int, dist_t_mea, dist_t_model);
    // TODO: FIXME division trough 0! (fix: alpha, beta are both >= 0 --> small number will prevent division through 0) 
    double w_int = fabs( (dist_t_int - alpha - beta - small_number)/(alpha + beta + small_number) );
    double w_mea = fabs( (dist_t_mea - alpha - beta - small_number)/(alpha + beta + small_number) );
    double w_model = fabs( (dist_t_model - alpha - beta - small_number)/(alpha + beta + small_number) );
    double arc_length = pow( pow(dist_t_int, w_int)*pow(dist_t_mea, w_mea)*pow(dist_t_model, w_model), 1.0/(w_int+w_mea+w_model));
    
    // For translation, use downscaling according to estimated arc length of rotation 
    double scale_factor = std::min(1.0, arc_length / tf_delta.getOrigin().distance(origin));
    tf::Vector3 scaled_translation = scale_factor * tf_delta.getOrigin();
    tf_delta.setOrigin(scaled_translation);

    // Use calculated delta to update last pose
    tf::Stamped<tf::Pose> current_pose;
    tf::poseStampedMsgToTF(filtered_pose_msg, current_pose);
    current_pose.mult(current_pose, tf_delta.inverse());

    // Construct msg
    tf::poseStampedTFToMsg(current_pose, filtered_pose_msg);
    filtered_pose_msg.header.frame_id = "map";
    filtered_pose_msg.header.stamp = stamp_current;
    filtered_pose_msg.header.seq = sequence++;

    // For rotation, interpolate between cam and imu 
    tf::Quaternion q_measured, q_interpolated;
    tf::quaternionMsgToTF(m->pose.orientation, q_measured);
    q_interpolated = pose_interpolated.getRotation();
    tf::Quaternion rotation = tf::slerp(q_measured, q_interpolated, 0.5);
    
    // Fix yaw angle
    tf::Matrix3x3 m_measured(rotation), m_interpolated(q_interpolated);
    double rm, pm, ym, ri, pi, yi;
    m_measured.getRPY(rm, pm, ym);
    m_interpolated.getRPY(ri, pi, yi);
    rotation.setRPY(rm, pm, yi);
    tf::quaternionTFToMsg(rotation, filtered_pose_msg.pose.orientation);
    // Publish
    filtered_pose_pub.publish( filtered_pose_msg );
    
    // Prep next iteration
    last_pose_interpolated = pose_interpolated; 
}

void imuMsgCallback(const geometry_msgs::PoseStamped::ConstPtr &m)
{
    // Initialization on first run
    if (!initialized_imu) {
        ROS_INFO("Imu init");
        filtered_pose_msg = *m;
        last_imu_pose = *m;
        initialized_imu = true;
        return;
    }
    
    // If cam interpolation is active, wait for cam timestamps to go past current time and do interpolation
    if (interpolate == CAM) {

        // Sanity check, can only interpolate if buffer is fully accumulated
        if (accumulator.size() == n_acc) {

            // Use delta filter and publish the pose msg
            apply_delta_filter_and_publish( m );
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
        ROS_INFO("INITIALIZED CAM");
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

            // Use delta filter and publish the pose msg
            apply_delta_filter_and_publish( m );
        }
    }
    
    // Debug msgs
    if (publish_debug_topic) 
    {
         // Testing, apply correct rotation
        tf::Stamped<tf::Pose> this_pose;
        tf::poseStampedMsgToTF(*m, this_pose);
        tf::Pose rotated_pose;

        rotated_pose.mult(this_pose, tf_axes_imu2cam);
        rotated_pose.mult(tf_map_imu2cam, rotated_pose);
        
        debug_pose_msg.header = m->header;
        debug_pose_msg.header.frame_id = "map";
        tf::poseTFToMsg(rotated_pose, debug_pose_msg.pose);
        debug_pose_pub.publish( debug_pose_msg );
    }

    // Save current pose to last pose for next iteration
    last_cam_pose = *m;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "delta_filter_node");
    ros::NodeHandle nh;

    // Nodehandle for the faster stream (should be vector for more than 2 estimators)
    ros::NodeHandle nh_fast;
    nh_fast.setCallbackQueue(&callbacks_fast);

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
    nh.param<double>("sphere_radius", r_sphere, 0.145);
    nh.param<bool>("debug_topics", publish_debug_topic, false);

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
    
    // Initialize variables
    initialized_cam = false;
    initialized_imu = false;

    // Get static tf between imu and camera frame
    tf::TransformListener tf_listener;
    tf_listener.waitForTransform(frame_id_imu, frame_id_cam, ros::Time(0), ros::Duration(1.0)); // wait 5 seconds for tf
    tf_listener.waitForTransform(std::string("axes_cam"), std::string("axes_imu"), ros::Time(0), ros::Duration(1.0)); // wait 5 seconds for tf
    tf_listener.lookupTransform(frame_id_imu, frame_id_cam, ros::Time(0), tf_map_imu2cam);
    tf_listener.lookupTransform(std::string("axes_cam"), std::string("axes_imu"), ros::Time(0), tf_axes_imu2cam);
    ROS_INFO("Found global transform imu -> cam:\n[x,y,z]=%f, %f, %f\n[x,y,z,w]=%f, %f, %f, %f", 
        tf_map_imu2cam.getOrigin().x(),
        tf_map_imu2cam.getOrigin().y(),
        tf_map_imu2cam.getOrigin().z(),
        tf_map_imu2cam.getRotation().x(),
        tf_map_imu2cam.getRotation().y(),
        tf_map_imu2cam.getRotation().z(),
        tf_map_imu2cam.getRotation().w()
    );
    ROS_INFO("Found axes definition transform imu -> cam:\n[x,y,z]=%f, %f, %f\n[x,y,z,w]=%f, %f, %f, %f", 
        tf_axes_imu2cam.getOrigin().x(),
        tf_axes_imu2cam.getOrigin().y(),
        tf_axes_imu2cam.getOrigin().z(),
        tf_axes_imu2cam.getRotation().x(),
        tf_axes_imu2cam.getRotation().y(),
        tf_axes_imu2cam.getRotation().z(),
        tf_axes_imu2cam.getRotation().w()
    );

    // Publishers and subscribers
    ros::Subscriber cam_pose_sub, imu_pose_sub;
    if (interpolate == IMU) {
        cam_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(topic_pose_cam, 1000, camMsgCallback);
        imu_pose_sub = nh_fast.subscribe<geometry_msgs::PoseStamped>(topic_pose_imu, 1000, imuMsgCallback);
    } else if (interpolate == CAM) {
        cam_pose_sub = nh_fast.subscribe<geometry_msgs::PoseStamped>(topic_pose_cam, 1000, camMsgCallback);
        imu_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(topic_pose_imu, 1000, imuMsgCallback);
    } else if (interpolate == NONE) {
        cam_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(topic_pose_cam, 1000, camMsgCallback);
        imu_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(topic_pose_imu, 1000, imuMsgCallback);
    }
    
    
    filtered_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(topic_publish, 1000);
    if (publish_debug_topic) debug_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/delta/debug", 1000);

    // Main processing loop, wait for callbacks to happen
    fast_rate = std::max(cam_rate, imu_rate);
    slow_rate = std::min(cam_rate, imu_rate);
    ros::Rate r(fast_rate);
    while(ros::ok()) {
        ros::spinOnce();
    	callbacks_fast.callOne();
	r.sleep();
    }

    return 0;
}


