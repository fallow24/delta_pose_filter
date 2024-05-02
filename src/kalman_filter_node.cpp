#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <queue>
#include <cmath>

#include <Eigen/Dense>
#include "../../../devel/include/realsense_pipeline_fix/CameraPoseAngularVelocity.h"

/** ---------------------------------------
 * ------------Rosparam server defines -----
 * ----------------------------------------*/

const char *topic_publish_default = "/lkf2/pose";
const char *topic_pose_imu_default = "/posePub_merged";
const char *topic_pose_cam_default = "/camera/pose";
const char *topic_pose_camImu_default = "/camera/poseAndImu";

const char *frame_id_imu_default = "imu_frame";
const char *frame_id_cam_default = "camera_frame";

int imu_rate, cam_rate;   // In Hz
int fast_rate, slow_rate; // Redefinition, same as above
int n_acc;                // Accumulation window of slower pose stream

double r_sphere; // Radius of sphere in m

/** ---------------------------------------
 * ------------Internal variables----------
 * ----------------------------------------*/

// Which pose stream to use for interpolation (the faster one)
enum INTERPOLATE_STREAM
{
    CAM,
    IMU,
    NONE
} interpolate;

// Flags to initialize pose streams
bool initialized_cam, initialized_imu;

// geometry_msgs Pose objects for IMU, CAM, and filtered pose
geometry_msgs::PoseStamped last_imu_pose, last_cam_pose, filtered_pose_msg;

// tf Pose objects for calculating interpolated delta
tf::Pose pose_interpolated, last_pose_interpolated;

// Pose accumulator for the faster stream
std::queue<realsense_pipeline_fix::CameraPoseAngularVelocity> accumulator;
// Callback queue for the faster stream. For more than 2 estimators this should be a vector of CallbackQueues.
ros::CallbackQueue callbacks_fast;

// Transformation between IMU and Camera pose frame (FROM Imu TO Cam)
tf::StampedTransform tf_map_imu2cam, tf_axes_imu2cam;

ros::Publisher filtered_pose_pub;

// Store current angular velocity provided by imus
sensor_msgs::Imu imu_angular_vel_curr;
sensor_msgs::Imu cam_angular_vel_curr;

// custom message type to store camera pose and gyro data
realsense_pipeline_fix::CameraPoseAngularVelocity camera_pose_imu;

// Save last interpolated confidence level of camera
float last_translated_confidence;

// Debug stuff
ros::Publisher debug_pose_pub;
geometry_msgs::PoseStamped debug_pose_msg;
bool publish_debug_topic;

// Global defines as objects on heap
tf::Vector3 origin(0, 0, 0);
tf::Quaternion rot_zero(0, 0, 0, 1);
const double small_number = 0.00001;

uint32_t sequence = 0; // Sequence number for publish msg

const float MIN_SCALING_FACTOR = 1.0;
const float MAX_SCALING_FACTOR = 1000.0;

/** ---------------------------------------
 * ------------Supporting methods----------
 * ----------------------------------------*/

// Advances the internal queue such that the timestamp argument is between front and end
inline void waitForAccumulator(double t)
{

    // Wait for the queue to be not empty
    while (accumulator.size() < 1 && ros::ok())
    {
        callbacks_fast.callOne(ros::WallDuration()); // callbacks_fast momentan für cam --- zeitstempel von imu und posePubMerged gleich -> nicht für imu interpolieren
    }

    // Check if lagging behind
    if (t < accumulator.front().header.stamp.toSec())
        return;

    // Wait for the query time to be between the accumulator times
    while (!(accumulator.front().header.stamp.toSec() < t && t < accumulator.back().header.stamp.toSec()) && ros::ok())
    {
        // If interpolation is disabled, all msgs have the same queue
        if (interpolate == NONE)
            ros::spinOnce();
        else
            callbacks_fast.callOne(ros::WallDuration());
    }
}

// Push a message to the internal queue, removes the oldest entry if full. 
inline void pushToAccumulator(const realsense_pipeline_fix::CameraPoseAngularVelocityConstPtr &m)
{
    accumulator.push(*m);
    if (accumulator.size() > n_acc)
        accumulator.pop();
}

// Helper to calculate the diff transform between two poses
inline tf::Transform pose_diff(const geometry_msgs::PoseStamped::ConstPtr &m, geometry_msgs::PoseStamped &last_pose_msg)
{
    tf::Stamped<tf::Pose> current_pose, last_pose;
    tf::poseStampedMsgToTF(*m, current_pose);
    tf::poseStampedMsgToTF(last_pose_msg, last_pose);
    return current_pose.inverseTimes(last_pose);
}

inline double getRollFromQuaternion(const geometry_msgs::Quaternion &q)
{
    tf::Quaternion q2;
    tf::quaternionMsgToTF(q, q2);
    double r, p, y;
    tf::Matrix3x3(q2).getRPY(r, p, y);
    return r;
}

inline double getPitchFromQuaternion(const geometry_msgs::Quaternion &q)
{
    tf::Quaternion q2;
    tf::quaternionMsgToTF(q, q2);
    double r, p, y;
    tf::Matrix3x3(q2).getRPY(r, p, y);
    return p;
}

inline double getYawFromQuaternion(const geometry_msgs::Quaternion &q)
{
    tf::Quaternion q2;
    tf::quaternionMsgToTF(q, q2);
    double r, p, y;
    tf::Matrix3x3(q2).getRPY(r, p, y);
    return y;
}

inline uint32_t confidenceTranslator(const uint8_t &confidence){
    /* Confidence level (0 = Failed, 1 = Low, 2 = Medium, 3 = High confidence) will be translated to a factor, which the pose variance will be multiplied with
    low confidence -> higher variance -> less influence in KF */

    return pow(10, 3 - confidence);
}

inline float getScalingFactorExp(const tf::Vector3 &angular_vel, bool isCam){

    float scalingFactor = 1.0;
    if(isCam){
        scalingFactor = last_translated_confidence * exp(angular_vel.length());
    }
    else scalingFactor = exp(angular_vel.length());

    // Clamp to be between MIN and MAX value
    scalingFactor = std::min(std::max(scalingFactor, MIN_SCALING_FACTOR), MAX_SCALING_FACTOR);
    return scalingFactor;
};

inline float getScalingFactorLog(const tf::Vector3 &angular_vel, bool isCam){

    float scalingFactor = 1.0;
    if(isCam){
        scalingFactor = last_translated_confidence * (log(angular_vel.length() + 1) + 1);
    }
    else scalingFactor = (log(angular_vel.length() + 1) + 1);

    // Clamp to be between MIN and MAX value
    scalingFactor = std::min(std::max(scalingFactor, MIN_SCALING_FACTOR), MAX_SCALING_FACTOR);
    return scalingFactor;
};


/** ----------------------------------------------------
 * ----------------- Linear Kalman Filter --------------
 * -----------------------------------------------------*/

// LKF state
Eigen::VectorXf state = Eigen::VectorXf::Zero(9); //(x, y, z, roll, pitch, yaw, w_x, w_y, w_z)

// State propagation matrix F
Eigen::MatrixXf F(9, 9);

// control input u
Eigen::VectorXf control_input = Eigen::VectorXf::Zero(9);

// control input matrix
Eigen::MatrixXf G(9, 9);

// state covariance matrix
Eigen::MatrixXf P = 10 * Eigen::MatrixXf::Identity(9, 9); //high number as initial value -> will convergate against smaller number

// measurement prediction
Eigen::VectorXf z = Eigen::VectorXf::Zero(9);

// measurement prediction matrix
Eigen::MatrixXf H = Eigen::MatrixXf::Identity(9, 9);

// process noise covariance matrix Q / System prediction noise
Eigen::MatrixXf Q = 0.1 * Eigen::MatrixXf::Identity(9, 9);

// measurement noise covariance matrix
Eigen::MatrixXf R_imu(9, 9);
Eigen::MatrixXf R_cam(9, 9);

// Prediction Step
void predict_state(const double dT, Eigen::VectorXf u, Eigen::MatrixXf Q)
{
    // Cross product with the normal vector of the ground
    // Note: Normal vector points INTO the ground (0,0,-1)
    F <<0, 0, 0, 0, 0, 0, 0, -(dT * r_sphere), 0,
        0, 0, 0, 0, 0, 0, (dT * r_sphere), 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0;

    // Predict state (x_pri)
    state = F * state + G * u;   // 9x9 * 9x1 = 9x1

    // Calculate P_pri
    P = F * P * F.transpose() + Q;

    // Predict measurement
    z = H * state; 
}

// Update Step -> measurement comes from either imu or cam callback
void update_state(const Eigen::VectorXf &measurement, Eigen::MatrixXf R)
{
    // Innovation covariance
    Eigen::MatrixXf S = H * P * H.transpose() + R;

    // Kalman gain
    Eigen::MatrixXf K = P * H.transpose() * S.inverse();

    // Calculate innovation
    Eigen::VectorXf innovation = measurement - z;
    
    // Update state estimation
    state = state + (K * innovation);
    
    // Calculate P_pos
    P = (Eigen::MatrixXf::Identity(9, 9) - K * H) * P;
}

// The magic function. Includes the interpolation of the faster stream.
void apply_lkf_and_publish(const geometry_msgs::PoseStamped::ConstPtr &m)
{
    /*
    * Interpolation of the faster stream, calculation in global frame 
    * will result in a Delta which applies in local frame.
    */

    // calculate dT
    double dT = (m->header.stamp - last_imu_pose.header.stamp).toSec();

    // Get timestamp from current and wait for accumulator to go past that
    ros::Time stamp_current = m->header.stamp;
    tfScalar t_current = stamp_current.toSec();

    // wait so that imu time stamp is inbetween cam time stamps
    waitForAccumulator(t_current); 

    /*
    * Interpolation of camera pose
    */ 

    // Convert geometry_msgs to Quaternion format
    tf::Quaternion q1, q2, q_res;
    tf::quaternionMsgToTF(accumulator.front().pose.pose.orientation, q1);
    tf::quaternionMsgToTF(accumulator.back().pose.pose.orientation, q2);

    // Convert geometry_msgs Points to tf Vectors
    tf::Vector3 v1, v2, v_res;
    tf::pointMsgToTF(accumulator.front().pose.pose.position, v1);
    tf::pointMsgToTF(accumulator.back().pose.pose.position, v2);

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

    // Rotate interpolated pose via basis change, into the frame of the faster stream
    pose_interpolated.mult(pose_interpolated, tf_axes_imu2cam);
    pose_interpolated.mult(tf_map_imu2cam, pose_interpolated);

    // Interpolation camera tracker confidence
    uint8_t pose_confidence_interpolated = (t * accumulator.front().tracker_confidence) + ((1 - t) * accumulator.back().tracker_confidence);
    last_translated_confidence = (float) confidenceTranslator(pose_confidence_interpolated);

    // interpolation camera angular velocities, which are given in LOCAL frame
    tf::Vector3 w1, w2, w_res;
    w1.setX(accumulator.front().imu.angular_velocity.x);
    w1.setY(-accumulator.front().imu.angular_velocity.y);
    w1.setZ(-accumulator.front().imu.angular_velocity.z);
    w2.setX(accumulator.back().imu.angular_velocity.x);
    w2.setY(-accumulator.back().imu.angular_velocity.y);
    w2.setZ(-accumulator.back().imu.angular_velocity.z);
    w_res = tf::lerp(w1, w2, t);
    // Rotate interpolated angular velocity via basis change into the frame of the faster stream
    tf::Pose angularVelPose;  // use tf::Pose to save angular velocity in x,y,z elements for multiplication operation
    angularVelPose.setOrigin(w_res);
    angularVelPose.setRotation(rot_zero);
    angularVelPose.mult(angularVelPose, tf_axes_imu2cam);
    angularVelPose.mult(tf_map_imu2cam, angularVelPose);

    // convert temporary pose back to sensor_msgs::Imu
    sensor_msgs::Imu cam_angular_vel_interpolated;
    cam_angular_vel_interpolated.angular_velocity.x = angularVelPose.getOrigin().getX();
    cam_angular_vel_interpolated.angular_velocity.y = angularVelPose.getOrigin().getY();
    cam_angular_vel_interpolated.angular_velocity.z = angularVelPose.getOrigin().getZ();

    /*
    * Rotate the local deltas into the global frame
    */

    // Angular velocities transformation from local to global
    tf::Vector3 tf_angular_velocity_imu;
    tf::vector3MsgToTF(imu_angular_vel_curr.angular_velocity, tf_angular_velocity_imu);
    tf::Vector3 tf_angular_velocity_cam;
    tf::vector3MsgToTF(cam_angular_vel_interpolated.angular_velocity, tf_angular_velocity_cam);

    // Use the current estimated rotation to make the angular velocities global
    tf::Stamped<tf::Pose> filteredPose;
    tf::poseStampedMsgToTF(filtered_pose_msg, filteredPose);     
    tf::Quaternion current_rotation = filteredPose.getRotation();
    tf::Transform current_tf(current_rotation);
    tf::Vector3 tf_angular_velocity_imu_rotated = current_tf * tf_angular_velocity_imu;
    tf::Vector3 tf_angular_velocity_cam_rotated = current_tf * tf_angular_velocity_cam;

    // Calculate delta of imu pose
    tf::Pose imu_diff = pose_diff(m, last_imu_pose);
    geometry_msgs::Pose imu_diff_geom_msgs;
    tf::poseTFToMsg(imu_diff, imu_diff_geom_msgs);
    tf::Pose imu_diff_rotated = current_tf * imu_diff;

    // In first iteration, last interpolation does not exist. Use the first msg instead
    if (fabs(last_pose_interpolated.getRotation().length() - 1.0) > small_number) {
        ROS_WARN("Uninitialized quaternion, length: %f", last_pose_interpolated.getRotation().length());
        tf::Stamped<tf::Pose> last_pose_cam_stamped;
        tf::poseStampedMsgToTF(last_cam_pose, last_pose_cam_stamped);
        last_pose_interpolated = last_pose_cam_stamped;
        last_pose_interpolated.mult(last_pose_interpolated, tf_axes_imu2cam);
        last_pose_interpolated.mult(tf_map_imu2cam, last_pose_interpolated);
        ROS_WARN("Attempted fix, length: %f", last_pose_interpolated.getRotation().length());
    }
    
    // Calculate delta of cam pose
    tf::Pose cam_diff_interpolated = pose_interpolated.inverseTimes(last_pose_interpolated);
    geometry_msgs::Pose cam_diff_geom_msgs;
    tf::poseTFToMsg(cam_diff_interpolated, cam_diff_geom_msgs);
    tf::Pose cam_diff_rotated = current_tf * cam_diff_interpolated;
    
    // Put everything into the LKF process control vector
    Eigen::Vector3f eigen_angular_velocity_imu_rotated;
    eigen_angular_velocity_imu_rotated[0] = tf_angular_velocity_imu_rotated.getX();
    eigen_angular_velocity_imu_rotated[1] = tf_angular_velocity_imu_rotated.getY();
    eigen_angular_velocity_imu_rotated[2] = tf_angular_velocity_imu_rotated.getZ();

    Eigen::Vector3f eigen_angular_velocity_cam_rotated;
    eigen_angular_velocity_cam_rotated[0] = tf_angular_velocity_cam_rotated.getX();
    eigen_angular_velocity_cam_rotated[1] = tf_angular_velocity_cam_rotated.getY();
    eigen_angular_velocity_cam_rotated[2] = tf_angular_velocity_cam_rotated.getZ();
   

    // Make vector 9dof for predict step
    // Cam pose as control input
    Eigen::VectorXf eigen_angular_velocity_rotated_9dof = Eigen::VectorXf::Zero(9);
    eigen_angular_velocity_rotated_9dof[3] = getRollFromQuaternion(cam_diff_geom_msgs.orientation);
    eigen_angular_velocity_rotated_9dof[4] = getPitchFromQuaternion(cam_diff_geom_msgs.orientation);
    eigen_angular_velocity_rotated_9dof[5] = getYawFromQuaternion(cam_diff_geom_msgs.orientation);
    eigen_angular_velocity_rotated_9dof[6] = eigen_angular_velocity_cam_rotated[0];
    eigen_angular_velocity_rotated_9dof[7] = eigen_angular_velocity_cam_rotated[1];
    eigen_angular_velocity_rotated_9dof[8] = eigen_angular_velocity_cam_rotated[2];
    
    // Imu pose as control input
    // eigen_angular_velocity_rotated_9dof[3] = getRollFromQuaternion(imu_diff_geom_msgs.orientation);
    // eigen_angular_velocity_rotated_9dof[4] = getPitchFromQuaternion(imu_diff_geom_msgs.orientation);
    // eigen_angular_velocity_rotated_9dof[5] = getYawFromQuaternion(imu_diff_geom_msgs.orientation);
    // eigen_angular_velocity_rotated_9dof[6] = eigen_angular_velocity_imu_rotated[0];
    // eigen_angular_velocity_rotated_9dof[7] = eigen_angular_velocity_imu_rotated[1];
    // eigen_angular_velocity_rotated_9dof[8] = eigen_angular_velocity_imu_rotated[2];

    /* 
     * Calculate Scaling factor for measurement noise covariance matrices
     + Take camera confidence and velocity into account
     + Low confidence -> higher variance -> more uncertainty
     + High angular velocity -> higher variance -> more uncertainty
     */
    
    // exp and confidence
    float scalingFactorCamera = getScalingFactorExp(tf_angular_velocity_cam_rotated, true);
    float scalingFactorImu = getScalingFactorExp(tf_angular_velocity_imu_rotated, false);

    // log and cofidence
    // float scalingFactorCamera = getScalingFactorLog(tf_angular_velocity_cam_rotated, true);
    // float scalingFactorImu = getScalingFactorLog(tf_angular_velocity_imu_rotated, false);

    Eigen::MatrixXf R_cam_scaled = R_cam * scalingFactorCamera;
    Eigen::MatrixXf R_imu_scaled = R_imu * scalingFactorImu;

    /*
     * Prediction step
     */

    predict_state(dT, eigen_angular_velocity_rotated_9dof, Q);

    /*
     * Update step
     */

    // update using IMU measurements
    Eigen::VectorXf measurementImu(9);
    measurementImu << imu_diff_rotated.getOrigin().getX(), imu_diff_rotated.getOrigin().getY(), imu_diff_rotated.getOrigin().getZ(),
        getRollFromQuaternion(imu_diff_geom_msgs.orientation), getPitchFromQuaternion(imu_diff_geom_msgs.orientation), getYawFromQuaternion(imu_diff_geom_msgs.orientation),
        tf_angular_velocity_imu_rotated.getX(), tf_angular_velocity_imu_rotated.getY(), tf_angular_velocity_imu_rotated.getZ();

    update_state(measurementImu, R_imu_scaled);

    // update using CAM measurements
    Eigen::VectorXf measurementCam(9);
    measurementCam << cam_diff_rotated.getOrigin().getX(), cam_diff_rotated.getOrigin().getY(), cam_diff_rotated.getOrigin().getZ(),
        getRollFromQuaternion(cam_diff_geom_msgs.orientation), getPitchFromQuaternion(cam_diff_geom_msgs.orientation), getYawFromQuaternion(cam_diff_geom_msgs.orientation),
        tf_angular_velocity_cam_rotated.getX(), tf_angular_velocity_cam_rotated.getY(), tf_angular_velocity_cam_rotated.getZ();

    update_state(measurementCam, R_cam_scaled);

    /*
     * Apply the LKF estimation to the filtered pose  
     */

    // The state contains the Deltas. Convert LKF state to delta tf
    tf::Pose filteredDeltaPose;
    filteredDeltaPose.setOrigin(current_tf.inverse() * tf::Vector3(state[0], state[1], state[2]));
    filteredDeltaPose.setRotation(tf::createQuaternionFromRPY(state[3], state[4], state[5]));

    // Apply delta 
    filteredPose.mult(filteredPose, filteredDeltaPose.inverse()); // update der filtered pose
    tf::poseStampedTFToMsg(filteredPose, filtered_pose_msg);      // update filtered_Pose_msg

    /*
     * Publish to topic
     */

    // Construct msg
    filtered_pose_msg.header.frame_id = "odom";
    filtered_pose_msg.header.stamp = stamp_current;
    filtered_pose_msg.header.seq = sequence++;

    // publish filtered_pose_msg
    filtered_pose_pub.publish(filtered_pose_msg);

    // Prep next iteration
    last_pose_interpolated = pose_interpolated;
}

void imuMsgCallback(const geometry_msgs::PoseStamped::ConstPtr &m)
{
    // Initialization on first run
    if (!initialized_imu)
    {
        ROS_INFO("Imu init");
        filtered_pose_msg = *m;
        last_imu_pose = *m;
        initialized_imu = true;
        return;
    }

    // If cam interpolation is active, wait for cam timestamps to go past current time and do interpolation
    if (interpolate == CAM)
    {

        // Sanity check, can only interpolate if buffer is fully accumulated
        if (accumulator.size() == n_acc)
        {

            // Use kalman filter and publish the pose msg
            apply_lkf_and_publish(m);
        }

        // Otherwise, if IMU interpolation is active, push current pose to queue
    }
    // TODO: Redefine pushToAccumulator to be templated. Not important right now
    /*else if (interpolate == IMU)
    {
        pushToAccumulator(m);
    }*/

    // Save current pose to last pose for next iteration
    last_imu_pose = *m;
}

void camMsgCallback(const realsense_pipeline_fix::CameraPoseAngularVelocityConstPtr &m)
{
    // Initialization on first run
    if (!initialized_cam)
    {
        ROS_INFO("INITIALIZED CAM");
        last_cam_pose = m->pose;
        cam_angular_vel_curr = m->imu;
        pushToAccumulator(m);
        initialized_cam = true;
        return;
    }

    // If cam interpolation is active, push cam poses to queue
    if (interpolate == CAM)
    {
        pushToAccumulator(m);
    }
    else if (interpolate == IMU)
    {

        // Sanity check, can only interpolate if buffer is fully accumulated
        if (accumulator.size() == n_acc)
        {

            // Use kalman filter and publish the pose msg
            const geometry_msgs::PoseStampedConstPtr temp_pose = boost::make_shared<const geometry_msgs::PoseStamped>(m->pose);
            apply_lkf_and_publish(temp_pose);
        }
    }

    // Debug msgs
    if (publish_debug_topic)
    {
        // Testing, apply correct rotation
        tf::Stamped<tf::Pose> this_pose;
        tf::poseStampedMsgToTF(m->pose, this_pose);
        tf::Pose rotated_pose;

        rotated_pose.mult(this_pose, tf_axes_imu2cam);
        rotated_pose.mult(tf_map_imu2cam, rotated_pose);

        debug_pose_msg.header = m->header;
        debug_pose_msg.header.frame_id = "odom";
        tf::poseTFToMsg(rotated_pose, debug_pose_msg.pose);
        debug_pose_pub.publish(debug_pose_msg);
    }

    // Save current pose to last pose for next iteration
    last_cam_pose = m->pose;
}

void orientationImuCallback(const sensor_msgs::Imu::ConstPtr &m)
{
    imu_angular_vel_curr = *m;
}

int main(int argc, char **argv)
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
    if (imu_rate < cam_rate)
    {
        interpolate = CAM;
        n_acc = std::ceil(imu_t / cam_t) + 1;
        ROS_INFO("Interpolating Camera Pose measurements using %d samples", n_acc);
    }
    else if (cam_rate < imu_rate)
    {
        interpolate = IMU;
        n_acc = std::ceil(cam_t / imu_t) + 1;
        ROS_INFO("Interpolating Imu Pose measurements using %d samples", n_acc);
    }
    else
    {
        interpolate = NONE;
        n_acc = 1;
        ROS_INFO("Interpolation disabled.");
    }

    // Initialize variables
    initialized_cam = false;
    initialized_imu = false;

    // Get static tf between imu and camera frame
    tf::TransformListener tf_listener;
    tf_listener.waitForTransform(std::string("axes_cam"), std::string("axes_imu"), ros::Time(0), ros::Duration(5.0)); // wait 2 seconds for tf
    tf_listener.waitForTransform(frame_id_imu, frame_id_cam, ros::Time(0), ros::Duration(5.0));                       // wait 2 seconds for tf
    tf_listener.lookupTransform(frame_id_imu, frame_id_cam, ros::Time(0), tf_map_imu2cam);
    tf_listener.lookupTransform(std::string("axes_cam"), std::string("axes_imu"), ros::Time(0), tf_axes_imu2cam);
    ROS_INFO("Found global transform imu -> cam:\n[x,y,z]=%f, %f, %f\n[x,y,z,w]=%f, %f, %f, %f",
             tf_map_imu2cam.getOrigin().x(),
             tf_map_imu2cam.getOrigin().y(),
             tf_map_imu2cam.getOrigin().z(),
             tf_map_imu2cam.getRotation().x(),
             tf_map_imu2cam.getRotation().y(),
             tf_map_imu2cam.getRotation().z(),
             tf_map_imu2cam.getRotation().w());
    ROS_INFO("Found axes definition transform imu -> cam:\n[x,y,z]=%f, %f, %f\n[x,y,z,w]=%f, %f, %f, %f",
             tf_axes_imu2cam.getOrigin().x(),
             tf_axes_imu2cam.getOrigin().y(),
             tf_axes_imu2cam.getOrigin().z(),
             tf_axes_imu2cam.getRotation().x(),
             tf_axes_imu2cam.getRotation().y(),
             tf_axes_imu2cam.getRotation().z(),
             tf_axes_imu2cam.getRotation().w());

    // Publishers and subscribers
    ros::Subscriber cam_pose_sub, imu_pose_sub, imu_vel_sub, cam_imu_vel_sub;
    if (interpolate == IMU)
    {
        cam_pose_sub = nh.subscribe<realsense_pipeline_fix::CameraPoseAngularVelocity>(topic_pose_camImu_default, 1000, camMsgCallback);
        imu_pose_sub = nh_fast.subscribe<geometry_msgs::PoseStamped>(topic_pose_imu, 1000, imuMsgCallback);
    }
    else if (interpolate == CAM)
    {
        cam_pose_sub = nh_fast.subscribe<realsense_pipeline_fix::CameraPoseAngularVelocity>(topic_pose_camImu_default, 1000, camMsgCallback);
        imu_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(topic_pose_imu, 1000, imuMsgCallback);
    }
    else if (interpolate == NONE)
    {
        cam_pose_sub = nh.subscribe<realsense_pipeline_fix::CameraPoseAngularVelocity>(topic_pose_camImu_default, 1000, camMsgCallback);
        imu_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(topic_pose_imu, 1000, imuMsgCallback);
    }
    imu_vel_sub = nh.subscribe<sensor_msgs::Imu>("orientation", 1000, orientationImuCallback);

    filtered_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(topic_publish, 1000);
    if (publish_debug_topic)
        debug_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/lkf2/debug", 1000);

    // Main processing loop, wait for callbacks to happen
    fast_rate = std::max(cam_rate, imu_rate);
    slow_rate = std::min(cam_rate, imu_rate);
    ros::Rate r(fast_rate);

    /*
    *  Initialise matrices & vectors
    */

    // Control input matrix
    G << 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1;

    // IMU:
    Eigen::VectorXf imu_variances = Eigen::VectorXf::Zero(9); //[x,y,z,r,p,y, angular_vel_x, angular_vel_y, angular_vel_z]^T
    
    // manually set
    imu_variances[0] = 0.1;     
    imu_variances[1] = 0.1;     
    imu_variances[2] = 0.001;   
    imu_variances[3] = 0.1;     
    imu_variances[4] = 0.1;     
    imu_variances[5] = 0.1;     
    imu_variances[6] = 0.1;     
    imu_variances[7] = 0.1;     
    imu_variances[8] = 1.0;     
    
    // calculated based on still standing recording
    // imu_variances[0] = 0.000000002876546546487205;  // 2.876546546487205e-09
    // imu_variances[1] = 0.00000001892371314707381;   // 1.892371314707381e-08 
    // imu_variances[2] = 0.001;                       // *not measured*
    // imu_variances[3] = 0.002958013472995309;        // 2.958013472995309e-03
    // imu_variances[4] = 0.00001947246991407008;      // 1.947246991407008e-05
    // imu_variances[5] = 0.0004273785397636673;       // 4.273785397636673e-04
    // imu_variances[6] = 0.00000613159511995296;      // 6.13159511995296e-06
    // imu_variances[7] = 0.00000303115278740476;      // 3.03115278740476e-06
    // imu_variances[8] = 0.000008382353540565755;     // 8.382353540565755e-06

    R_imu << imu_variances[0], 0, 0, 0, 0, 0, 0, 0, 0,              // var(x)
             0, imu_variances[1], 0, 0, 0, 0, 0, 0, 0,              // var(y)
             0, 0, imu_variances[2], 0, 0, 0, 0, 0, 0,              // var(z)
             0, 0, 0, imu_variances[3], 0, 0, 0, 0, 0,              // var(roll)
             0, 0, 0, 0, imu_variances[4], 0, 0, 0, 0,              // var(pitch)
             0, 0, 0, 0, 0, imu_variances[5], 0, 0, 0,              // var(yaw)
             0, 0, 0, 0, 0, 0, imu_variances[6], 0, 0,              // var(angular_vel_x)
             0, 0, 0, 0, 0, 0, 0, imu_variances[7], 0,              // var(angular_vel_y)
             0, 0, 0, 0, 0, 0, 0, 0, imu_variances[8];              // var(angular_vel_z)

    // CAM:
    Eigen::VectorXf cam_variances = Eigen::VectorXf::Zero(9); //[x,y,z,r,p,y, angular_vel_x, angular_vel_y, angular_vel_z]^T

    // manually set
    cam_variances[0] = 0.1;  
    cam_variances[1] = 0.1;
    cam_variances[2] = 1.0;
    cam_variances[3] = 0.1; 
    cam_variances[4] = 0.1; 
    cam_variances[5] = 1.0; 
    cam_variances[6] = 0.1;
    cam_variances[7] = 0.1;
    cam_variances[8] = 0.1;

    // calculated based on still standing recording
    // cam_variances[0] = 0.000000001229815901018121;      // 1.229815901018121e-09
    // cam_variances[1] = 0.0000000002974322429504026;     // 2.974322429504026e-10
    // cam_variances[2] = 0.000008382353540565755;         // 8.382353540565755e-06
    // cam_variances[3] = 0.00003712583228935403;          // 3.712583228935403e-05
    // cam_variances[4] = 0.00004393695468057207;          // 4.393695468057207e-05
    // cam_variances[5] = 0.00003989540596116565;          // 3.989540596116565e-05
    // cam_variances[6] = 0.000004191121769533953;         // 4.191121769533953e-06
    // cam_variances[7] = 0.000004838667659019292;         // 4.838667659019292e-06
    // cam_variances[8] = 0.000007326272172174005;         // 7.326272172174005e-06

    R_cam << cam_variances[0], 0, 0, 0, 0, 0, 0, 0, 0,
             0, cam_variances[1], 0, 0, 0, 0, 0, 0, 0,
             0, 0, cam_variances[2], 0, 0, 0, 0, 0, 0,
             0, 0, 0, cam_variances[3], 0, 0, 0, 0, 0,
             0, 0, 0, 0, cam_variances[4], 0, 0, 0, 0,
             0, 0, 0, 0, 0, cam_variances[5], 0, 0, 0,
             0, 0, 0, 0, 0, 0, cam_variances[6], 0, 0,
             0, 0, 0, 0, 0, 0, 0, cam_variances[7], 0,
             0, 0, 0, 0, 0, 0, 0, 0, cam_variances[8];

    while (ros::ok())
    {
        ros::spinOnce();
        callbacks_fast.callOne();
        r.sleep();
    }

    return 0;
}