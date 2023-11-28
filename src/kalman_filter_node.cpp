#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <queue>

#include <ros/ros.h>

#include <Eigen/Dense>
#include <geometry_msgs/Twist.h> //for angular velocity

/*
TODO:   (!)use systeminput either as u(t) and with G matrix or do everything in 9DoF
        (!)fix gimbal lock due to r,p,y -> either quaternions or use angle differences
        (?)covariances into matrices

        wird x achse und y achse von cam und imu angepasst, momentan vertauscht


        -entweder delta rpy oder quaternionen und bei R matrix erst mal kleine werte annehmen


        -angular vel muss in richtigen frame gerückt werden, bevor linear vel aufgestellt, entweder letzten wert oder nächste prediction nutzen um von imu frame in map frame
        dafür last_filtered_pose nutzen

        -in R matrix z achse von imu hohen wert geben? selbe bei 7,8,9 von cam
        -cam werte doch lieber als system input mit G und u, sodass R nicht addiert werden muss? macht ja varianzen kaputt oder?
        -lieber quaternionen statt rpy oder? varianzne aus csv?
*/

// rohdaten in orientation topic -> angular vel und accelerometer werte -> als systeminput angular vel

// Rosparam parameters
const char *topic_publish_default = "/delta/pose";
const char *topic_pose_imu_default = "/posePub_merged";
const char *topic_pose_cam_default = "/camera/pose";

const char *frame_id_imu_default = "map"; //"imu"; // SHOULD BE IMU!!!
const char *frame_id_cam_default = "camera_frame";

int imu_rate, cam_rate;   // In Hz
int fast_rate, slow_rate; // Redefinition, same as above
int n_acc;                // Accumulation window of slower pose stream

double r_sphere; // Radius of sphere in m

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
std::queue<geometry_msgs::PoseStamped> accumulator;
// Callback queue for the faster stream. For more than 2 estimators this should be a vector of CallbackQueues.
ros::CallbackQueue callbacks_fast;

// Transformation between IMU and Camera pose frame (FROM Imu TO Cam)
tf::StampedTransform tf_map_imu2cam, tf_axes_imu2cam;
ros::Publisher filtered_pose_pub;

// Store current angular velocity provided by imus
sensor_msgs::Imu angular_vel_curr;

// norm vector for rotation of angular velocity
Eigen::Vector3f eigen_norm_vector;
tf::Vector3 tf_norm_vector;

// Store sum of filtered delta values for rpy
Eigen::VectorXf filtered_rpy_sum = Eigen::VectorXf::Zero(3);

// Debug stuff
ros::Publisher debug_pose_pub;
geometry_msgs::PoseStamped debug_pose_msg;
bool publish_debug_topic;

// Global defines as objects on heap
tf::Vector3 origin(0, 0, 0);
tf::Quaternion rot_zero(0, 0, 0, 1);
const double small_number = 0.00001;

uint32_t sequence = 0; // Sequence number for publish msg

inline void waitForAccumulator(double t)
{

    // Wait for the queue to be not empty
    while (accumulator.size() < 1 && ros::ok())
    {
        callbacks_fast.callOne(ros::WallDuration()); // callbacks_fast momentan für cam --- zeitstempel von imu und posePubMerged gleich -> nicht für imu interpolieren
        
    }

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
    return std::max(std::max(a, b), c);
}

inline double min(double a, double b, double c)
{
    return std::min(std::min(a, b), c);
}

inline double mid(double a, double b, double c)
{
    return std::min(std::max(a, b), std::max(b, c));
}

//------------supporting methods----------
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

//--------------LKF----------------

// noch ohne Systemeingang, also ohne G matrix und u(k)
// H, F, R, Q, nicht verändert pro step

// LKF state
Eigen::VectorXf state = Eigen::VectorXf::Zero(9); //(x, y, z, roll, pitch, yaw, w_x, w_y, w_z) -> initial 0

// State propagation matrix F
Eigen::MatrixXf F(9, 9);

// control input u
Eigen::VectorXf control_input = Eigen::VectorXf::Zero(9);

// control input matrix
Eigen::MatrixXf G(9, 9);

// state covariance matrix
Eigen::MatrixXf P = 10 * Eigen::MatrixXf::Identity(9, 9); // for uncertainty of the state --- high number as initial value -> will convergate against smaller number

// measurement prediction
Eigen::VectorXf z = Eigen::VectorXf::Zero(9); // used in update step for innovation -> initial 0

// measurement prediction matrix
Eigen::MatrixXf H = Eigen::MatrixXf::Identity(9, 9); // check depending on sensors -> measurement provides all variables in state -> all ones?

Eigen::MatrixXf Q = 0.1 * Eigen::MatrixXf::Identity(9, 9); // process noise covariance matrix Q / System prediction noise -> how accurate is model
// takes influences like wind, bumps etc into account -> should be rather small compared to P
// TODO: determine Q -> modell konstant

// measurement noise covariance matrix
// Eigen::MatrixXf R(9,9);
Eigen::MatrixXf R_imu(9, 9);
Eigen::MatrixXf R_cam(9, 9); // evtl z achse manuell hohe varianz geben


// Prediction Step
void predict_state(const double dT, Eigen::VectorXf u)
{

    F <<0, 0, 0, 0, 0, 0, 0, (dT * r_sphere), 0,
        0, 0, 0, 0, 0, 0, -(dT * r_sphere), 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0;

    G << 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1;

    // predict state
    state = F * state + G * u;   // 9x9 * 9x1 = 9x1   //here: x_pri

    P = F * P * F.transpose() + Q; // here: calculate P_pri   // 9x9 * 9x9 * 9x9 + 9x9 = 9x9

    // predict measurement
    z = H * state; 
}

// Update Step -> measurement comes from either imu or cam callback
void update_state(const Eigen::VectorXf &measurement, const Eigen::MatrixXf R)
{
    // innovation covariance
    Eigen::MatrixXf S = H * P * H.transpose() + R; // all 9x9

    // kalman gain
    Eigen::MatrixXf K = P * H.transpose() * S.inverse(); // all 9x9

    // update state estimation -> correction of state
    // calculate innovation
    Eigen::VectorXf innovation = measurement - z;      // 9x1 - 9x1 = 9x1
    state = state + (K * innovation);                  // calculate x_pos    // 9x1 + (9x9 * 9x1) = 9x1 + 9x1 = 9x1
    P = (Eigen::MatrixXf::Identity(9, 9) - K * H) * P; // (9x9 - (9x9 * 9x9)) * 9x9 = 9x9

}

// PoseStamped -> Pose, Header -> Pose: Point, Quaternion -> Point: x,y,z; Quaternion: x,y,z,w
void apply_lkf_and_publish(const geometry_msgs::PoseStamped::ConstPtr &m)
{

    // calculate dT
    double dT = (m->header.stamp - last_imu_pose.header.stamp).toSec();

    // Get timestamp from current and wait for accumulator to go past that
    ros::Time stamp_current = m->header.stamp;
    tfScalar t_current = stamp_current.toSec();

    waitForAccumulator(t_current); // wait so that imu time stamp is inbetween cam time stamps

    // interpolation
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

    tf::Vector3 tf_angular_velocity;
    tf::vector3MsgToTF(angular_vel_curr.angular_velocity, tf_angular_velocity);

    tf::Matrix3x3 rotation_matrix;
    rotation_matrix.setRotation(tf::createQuaternionFromRPY(state[3], state[4], state[5]));
    tf::Vector3 tf_angular_velocity_rotated = rotation_matrix.inverse() * tf_angular_velocity;

    Eigen::Vector3f eigen_angular_velocity_rotated;
    eigen_angular_velocity_rotated[0] = tf_angular_velocity_rotated.getX();
    eigen_angular_velocity_rotated[1] = tf_angular_velocity_rotated.getY();
    eigen_angular_velocity_rotated[2] = tf_angular_velocity_rotated.getZ();

    tf::Pose imu_diff = pose_diff(m, last_imu_pose);
    geometry_msgs::Pose imu_diff_geom_msgs;
    tf::poseTFToMsg(imu_diff, imu_diff_geom_msgs);

    // make vector 9dof for predict step
    Eigen::VectorXf eigen_angular_velocity_rotated_9dof = Eigen::VectorXf::Zero(9);
    eigen_angular_velocity_rotated_9dof[6] = eigen_angular_velocity_rotated[0];
    eigen_angular_velocity_rotated_9dof[7] = eigen_angular_velocity_rotated[1];
    eigen_angular_velocity_rotated_9dof[8] = eigen_angular_velocity_rotated[2];

    // prediction step
    predict_state(dT, eigen_angular_velocity_rotated_9dof);

    // update step
    Eigen::VectorXf measurement(9);

    if (fabs(last_pose_interpolated.getRotation().length() - 1.0) > small_number) {
        ROS_WARN("Uninitialized quaternion, length: %f", last_pose_interpolated.getRotation().length());
        last_pose_interpolated = pose_interpolated;
        ROS_WARN("Attempted fix, length: %f", last_pose_interpolated.getRotation().length());
    }
    
    tf::Pose cam_diff_interpolated = pose_interpolated.inverseTimes(last_pose_interpolated);
    geometry_msgs::Pose cam_diff_geom_msgs;
    tf::poseTFToMsg(cam_diff_interpolated, cam_diff_geom_msgs);

    measurement << imu_diff.getOrigin().getX(), imu_diff.getOrigin().getY(), imu_diff.getOrigin().getZ(),
        getRollFromQuaternion(imu_diff_geom_msgs.orientation), getPitchFromQuaternion(imu_diff_geom_msgs.orientation), getYawFromQuaternion(imu_diff_geom_msgs.orientation),
        tf_angular_velocity_rotated.getX(), tf_angular_velocity_rotated.getY(), tf_angular_velocity_rotated.getZ();

    update_state(measurement, R_imu);

    measurement << cam_diff_interpolated.getOrigin().getX(), cam_diff_interpolated.getOrigin().getY(), cam_diff_interpolated.getOrigin().getZ(),
        getRollFromQuaternion(cam_diff_geom_msgs.orientation), getPitchFromQuaternion(cam_diff_geom_msgs.orientation), getYawFromQuaternion(cam_diff_geom_msgs.orientation),
        tf_angular_velocity_rotated.getX(), tf_angular_velocity_rotated.getY(), tf_angular_velocity_rotated.getZ();

    update_state(measurement, R_cam);

    tf::Pose filteredDeltaPose;
    filteredDeltaPose.setOrigin(tf::Vector3(state[0], state[1], state[2]));
    filteredDeltaPose.setRotation(tf::createQuaternionFromRPY(state[3], state[4], state[5]));

    tf::Stamped<tf::Pose> filteredPose;
    tf::poseStampedMsgToTF(filtered_pose_msg, filteredPose);      // put last published msg into filteredPose
    filteredPose.mult(filteredPose, filteredDeltaPose.inverse()); // update der filtered pose
    tf::poseStampedTFToMsg(filteredPose, filtered_pose_msg);      // update filtered_Pose_msg
    
    // double tmp_y, tmp_p, tmp_r;
    // tf::Matrix3x3(filteredDeltaPose.inverse().getRotation()).getRPY(tmp_r, tmp_p, tmp_y);
    // printf("Diff: %f %f %f\n", tmp_r, tmp_p, tmp_y);
    // printf("Angular vel: %f %f %f\n", state[6], state[7], state[8]);

    // Construct msg
    filtered_pose_msg.header.frame_id = "map";
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

            // Use delta filter and publish the pose msg
            apply_lkf_and_publish(m);
        }

        // Otherwise, if IMU interpolation is active, push current pose to queue
    }
    else if (interpolate == IMU)
    {
        pushToAccumulator(m);
    }

    // Save current pose to last pose for next iteration
    last_imu_pose = *m;
}

void camMsgCallback(const geometry_msgs::PoseStamped::ConstPtr &m)
{
    // Initialization on first run
    if (!initialized_cam)
    {
        ROS_INFO("INITIALIZED CAM");
        last_cam_pose = *m;
        initialized_cam = true;
        return;
    }

    // If cam interpolation is active, push cam poses to queue
    if (interpolate == CAM)
    {
        pushToAccumulator(m);

        // Otherwise, if IMU interpolation is active, wait for the IMU poses in the queue
    }
    else if (interpolate == IMU)
    {

        // Sanity check, can only interpolate if buffer is fully accumulated
        if (accumulator.size() == n_acc)
        {

            // Use delta filter and publish the pose msg
            apply_lkf_and_publish(m);
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
        debug_pose_pub.publish(debug_pose_msg);
    }

    // Save current pose to last pose for next iteration
    last_cam_pose = *m;
}

void orientationImuCallback(const sensor_msgs::Imu::ConstPtr &m)
{
    angular_vel_curr = *m;
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
    // TODO topic param für imu raw
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
    tf_listener.waitForTransform(frame_id_imu, frame_id_cam, ros::Time(0), ros::Duration(1.0));                       // wait 5 seconds for tf
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
    // TODO noch callback für sensor_msgs/IMU
    ros::Subscriber cam_pose_sub, imu_pose_sub, imu_vel_sub;
    if (interpolate == IMU)
    {
        cam_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(topic_pose_cam, 1000, camMsgCallback);
        imu_pose_sub = nh_fast.subscribe<geometry_msgs::PoseStamped>(topic_pose_imu, 1000, imuMsgCallback);
    }
    else if (interpolate == CAM)
    {
        cam_pose_sub = nh_fast.subscribe<geometry_msgs::PoseStamped>(topic_pose_cam, 1000, camMsgCallback);
        imu_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(topic_pose_imu, 1000, imuMsgCallback);
    }
    else if (interpolate == NONE)
    {
        cam_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(topic_pose_cam, 1000, camMsgCallback);
        imu_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(topic_pose_imu, 1000, imuMsgCallback);
    }
    imu_vel_sub = nh.subscribe<sensor_msgs::Imu>("orientation", 1000, orientationImuCallback);

    filtered_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(topic_publish, 1000);
    if (publish_debug_topic)
        debug_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/delta/debug", 1000);

    // Main processing loop, wait for callbacks to happen
    fast_rate = std::max(cam_rate, imu_rate);
    slow_rate = std::min(cam_rate, imu_rate);
    ros::Rate r(fast_rate);

    // initialise matrices & vectors
    // norm vector
    eigen_norm_vector[0] = 0.0;
    eigen_norm_vector[1] = 0.0;
    eigen_norm_vector[2] = 1.0;

    // IMU:
    Eigen::VectorXf imu_variances = Eigen::VectorXf::Zero(9); //[x,y,z,r,p,y, angular_vel_x, angular_vel_y, angular_vel_z]^T
    imu_variances[0] = 0.1; // 0.000000002685523762832521;
    imu_variances[1] = 0.1; //0.0000001275576292974622;
    imu_variances[2] = 0.001;  //not measured
    imu_variances[3] = 0.1; //0.009465788593315629;
    imu_variances[4] = 0.1; //0.0001922401945712851;
    imu_variances[5] = 0.1; //0.00007255917842660958;
    imu_variances[6] = 0.1; //0.00008535226550528127;
    imu_variances[7] = 0.1; //0.00002174349644727122;
    imu_variances[8] = 0.1; //0.00001210644017747147;

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
    cam_variances[0] = 0.1; //0.00000001217939299950571;
    cam_variances[1] = 0.1; //0.0000000008446764545249315;
    cam_variances[2] = 100.0; //0.000000007498298810942597;
    cam_variances[3] = 0.1; //0.0002543484849699809;
    cam_variances[4] = 0.1; //0.004764144829708403;
    cam_variances[5] = 0.01; //0.0001030990187090913;
    imu_variances[6] = 0.1;  //not measured
    imu_variances[7] = 0.1;  //not measured
    imu_variances[8] = 0.1;  //not measured

    R_cam << cam_variances[0], 0, 0, 0, 0, 0, 0, 0, 0,
             0, cam_variances[1], 0, 0, 0, 0, 0, 0, 0,
             0, 0, cam_variances[2], 0, 0, 0, 0, 0, 0,
             0, 0, 0, cam_variances[3], 0, 0, 0, 0, 0,
             0, 0, 0, 0, cam_variances[4], 0, 0, 0, 0,
             0, 0, 0, 0, 0, cam_variances[5], 0, 0, 0,
             0, 0, 0, 0, 0, 0, cam_variances[6], 0, 0,
             0, 0, 0, 0, 0, 0, 0, cam_variances[7], 0,
             0, 0, 0, 0, 0, 0, 0, 0, cam_variances[8];

    /*Q <<     0, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 1, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 1, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 1;

    Q = Q * R_imu;*/

    while (ros::ok())
    {
        ros::spinOnce();
        callbacks_fast.callOne();
        r.sleep();
    }

    return 0;
}
