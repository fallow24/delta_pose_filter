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

//rohdaten in orientation topic -> angular vel und accelerometer werte -> als systeminput angular vel



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

// Store current angular velocity provided by imus
sensor_msgs::Imu angular_vel_curr;

// Store last iterations orientation angles to calculate delta for KF
Eigen::VectorXf last_rpy_imu = Eigen::VectorXf::Zero(3);
Eigen::VectorXf last_rpy_cam = Eigen::VectorXf::Zero(3);

// Store delta of orientation angles
Eigen::VectorXf d_rpy_cam = Eigen::VectorXf::Zero(3);
Eigen::VectorXf d_rpy_imu = Eigen::VectorXf::Zero(3);

// Store sum of filtered delta values for rpy
Eigen::VectorXf filtered_rpy_sum = Eigen::VectorXf::Zero(3);


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
        callbacks_fast.callOne( ros::WallDuration() );  //callbacks_fast momentan für cam --- zeitstempel von imu und posePubMerged gleich -> nicht für imu interpolieren
	rate.sleep();
    }

    if(t < accumulator.front().header.stamp.toSec()) return;
	
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


//------------supporting methods----------
inline double getRollFromQuaternion(const geometry_msgs::Quaternion &q){
    tf::Quaternion q2;
    tf::quaternionMsgToTF(q, q2);
    double r,p,y;
    tf::Matrix3x3(q2).getRPY(r,p,y);
    return r;
}

inline double getPitchFromQuaternion(const geometry_msgs::Quaternion &q){
    tf::Quaternion q2;
    tf::quaternionMsgToTF(q, q2);
    double r,p,y;
    tf::Matrix3x3(q2).getRPY(r,p,y);
    return p;
}

inline double getYawFromQuaternion(const geometry_msgs::Quaternion &q){
    tf::Quaternion q2;
    tf::quaternionMsgToTF(q, q2);
    double r,p,y;
    tf::Matrix3x3(q2).getRPY(r,p,y);
    return y;
}


//--------------LKF----------------

//noch ohne Systemeingang, also ohne G matrix und u(k)
//H, F, R, Q, nicht verändert pro step

//LKF state
Eigen::VectorXf state = Eigen::VectorXf::Zero(9);//(x, y, z, roll, pitch, yaw, w_x, w_y, w_z) -> initial 0

Eigen::MatrixXf P = 10 * Eigen::MatrixXf::Identity(9,9);//state covariance matrix for uncertainty of the state --- high number as initial value -> will convergate against smaller number*/

//measurement prediction
Eigen::VectorXf z = Eigen::VectorXf::Zero(9); //used in update step for innovation -> initial 0

//measurement prediction matrix
Eigen::MatrixXf H = Eigen::MatrixXf::Identity(9,9);//check depending on sensors -> measurement provides all variables in state -> all ones?

Eigen::MatrixXf Q = 0.001 * Eigen::MatrixXf::Identity(9,9);//process noise covariance matrix Q / System prediction noise -> how accurate is model    
//takes influences like wind, bumps etc into account -> should be rather small compared to P 
//TODO: determine Q -> modell konstant 

//measurement noise covariance matrix
Eigen::MatrixXf R(9,9);
//Eigen::MatrixXf R = Eigen::MatrixXf::Identity(6,6); //determine R -> R_imu + R_cam

//R = (sigma_imu)² || (sigma_cam)² = standard deviation for sensor -> determine R for imu and R for Cam

/*
R<< sigma_imu_x² + sigma_cam_x² 0 0 0 0 0
    0 sigma_imu_y² + sigma_cam_y² 0 0 0 0
    0 0 sigma_imu_z² + sigma_cam_z² 0 0 0 
    ...
*/

//Prediction Step
void predict_state(const double dT){

    //std::cout << "Eigen Vector:\n" << state << "\n";
    //State propagation matrix F
        Eigen::MatrixXf F (9,9);
        /*F <<1, 0, 0, 0, 0, 0, dT * r_sphere, 0, 0,
            0, 1, 0, 0, 0, 0, 0, dT * r_sphere, 0,
            0, 0, 1, 0, 0, 0, 0, 0, dT * r_sphere,
            0, 0, 0, 1, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 1, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 1;*/
            F <<1, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 1, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 1, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 1;
        // result is 9x1 vector with [x+x'*dt, y+y'*dt, z+z'*dt, r, p ,y, w_x, w_y, w_z]^T

            //std::cout << "F:\n" << F << "\n";
            //std::cout << "post F, pre R_imu";


    Eigen::VectorXf imu_variances = Eigen::VectorXf::Zero(9);       //[x,y,z,r,p,y, angular_vel_x, angular_vel_y, angular_vel_z]^T
    imu_variances[0] = 0.000000008625475089899384;
    imu_variances[1] = 0.0000001300695165234252;
    imu_variances[2] = 0.0;
    //TODO complete this
    /*
    varianzen der quaternionen einträge: (imu)
    qx: 0.002283471474404559
    qy: 0.00000008585881265633921
    qz: 0.000000009046559957449105
    qw: 0.000005104138561091153
    */

    Eigen::MatrixXf R_imu (9,9);
    R_imu <<imu_variances[0], 0, 0, 0, 0, 0, 0, 0, 0,                   //sigma_x²
            0, imu_variances[1], 0, 0, 0, 0, 0, 0, 0,                   //sigma_y²
            0, 0, imu_variances[2], 0, 0, 0, 0, 0, 0,                   //sigma_z²
            0, 0, 0, 0.000001, 0, 0, 0, 0, 0,                                //sigma_roll²
            0, 0, 0, 0, 0.000001, 0, 0, 0, 0,                              //sigma_pitch²
            0, 0, 0, 0, 0, 0.000001, 0, 0, 0,                              //sigma_yaw²
            0, 0, 0, 0, 0, 0, 0.000001, 0, 0,                              //sigma_angular_vel_x
            0, 0, 0, 0, 0, 0, 0, 0.000001, 0,                              //sigma_angular_vel_y
            0, 0, 0, 0, 0, 0, 0, 0, 0.000001;                              //sigma_angular_vel_z

    //std::cout << "Rimu:\n" << R_imu << "\n";

    
    Eigen::VectorXf cam_variances = Eigen::VectorXf::Zero(9);       //[x,y,z,r,p,y, angular_vel_x, angular_vel_y, angular_vel_z]^T     
    cam_variances[0] = 0.00000001667001288387107;
    cam_variances[1] = 0.0000000008720263515496412;
    cam_variances[2] = 0.000000008970177859979189;
    //TODO complete this
    /*
    varianzen der quaternionen einträge: (cam)
    qx: 0.000000246476785231986
    qy: 0.0000003675303217650841
    qz: 0.000000009046559957449105
    qw: 0.001333102683144191
    */

    Eigen::MatrixXf R_cam(9,9); //evtl z achse manuell hohe varianz geben
    R_cam <<cam_variances[0], 0, 0, 0, 0, 0, 0, 0, 0,
            0, cam_variances[1], 0, 0, 0, 0, 0, 0, 0,
            0, 0, cam_variances[2], 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0.000001, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0.000001, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0.000001, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0,                
            0, 0, 0, 0, 0, 0, 0, 0, 0,                
            0, 0, 0, 0, 0, 0, 0, 0, 0;  
            
    //std::cout << "Rc:\n" << R_cam << "\n";
          
    R = R_imu + R_cam;

    //std::cout << "R:\n" << R << "\n";

    //predict state
    state = F * state;//9x9 * 9x1 = 9x1   //here: x_pri
    P = F * P * F.transpose() + Q;//here: calculate P_pri   // 9x9 * 9x9 * 9x9 + 9x9 = 9x9

    //std::cout << "P:\n" << P << "\n";

    //predict measurement
    z = H * state; //H * x_pri  //if H = identity matrix -> z = x ?    // 9x9 * 9x1 = 9x1
}

//Update Step -> measurement comes from either imu or cam callback
void update_state(const Eigen::VectorXf &measurement){      //TODO: hier noch R übergeben je nachdem ob cam oder imu
    //innovation covariance
    Eigen::MatrixXf S = H * P * H.transpose() + R; // all 9x9
    
    //kalman gain
    Eigen::MatrixXf K = P * H.transpose() * S.inverse();    // all 9x9

    //update state estimation -> correction of state
    //calculate innovation
    Eigen::VectorXf innovation = measurement - z;   // 9x1 - 9x1 = 9x1
    state = state + (K * innovation);//calculate x_pos    // 9x1 + (9x9 * 9x1) = 9x1 + 9x1 = 9x1
    P = (Eigen::MatrixXf::Identity(9,9) - K * H) * P;   // (9x9 - (9x9 * 9x9)) * 9x9 = 9x9

    /*std::cout << "S" << S << std::endl;
    std::cout << "innovation" << innovation<< std::endl;
    std::cout << "K" << K<< std::endl;
    std::cout << "S_inverse" << S.inverse() << std::endl;
    std::cout << "new state" << state<< std::endl;*/
}

//PoseStamped -> Pose, Header -> Pose: Point, Quaternion -> Point: x,y,z; Quaternion: x,y,z,w
void apply_lkf_and_publish(const geometry_msgs::PoseStamped::ConstPtr &m){

    //calculate dT
    double dT = (m->header.stamp - last_imu_pose.header.stamp).toSec();

    //printf("dT: %f",dT);

    //TODO: hier angular vel in richtigen frame rücken und an state und u übergeben


    //prediction step
    predict_state(dT);

    // Get timestamp from current and wait for accumulator to go past that  
    ros::Time stamp_current = m->header.stamp;
    tfScalar t_current = stamp_current.toSec();

    waitForAccumulator(t_current); //wait so that imu time stamp is inbetween cam time stamps

    //interpolation
    //Convert geometry_msgs to Quaternion format
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


    //update step cam (interpolated)
    Eigen::VectorXf measurement(9);
    geometry_msgs::Pose cam_pose;
    tf::poseTFToMsg(pose_interpolated, cam_pose);

    //std::cout << "cam_pose: " << cam_pose << " , imu pose: " << m->pose << std::endl;

    //calculate delta for rpy of cam
    d_rpy_cam[0] = getRollFromQuaternion(cam_pose.orientation) - last_rpy_cam[0];
    d_rpy_cam[1] = getPitchFromQuaternion(cam_pose.orientation) - last_rpy_cam[1];
    d_rpy_cam[2] = getYawFromQuaternion(cam_pose.orientation) - last_rpy_cam[2];

    /*
    tf::Pose diff_interpolated;
    diff_interpolated = pose_interpolated.inverseTimes(last_pose_interpolated);
    */

    tf::Pose cam_diff_interpolated = pose_interpolated.inverseTimes(last_pose_interpolated);
    geometry_msgs::Quaternion cam_diff_geom_msgs;
    tf::quaternionTFToMsg(cam_diff_interpolated.getRotation(), cam_diff_geom_msgs);


    //version without delta angles commented out
    /*measurement <<  cam_pose.position.x, cam_pose.position.y, cam_pose.position.z,
                    getRollFromQuaternion(cam_pose.orientation), getPitchFromQuaternion(cam_pose.orientation), getYawFromQuaternion(cam_pose.orientation),
                    0, 0, 0;*/ //[x,y,z,r,p,y, angular_vel_x, angular_vel_y, angular_vel_z]^T

    /*measurement <<  cam_pose.position.x, cam_pose.position.y, cam_pose.position.z,
                    d_rpy_cam[0], d_rpy_cam[1], d_rpy_cam[2],
                    0, 0, 0;*/

    measurement <<  cam_diff_interpolated.getOrigin().getX(), cam_diff_interpolated.getOrigin().getX(),
                    getRollFromQuaternion(cam_diff_geom_msgs), getPitchFromQuaternion(cam_diff_geom_msgs), getYawFromQuaternion(cam_diff_geom_msgs),
                    0, 0, 0;
                    
    update_state(measurement);
    

    //update step IMU

    //calculate delta for rpy of cam
    d_rpy_imu[0] = getRollFromQuaternion(m->pose.orientation) - last_rpy_imu[0];
    d_rpy_imu[1] = getPitchFromQuaternion(m->pose.orientation) - last_rpy_imu[1];
    d_rpy_imu[2] = getYawFromQuaternion(m->pose.orientation) - last_rpy_imu[2];

    tf::Pose imu_diff = pose_diff(m, last_imu_pose);
    geometry_msgs::Quaternion imu_diff_geom_msgs;
    tf::quaternionTFToMsg(imu_diff.getRotation(), imu_diff_geom_msgs);

    //version without delta angles commented out
    /*measurement <<  m->pose.position.x, m->pose.position.y, m->pose.position.z,
                    getRollFromQuaternion(m->pose.orientation), getPitchFromQuaternion(m->pose.orientation), getYawFromQuaternion(m->pose.orientation),
                    angular_vel_curr.angular_velocity.x, angular_vel_curr.angular_velocity.y, angular_vel_curr.angular_velocity.z;*/ //[x,y,z,r,p,y, angular_vel_x, angular_vel_y, angular_vel_z]^T
    /*measurement <<  m->pose.position.x, m->pose.position.y, m->pose.position.z,
                    d_rpy_imu[0], d_rpy_imu[1], d_rpy_imu[2],
                    angular_vel_curr.angular_velocity.x, angular_vel_curr.angular_velocity.y, angular_vel_curr.angular_velocity.z;*/
                    
    measurement <<  imu_diff.getOrigin().getX(), imu_diff.getOrigin().getY(), imu_diff.getOrigin().x(),
                    getRollFromQuaternion(imu_diff_geom_msgs), getPitchFromQuaternion(imu_diff_geom_msgs), getYawFromQuaternion(imu_diff_geom_msgs),
                    angular_vel_curr.angular_velocity.x, angular_vel_curr.angular_velocity.y, angular_vel_curr.angular_velocity.z;


    update_state(measurement);


    //TODO: state to 9dof -> angular velocity as for system input -> vorher in frame rücken
    //rotieren von angular vel über:(inverse) rotationsmatrix von rpy * vektor von angular vel
    


    // save filtered state
    // put rpy back into quaternion and save it to be published as orientation
    // TODO: currently holds delta values and not full angles

    // add filtered delta angel to sum of filtered rpy angles
    //anstatt summe: 

    /*filtered_rpy_sum[0] += state[3];
    filtered_rpy_sum[1] += state[4];
    filtered_rpy_sum[2] += state[5];*/

    tf::Pose filteredDeltaPose;
    filteredDeltaPose.setOrigin(tf::Vector3(state[0],state[1],state[2]));       //TODO: state nur mit deltas
    filteredDeltaPose.setRotation(tf::createQuaternionFromRPY(state[3],state[4],state[5]));

    tf::Stamped<tf::Pose> filteredPose;
    tf::poseStampedMsgToTF(filtered_pose_msg, filteredPose);        //put last published msg into filteredPose
    filteredPose.mult(filteredPose, filteredDeltaPose.inverse());   //update der filtered pose
    tf::poseStampedTFToMsg(filteredPose, filtered_pose_msg);        //update filtered_Pose_msg


    /*std::cout << "raw_cam_rpy (raw whole angle of this iteration):" << std::endl << std::endl;
    std::cout << "roll:" << getRollFromQuaternion(cam_pose.orientation) << ", ";
    std::cout << "pitch:" << getPitchFromQuaternion(cam_pose.orientation) << ", ";
    std::cout << "yaw:" << getYawFromQuaternion(cam_pose.orientation) << std::endl << std::endl;
    std::cout << "raw_imu_rpy (raw whole angle of this iteration):" << std::endl << std::endl;
    std::cout << "roll:" << getRollFromQuaternion(m->pose.orientation) << ", ";
    std::cout << "pitch:" << getPitchFromQuaternion(m->pose.orientation) << ", ";
    std::cout << "yaw:" << getYawFromQuaternion(m->pose.orientation) << std::endl << std::endl;
    std::cout << "last_imu_rpy (raw whole angle of last iteration):" << std::endl << last_rpy_imu << std::endl << std::endl;
    std::cout << "delta_imu_rpy (raw delta):" << std::endl << d_rpy_imu << std::endl << std::endl;
    std::cout << "state (filtered delta):" << std::endl << state << std::endl << std::endl;
    std::cout << "filtered_sum_rpy (filtered whole angle):" << std::endl << filtered_rpy_sum << std::endl << std::endl;
    std::cout << "--------------------------next iteration---------------------------" << std::endl << std::endl;*/

    /*tf::Quaternion filtered_orientation;
    //filtered_orientation.setRPY(state[3], state[4], state[5]); //use if not using delta angles for KF
    filtered_orientation.setRPY(filtered_rpy_sum[0], filtered_rpy_sum[1], filtered_rpy_sum[2]);
    geometry_msgs::Quaternion filtered_quaternion;
    tf::quaternionTFToMsg(filtered_orientation, filtered_quaternion);

    filtered_pose_msg.pose.orientation = filtered_quaternion;
    filtered_pose_msg.pose.position.x = state[0];
    filtered_pose_msg.pose.position.y = state[1];
    filtered_pose_msg.pose.position.z = state[2];*/

    // Construct msg
    filtered_pose_msg.header.frame_id = "odom";
    filtered_pose_msg.header.stamp = stamp_current;
    filtered_pose_msg.header.seq = sequence++;

    // publish filtered_pose_msg
    filtered_pose_pub.publish(filtered_pose_msg);

    // Prep next iteration
    last_pose_interpolated = pose_interpolated; 


    // update the last rpy vectors for the next delta calculation
    /*last_rpy_cam[0] = getRollFromQuaternion(cam_pose.orientation);
    last_rpy_cam[1] = getPitchFromQuaternion(cam_pose.orientation);
    last_rpy_cam[2] = getYawFromQuaternion(cam_pose.orientation);

    last_rpy_imu[0] = getRollFromQuaternion(m->pose.orientation);
    last_rpy_imu[1] = getPitchFromQuaternion(m->pose.orientation);
    last_rpy_imu[2] = getYawFromQuaternion(m->pose.orientation);*/
    
    
    /*
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
    */
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
            apply_lkf_and_publish(m);
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
        debug_pose_pub.publish( debug_pose_msg );
    }

    // Save current pose to last pose for next iteration
    last_cam_pose = *m;
}

void orientationImuCallback(const sensor_msgs::Imu::ConstPtr &m){
    angular_vel_curr = *m;
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
    //TODO topic param für imu raw
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
    //TODO noch callback für sensor_msgs/IMU
    ros::Subscriber cam_pose_sub, imu_pose_sub, imu_vel_sub;
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
    imu_vel_sub = nh.subscribe<sensor_msgs::Imu>("orientation", 1000, orientationImuCallback);
    
    
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


