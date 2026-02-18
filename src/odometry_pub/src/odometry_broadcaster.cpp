#include <ros/ros.h>
#include <cmath>
#include <ros/time.h>
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class OdometryBroadcaster
{
public:
    OdometryBroadcaster();
    ~OdometryBroadcaster();
private:
    void wheelVelCallback(const std_msgs::Float32MultiArray& msg);

    void runOdometry(const std_msgs::Float32MultiArray& msg_,
                     const ros::Time& time_stamp);

    void computeOdometry(nav_msgs::Odometry& odom,
                         const float& FLWheelVel,
                         const float& FRWheelVel,
                         const float& RLWheelVel, 
                         const float& RRWheelVel, 
                         const float& time_interval_ms);

    geometry_msgs::Twist computeVel(const float& FLWheelVel,
                                    const float& FRWheelVel,
                                    const float& RLWheelVel,
                                    const float& RRWheelVel);

    nav_msgs::Odometry computeRelativeMotion(const float& FLWheelVel,
                                             const float& FRWheelVel,
                                             const float& RLWheelVel,
                                             const float& RRWheelVel,
                                             const float& timeSeconds);

    //ROS variables
    void setSubAndPub(ros::NodeHandle& nh_);
    std::string ns; //Parameters namespace
    
    //Subscriber and publishers
    ros::Subscriber feedback_sub;
    ros::Publisher odom_pub;
    tf2_ros::TransformBroadcaster transform_broadcaster;

    // Parameters
    // For Mecanum: robot_wheelbase = L1 (half-width) + L2 (half-length) = 0.20 + 0.175 = 0.375
    // For Differential: robot_wheelbase = track width
    float robot_wheelbase = 0.375; // L1 + L2 for holonomic geometry
    float robot_track_width = 0.2; // For differential drive
    std::string kinematics_type; // "holonomic" or "differential"
    bool publish_tf;

    ros::Time last_received_data;
    ros::Time time_now;
    bool init = false;
    nav_msgs::Odometry odom_msg;
    geometry_msgs::TransformStamped odom_transform;

};

//=====================
//        constructor
//=====================
OdometryBroadcaster::OdometryBroadcaster(){
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ns = nh.getNamespace()+"/";
    if (ns == "//") ns = "";

    // Load parameters
    nh_private.param<std::string>("kinematics_type", kinematics_type, "holonomic");
    nh_private.param<bool>("publish_tf", publish_tf, true);
    // You might want to load dimensions here too
    // nh_private.param<float>("wheelbase", robot_wheelbase, 0.2);

    ROS_INFO_STREAM(ns << "Odometry broadcaster: startup... Mode: " << kinematics_type);

    //Setup ROS subscribers and publishers
    setSubAndPub(nh);
}

OdometryBroadcaster::~OdometryBroadcaster(){}

//=======================================
//   Setup ROS subscribers, publishers
//        and pre-fill messages
//=======================================
void OdometryBroadcaster::setSubAndPub(ros::NodeHandle& nh_){
    ROS_INFO_STREAM(ns << "Odometry broadcaster: setting up publishers and subscribers...");

    //------------------------------------------
    // Setup wheel velocity feedback subscriber
    //------------------------------------------
    // Expecting [FL, FR, RL, RR]
    feedback_sub = nh_.subscribe("wheel_velocity", 1000, &OdometryBroadcaster::wheelVelCallback, this);

    //--------------------------
    // Setup odometry publisher
    //--------------------------
    odom_pub = nh_.advertise<nav_msgs::Odometry>("odom", 1000);

    //-----------------------------
    // Initialize odometry message
    //-----------------------------
    odom_msg.header.frame_id = "odom"; // Standard frame names
    odom_msg.child_frame_id = "base_footprint";
    odom_msg.pose.pose.orientation.w = 1; //unit quaternion
    odom_msg.pose.pose.orientation.x = 0;
    odom_msg.pose.pose.orientation.y = 0;
    odom_msg.pose.pose.orientation.z = 0;

    //------------------------------
    // Initialize transform message
    //------------------------------
    odom_transform.header.frame_id = "odom";
    odom_transform.child_frame_id = "base_footprint";
}


void OdometryBroadcaster::wheelVelCallback(const std_msgs::Float32MultiArray& msg){
    if (msg.data.size() < 4){
        ROS_WARN_THROTTLE(1.0, "Received wheel velocity msg with insufficient data");
        return;
    }
    
    if (!init){
        last_received_data = ros::Time::now();
        init = true;
        ROS_INFO_STREAM(ns << "Odometry broadcaster: initialized and receiving data!");
    }
    else{
        time_now = ros::Time::now();
        ros::Duration time_interval = time_now - last_received_data;
        float time_interval_sec = time_interval.toSec();

        // Sanity check for time jump
        if (time_interval_sec < 0) {
             last_received_data = time_now;
             return;
        }

        if (time_interval_sec < 1.0){ // slightly more tolerant
            runOdometry(msg, time_now);
            last_received_data = time_now;
        }
        else{
            ROS_WARN_STREAM(ns << "Odometry broadcaster: time jump or lag detected " << time_interval_sec << " sec");
            last_received_data = time_now;
        }
    }
}

void OdometryBroadcaster::runOdometry(const std_msgs::Float32MultiArray& msg_, const ros::Time& time_stamp){
        // Assuming Arduino sends [FL, FR, RL, RR] in msg_.data[0..3]
        // If the array has 5 elements (last is time), we ignore the last one as we use ROS time.
        
        computeOdometry(odom_msg, msg_.data[0], msg_.data[1], msg_.data[2], msg_.data[3], (time_stamp - last_received_data).toSec() * 1000.0);

        //Publish odometry
        odom_msg.header.stamp = time_stamp;
        odom_pub.publish(odom_msg);

        //Publish transform
        odom_transform.header.stamp = time_stamp;
        odom_transform.transform.translation.x = odom_msg.pose.pose.position.x;
        odom_transform.transform.translation.y = odom_msg.pose.pose.position.y;
        odom_transform.transform.translation.z = odom_msg.pose.pose.position.z;
        odom_transform.transform.rotation = odom_msg.pose.pose.orientation;
        
        // Removed the 180 degree rotation hack. Ensure hardware matches standard ROS frames.
        
        if (publish_tf) {
            transform_broadcaster.sendTransform(odom_transform);
        }
}

void OdometryBroadcaster::computeOdometry(nav_msgs::Odometry& odom,
                     const float& FL,
                     const float& FR,
                     const float& RL, 
                     const float& RR, 
                     const float& time_interval_ms)
{   
    float time_sec = time_interval_ms / 1000.0;
    if(time_sec <= 0) return;

    const nav_msgs::Odometry relativeMotion = computeRelativeMotion(FL, FR, RL, RR, time_sec);
    
    tf2::Quaternion q_prev, q_rot, q_new;
    tf2::Vector3 new_translation;
    tf2::Vector3 rel_translation;

    //Convert to quaternion type for computation
    tf2::convert(odom.pose.pose.orientation, q_prev);
    tf2::convert(relativeMotion.pose.pose.orientation, q_rot);
    
    // Update orientation
    q_new = q_prev * q_rot; // Rotation composition
    q_new.normalize();
    tf2::convert(q_new, odom.pose.pose.orientation);

    // Update Position
    // Rotate relative translation vector by previous orientation
    tf2::convert(relativeMotion.pose.pose.position, rel_translation);
    
    // In ROS TF2: q * v rotates vector v by quaternion q
    new_translation = tf2::quatRotate(q_prev, rel_translation);

    odom.pose.pose.position.x += new_translation.x();
    odom.pose.pose.position.y += new_translation.y();
    odom.pose.pose.position.z += new_translation.z();

    // Set Twist (velocity in child frame / base_link)
    odom.twist = relativeMotion.twist;
}

//======================================
//    Compute velocity in base link frame
//======================================
geometry_msgs::Twist OdometryBroadcaster::computeVel(const float& FL, const float& FR, const float& RL, const float& RR){
    geometry_msgs::Twist vel;

    if (kinematics_type == "holonomic") {
        // Mecanum Kinematics (Forward Kinematics)
        // Note: RL and RR are hardware-inverted, so negate them to get actual wheel velocities
        float RL_actual = -RL;  // Hardware inverted
        float RR_actual = -RR;  // Hardware inverted
        
        // Standard Mecanum equations:
        // Vx = (FL + FR + RL + RR) / 4
        // Vy = (-FL + FR + RL - RR) / 4
        // Wz = (-FL + FR - RL + RR) / (4 * (L1 + L2))
        
        vel.linear.x = (FL + FR + RL_actual + RR_actual) / 4.0;
        vel.linear.y = (-FL + FR + RL_actual - RR_actual) / 4.0; 
        vel.linear.z = 0.0;
        
        float geom_factor = 1.0 / (4.0 * robot_wheelbase);
        vel.angular.z = (-FL + FR - RL_actual + RR_actual) * geom_factor;
        // vel.angular.z = (FL - FR - RL_actual + RR_actual) * geom_factor;
        ROS_DEBUG_THROTTLE(1.0, "Mecanum velocities - Raw: [FL=%.4f, FR=%.4f, RL=%.4f, RR=%.4f] Actual: [%.4f, %.4f, %.4f, %.4f] Computed: [Vx=%.4f, Vy=%.4f, Wz=%.4f]",
                          FL, FR, RL, RR, FL, FR, RL_actual, RR_actual, vel.linear.x, vel.linear.y, vel.angular.z);

    } else { 
        // Differential Drive (Skid Steer 4WD)
        float RL_actual = -RL;  // Hardware inverted
        float RR_actual = -RR;  // Hardware inverted
        
        float v_left = (FL + RL_actual) / 2.0;
        float v_right = (FR + RR_actual) / 2.0;

        vel.linear.x = (v_right + v_left) / 2.0;
        vel.linear.y = 0.0;
        vel.linear.z = 0.0;
        
        vel.angular.z = (v_right - v_left) / robot_wheelbase;
        
        ROS_DEBUG_THROTTLE(1.0, "Differential velocities - Raw: [FL=%.4f, FR=%.4f, RL=%.4f, RR=%.4f] Computed: [Vx=%.4f, Wz=%.4f]",
                          FL, FR, RL, RR, vel.linear.x, vel.angular.z);
    }

    return vel;
}

nav_msgs::Odometry OdometryBroadcaster::computeRelativeMotion(const float& FL, const float& FR, const float& RL, const float& RR, const float& timeSeconds){
    nav_msgs::Odometry rel_motion;
    const geometry_msgs::Twist vel = computeVel(FL, FR, RL, RR);
    rel_motion.twist.twist = vel;

    double angleChange = vel.angular.z * timeSeconds;
    
    // Runge-Kutta integration or Exact Arc integration
    // For small time steps, straight line approx or arc approx is fine.

    if (std::abs(vel.angular.z) < 0.001) {
        // Linear motion approximation
        rel_motion.pose.pose.position.x = vel.linear.x * timeSeconds;
        rel_motion.pose.pose.position.y = vel.linear.y * timeSeconds;
    } else {
        // Arc motion
        // Exact integration of constant velocity commands:
        // x(t) = x0 + (vx/w)*sin(wt) + (vy/w)*(cos(wt)-1)   <-- Note: Vy exists for holonomic
        // y(t) = y0 - (vx/w)*(cos(wt)-1) + (vy/w)*sin(wt)
        // theta(t) = theta0 + w*t
        
        double av_z = vel.angular.z;
        double lin_x = vel.linear.x;
        double lin_y = vel.linear.y;
        
        rel_motion.pose.pose.position.x = (lin_x * std::sin(angleChange) + lin_y * (std::cos(angleChange) - 1.0)) / av_z;
        rel_motion.pose.pose.position.y = (lin_x * (1.0 - std::cos(angleChange)) + lin_y * std::sin(angleChange)) / av_z;
    }
    
    rel_motion.pose.pose.position.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, angleChange);
    q.normalize();
    rel_motion.pose.pose.orientation = tf2::toMsg(q);
    
    return rel_motion;
}


//==============================
//             Main
//==============================
int main(int argc, char** argv){
    ros::init(argc, argv, "odometry_node");
    
    try{
        OdometryBroadcaster odometry_broadcaster;
        ros::spin();
    }
    catch (std::exception& e){
         ROS_FATAL_STREAM("Node encountered an error: " << e.what());
         return 1;
    }
    return 0;
}
