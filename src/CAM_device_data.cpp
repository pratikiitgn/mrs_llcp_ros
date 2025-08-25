#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <mrs_modules_msgs/Llcp.h>
#include <sensor_msgs/Imu.h>

// | ----------------- Calling required libraries ----------------- |
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <mrs_lib/attitude_converter.h>
#include <nav_msgs/Odometry.h>

// | ----------------- Calling libraries for filtering the data ----------------- |
#include <mrs_lib/median_filter.h>

float encoder_1_in_deg_ = 0.0;
float encoder_2_in_deg_ = 0.0;
float encoder_1_in_rad_ = 0.0;
float encoder_2_in_rad_ = 0.0;
Eigen::Vector3d e_3_neg(0.0,0.0,-1.0);

int raw_encoder_1_data = 0;
int raw_encoder_2_data = 0;

Eigen::Matrix3d R_curr;

// | ----------------- Cable attitude State ----------------- |

Eigen::Vector3d     q (0.0,0.0,-1.0);
Eigen::Vector3d q_dot (0.0,0.0, 0.0);
Eigen::Vector3d q_old (0.0,0.0, 0.0);
Eigen::Matrix3d CAM_R_y;
Eigen::Matrix3d CAM_R_x;

class MedianFilter {
public:
  MedianFilter(size_t window_size) : window_size_(window_size) {}

  double update(double x) {
    buffer_.push_back(x);

    if (buffer_.size() > window_size_) {
      buffer_.pop_front();
    }

    auto buffer_sorted = buffer_;

    std::sort(std::begin(buffer_sorted), std::end(buffer_sorted));
    const auto middle = buffer_sorted.size()/2;
    const auto median = buffer_sorted.at(middle);

    return median;
  }

private:
  size_t window_size_;
  std::deque<double> buffer_;
};

class MovingAverageFilter {
public:
  MovingAverageFilter(size_t window_size) : window_size_(window_size), sum_(0.0) {}

  double update(double x) {
    buffer_.push_back(x);
    sum_ += x;

    if (buffer_.size() > window_size_) {
      sum_ -= buffer_.front();
      buffer_.pop_front();
    }

    return sum_ / buffer_.size();
  }

private:
  size_t window_size_;
  std::deque<double> buffer_;
  double sum_;
};

class LowPassFilter {
public:
  LowPassFilter(double alpha = 1.0) : alpha_(alpha), initialized_(false) {}

  double update(double x) {
    if (!initialized_) {
      y_ = x;
      initialized_ = true;
    } else {
      y_ = y_ + alpha_ * (x - y_);
    }
    return y_;
  }

  void reset(double x = 0.0) {
    y_ = x;
    initialized_ = false;
  }

private:
  double alpha_;   // smoothing factor: smaller = smoother, but slower
  double y_;
  bool initialized_;
};

namespace mrs_llcp_ros
{

class CAMDeviceData : public nodelet::Nodelet
{
public:
  CAMDeviceData() = default;
  ~CAMDeviceData() override = default;

  // Filters for q and q_dot
  LowPassFilter q_filters_[3]   = { LowPassFilter(0.4), LowPassFilter(0.4), LowPassFilter(0.4) };
  LowPassFilter qdot_filters_[3]= { LowPassFilter(0.2), LowPassFilter(0.2), LowPassFilter(0.2) };

  MovingAverageFilter q_filters_MA[3]   = { MovingAverageFilter(10),  MovingAverageFilter(10),  MovingAverageFilter(10) };
  MovingAverageFilter qdot_filters_MA[3]= { MovingAverageFilter(15),  MovingAverageFilter(15),  MovingAverageFilter(15) };

  MedianFilter q_filters_med[3]   = { MedianFilter(4),  MedianFilter(4),  MedianFilter(4) };
  MedianFilter qdot_filters_med[3]= { MedianFilter(4),  MedianFilter(4),  MedianFilter(4) };

  ros::Time last_llcp_time_;
  bool first_llcp_msg_ = true;

private:
  ros::Subscriber sub_raw_CAM_data;
  ros::Subscriber sub_IMU_data;
  ros::Publisher  pub_cable_state;

  struct HeartbeatMsg {
    uint8_t   id;
    bool      is_running;
    uint16_t  messages_received;
    uint16_t  servo1_pos;
    uint16_t  servo2_pos;
    bool      last_trigger;
    uint8_t   last_trigger_num;
  };

  // Helper to decode 16-bit little-endian integer
  uint16_t read_u16(const std::vector<uint8_t>& data, size_t offset) {
    return static_cast<uint16_t>(data[offset]) |
           (static_cast<uint16_t>(data[offset + 1]) << 8);
  }

  void llcpCallback(const mrs_modules_msgs::Llcp::ConstPtr& msg) {
    if (!msg->checksum_matched) {
      NODELET_WARN("Received LLCP message with bad checksum.");
      return;
    }

    const std::vector<uint8_t>& payload = msg->payload;

    if (payload.size() < 10) {
      NODELET_WARN("Payload too short to be a heartbeat message.");
      return;
    }

    HeartbeatMsg hb;

    hb.id                   = payload[0];
    hb.is_running           = payload[1];
    hb.messages_received    = read_u16(payload, 2);
    hb.servo1_pos           = read_u16(payload, 4);
    hb.servo2_pos           = read_u16(payload, 6);
    hb.last_trigger         = payload[8];
    hb.last_trigger_num     = payload[9];

    if (hb.id != 52) {  // HEARTBEAT_MSG_ID
      NODELET_WARN("Received message with unexpected ID: %d", hb.id);
      return;
    }

    int offset_encoder_1 = 2361;
    int offset_encoder_2 = 2799;

    raw_encoder_1_data  = hb.servo1_pos;
    raw_encoder_2_data  = hb.servo2_pos;

    int offsetted_encoder_1 = hb.servo1_pos - offset_encoder_1;
    int offsetted_encoder_2 = hb.servo2_pos - offset_encoder_2;

    encoder_1_in_deg_ = static_cast<float>(offsetted_encoder_1 * 0.0878906);
    encoder_2_in_deg_ = -static_cast<float>(offsetted_encoder_2 * 0.0878906);

    encoder_1_in_rad_ = encoder_1_in_deg_ * M_PI / 180.0;
    encoder_2_in_rad_ = encoder_2_in_deg_ * M_PI / 180.0;

    // NODELET_INFO_STREAM("Encoder 1 (deg): " << encoder_1_in_deg_);
    // NODELET_INFO_STREAM("Encoder 2 (deg):" << encoder_2_in_deg_);

    // Calculate rotation about pitch axis of CAM device
    CAM_R_y << 
      std::cos(static_cast<double>(encoder_2_in_rad_)), 0, std::sin(static_cast<double>(encoder_2_in_rad_)),
      0, 1, 0,
    -std::sin(static_cast<double>(encoder_2_in_rad_)), 0, std::cos(static_cast<double>(encoder_2_in_rad_));

    // Calculate rotation about roll axis of CAM device
    CAM_R_x << 
      1, 0, 0,
      0, std::cos(static_cast<double>(encoder_1_in_rad_)), -std::sin(static_cast<double>(encoder_1_in_rad_)),
      0, std::sin(static_cast<double>(encoder_1_in_rad_)),  std::cos(static_cast<double>(encoder_1_in_rad_));

    // NODELET_INFO_STREAM("Quat x: " << CAM_R_x(1,1));

    Eigen::Vector3d roll_pitch_yaw  = Rotation_matrix_to_Euler_angle(R_curr);

    roll_pitch_yaw                  = roll_pitch_yaw * 180.0 / M_PI;

    // NODELET_INFO_STREAM(": [" 
    //   << std::setprecision(4) << roll_pitch_yaw[0] << " " 
    //   << std::setprecision(4) << roll_pitch_yaw[1] << " " 
    //   << std::setprecision(5) << roll_pitch_yaw[2] << "],  "
    //   "[" << std::setprecision(4) << encoder_1_in_deg_ << ", "
    //       << std::setprecision(4) << encoder_2_in_deg_ << "]");

    // NODELET_INFO_STREAM(0.5, ": [" 
    //   << std::setprecision(4) << roll_pitch_yaw[0] << " " 
    //   << std::setprecision(4) << roll_pitch_yaw[1] << " " 
    //   << std::setprecision(5) << roll_pitch_yaw[2] << "],  "
    //   "[" << std::setprecision(4) << raw_encoder_1_data << ", "
    //       << std::setprecision(4) << raw_encoder_2_data << "]");

    q = Matrix_vector_mul(R_curr,Matrix_vector_mul(CAM_R_x,Matrix_vector_mul(CAM_R_y,e_3_neg)));
    q = sat_q(q);

    // Compute delta T from message timestamp
    ros::Time current_time = ros::Time::now();
    double delta_T_of_llcp;

    if (first_llcp_msg_) {
      delta_T_of_llcp = 0.01; // assume ~100 Hz for the very first msg
      first_llcp_msg_ = false;
    } else {
      delta_T_of_llcp = (current_time - last_llcp_time_).toSec();
      if (delta_T_of_llcp <= 0.0 || delta_T_of_llcp > 1.0) {
        // invalid or too large, clamp
        delta_T_of_llcp = 0.01;
      }
    }
    last_llcp_time_ = current_time;

// /- Publish cable state data  --------------------------------------
    nav_msgs::Odometry Cable_state_odom;
    Cable_state_odom.header.stamp         = ros::Time::now();

// /- Before implementing the filter-  ---------------------------------
    Cable_state_odom.pose.pose.orientation.x  = q[0];
    Cable_state_odom.pose.pose.orientation.y  = q[1];
    Cable_state_odom.pose.pose.orientation.z  = q[2];
    // --- Apply low-pass filters ---
    for (int i = 0; i < 3; i++) {
      // q[i]     = q_filters_med[i].update(q[i]);
      q[i]     = q_filters_MA[i].update(q[i]);
    }
// /- After implementing the filter-  ---------------------------------
    Cable_state_odom.pose.pose.position.x     = q[0];
    Cable_state_odom.pose.pose.position.y     = q[1];
    Cable_state_odom.pose.pose.position.z     = q[2];
// --------------------------------------------------------------------

    // Now compute q_dot using real Δt
    q_dot = (q - q_old) / delta_T_of_llcp;
    q_dot = sat_q_dot(q_dot);
    q_old = q;
// /- Before implementing the filter-  ---------------------------------
    Cable_state_odom.twist.twist.angular.x = q_dot[0];
    Cable_state_odom.twist.twist.angular.y = q_dot[1];

    // Cable_state_odom.twist.twist.angular.x = encoder_1_in_deg_;
    // Cable_state_odom.twist.twist.angular.y = encoder_2_in_deg_;

    Cable_state_odom.twist.twist.angular.z = q_dot[2];
    // --- Apply low-pass filters ---
    for (int ii = 0; ii < 3; ii++) {
      // q_dot[ii] = qdot_filters_med[ii].update(q_dot[ii]);
      q_dot[ii] = qdot_filters_MA[ii].update(q_dot[ii]);
    }
    q_dot = sat_q_dot(q_dot);
// /- After implementing the filter-  ---------------------------------
    Cable_state_odom.twist.twist.linear.x = q_dot[0];
    Cable_state_odom.twist.twist.linear.y = q_dot[1];
    Cable_state_odom.twist.twist.linear.z = q_dot[2];

    // Cable_state_odom.pose.pose.orientation.roll     = encoder_1_in_rad_;
    // Cable_state_odom.pose.pose.orientation.pitch    = encoder_2_in_rad_;
// --------------------------------------------------------------------

    // NODELET_INFO_STREAM(": [" 
    // << std::setprecision(4) << q[0] << " " 
    // << std::setprecision(4) << q[1] << " " 
    // << std::setprecision(5) << q[2] << "],  "
    // "[" << std::setprecision(4) << roll_pitch_yaw[2] << "]");

  // | ----------------- Publish Cable States ----------------- |
    pub_cable_state.publish(Cable_state_odom);

  }

Eigen::Vector3d sat_q_dot(Eigen::Vector3d vec){
    float sat_lim = 10.0;
    for (int ii=0; ii<3; ii++){
        if (vec[ii] > sat_lim){
            vec[ii] = sat_lim;
        }
        if (vec[ii] < -sat_lim){
            vec[ii] = -sat_lim;
        }
    }
    return vec;
}

Eigen::Vector3d sat_q(Eigen::Vector3d vec){
  float sat_lim = 1;
  for (int ii=0; ii<3; ii++){
      if (vec[ii] > sat_lim){
          vec[ii] = sat_lim;
      }
      if (vec[ii] < -sat_lim){
          vec[ii] = -sat_lim;
      }
  }
  return vec;
}

Eigen::Vector3d Matrix_vector_mul(Eigen::Matrix3d R, Eigen::Vector3d v){
  Eigen::Vector3d mul_vector (R(0,0)*v[0] + R(0,1)*v[1] + R(0,2)*v[2], R(1,0)*v[0] + R(1,1)*v[1] + R(1,2)*v[2],  R(2,0)*v[0] + R(2,1)*v[1] + R(2,2)*v[2]);
  return mul_vector;
}

Eigen::Vector3d Rotation_matrix_to_Euler_angle(Eigen::Matrix3d R){

  float sy    = sqrtf( R(0,0) * R(0,0) +  R(1,0) * R(1,0) );
  float ph_   = atan2f(R(2,1) , R(2,2));
  float th_   = atan2f(-R(2,0), sy);
  float ps_   = atan2f(R(1,0), R(0,0));

  Eigen::Vector3d des_roll_pitch_yaw(ph_, th_, ps_);
  return des_roll_pitch_yaw;
}

void imu_data_callback(const sensor_msgs::Imu::ConstPtr& msg) {

  Eigen::Quaterniond quad_rot_in_quat(msg->orientation.w,msg->orientation.x,msg->orientation.y,msg->orientation.z);
  R_curr = mrs_lib::AttitudeConverter(quad_rot_in_quat);

}

void onInit() override {
  ros::NodeHandle& nh = getNodeHandle();

  // Get UAV name from parameter or environment variable
  std::string uav_name;
  if (!nh.getParam("uav_name", uav_name)) {
    const char* uav_env = std::getenv("UAV_NAME");
    uav_name = uav_env ? uav_env : "uav1";
  }

  std::string topic_name_CAM_device_raw = "/" + uav_name + "/llcp/received_message";
  std::string topic_name_IMU_raw        = "/" + uav_name + "/mavros/imu/data";

  sub_raw_CAM_data  = nh.subscribe(topic_name_CAM_device_raw, 10, &CAMDeviceData::llcpCallback, this);
  sub_IMU_data      = nh.subscribe(topic_name_IMU_raw, 10, &CAMDeviceData::imu_data_callback, this);

  pub_cable_state   = nh.advertise<nav_msgs::Odometry>("cable_state", 1);

  NODELET_INFO("CAMDeviceData nodelet started.");
}

};

} // namespace mrs_llcp_ros

// Register nodelet
PLUGINLIB_EXPORT_CLASS(mrs_llcp_ros::CAMDeviceData, nodelet::Nodelet)
