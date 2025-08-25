#include <ros/ros.h>
#include <mrs_modules_msgs/Llcp.h>

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
    ROS_WARN("Received LLCP message with bad checksum.");
    return;
  }

  const std::vector<uint8_t>& payload = msg->payload;

  if (payload.size() < 10) {
    ROS_WARN("Payload too short to be a heartbeat message.");
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

  // Validate the message type
  if (hb.id != 52) {  // HEARTBEAT_MSG_ID = 52
    ROS_WARN("Received message with unexpected ID: %d", hb.id);
    return;
  }

  // Print message contents
  // ROS_INFO_STREAM("---- Heartbeat Received ----");
  // ROS_INFO_STREAM("is_running: " << hb.is_running);
  // ROS_INFO_STREAM("messages_received: " << hb.messages_received);
  // ROS_INFO_STREAM("servo1_pos: " << hb.servo1_pos);
  // ROS_INFO_STREAM("servo2_pos: " << hb.servo2_pos);
  // ROS_INFO_STREAM("last_trigger: " << hb.last_trigger);
  // ROS_INFO_STREAM("last_trigger_num: " << static_cast<int>(hb.last_trigger_num));

  int offset_encoder_1    = 1800;
  int offset_encoder_2    = 2100;

  int offsetted_encoder_1 = hb.servo1_pos - offset_encoder_1;
  int offsetted_encoder_2 = hb.servo2_pos - offset_encoder_2;

  float encoder_1_in_deg  = (float) (offsetted_encoder_1 * 0.0878906);
  float encoder_2_in_deg  = (float) (offsetted_encoder_2 * 0.0878906);

  // ROS_INFO_STREAM("Encoder 1: " << encoder_1_in_deg);
  // ROS_INFO_STREAM("Encoder 2: " << encoder_2_in_deg);

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "heartbeat_listener");
  ros::NodeHandle nh;

  ros::Subscriber sub     = nh.subscribe("llcp_in", 10, llcpCallback);

  ROS_INFO("Heartbeat listener node started.");
  ros::spin();
  return 0;
}