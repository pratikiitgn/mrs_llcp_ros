#include <ros/package.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <string>
#include <serial_port.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <mrs_msgs/Llcp.h>

#include <llcp.h>
#include <thread>

#define SERIAL_BUFFER_SIZE 512

namespace llcp_ros
{

/* class LlcpRos //{ */

class LlcpRos : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  LLCP_Receiver_t llcp_receiver;

  serial_port::SerialPort serial_port_;

  bool openSerialPort(std::string portname, int baudrate);
  void serialThread(void);

  std::thread serial_thread_;

  void callbackSendMessage(const mrs_msgs::LlcpConstPtr &msg);

  void callbackMaintainerTimer(const ros::TimerEvent &event);
  void callbackConnectTimer(const ros::TimerEvent &event);

  ros::NodeHandle nh_;

  bool running_     = true;
  bool initialized_ = false;
  bool connected_ = false;

  std::string portname_;

  ros::Publisher  llcp_publisher_;
  ros::Subscriber llcp_subscriber_;

  ros::Timer maintainer_timer_;
};

//}

/* onInit() //{ */

void LlcpRos::onInit() {

  // Get paramters
  nh_ = ros::NodeHandle("~");

  ros::Time::waitForValid();

  ROS_INFO("[LlcpRos]: node initialized");


  llcp_initialize(&llcp_receiver);

  ROS_INFO("[LlcpRos]: llcp receiver initialized");

  serial_thread_ = std::thread(&LlcpRos::serialThread, this);
  /* nh_.param("uav_name", uav_name_, std::string("uav")); */
  nh_.param("portname", portname_);
  /* nh_.param("baudrate", baudrate_, 115200); */
  /* nh_.param("publish_bad_checksum", publish_bad_checksum, false); */
  /* nh_.param("simulate_fake_garmin", simulate_fake_garmin, false); */
  /* nh_.param("use_timeout", use_timeout, true); */
  /* nh_.param("swap_garmins", swap_garmins, false); */
  /* nh_.param("serial_rate", serial_rate_, 5000); */
  /* nh_.param("serial_buffer_size", serial_buffer_size_, 1024); */

  /* ser_send_int     = nh_.advertiseService("send_int", &LlcpRos::callbackSendInt, this); */
  /* ser_send_int_raw = nh_.advertiseService("send_int_raw", &LlcpRos::callbackSendIntRaw, this); */

  /* // Publishers */
  /* std::string postfix_A = swap_garmins ? "_up" : ""; */
  /* std::string postfix_B = swap_garmins ? "" : "_up"; */
  /* range_publisher_A_    = nh_.advertise<sensor_msgs::Range>("range" + postfix_A, 1); */
  /* garmin_A_frame_       = uav_name_ + "/garmin" + postfix_A; */
  /* garmin_B_frame_       = uav_name_ + "/garmin" + postfix_B; */

  /* llcp_ros_publisher_ = nh_.advertise<mrs_msgs::LlcpRos>("baca_protocol_out", 1); */

  /* llcp_ros_subscriber = nh_.subscribe("baca_protocol_in", 10, &LlcpRos::callbackSendMessage, this, ros::TransportHints().tcpNoDelay()); */
  llcp_publisher_ = nh_.advertise<mrs_msgs::Llcp>("llcp_out", 1);

  llcp_subscriber_ = nh_.subscribe("llcp_in", 10, &LlcpRos::callbackSendMessage, this, ros::TransportHints().tcpNoDelay());

  /* // Output loaded parameters to console for double checking */
  /* ROS_INFO_THROTTLE(1.0, "[%s] is up and running with the following parameters:", ros::this_node::getName().c_str()); */
  /* ROS_INFO_THROTTLE(1.0, "[%s] portname: %s", ros::this_node::getName().c_str(), portname_.c_str()); */
  /* ROS_INFO_THROTTLE(1.0, "[%s] baudrate: %i", ros::this_node::getName().c_str(), baudrate_); */
  /* ROS_INFO_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName().c_str() << "] publishing messages with wrong checksum: " << publish_bad_checksum); */


  /* serial_timer_     = nh_.createTimer(ros::Rate(serial_rate_), &LlcpRos::callbackSerialTimer, this); */
  /* fake_timer_       = nh_.createTimer(ros::Rate(fake_garmin_rate_), &LlcpRos::callbackFakeTimer, this); */
  
  maintainer_timer_ = nh_.createTimer(ros::Rate(1), &LlcpRos::callbackMaintainerTimer, this);

  connected_ = openSerialPort(portname_, 115200);


  initialized_ = true;
}
//}

/*  openSerialPort()//{ */

bool LlcpRos::openSerialPort(std::string portname, int baudrate) {

  ROS_INFO_THROTTLE(1.0, "[%s]: Openning serial port.", ros::this_node::getName().c_str());

  if (!serial_port_.connect(portname, baudrate)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: Could not connect to the serial port", ros::this_node::getName().c_str());
    /* is_connected_ = false; */
    return false;
  }

  ROS_INFO_THROTTLE(1.0, "[%s]: Connected to sensor.", ros::this_node::getName().c_str());
  /* is_connected_  = true; */
  /* last_received_ = ros::Time::now(); */

  return true;
}

//}

/*  serialThread()//{ */

void LlcpRos::serialThread(void) {

  uint8_t  rx_buffer[SERIAL_BUFFER_SIZE];
  uint16_t bytes_read;

  while (!initialized_) {
    //TODO : do mutex initialized_ and connected_ variables!!!
    ROS_INFO("[LlcpRos]: waiting for initialization");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  while (!connected_) {
    //TODO : do mutex initialized_ and connected_ variables!!!
    ROS_INFO("[LlcpRos]: waiting for connection");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  /* { */
  /*   while (!serial_port_.checkConnected()) { */
  /*     ROS_INFO("[LlcpRos]: serial thread waiting for serial port"); */
  /*     std::this_thread::sleep_for(std::chrono::milliseconds(100)); */
  /*   } */
  /* } */

  ROS_INFO("[LlcpRos]: serial thread starting");

  while (running_) {
    bytes_read = serial_port_.readSerial(rx_buffer, SERIAL_BUFFER_SIZE);
    if (bytes_read > 0) {

      // feed all the incoming bytes into the MiniPIX interface
      for (uint16_t i = 0; i < bytes_read; i++) {

        LLCP_Message_t *message_in;

        bool checksum_matched = false;

        if (llcp_processChar(rx_buffer[i], &llcp_receiver, &message_in, &checksum_matched)) {
          ROS_INFO_STREAM("[LlcpRos]: received message with id " << message_in->id);
          ROS_INFO_STREAM("[LlcpRos]: checksum is: " << checksum_matched);
          mrs_msgs::Llcp msg_out;

          msg_out.checksum_matched = checksum_matched;
          msg_out.id               = message_in->id;

          uint8_t payload_size = llcp_receiver.payload_size;

          /* msg_out.payload.push_back(message_in->id); */

          for (uint8_t i = 0; i < payload_size; i++) {
            msg_out.payload.push_back(message_in->payload[i]);
          }
          llcp_publisher_.publish(msg_out);
        }
      }
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

//}

/* callbackSendMessage() //{ */

void LlcpRos::callbackSendMessage(const mrs_msgs::LlcpConstPtr &msg) {

  if (!connected_) {
    return;
  }

  if (!initialized_) {
    ROS_INFO_STREAM_THROTTLE(1.0, "[LlcpRos]: Cannot send message, nodelet not initialized!");
    return;
  }

  /* ROS_INFO_STREAM_THROTTLE(1.0, "SENDING: " << msg->id); */

  uint8_t out_buffer[512];

  // llcp is working with arrays, so we need to convert the payload from the ROS message into an array
  std::vector<uint8_t> payload_vec  = msg->payload;
  uint8_t              payload_size = payload_vec.size();
  uint8_t              payload_arr[payload_size];
  std::copy(payload_vec.begin(), payload_vec.end(), payload_arr);

  uint16_t msg_len = llcp_prepareMessage((uint8_t *)&payload_arr, payload_size, out_buffer, msg->id);
  serial_port_.sendCharArray(out_buffer, msg_len);
}

//}

/* callbackMaintainerTimer() //{ */

void LlcpRos::callbackMaintainerTimer(const ros::TimerEvent &event) {

  if (connected_) {

    if (!serial_port_.checkConnected()) {
      connected_ = false;
      ROS_ERROR("[LlcpRos] Serial device is disconnected! ");
    }
  }

  /* if (((ros::Time::now() - last_received_).toSec() > MAXIMAL_TIME_INTERVAL) && use_timeout && is_connected_) { */

  /*   connected_ = false; */

  /*   ROS_ERROR_STREAM("[" << ros::this_node::getName().c_str() << "] Serial port timed out - no messages were received in " << MAXIMAL_TIME_INTERVAL */
  /*                        << " seconds"); */
  /* } */

  /* if (connected_) { */

    /* ROS_INFO_STREAM("Got msgs - Garmin: " << received_msg_ok_garmin << " Generic msg: " << received_msg_ok << "  Wrong checksum: " << received_msg_bad_checksum */
    /*                                       << "; in the last " << (ros::Time::now() - interval_).toSec() << " s"); */
    /* interval_ = ros::Time::now(); */

  /* } else { */

  /*   connectToSensor(); */
  /* } */
}

//}

}  // namespace llcp_ros

PLUGINLIB_EXPORT_CLASS(llcp_ros::LlcpRos, nodelet::Nodelet);
