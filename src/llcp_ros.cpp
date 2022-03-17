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

namespace mrs_llcp_ros
{

/* class MrsLlcpRos //{ */

class MrsLlcpRos : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  LLCP_Receiver_t llcp_receiver;

  serial_port::SerialPort serial_port_;

  bool openSerialPort(std::string portname, int baudrate);
  void serialThread(void);

  std::thread serial_thread_;

  void callbackSendMessage(const mrs_msgs::LlcpConstPtr &msg);
  void connectToSerial();
  void callbackMaintainerTimer(const ros::TimerEvent &event);
  void callbackConnectTimer(const ros::TimerEvent &event);

  ros::NodeHandle nh_;

  bool running_     = true;
  bool initialized_ = false;
  bool connected_   = false;

  std::string portname_;
  int         baudrate_;

  ros::Publisher  llcp_publisher_;
  ros::Subscriber llcp_subscriber_;

  ros::Timer maintainer_timer_;

  std::mutex mutex_connected_;
};

//}

/* onInit() //{ */

void MrsLlcpRos::onInit() {

  // Get paramters
  nh_ = ros::NodeHandle("~");

  ros::Time::waitForValid();

  ROS_INFO("[MrsLlcpRos]: node initialized");

  llcp_initialize(&llcp_receiver);

  ROS_INFO("[MrsLlcpRos]: llcp receiver initialized");

  nh_.getParam("portname", portname_);
  nh_.getParam("baudrate", baudrate_);

  llcp_publisher_ = nh_.advertise<mrs_msgs::Llcp>("llcp_out", 1);

  llcp_subscriber_ = nh_.subscribe("llcp_in", 10, &MrsLlcpRos::callbackSendMessage, this, ros::TransportHints().tcpNoDelay());

  maintainer_timer_ = nh_.createTimer(ros::Rate(1), &MrsLlcpRos::callbackMaintainerTimer, this);

  {
    std::scoped_lock lock(mutex_connected_);
    connected_ = false;
  }

  connectToSerial();
  initialized_ = true;
}
//}

/*  connectToSerial()//{ */

void MrsLlcpRos::connectToSerial() {

  if (serial_thread_.joinable()) {
    serial_thread_.join();
  }

  bool tmp_conected = false;

  while (!tmp_conected) {
    tmp_conected = openSerialPort(portname_, baudrate_);
  }

  {
    std::scoped_lock lock(mutex_connected_);
    connected_ = true;
  }

  serial_thread_ = std::thread(&MrsLlcpRos::serialThread, this);
}

//}

/*  openSerialPort()//{ */

bool MrsLlcpRos::openSerialPort(std::string portname, int baudrate) {

  ROS_INFO_THROTTLE(1.0, "[%s]: Openning serial port.", ros::this_node::getName().c_str());
  ROS_INFO_STREAM_THROTTLE(1.0, "Portname: " << portname << " baudrate: " << baudrate);

  if (!serial_port_.connect(portname, baudrate)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: Could not connect to the serial port", ros::this_node::getName().c_str());
    return false;
  }

  ROS_INFO_THROTTLE(1.0, "[%s]: Connected to sensor.", ros::this_node::getName().c_str());

  return true;
}

//}

/*  serialThread()//{ */

void MrsLlcpRos::serialThread(void) {

  uint8_t  rx_buffer[SERIAL_BUFFER_SIZE];
  uint16_t bytes_read;

  ROS_INFO("[MrsLlcpRos]: serial thread starting");

  while (running_) {

    bool connected;
    {
      std::scoped_lock lock(mutex_connected_);
      connected = connected_;
    }
    if (!connected) {
      ROS_WARN("[MrsLlcpRos]: terminating serial thread because the serial port was disconnected");
      return;
    }

    bytes_read = serial_port_.readSerial(rx_buffer, SERIAL_BUFFER_SIZE);
    if (bytes_read > 0 && bytes_read < SERIAL_BUFFER_SIZE) {
      /* If the serial device is disconnected and readSerial() is called, it will return max_int number of read bytes, thats why we check it against the
       * SERIAL_BUFFER_SIZE */

      for (uint16_t i = 0; i < bytes_read; i++) {

        LLCP_Message_t *message_in;

        bool checksum_matched = false;

        if (llcp_processChar(rx_buffer[i], &llcp_receiver, &message_in, &checksum_matched)) {
          ROS_INFO_STREAM("[MrsLlcpRos]: received message with id " << message_in->id);
          ROS_INFO_STREAM("[MrsLlcpRos]: checksum is: " << checksum_matched);
          mrs_msgs::Llcp msg_out;

          msg_out.checksum_matched = checksum_matched;
          msg_out.id               = message_in->id;

          uint8_t payload_size = llcp_receiver.payload_size;

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

void MrsLlcpRos::callbackSendMessage(const mrs_msgs::LlcpConstPtr &msg) {

  bool connected;

  {
    std::scoped_lock lock(mutex_connected_);
    connected = connected_;
  }
  if (!connected) {
    return;
  }

  if (!initialized_) {
    ROS_INFO_STREAM_THROTTLE(1.0, "[MrsLlcpRos]: Cannot send message, nodelet not initialized!");
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

void MrsLlcpRos::callbackMaintainerTimer(const ros::TimerEvent &event) {

  bool connected;

  {
    std::scoped_lock lock(mutex_connected_);
    connected = connected_;
  }

  if (connected) {

    if (!serial_port_.checkConnected()) {
      {
        std::scoped_lock lock(mutex_connected_);
        connected_ = false;
      }
      ROS_ERROR("[MrsLlcpRos] Serial device is disconnected! ");
      connectToSerial();
    }
  }

  /* if (((ros::Time::now() - last_received_).toSec() > MAXIMAL_TIME_INTERVAL) && use_timeout && is_connected_) { */

  /*   connected_ = false; */

  /*   ROS_ERROR_STREAM("[" << ros::this_node::getName().c_str() << "] Serial port timed out - no messages were received in " << MAXIMAL_TIME_INTERVAL */
  /*                        << " seconds"); */
  /* } */

  /* if (connected_) { */

  /* ROS_INFO_STREAM("Got msgs - Garmin: " << received_msg_ok_garmin << " Generic msg: " << received_msg_ok << "  Wrong checksum: " << received_msg_bad_checksum
   */
  /*                                       << "; in the last " << (ros::Time::now() - interval_).toSec() << " s"); */
  /* interval_ = ros::Time::now(); */

  /* } else { */

  /*   connectToSensor(); */
  /* } */
}

//}

}  // namespace mrs_llcp_ros

PLUGINLIB_EXPORT_CLASS(mrs_llcp_ros::MrsLlcpRos, nodelet::Nodelet);
