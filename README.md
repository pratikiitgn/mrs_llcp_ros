# mrs_llcp_ros
This package is the ROS interface for the low-level communication protocol (LLCP) which is used by the MRS group to communicate with lower level devices.
The management of the serial port on Linux can be a hassle, so this packages takes care of that for you, you just specify the serial port and baud rate in a config or a launch file.
An example of how to use this package can be found in the [llcp_example](https://github.com/ctu-mrs/llcp_example) package.

Your ROS node communicates with mrs_llcp_ros through ROS messages, and mrs_llcp_ros then handles the physical serial port communication with the low-level device:

```mermaid
flowchart LR
A[Your ROS node] <-->|ROS messages| B[mrs_llcp_ros]
B <-->|UART| C[Low level device]
```

Mrs_llcp_ros uses two ROS topics, using the mrs_msgs::Llcp message:

```
/$UAV_NAME/llcp/received_message
/$UAV_NAME/llcp/send_message
```

All received messages are published on the `received_message` topic.
If you want to send a message to the low-level device, publish it to the `send_message` topic.

Here is some example code (taken from [llcp_example](https://github.com/ctu-mrs/llcp_example)) showing you how to send and receive messages:

## message definitions

```c
#define DATA_MSG_ID 52
#define HEARTBEAT_MSG_ID 51

struct __attribute__((__packed__)) data_msg
{
  const uint8_t id = DATA_MSG_ID;
  uint8_t       data1_uint8;
  uint32_t      data2_uint32;
  float         data3_float;
};

struct __attribute__((__packed__)) heartbeat_msg
{
  const uint8_t id = HEARTBEAT_MSG_ID;
  bool          is_running;
  uint16_t      messages_received;
};

```

## receiving a message

```c
void LlcpExample::callbackReceiveMessage(const mrs_msgs::LlcpConstPtr &msg) {

  // llcp is working with arrays, so we need to convert the payload from the ROS message into an array
  uint8_t payload_size = msg->payload.size();
  uint8_t payload_array[payload_size];
  std::copy(msg->payload.begin(), msg->payload.end(), payload_array);

  // the first byte is the ID, so we can use it to determine the message type
  switch (payload_array[0]) {
    case DATA_MSG_ID: {

      data_msg *received_msg = (data_msg *)payload_array;
      /* do your stuff here */
      break;
    }
    case HEARTBEAT_MSG_ID: {
      heartbeat_msg *received_msg = (heartbeat_msg *)payload_array;
      /* do your stuff here */
      break;
    }
  }
}
```

## sending a message

```c
  data_msg msg_out;

  msg_out.id  = DATA_MSG_ID;
  msg_out.data1_uint8  = 23;
  msg_out.data2_uint32 = 178491;
  msg_out.data3_float  = 420.69;

  mrs_msgs::Llcp llcp_msg;

  uint8_t *msg_ptr = (uint8_t *)&msg_out;

  for (int i = 0; i < sizeof(msg_out); i++) {
    llcp_msg.payload.push_back(msg_ptr[i]);
  }

  llcp_publisher_.publish(llcp_msg);
```
