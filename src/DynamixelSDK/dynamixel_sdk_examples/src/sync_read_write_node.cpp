// Author: Karanjot Singh
*******************************************************************************/

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/sync_set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/sync_get_position.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "read_write_node.hpp"

// Control table address for X series (except XL-320)
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_PROFILE_VELOCITY 112
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define BAUDRATE 1000000  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/Dyna0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;

uint32_t profile_velocity = 30;

uint8_t dxl_error = 0;

uint32_t goal_position1 = 0;
uint32_t goal_position2 = 0;
uint32_t goal_position3 = 0;
uint32_t goal_position4 = 0;
uint32_t goal_position5 = 0;
uint32_t goal_position6 = 0;
uint32_t goal_position7 = 0;
uint32_t goal_position8 = 0;
int dxl_comm_result = COMM_TX_FAIL;

SyncReadWriteNode::SyncReadWriteNode()
: Node("sync_read_write_node")
{
  RCLCPP_INFO(this->get_logger(), "Run sync read write node");

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  set_position_subscriber_ =
    this->create_subscription<SyncSetPosition>(
    "sync_set_position",
    QOS_RKL10V,
    [this](const SyncSetPosition::SharedPtr msg) -> void
    {
      uint8_t dxl_error = 0;
      // Position Value of X series is 4 byte data.
      uint32_t goal_position1 = (unsigned int)msg->position1;  // Convert int32 -> uint32
      // Write Goal Position (length : 4 bytes)
      dxl_comm_result =
      packetHandler->write4ByteTxRx(
        portHandler,
        (uint8_t) msg->id1,
        ADDR_GOAL_POSITION,
        goal_position1,
        &dxl_error
      );

      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
      } else {
        RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]", msg->id1, msg->position1);
      }

      //uint8_t dxl_error = 0;
      // Position Value of X series is 4 byte data.
      uint32_t goal_position2 = (unsigned int)msg->position2;  // Convert int32 -> uint32
      // Write Goal Position (length : 4 bytes)
      dxl_comm_result =
      packetHandler->write4ByteTxRx(
        portHandler,
        (uint8_t) msg->id2,
        ADDR_GOAL_POSITION,
        goal_position2,
        &dxl_error
      );

      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
      } else {
        RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]", msg->id2, msg->position2);
      }

      //uint8_t dxl_error = 0;
      // Position Value of X series is 4 byte data.
      uint32_t goal_position3 = (unsigned int)msg->position3;  // Convert int32 -> uint32
      // Write Goal Position (length : 4 bytes)
      dxl_comm_result =
      packetHandler->write4ByteTxRx(
        portHandler,
        (uint8_t) msg->id3,
        ADDR_GOAL_POSITION,
        goal_position3,
        &dxl_error
      );

      //uint8_t dxl_error = 0;
      // Position Value of X series is 4 byte data.
      uint32_t goal_position4 = (unsigned int)msg->position4;  // Convert int32 -> uint32
      // Write Goal Position (length : 4 bytes)
      dxl_comm_result =
      packetHandler->write4ByteTxRx(
        portHandler,
        (uint8_t) msg->id4,
        ADDR_GOAL_POSITION,
        goal_position4,
        &dxl_error
      );

      //uint8_t dxl_error = 0;
      // Position Value of X series is 4 byte data.
      uint32_t goal_position5 = (unsigned int)msg->position5;  // Convert int32 -> uint32
      // Write Goal Position (length : 4 bytes)
      dxl_comm_result =
      packetHandler->write4ByteTxRx(
        portHandler,
        (uint8_t) msg->id5,
        ADDR_GOAL_POSITION,
        goal_position5,
        &dxl_error
      );

      //uint8_t dxl_error = 0;
      // Position Value of X series is 4 byte data.
      uint32_t goal_position6 = (unsigned int)msg->position6;  // Convert int32 -> uint32
      // Write Goal Position (length : 4 bytes)
      dxl_comm_result =
      packetHandler->write4ByteTxRx(
        portHandler,
        (uint8_t) msg->id6,
        ADDR_GOAL_POSITION,
        goal_position6,
        &dxl_error
      );

      //uint8_t dxl_error = 0;
      // Position Value of X series is 4 byte data.
      uint32_t goal_position7 = (unsigned int)msg->position7;  // Convert int32 -> uint32
      // Write Goal Position (length : 4 bytes)
      dxl_comm_result =
      packetHandler->write4ByteTxRx(
        portHandler,
        (uint8_t) msg->id7,
        ADDR_GOAL_POSITION,
        goal_position7,
        &dxl_error
      );

      //uint8_t dxl_error = 0;
      // Position Value of X series is 4 byte data.
      uint32_t goal_position8 = (unsigned int)msg->position8;  // Convert int32 -> uint32
      // Write Goal Position (length : 4 bytes)
      dxl_comm_result =
      packetHandler->write4ByteTxRx(
        portHandler,
        (uint8_t) msg->id8,
        ADDR_GOAL_POSITION,
        goal_position8,
        &dxl_error
      );
    }
    );

  auto get_present_position =
    [this](
    const std::shared_ptr<SyncGetPosition::Request> request,
    std::shared_ptr<SyncGetPosition::Response> response) -> void
    {
      // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
      dxl_comm_result = packetHandler->read4ByteTxRx(
        portHandler,
        (uint8_t) request->id1,
        ADDR_PRESENT_POSITION,
        reinterpret_cast<uint32_t *>(&present_position1),
        &dxl_error
      );

      RCLCPP_INFO(
        this->get_logger(),
        "Get [ID: %d] [Present Position: %d]",
        request->id1,
        present_position1
      );
      response->position1 = present_position1;

      // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
      dxl_comm_result = packetHandler->read4ByteTxRx(
        portHandler,
        (uint8_t) request->id2,
        ADDR_PRESENT_POSITION,
        reinterpret_cast<uint32_t *>(&present_position2),
        &dxl_error
      );

      RCLCPP_INFO(
        this->get_logger(),
        "Get [ID: %d] [Present Position: %d]",
        request->id2,
        present_position2
      );
      response->position2 = present_position2;

      // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
      dxl_comm_result = packetHandler->read4ByteTxRx(
        portHandler,
        (uint8_t) request->id3,
        ADDR_PRESENT_POSITION,
        reinterpret_cast<uint32_t *>(&present_position3),
        &dxl_error
      );
      response->position3 = present_position3;

      // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
      dxl_comm_result = packetHandler->read4ByteTxRx(
        portHandler,
        (uint8_t) request->id4,
        ADDR_PRESENT_POSITION,
        reinterpret_cast<uint32_t *>(&present_position4),
        &dxl_error
      );
      response->position4 = present_position4;

      // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
      dxl_comm_result = packetHandler->read4ByteTxRx(
        portHandler,
        (uint8_t) request->id5,
        ADDR_PRESENT_POSITION,
        reinterpret_cast<uint32_t *>(&present_position5),
        &dxl_error
      );
      response->position5 = present_position5;

      // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
      dxl_comm_result = packetHandler->read4ByteTxRx(
        portHandler,
        (uint8_t) request->id6,
        ADDR_PRESENT_POSITION,
        reinterpret_cast<uint32_t *>(&present_position6),
        &dxl_error
      );
      response->position6 = present_position6;

      // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
      dxl_comm_result = packetHandler->read4ByteTxRx(
        portHandler,
        (uint8_t) request->id7,
        ADDR_PRESENT_POSITION,
        reinterpret_cast<uint32_t *>(&present_position7),
        &dxl_error
      );
      response->position7 = present_position7;

      // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
      dxl_comm_result = packetHandler->read4ByteTxRx(
        portHandler,
        (uint8_t) request->id8,
        ADDR_PRESENT_POSITION,
        reinterpret_cast<uint32_t *>(&present_position8),
        &dxl_error
      );
      response->position8 = present_position8;
    };

  get_position_server_ = create_service<SyncGetPosition>("sync_get_position", get_present_position);
}

SyncReadWriteNode::~SyncReadWriteNode()
{
}

void setupDynamixel(uint8_t dxl_id)
{
  // Use Position Control Mode
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_OPERATING_MODE,
    3,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("sync_read_write_node"), "Failed to set Position Control Mode.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("sync_read_write_node"), "Succeeded to set Position Control Mode.");
  }

  // Enable Torque of DYNAMIXEL
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_TORQUE_ENABLE,
    1,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("sync_read_write_node"), "Failed to enable torque.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("sync_read_write_node"), "Succeeded to enable torque.");
  }

  // Setup Profile velocity for operation
  dxl_comm_result = packetHandler->write4ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_PROFILE_VELOCITY,
    profile_velocity,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("sync_read_write_node"), "Failed to set Profile Velocity.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("sync_read_write_node"), "Succeeded to set Profile Velocity.");
  }
}

int main(int argc, char * argv[])
{
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open Serial Port
  dxl_comm_result = portHandler->openPort();
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("sync_read_write_node"), "Failed to open the port!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("sync_read_write_node"), "Succeeded to open the port.");
  }

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("sync_read_write_node"), "Failed to set the baudrate!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("sync_read_write_node"), "Succeeded to set the baudrate.");
  }

  setupDynamixel(BROADCAST_ID);

  rclcpp::init(argc, argv);

  auto syncreadwritenode = std::make_shared<SyncReadWriteNode>();
  rclcpp::spin(syncreadwritenode);
  rclcpp::shutdown();

  // Disable Torque of DYNAMIXEL
  packetHandler->write1ByteTxRx(
    portHandler,
    BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    0,
    &dxl_error
  );

  return 0;
}
