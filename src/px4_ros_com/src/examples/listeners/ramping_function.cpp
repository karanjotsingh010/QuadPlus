
// DO NOT DECLARE VARIABLES AS GLOBAL WHICH ARE ADDED ONTO THEMSLEVES IN THE CODE. e.g., u = u + delu_res;
// THESE VARIABLE VALUES ARE RETIANED OVER EVERY CYCLE AND THIS LEADS TO ERROR
#include <cstdio>
#include <memory>
#include <string>
#include <chrono>
#include <iostream>

// dynamixel start
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <dynamixel_sdk_custom_interfaces/msg/set_position.hpp>
#include <dynamixel_sdk_custom_interfaces/srv/get_position.hpp>
#include <dynamixel_sdk_custom_interfaces/msg/sync_set_position.hpp>
#include <dynamixel_sdk_custom_interfaces/srv/sync_get_position.hpp>

#include "rcutils/cmdline_parser.h"
// dynamixel end

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/rc_channels.hpp>
#include <stdint.h>
#include </usr/include/armadillo>

using namespace std::chrono_literals;
// dynamixel start

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
double delu_max_step = 2.0*arma::datum::pi*10.0/(60.0*200.0);	// delu_max = 2*pi*servo rpm / (60*running frequency in HZ)
double servo_bits_rad = 4095/(2*arma::datum::pi);			// convert servo angle to bits (4096 bits)

uint8_t dxl_error = 0;

uint8_t id1 = 1;
uint8_t id2 = 2;
uint8_t id3 = 3;
uint8_t id4 = 4;
uint8_t id5 = 5;
uint8_t id6 = 6;
uint8_t id7 = 7;
uint8_t id8 = 8;

uint32_t zero_position1 = 3073;
uint32_t zero_position2 = 2048;
uint32_t zero_position3 = 3073;
uint32_t zero_position4 = 2048;
uint32_t zero_position5 = 3073;
uint32_t zero_position6 = 2048;
uint32_t zero_position7 = 3073;
uint32_t zero_position8 = 2048;

uint32_t goal_position1 = zero_position1;
uint32_t goal_position2 = zero_position2;
uint32_t goal_position3 = zero_position3;
uint32_t goal_position4 = zero_position4;
uint32_t goal_position5 = zero_position5;
uint32_t goal_position6 = zero_position6;
uint32_t goal_position7 = zero_position7;
uint32_t goal_position8 = zero_position8;

int dxl_comm_result = COMM_TX_FAIL;

// dynamixel end

// INDI variables declaration
arma::vec u = arma::zeros(8);	
double phi_d;
double theta_d;
double upper_lim = 0.17;
double lower_lim = -0.17;


//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@


class SensorIntegrator : public rclcpp::Node {
public:
	SensorIntegrator() : Node("sensor_integrator") {
	// subscriber 1
		rc_channels_listener_ = this->create_subscription<px4_msgs::msg::RcChannels>("fmu/rc_channels/out", 
		#ifdef ROS_DEFAULT_API
            	10,
		#endif
            	[this](const px4_msgs::msg::RcChannels::UniquePtr msg) {
           
            	phi_d = 0.17*msg->channels[1];
            	theta_d = 0.17*msg->channels[2];

            	// Overdamped
            	u(0) = phi_d;
            	u(1) = phi_d;
            	u(2) = phi_d;
            	u(3) = phi_d;
            	
            	u(4) = theta_d;
            	u(5) = theta_d;
            	u(6) = theta_d;
            	u(7) = theta_d;
            	
            	u = arma::clamp(u, lower_lim, upper_lim);
            	        	
            	goal_position1 = (unsigned int)(zero_position1 - (std::ceil(u(0)*servo_bits_rad)));
		        goal_position2 = (unsigned int)(zero_position2 + (std::ceil(u(4)*servo_bits_rad)));
		        goal_position3 = (unsigned int)(zero_position3 + (std::ceil(u(1)*servo_bits_rad)));
		        goal_position4 = (unsigned int)(zero_position4 - (std::ceil(u(5)*servo_bits_rad)));
		        goal_position5 = (unsigned int)(zero_position5 - (std::ceil(u(6)*servo_bits_rad)));
		        goal_position6 = (unsigned int)(zero_position6 - (std::ceil(u(2)*servo_bits_rad)));
		        goal_position7 = (unsigned int)(zero_position7 + (std::ceil(u(7)*servo_bits_rad)));
		        goal_position8 = (unsigned int)(zero_position8 + (std::ceil(u(3)*servo_bits_rad)));

		        
		        std::cout << msg->timestamp << " " << goal_position1 << " " << goal_position3 << " " << goal_position6 << " " << goal_position8 << " " << goal_position2 << " " << goal_position4 << " " << goal_position5 << " " << goal_position7 << std::endl;
        	});
	}
private:
	rclcpp::Subscription<px4_msgs::msg::RcChannels>::SharedPtr rc_channels_listener_;				// subscriber 1
};

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
// dynamixel start

class PubSyncWriteNode : public rclcpp::Node
{
  public:
	PubSyncWriteNode() : Node("pub_sync_write_node") {
	    RCLCPP_INFO(this->get_logger(), "Running pub sync write node");

      this->declare_parameter("qos_depth", 10);
      int8_t qos_depth = 0;
      this->get_parameter("qos_depth", qos_depth);

      const auto QOS_RKL10V =
        rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

      set_position_publisher_ =
        this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SyncSetPosition>("sync_set_position", QOS_RKL10V);
      auto timer_callback =
      [this]() -> void {
          uint8_t dxl_error = 0;
          auto position = dynamixel_sdk_custom_interfaces::msg::SyncSetPosition();
          // Position Value of X series is 4 byte data.
          position.position1 = goal_position1;
          // Write Goal Position (length : 4 bytes)
          dxl_comm_result =
          packetHandler->write4ByteTxRx(
            portHandler,
            id1,
            ADDR_GOAL_POSITION,
            goal_position1,
            &dxl_error
          );

         /* if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
          } else if (dxl_error != 0) {
            RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
          } else {
            RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]", id1, goal_position1);
          }*/
          // Position Value of X series is 4 byte data.
          position.position2 = goal_position2;
          // Write Goal Position (length : 4 bytes)
          dxl_comm_result =
          packetHandler->write4ByteTxRx(
            portHandler,
            id2,
            ADDR_GOAL_POSITION,
            goal_position2,
            &dxl_error
          );

          /*if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
          } else if (dxl_error != 0) {
            RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
          } else {
            RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]", id2, goal_position2);
          }*/
          // Position Value of X series is 4 byte data.
          position.position3 = goal_position3;
          // Write Goal Position (length : 4 bytes)
          dxl_comm_result =
          packetHandler->write4ByteTxRx(
            portHandler,
            id3,
            ADDR_GOAL_POSITION,
            goal_position3,
            &dxl_error
          );
          // Position Value of X series is 4 byte data.
          position.position4 = goal_position4;
          // Write Goal Position (length : 4 bytes)
          dxl_comm_result =
          packetHandler->write4ByteTxRx(
            portHandler,
            id4,
            ADDR_GOAL_POSITION,
            goal_position4,
            &dxl_error
          );
          // Position Value of X series is 4 byte data.
          position.position5 = goal_position5;
          // Write Goal Position (length : 4 bytes)
          dxl_comm_result =
          packetHandler->write4ByteTxRx(
            portHandler,
            id5,
            ADDR_GOAL_POSITION,
            goal_position5,
            &dxl_error
          );
          // Position Value of X series is 4 byte data.
          position.position6 = goal_position6;
          // Write Goal Position (length : 4 bytes)
          dxl_comm_result =
          packetHandler->write4ByteTxRx(
            portHandler,
            id6,
            ADDR_GOAL_POSITION,
            goal_position6,
            &dxl_error
          );
          // Position Value of X series is 4 byte data.
          position.position7 = goal_position7;
          // Write Goal Position (length : 4 bytes)
          dxl_comm_result =
          packetHandler->write4ByteTxRx(
            portHandler,
            id7,
            ADDR_GOAL_POSITION,
            goal_position7,
            &dxl_error
          );
          // Position Value of X series is 4 byte data.
          position.position8 = goal_position8;
          // Write Goal Position (length : 4 bytes)
          dxl_comm_result =
          packetHandler->write4ByteTxRx(
            portHandler,
            id8,
            ADDR_GOAL_POSITION,
            goal_position8,
            &dxl_error
          );
          this->set_position_publisher_->publish(position);
        };
        timer_ = this->create_wall_timer(1ms, timer_callback);
    }
  private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SyncSetPosition>::SharedPtr set_position_publisher_;
};

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
	    RCLCPP_ERROR(rclcpp::get_logger("pub_sync_write_node"), "Failed to set Position Control Mode.");
	  } else {
	    RCLCPP_INFO(rclcpp::get_logger("pub_sync_write_node"), "Succeeded to set Position Control Mode.");
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
	    RCLCPP_ERROR(rclcpp::get_logger("pub_sync_write_node"), "Failed to enable torque.");
	  } else {
	    RCLCPP_INFO(rclcpp::get_logger("pub_sync_write_node"), "Succeeded to enable torque.");
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
	    RCLCPP_ERROR(rclcpp::get_logger("pub_sync_write_node"), "Failed to set Profile Velocity.");
	  } else {
	    RCLCPP_INFO(rclcpp::get_logger("pub_sync_write_node"), "Succeeded to set Profile Velocity.");
	  }
}
// dynamixel end

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

int main(int argc, char *argv[])
{ 
    // dynamixel start
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Open Serial Port
    dxl_comm_result = portHandler->openPort();
    if (dxl_comm_result == false) {
      RCLCPP_ERROR(rclcpp::get_logger("pub_sync_write_node"), "Failed to open the port!");
      return -1;
    } else {
      RCLCPP_INFO(rclcpp::get_logger("pub_sync_write_node"), "Succeeded to open the port.");
    }

    // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
    dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
    if (dxl_comm_result == false) {
      RCLCPP_ERROR(rclcpp::get_logger("pub_sync_write_node"), "Failed to set the baudrate!");
      return -1;
    } else {
      RCLCPP_INFO(rclcpp::get_logger("pub_sync_write_node"), "Succeeded to set the baudrate.");
    }

    setupDynamixel(BROADCAST_ID);
    // dynamixel end
    std::cout << "Starting listener nodes..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    
    auto node1 = std::make_shared<PubSyncWriteNode>();
    auto node2 = std::make_shared<SensorIntegrator>();		

    
    exec.add_node(node1);
    exec.add_node(node2);

    exec.spin();
    rclcpp::shutdown();
    // dynamixel start
    // Disable Torque of DYNAMIXEL
    packetHandler->write1ByteTxRx(
        portHandler,
        BROADCAST_ID,
        ADDR_TORQUE_ENABLE,
        0,
        &dxl_error
    );
    // dynamixel end
    return 0;
}
