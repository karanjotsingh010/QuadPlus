/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  QUADPLUS: OVERACTUATED MULTIROTOR  >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
				
#### This file contains the source code for the Sensor Integrator

# OUTPUT:: 12 STATES (SENSORS)
	   Inertial frame- Current Velocities (vx,vy,vz),
	   		    Current Accelerations (ax,ay,az).
	   Body frame- Current Angular Rates (phi_dot, theta_dot, psi_dot).
	   Current Attitude- Roll, Pitch, Yaw (phi, theta, psi).
	   
	   RC CHANNELS
	   Inertial frame- Commanded Velocities (vel_d(0),vel_d(1),vel_d(2)),
	   		    Commanded Attitude (phi_d,theta_d),
	   		    Commanded Angular Rate (psi_dot_desired),
	   		    Kill switch (rc_ks).
	   		    
# RUNNING FREQUENCY:: 250 Hz.
	   		    
**Note- Coordinate convention used is FLU (Front-Left-Up) using Right-hand rule.

<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/

#include <memory>
#include <chrono>
#include <iostream>
#include <deque>
#include <iomanip>

using namespace std::chrono_literals;

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
//#include <px4_msgs/msg/vehicle_local_position_setpoint.hpp>
//#include <px4_msgs/msg/actuator_outputs.hpp>
//#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
//#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <px4_msgs/msg/rc_channels.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include </usr/include/armadillo>

// Sensor states feedback
arma::vec acc_0b_meas = arma::zeros(3);			    // previous time step acceleration in body frameused for delta forces calculation

arma::vec att_meas = arma::zeros(3);			    // attitude measured from FRD (body) to NED (inertial) frame: {phi,theta,psi}, PX4 msg: vehicle_attitude
arma::vec vel_iner_meas = arma::zeros(3);   	    // measured current velocities in inertial frame
double psi_act;
double timestamp;
double T1_0,T2_0,T3_0,T4_0;
arma::vec M_sp = arma::zeros(3);
arma::vec u = arma::zeros(8);					        // absolute actuator output

// RC channel input
double rc_throttle = 0.0;
double ks, rc_ks;				                    // kill switch
arma::vec vel_d = arma::zeros(3);   	            // commanded velocities
double phi_d = 0.0, theta_d = 0.0;	                // commanded angles and yaw rate

// Transforms and commands
arma::mat R = arma::zeros(3,3);				        // inertial to body frame transformation matrix
arma::vec G = arma::zeros(3);					    // gravitational forces vector in body frame
arma::vec Fb_des = arma::zeros(3);				    // desired forces from velocity commands (RC) in body frame

arma::vec cmd_dd = arma::zeros(3);				    // command velocity vector
arma::vec del_F_M_b = arma::zeros(6);				// incremental forces and moments in body frame
arma::mat b_inv = arma::zeros(12,3);				// desired forces (Fb_des) to propeller forces

// PI parameters
//Velocity control gains		
double XY_KP = 1.3;
double XY_KI = 0.0;
double VEL_KP_factor = 0.0, VEL_KI_factor = 0.0;

arma::vec POS_KP_GAIN = {XY_KP,XY_KP,3.0}; 			// Velocity control: Proportional Gains
arma::vec POS_KI_GAIN = {XY_KI,XY_KI,0.4}; 			// Velocity control: Integral Gains			
//arma::vec I_SAT = {0.3,0.3,0.1};				    // Integrator limit

//Limits
arma::vec POS_V_MIN = {-6.0,-6.0,-1.0}; 			// NORMALIZATION: Velocity controller input: min velocity limits
arma::vec POS_V_MAX = {6.0,6.0,3.0}; 				// NORMALIZATION: Velocity controller input: max velocity limits
arma::vec POS_A_MIN = {-1.0,-1.0,-1.0}; 			// NORMALIZATION: Velocity controller output: min acceleration limits
arma::vec POS_A_MAX = {1.0,1.0,1.0}; 				// NORMALIZATION: Velocity controller output: max acceleration limits
arma::vec NORM_MAX = {1.0,1.0,1.0};				    // NORMALIZATION: Max value
arma::vec NORM_MIN = {-1.0,-1.0,-1.0};			    // NORMALIZATION: Min value
arma::vec POS_CUM_ERROR = arma::zeros(3);			// Cumulative error- velocity control integrator

// INDI variables declaration
bool sigma1 = 1;
bool sigma2 = 1;
bool sigma3 = 1;
bool sigma4 = 1;

// Default parameters
double mass = 3.5;						            // mass of the system (kg)
double gravity = 9.78;						        // gravity acceleration (m/s^2)

// declare constants
double KT = 2.5E-5;						            // coefficient of thrust
double KM = 4.0E-7;						            // coefficient of drag moment
double co_ratio = KM / KT;		
double l3 = 0.075;						            // displacement from bottom centre of rotation to top centre of rotation
double hb = -0.0066;						        // displacement of bottom centre of rotation from CG
arma::vec la = {-0.265,0.265,0.265,-0.265};			// arm length vector

// Low level declaration
arma::mat mixing_mat = arma::zeros(4,4);			// mixing matrix
arma::vec control_vec = arma::zeros(4);			    // commanded thrust vector (low level controller output)

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@    NODE: SENSOR INTEGRATOR   @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
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
                    rc_throttle = msg->channels[0];
                    vel_d(1) = -msg->channels[1]; 
                    vel_d(0) = msg->channels[2]; 
                    ks = msg->channels[14];
                   	if (ks <= 0.5) {
                   		rc_ks = 1.0;}
                   	else {
                   		rc_ks = 0.0;}
                   	
                   	if (rc_throttle >= 0.0 && rc_throttle <= 0.50) {
                   		vel_d(2) = (rc_throttle/0.50) - 1.0;}
                   	else if (rc_throttle > 0.50 && rc_throttle <= 1.0) {
                   		vel_d(2) = (rc_throttle-0.50)/0.50;}
                   	else {
                   		vel_d(2) = 0.0;}
                   	//std::cout << "vel des: " << vel_d(0) << " " << vel_d(1) << " " << vel_d(2) << std::endl;
                  
                   	phi_d = 1.0*msg->channels[4];
                   	theta_d = 1.0*msg->channels[5];  
                   	
                   	if (msg->channels[4] >= 0.0) {
                   		VEL_KP_factor = msg->channels[6]*4.0;}
                   	else {
                   		VEL_KP_factor = 0.0;}
                   	if (msg->channels[5] >= 0.0) {
                   		VEL_KI_factor = msg->channels[7]*4.0;}
                   	else {
                   		VEL_KI_factor = 0.0;}             	
        });
        	
	// subscriber 2 (~250 Hz)
		vehicle_attitude_listener_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>("fmu/vehicle_attitude/out",
		#ifdef ROS_DEFAULT_API
           	10,
		#endif
            [this](const px4_msgs::msg::VehicleAttitude::UniquePtr msg) {
           	        double q0 = msg->q[0];
           	        double q1 = msg->q[1];
            	    double q2 = msg->q[2];
            	    double q3 = msg->q[3];
            	    att_meas(0) = std::atan2(2. * (q0 * q1 + q2 * q3), 1. - 2. * (q1 * q1 + q2 * q2));
            	    att_meas(1) = -std::asin(2. * (q0 * q2 - q3 * q1)); // negative sign added to account for rotation transform from FRD (Front-Right-Down) to FLU (Front-Left-Up) frame
            	    psi_act = std::atan2(2. * (q0 * q3 + q1 * q2), 1. - 2. * (q2 * q2 + q3 * q3));; // negative sign added to account for rotation transform from FRD (Front-Right-Down) to FLU (Front-Left-Up) frame, provides correction for velocities and accelerations which are expressed in NED frame
            	    att_meas(2) = 0.0;		// yaw angle drifts a lot, controlling yaw rate instead
            	    timestamp = msg->timestamp;
            	    //std::cout << "att meas: " << att_meas(0) << " " << att_meas(1) << " " << psi_act << std::endl;
		});
        
        // subscriber 3 (~100 Hz)
		vehicle_local_pos_listener_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("fmu/vehicle_local_position/out",
		#ifdef ROS_DEFAULT_API
            	10,
		#endif
            [this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
                    vel_iner_meas(0) = ((msg->vx)*std::cos(psi_act)) + ((msg->vy)*std::sin(psi_act));
            	    vel_iner_meas(1) = ((msg->vx)*std::sin(psi_act)) - ((msg->vy)*std::cos(psi_act)); // negative sign added to account for rotation transform from FRD (Front-Right-Down) to FLU (Front-Left-Up) frame
            	    vel_iner_meas(2) = -msg->vz; // negative sign added to account for rotation transform from FRD (Front-Right-Down) to FLU (Front-Left-Up) frame
        });
        
        // subscriber 4 (~200-250 Hz)
		/*sensor_combined_listener_ = this->create_subscription<px4_msgs::msg::SensorCombined>("/fmu/sensor_combined/out",
		#ifdef ROS_DEFAULT_API
           	10,
		#endif
            [this](const px4_msgs::msg::SensorCombined::UniquePtr msg) {
           	        acc_meas(0) = msg->accelerometer_m_s2[0];   //((msg->accelerometer_m_s2[0])*std::cos(psi_act)) + ((msg->accelerometer_m_s2[1])*std::sin(psi_act));
           	        acc_meas(1) = -msg->accelerometer_m_s2[1];  //((msg->accelerometer_m_s2[0])*std::sin(psi_act)) - ((msg->accelerometer_m_s2[1])*std::cos(psi_act)); // negative sign added to account for rotation transform from FRD (Front-Right-Down) to FLU (Front-Left-Up) frame
           	        acc_meas(2) = -msg->accelerometer_m_s2[2]; // negative sign added to account for rotation transform from FRD (Front-Right-Down) to FLU (Front-Left-Up) frame
           	   
           	        //std::cout << "acc_dt: " << msg->accelerometer_integral_dt << std::endl;
		});*/
		
		// subscriber 5 (~500 Hz)
		servo_angles_listener_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/servo_angles",
		#ifdef ROS_DEFAULT_API
            	10,
		#endif
            [this](const geometry_msgs::msg::PoseArray::UniquePtr msg) {
                    u(0) = msg->poses[0].position.x;
		            u(1) = msg->poses[0].position.y;
	                u(2) = msg->poses[0].position.z;
	       
	                u(3) = msg->poses[0].orientation.x;
	                u(4) = msg->poses[0].orientation.y;
	                u(5) = msg->poses[0].orientation.z;
	    	
	                u(6) = msg->poses[1].position.x;
	                u(7) = msg->poses[1].position.y;
        });
        
        // subscriber 6 (~0 Hz)
		moment_sp_listener_ = this->create_subscription<px4_msgs::msg::VehicleTorqueSetpoint>("/fmu/vehicle_torque_setpoint/out",
		#ifdef ROS_DEFAULT_API
            	10,
		#endif
            [this](const px4_msgs::msg::VehicleTorqueSetpoint::UniquePtr msg) {
            	    M_sp(0) = msg->xyz[0];
            	    M_sp(1) = -msg->xyz[1]; 		// negative sign to transform FRD to FLU
            	    M_sp(2) = -msg->xyz[2]; 		// negative sign to transform FRD to FLU
        });
 		
        // Publisher
        
        #ifdef ROS_DEFAULT_API
		    pos_force_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/pos_forces", 10);
		#else
		    pos_force_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/pos_forces");
		#endif
	    	auto timer_callback =
		    [this]()->void {
		            pseudo_control_gen();
		    }; 
		timer_ = this->create_wall_timer(2ms, timer_callback);
	} // closing public constructor
	
	void pseudo_control_gen();
private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pos_force_publisher_;				                    // Publisher
	rclcpp::Subscription<px4_msgs::msg::RcChannels>::SharedPtr rc_channels_listener_;				                    // subscriber 1
	rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_listener_;			                // subscriber 2
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_pos_listener_;		            // subscriber 3
	//rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sensor_combined_listener_;			                // subscriber 4
	rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr servo_angles_listener_;		                        // subscriber 5
	rclcpp::Subscription<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr moment_sp_listener_;		                    // subscriber 6
};
	  
	 
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@    FUNCTION: PSEUDO CONTROL CALCULATION   @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

void SensorIntegrator::pseudo_control_gen() {

        auto allocation_vect = geometry_msgs::msg::PoseArray();
		allocation_vect.poses.resize(4);
		
		POS_KP_GAIN(0) = XY_KP * (1.0 + VEL_KP_factor);
		POS_KP_GAIN(1) = XY_KP * (1.0 + VEL_KP_factor);
		
		POS_KI_GAIN(0) = XY_KI * (1.0 + VEL_KI_factor);
		POS_KI_GAIN(1) = XY_KI * (1.0 + VEL_KI_factor);
		
    // inertial to body frame transform
		R(0,0) = std::cos(att_meas(2))*std::cos(att_meas(1));
		R(0,1) = std::sin(att_meas(2))*std::cos(att_meas(1));
		R(0,2) = -std::sin(att_meas(1));
					  
		R(1,0) = std::cos(att_meas(2))*std::sin(att_meas(1))*std::sin(att_meas(0))-std::sin(att_meas(2))*std::cos(att_meas(0));
		R(1,1) = std::sin(att_meas(2))*std::sin(att_meas(1))*std::sin(att_meas(0))+std::cos(att_meas(2))*std::cos(att_meas(0));
		R(1,2) = std::cos(att_meas(1))*std::sin(att_meas(0));
					  
		R(2,0) = std::cos(att_meas(2))*std::sin(att_meas(1))*std::cos(att_meas(0))+std::sin(att_meas(2))*std::sin(att_meas(0));
		R(2,1) = std::sin(att_meas(2))*std::sin(att_meas(1))*std::cos(att_meas(0))-std::cos(att_meas(2))*std::sin(att_meas(0));
		R(2,2) = std::cos(att_meas(1))*std::cos(att_meas(0));				

		G(0) = -std::sin(theta_d)*mass*gravity;
		G(1) = std::cos(theta_d)*std::sin(phi_d)*mass*gravity;
		G(2) = std::cos(theta_d)*std::cos(phi_d)*mass*gravity;
		
	// Clamping to maximum and minimum
		/*vel_d(0) = std::max(vel_d(0),POS_V_MIN(0));
		vel_d(0) = std::min(vel_d(0),POS_V_MAX(0));
		vel_d(1) = std::max(vel_d(1),POS_V_MIN(1));
		vel_d(1) = std::min(vel_d(1),POS_V_MAX(1));
		vel_d(2) = std::max(vel_d(2),POS_V_MIN(2));
		vel_d(2) = std::min(vel_d(2),POS_V_MAX(2));	// RC input already normalized and clamped	
		
	// Normalisation
		if(vel_d(0)>=0) {
			vel_d(0) = vel_d(0)/std::abs(POS_V_MAX(0));}		
		else {
			vel_d(0) = vel_d(0)/std::abs(POS_V_MIN(0));}
		if(vel_d(1)>=0) {
			vel_d(1) = vel_d(1)/std::abs(POS_V_MAX(1));}
		else {
			vel_d(1) = vel_d(1)/std::abs(POS_V_MIN(1));}
		if(vel_d(2)>=0) {
		vel_d(2) = vel_d(2)/std::abs(POS_V_MAX(2));}
		else {
		vel_d(2) = vel_d(2)/std::abs(POS_V_MIN(2));}*/  	// RC input already normalized and clamped	
	
		if(vel_iner_meas(0)>=0) {
			vel_iner_meas(0) = vel_iner_meas(0)/std::abs(POS_V_MAX(0));}
		else {
			vel_iner_meas(0) = vel_iner_meas(0)/std::abs(POS_V_MIN(0));}
		if(vel_iner_meas(1)>=0) {
			vel_iner_meas(1) = vel_iner_meas(1)/std::abs(POS_V_MAX(1));}
		else {
			vel_iner_meas(1) = vel_iner_meas(1)/std::abs(POS_V_MIN(1));}
		if(vel_iner_meas(2)>=0) {
			vel_iner_meas(2) = vel_iner_meas(2)/std::abs(POS_V_MAX(2));}
		else {
			vel_iner_meas(2) = vel_iner_meas(2)/std::abs(POS_V_MIN(2));}
			
	// VELOCITY CONTROLLER
		arma::vec velocity_err = vel_d - vel_iner_meas;
		arma::vec acceleration_d = velocity_err % POS_KP_GAIN + POS_CUM_ERROR % POS_KI_GAIN; //Desired acceleration (PI controller)
		POS_CUM_ERROR = POS_CUM_ERROR + velocity_err;
		POS_CUM_ERROR = arma::clamp(POS_CUM_ERROR,-0.1,0.1);
				
		//std::cout << "acceleration_d: " << acceleration_d  << std::endl;
				
	// Clamping to maximum and minimum
		acceleration_d(0) = std::max(acceleration_d(0),NORM_MIN(0));
		acceleration_d(0) = std::min(acceleration_d(0),NORM_MAX(0));
		acceleration_d(1) = std::max(acceleration_d(1),NORM_MIN(1));
		acceleration_d(1) = std::min(acceleration_d(1),NORM_MAX(1));
		acceleration_d(2) = std::max(acceleration_d(2),NORM_MIN(2));
		acceleration_d(2) = std::min(acceleration_d(2),NORM_MAX(2));
	// De-normalisation
		if(acceleration_d(0)>=0) {
			acceleration_d(0) = acceleration_d(0) * std::abs(POS_A_MAX(0));}
		else {
			acceleration_d(0) = acceleration_d(0) * std::abs(POS_A_MIN(0));}
		if(acceleration_d(1)>=0) {
			acceleration_d(1) = acceleration_d(1) * std::abs(POS_A_MAX(1));}
		else {
			acceleration_d(1) = acceleration_d(1) * std::abs(POS_A_MIN(1));}
		if(acceleration_d(2)>=0) {
			acceleration_d(2) = acceleration_d(2) * std::abs(POS_A_MAX(2));}
		else {
			acceleration_d(2) = acceleration_d(2) * std::abs(POS_A_MIN(2));}
				
	// Desired forces
		arma::vec force_d = R * acceleration_d * mass;						    // controller output
		
	// delta forces (body frame)
	    arma::vec del_F_body = force_d - (R * acc_0b_meas * mass);
	    
	// delta forces and moments
	    del_F_M_b(0) = del_F_body(0);
		del_F_M_b(1) = del_F_body(1);
		del_F_M_b(2) = del_F_body(2);
		del_F_M_b(3) = 0.0;
		del_F_M_b(4) = 0.0;
		del_F_M_b(5) = 0.0;
		
	// De-normalisation
	    if(vel_d(0)>=0){
			cmd_dd(0) = vel_d(0) * std::abs(POS_V_MAX(0));}
		else{
			cmd_dd(0) = vel_d(0) * std::abs(POS_V_MIN(0));}
		if(vel_d(1)>=0){
			cmd_dd(1) = vel_d(1) * std::abs(POS_V_MAX(1));}
		else{
			cmd_dd(1) = vel_d(1) * std::abs(POS_V_MIN(1));}
		if(vel_d(2)>=0){
			cmd_dd(2) = vel_d(2) * std::abs(POS_V_MAX(2));}
		else{
			cmd_dd(2) = vel_d(2) * std::abs(POS_V_MIN(2));}
			    
	// calculating Force desired (body frame)
        	Fb_des = R * cmd_dd + G; 			// global to body frame conversion of desired forces (from RC input or trajectory generation)
		
	// static input matrix
	  	for (int i = 0; i < 3; i++)
	  	{
		  	b_inv(i,i) = sigma1 * 0.25;
		  	b_inv(i+3,i) = sigma2 * 0.25;
		  	b_inv(i+6,i) = sigma3 * 0.25;
		  	b_inv(i+9,i) = sigma4 * 0.25;
	  	}
		
	// desired body frame to propeller frame pseudo control: Fb_des to F_P)
		arma::vec F_P = b_inv * Fb_des;
		
	// individual propeller thrust estimation
		arma::vec ctrl_des = {G(2),M_sp(0),M_sp(1),M_sp(2)};
		// determine cos and sin of output servo angles
		double out_c_a0 = std::cos(u(0));
		double out_c_a1 = std::cos(u(1));
		double out_c_a2 = std::cos(u(2));
		double out_c_a3 = std::cos(u(3));
		double out_s_a0 = std::sin(u(0));
		double out_s_a1 = std::sin(u(1));
		double out_s_a2 = std::sin(u(2));
		double out_s_a3 = std::sin(u(3));

		double out_c_b0 = std::cos(u(4));
		double out_c_b1 = std::cos(u(5));
		double out_c_b2 = std::cos(u(6));
		double out_c_b3 = std::cos(u(7));
		double out_s_b0 = std::sin(u(4));
		double out_s_b1 = std::sin(u(5));
		double out_s_b2 = std::sin(u(6));
		double out_s_b3 = std::sin(u(7));
			
	// mixing matrix for control allocation
		mixing_mat(0,0) = out_c_a0*out_c_b0;
		mixing_mat(0,1) = out_c_a1*out_c_b1;
		mixing_mat(0,2) = out_c_a2*out_c_b2;
		mixing_mat(0,3) = out_c_a3*out_c_b3;

		mixing_mat(1,0) = out_c_b0*(hb*out_s_a0 - la(1)*out_c_a0) - co_ratio*out_s_b0;
		mixing_mat(1,1) = out_c_b1*(hb*out_s_a1 + la(1)*out_c_a1) - co_ratio*out_s_b1;
		mixing_mat(1,2) = out_s_a2*(l3*out_c_b2 + hb) + co_ratio*out_s_b2*out_c_a2;
		mixing_mat(1,3) = out_s_a3*(l3*out_c_b3 + hb) + co_ratio*out_s_b3*out_c_a3;
		
		mixing_mat(2,0) = out_s_b0*(l3*out_c_a0 + hb) + co_ratio*out_s_a0*out_c_b0;
		mixing_mat(2,1) = out_s_b1*(l3*out_c_a1 + hb) + co_ratio*out_s_a1*out_c_b1;
		mixing_mat(2,2) = out_c_a2*(hb*out_s_b2 - la(1)*out_c_b2) - co_ratio*out_s_a2;
		mixing_mat(2,3) = out_c_a3*(hb*out_s_b3 + la(1)*out_c_b3) - co_ratio*out_s_a3;

		mixing_mat(3,0) = -co_ratio*out_c_a0*out_c_b0;
		mixing_mat(3,1) = -co_ratio*out_c_a1*out_c_b1;
		mixing_mat(3,2) = co_ratio*out_c_a2*out_c_b2;
		mixing_mat(3,3) = co_ratio*out_c_a3*out_c_b3;
		
	// finding thrust output from mixing matrix and force, moments desired 
		control_vec = arma::pinv(mixing_mat) * ctrl_des;
		
	// forces and moments output (combined controller output)   
		allocation_vect.poses[0].position.x = del_F_M_b(0);
	    allocation_vect.poses[0].position.y = del_F_M_b(1);
	    allocation_vect.poses[0].position.z = del_F_M_b(2);
	       
	    allocation_vect.poses[0].orientation.x = del_F_M_b(3);
	    allocation_vect.poses[0].orientation.y = del_F_M_b(4);
	    allocation_vect.poses[0].orientation.z = del_F_M_b(5);
	    	
	    allocation_vect.poses[1].position.x = F_P(0);
	    allocation_vect.poses[1].position.y = F_P(1);
	    allocation_vect.poses[1].position.z = F_P(2);
	    	
	    allocation_vect.poses[1].orientation.x = F_P(3);
	    allocation_vect.poses[1].orientation.y = F_P(4);
	    allocation_vect.poses[1].orientation.z = F_P(5);
	    	
	    allocation_vect.poses[2].position.x = F_P(6);
	    allocation_vect.poses[2].position.y = F_P(7);
	    allocation_vect.poses[2].position.z = F_P(8);
	    	
	    allocation_vect.poses[2].orientation.x = F_P(9);
	    allocation_vect.poses[2].orientation.y = F_P(10);
	    allocation_vect.poses[2].orientation.z = F_P(11);
	    
	    allocation_vect.poses[3].position.x = control_vec(0);
	    allocation_vect.poses[3].position.y = control_vec(1);
	    allocation_vect.poses[3].position.z = control_vec(2);
	    	
	    allocation_vect.poses[3].orientation.x = control_vec(3);
	    allocation_vect.poses[3].orientation.y = rc_ks;
	    allocation_vect.poses[3].orientation.z = timestamp;
	    
	    acc_0b_meas = acceleration_d;
	    
        pos_force_publisher_->publish(allocation_vect);
}


//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

int main(int argc, char *argv[])
{ 
    std::cout << "Starting sensor node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    // spin the node   
    rclcpp::spin(std::make_shared<SensorIntegrator>());
    rclcpp::shutdown();
    return 0;
}
