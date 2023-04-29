
uint64 timestamp				# time since system start (microseconds)

float32 x_g				# North position in NED earth-fixed frame, (metres)
float32 y_g				# East position in NED earth-fixed frame, (metres)
float32 z_g				# Down position (negative altitude) in NED earth-fixed frame, (metres)
float32 vx_g 				# North velocity in NED earth-fixed frame, (metres/sec)
float32 vy_g				# East velocity in NED earth-fixed frame, (metres/sec)
float32 vz_g				# Down velocity in NED earth-fixed frame, (metres/sec)
float32 ax_g        			# North velocity derivative in NED earth-fixed frame, (metres/sec^2)
float32 ay_g        			# East velocity derivative in NED earth-fixed frame, (metres/sec^2)
float32 az_g        			# Down velocity derivative in NED earth-fixed frame, (metres/sec^2)

float32[3] a_b		# average value acceleration measured in the FRD body frame XYZ-axis in m/s^2 over the last accelerometer sampling period

float32[4] q_g			    # Quaternion rotation from the FRD body frame to the NED earth frame
float32[3] av_xyz_b		# Bias corrected angular velocity about the FRD body frame XYZ-axis in rad/s
float32[3] aa_xyz_b         # angular acceleration about the FRD body frame XYZ-axis in rad/s^2

float32[4] alpha
float32[4] beta
float32[4] thrust




