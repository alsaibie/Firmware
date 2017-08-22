# Optical pose in NED body frame in SI units.
# @see http://en.wikipedia.org/wiki/International_System_of_Units
 
uint8 sensor_id			# id of the sensor emitting the pose value
float32 Q1	# Quaternion 1 of Pose
float32 Q2	# Quaternion 2 of Pose
float32 Q3	# Quaternion 3 of Pose
float32 Q4	# Quaternion 4 of Pose
float32 ecx	# Eccentricity in plane - x
float32 ecy	# Eccentricity in plane - y
float32 gyro_x_rate_integral	# accumulated gyro value in radians around x axis
float32 gyro_y_rate_integral	# accumulated gyro value in radians around y axis
float32 gyro_z_rate_integral	# accumulated gyro value in radians around z axis
float32 headon_distance_m		# Distance to Head On Object
uint32 integration_timespan	# accumulation timespan in microseconds
uint32 time_since_last_sonar_update	# time since last sonar update in microseconds
uint16 frame_count_since_last_readout	# number of accumulated frames in timespan
int16 gyro_temperature	# Temperature * 100 in centi-degrees Celsius
uint8 quality	# Average of quality of accumulated frames, 0: bad quality, 255: maximum quality

