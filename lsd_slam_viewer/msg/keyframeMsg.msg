int32 id
float64 time
bool isKeyframe

# camToWorld as serialization of sophus sim(3).
# may change with keyframeGraph-updates.
float32[7] camToWorld 

# Append for check Lie group convertions ****
float32[16] fullMatrix		# Full tranformation matrix
float32 scale			# Using for temporary solution

int32 kfID			# Id of current key frame

uint8 sourceType		# Type of ground truth source
				# 0 - Blender
				# 1 - Sawyer joints state
				# 2 - IMU data

# Sawyer angels *****************************

float32[7] jointAngles		# Angles of Sawyer joints

# Blender data ******************************

float32[7] blenderData		# Poisition and orientation from external sources

#********************************************

# camera parameter (fx fy cx cy), width, height
# will never change, but required for display.
float32 fx
float32 fy
float32 cx
float32 cy
uint32 height
uint32 width


# data as InputPointDense (float idepth, float idepth_var, uchar color[4]), width x height
# may be empty, in that case no associated pointcloud is ever shown.
uint8[] pointcloud
