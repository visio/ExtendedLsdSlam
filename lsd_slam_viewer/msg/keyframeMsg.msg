int32 id
float64 time
bool isKeyframe

# camToWorld as serialization of sophus sim(3).
# may change with keyframeGraph-updates.
float32[7] camToWorld 

# Append for check Lie group convertions ****
# Full tranformation matrix
float32[16] fullMatrix

# Using for temporary solution	
float32 scale			

# Id of current key frame
int32 kfID			

# Type of ground truth source
# 0 - Blender
# 1 - Sawyer joints state
# 2 - IMU data

uint8 sourceType		

# Sawyer angels *****************************
# Angles of Sawyer joints
float32[7] jointAngles		

# Blender data ******************************
# Poisition and orientation from external sources
float32[7] blenderData		

# External position data ******************************
#  Ground truth tranformation matrix
float32[16] gtMatrix	

# Ground truth  Translation matrix
float32[3] gtTranslation
# Ground truth Quaternion
float32[4] gtQuaternion	

# Ground truth scale
float32 gtScale		
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
