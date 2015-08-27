# The stamp of the grasp in the original video
time stamp

# The relevant grasp information
int32 obj_num
int32 sub_num
int32 grasp_num    # If this was the first grasp in a file, this index might not be zero (carryover from prevoius trial)
int32 grasp_idx    # If this was the first grasp in the file, it would belong to grasp_idx set 0
int32 extreme_num

# Processed data at the time of the stamp above
sensor_msgs/PointCloud2 cloud_image
sensor_msgs/Image rgb_image
sensor_msgs/Image depth_image
sensor_msgs/CameraInfo cam_info

# Robot joint states at this time
sensor_msgs/JointState wam_joints
sensor_msgs/JointState hand_joints
