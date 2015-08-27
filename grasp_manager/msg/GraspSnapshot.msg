# The stamp of the grasp in the original video
time stamp

# The relevant grasp information
int32 obj_num
int32 sub_num
int32 grasp_num
int32 extreme_num

# Processed data at the time of the stamp above
sensor_msgs/PointCloud2 cloud_image
sensor_msgs/Image rgb_image
sensor_msgs/Image depth_image
sensor_msgs/CameraInfo cam_info
