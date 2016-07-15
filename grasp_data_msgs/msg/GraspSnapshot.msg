# The stamp of the grasp in the original video
time stamp

# The relevant grasp information
int32 obj_num
int32 sub_num
int32 grasp_num   
int32 grasp_idx    
bool is_optimal
int32 extreme_num
int32 optimal_num

string task

# Processed data from kinect at the time of the stamp above
sensor_msgs/PointCloud2 cloud_image		# May be empty since kinect depth stream was intermittently down
sensor_msgs/Image rgb_image
sensor_msgs/Image depth_image
sensor_msgs/CameraInfo cam_info

# Processed data from realsense cameras
sensor_msgs/PointCloud2 realsense_feed1
sensor_msgs/PointCloud2 realsense_feed2

# Robot joint states at this time
sensor_msgs/JointState wam_joints
sensor_msgs/JointState hand_joints
