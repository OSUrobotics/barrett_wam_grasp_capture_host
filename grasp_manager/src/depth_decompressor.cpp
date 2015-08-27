#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

#include <string.h>

using std::string;

ros::Publisher* depth_repub;

void depth_repub_cb(const sensor_msgs::ImageConstPtr& img)
{
	ROS_INFO("Got a depth image!");
	depth_repub->publish(*img);
}

void test_cb(const sensor_msgs::CompressedImageConstPtr& img)
{
	ROS_INFO("MSG RECV.");
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "depth_republish");
	ros::NodeHandle nh, nh2;

	// Get the topics to use
	string input_topic;
	string output_topic;
	if (!ros::param::get("~input_topic", input_topic)) {
		ROS_ERROR("No input topic received for depth subscription");
		return 1;
	}

	if (!ros::param::get("~output_topic", output_topic)) {
		ROS_ERROR("No output topic specified for depth repub.");
		return 2;
	}
	ROS_INFO_STREAM("Subscribing to " << input_topic << " and republising on " << output_topic);

	image_transport::ImageTransport it(nh2);
	ROS_WARN_STREAM("After it nh.");
	image_transport::TransportHints hints("compressed", ros::TransportHints().tcpNoDelay(), nh2);
	ROS_WARN_STREAM("After transport hints.");
	ros::Publisher temp_repub = nh.advertise<sensor_msgs::Image>(output_topic, 5);
	depth_repub = &temp_repub;
	ROS_WARN_STREAM("Made it past publishing");
	image_transport::Subscriber compressed_depth_sub = it.subscribe(input_topic, 0, depth_repub_cb, hints);
	ros::Subscriber subby = nh2.subscribe(input_topic + "/compressed", 0, test_cb);

	ROS_WARN_STREAM("Made it past subscribing.");

	/*while (ros::ok()) {
		ROS_INFO_STREAM("Number of publishers: " << compressed_depth_sub.getNumPublishers());
		sleep(1);
	}*/
	ros::spin();

	return 0;
}
