/// ROS
#include <ros/ros.h>

/// Messages
#include <can_msgs/Frame.h>

//publisher objects
ros::NodeHandle *nh;
ros::Publisher can_pub;

void pack_8(unsigned char &least_significant_byte, 
            unsigned int   value_in)
{
    least_significant_byte = (value_in & 0xFF);
}

void pack_16(unsigned char &most_significant_byte,
             unsigned char &least_significant_byte, 
             unsigned int   value_in)
{
    most_significant_byte = ((value_in >> 8) & 0xFF);
    least_significant_byte = (value_in & 0xFF);
}

void pack_24(unsigned char &most_significant_byte,
             unsigned char &more_significant_byte, 
             unsigned char &least_significant_byte, 
             unsigned int   value_in)
{
    most_significant_byte = ((value_in >> 16) & 0xFF);
    more_significant_byte = ((value_in >> 8) & 0xFF);
    least_significant_byte = (value_in & 0xFF);
}

void pack_32(unsigned char &most_significant_byte,
             unsigned char &even_more_significant_byte, 
             unsigned char &more_significant_byte, 
             unsigned char &least_significant_byte, 
             unsigned int   value_in)
{
    most_significant_byte = ((value_in >> 24) & 0xFF);
    even_more_significant_byte = ((value_in >> 16) & 0xFF);
    more_significant_byte = ((value_in >> 8) & 0xFF);
    least_significant_byte = (value_in & 0xFF);
}

void callback(const ros::TimerEvent&)
{
	can_msgs::Frame can_msg;

	//can command specifics
	can_msg.is_extended = true;
	can_msg.is_rtr = false;
	can_msg.is_error = false;
	//header frame doesnt matter
	can_msg.header.frame_id = "0";  // "0" for no frame.
	can_msg.header.stamp = ros::Time::now();

	can_msg.id = 0xFADE;

    /// Pack the timestamp into the message so we can measure delays or 
    unsigned int seconds = can_msg.header.stamp.sec;
    unsigned int microseconds = can_msg.header.stamp.nsec / 1000;
	pack_16(can_msg.data[1],can_msg.data[0],velocity);
	pack_24(can_msg.data[3],can_msg.data[2],can_msg.data[1],velocity);
    
	can_pub.publish(can_msg);

}

/// Callback when we get a new can message
void can_receive_callback(const can_msgs::Frame::ConstPtr& msg)
{

	/// This receives all message IDs. We parse through to find the desired feedback messages
	switch(msg->id) {

        /// Example message catcher
		case 0xFADE:	{
            ROS_INFO("Got FADE message, 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
                     msg->data[0],
                     msg->data[1],
                     msg->data[2],
                     msg->data[3],
                     msg->data[4],
                     msg->data[5],
                     msg->data[6],
                     msg->data[7]);

			break;
		}
	}
}

int main (int argc, char** argv)
{
	//// Initialize ROS
	ros::init (argc, argv, "rudican-demo_node");
	nh = new ros::NodeHandle("~");

    /// Subscribe to can messages
    ros::Subscriber sub_can = nh->subscribe("/received_messages",1,can_receive_callback);

    /// Publish can messages
	can_pub = nh->advertise<can_msgs::Frame>("/sent_messages", 50);

    /// Time based callback, 100Hz
    ros::Timer timer = nh.createTimer(ros::Duration(0.01), timerCallback);

	// Spin
	ros::spin ();
}


