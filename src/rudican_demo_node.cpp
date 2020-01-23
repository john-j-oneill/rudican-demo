/// ROS
#include <ros/ros.h>

/// Messages
#include <can_msgs/Frame.h>

//publisher objects
ros::NodeHandle *nh;
ros::Publisher can_pub;

/// Globals
bool debug = true;

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

unsigned int unpack_24(const unsigned char most_significant_byte,
                       const unsigned char more_significant_byte,
                       const unsigned char least_significant_byte)
{
    unsigned int retval = (((unsigned int) most_significant_byte) << 16)
                        + (((unsigned int) more_significant_byte) << 8)
                        + (((unsigned int) least_significant_byte) << 0);
    return retval;
}


void timer_callback(const ros::TimerEvent&)
{
	can_msgs::Frame can_msg;

	//can command specifics
	can_msg.is_extended = true;
	can_msg.is_rtr = false;
	can_msg.is_error = false;
	can_msg.dlc = 8;
	//header frame doesnt matter
	can_msg.header.frame_id = "0";  // "0" for no frame.
	can_msg.header.stamp = ros::Time::now();

	can_msg.id = 0xFACE;

    /// Pack the timestamp into the message so we can measure delays or drops
    unsigned int seconds = can_msg.header.stamp.sec;
    unsigned int microseconds = can_msg.header.stamp.nsec / 1000;
    /// Only grab the 8 least significant bits of seconds,
    /// 255 seconds is plenty long to detect delays/drops/etc.
    pack_8(can_msg.data[0],seconds);
    /// Microseconds only require 10^6 or ~2^20 bits, so use 24 bits
    pack_24(can_msg.data[3],can_msg.data[2],can_msg.data[1],microseconds);
    /// Leave the remaining 32 bits empty so they can be added by the response
    pack_32(can_msg.data[7],can_msg.data[6],can_msg.data[5],can_msg.data[4],0);
    
	can_pub.publish(can_msg);
    ROS_INFO_COND(debug,"Sent FACE message, 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
                  can_msg.data[0],
                  can_msg.data[1],
                  can_msg.data[2],
                  can_msg.data[3],
                  can_msg.data[4],
                  can_msg.data[5],
                  can_msg.data[6],
                  can_msg.data[7]);

}

/// Callback when we get a new can message
void can_receive_callback(const can_msgs::Frame::ConstPtr& msg)
{

	/// This receives all message IDs. We parse through to find the desired feedback messages
	switch(msg->id) {

        /// Example message catcher
		case 0xFADE:	{
            ROS_INFO_COND(debug,"Got FADE message, 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
                          msg->data[0],
                          msg->data[1],
                          msg->data[2],
                          msg->data[3],
                          msg->data[4],
                          msg->data[5],
                          msg->data[6],
                          msg->data[7]);
            unsigned int sent_us = unpack_24(msg->data[3],msg->data[2],msg->data[1]);
            unsigned int sent_sec = (unsigned int) msg->data[0];
            unsigned int bounce_us = unpack_24(msg->data[7],msg->data[6],msg->data[5]);
            unsigned int bounce_sec = (unsigned int) msg->data[4];
            ros::Time time = ros::Time::now();
            unsigned int recv_sec = (time.sec) & 0xFF;
            unsigned int recv_us = time.nsec / 1000;
            double dt = ((double) recv_sec + ((double) recv_us / 1e6))
                      - ((double) sent_sec + ((double) sent_us / 1e6));
            ROS_INFO("Got echo, sent %03d.%06d, bounce %03d.%06d, recv %03d.%06d, dt=%0.6f",
                     sent_sec,sent_us,bounce_sec,bounce_us,recv_sec,recv_us,dt);
			break;
		}
	}
}

int main (int argc, char** argv)
{
	//// Initialize ROS
	ros::init (argc, argv, "rudican_demo_node");
	nh = new ros::NodeHandle("~");

    /// Subscribe to can messages
    ros::Subscriber sub_can = nh->subscribe("/received_messages",1,can_receive_callback);

    /// Publish can messages
	can_pub = nh->advertise<can_msgs::Frame>("/sent_messages", 50);

    /// Time based callback, 100Hz
    double dt = 1.0/100.0;
    nh->getParam("dt",dt);
    nh->getParam("debug",debug);
    ros::Timer timer = nh->createTimer(ros::Duration(dt), timer_callback);

	// Spin
	ros::spin ();
}


