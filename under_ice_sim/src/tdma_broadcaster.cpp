// A tdma broadcaster node that publishes integers over a given topic at regularly
// specified intervals. Subscribing nodes should then act upon receipt of the appropriate
// integer by performing their desired action.
//
// Laughlin Barker - August 2017 - Dynamical Systems and Control Laboratory, JHU

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <iostream>

class TDMA
{
public:
	TDMA();

protected:
	ros::NodeHandle nh;
	ros::Publisher broadcast_topic;

	std_msgs::Int16 msg;

	void timer1callback(const ros::TimerEvent& event);
	void timer2callback(const ros::TimerEvent& event);
	void timer2delaycallback(const ros::TimerEvent& event);

	ros::Timer timer1;
	ros::Timer timer2;
	ros::Timer timer2delay;

};

TDMA::TDMA()
{
	broadcast_topic = nh.advertise<std_msgs::Int16>("tdma", 10);

	//create first timer to fire off #1 eveyr 120 seconds
	timer1 = nh.createTimer(ros::Duration(120), &TDMA::timer1callback, this);

	//fire first timer now
	msg.data = 1;
	broadcast_topic.publish(msg);
	
	//use a oneshot timer to start the second timer, which will also fire every 120s
	//but offset from the first timer by some ammount of delay
	bool oneshot = true;
	timer2delay = nh.createTimer(ros::Duration(60), &TDMA::timer2delaycallback, this, oneshot);
}



void TDMA::timer1callback(const ros::TimerEvent& event)
{
	msg.data = 1;
	broadcast_topic.publish(msg);
}

void TDMA::timer2callback(const ros::TimerEvent& event)
{
	msg.data = 2;
	broadcast_topic.publish(msg);
}

void TDMA::timer2delaycallback(const ros::TimerEvent& event)
{	
	//create second timer for next cycle in 120s
	timer2 = nh.createTimer(ros::Duration(120), &TDMA::timer2callback, this);

	//fire off a signal
	msg.data = 2;
	broadcast_topic.publish(msg);
}


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "tdma_publisher");

    //create TDMABroadcaster object
    TDMA broadcaster;

    while(ros::ok())
    {
    	ros::spin();
    }

	return 0;
}

