/*------------------------------------------------------------------------
 *---------------------           ros_aceous              --------------------
 *------------------------------------------------------------------------
 *                                                         V0.1B  15/09/11
 *
 *
 *  File: ./src/main.cpp
 *  Authors: Danilo Tardioli
 *  ----------------------------------------------------------------------
 *  Copyright (C) 2000-2011, Universidad de Zaragoza, SPAIN
 *
 *  Contact Addresses: Danilo Tardioli                   dantard@unizar.es
 *
 *  RT-WMP is free software; you can  redistribute it and/or  modify it
 *  under the terms of the GNU General Public License  as published by the
 *  Free Software Foundation;  either  version 2, or (at  your option) any
 *  later version.
 *
 *  RT-WMP  is distributed  in the  hope  that  it will be   useful, but
 *  WITHOUT  ANY  WARRANTY;     without  even the   implied   warranty  of
 *  MERCHANTABILITY  or  FITNESS FOR A  PARTICULAR PURPOSE.    See the GNU
 *  General Public License for more details.
 *
 *  You should have received  a  copy of  the  GNU General Public  License
 *  distributed with RT-WMP;  see file COPYING.   If not,  write to the
 *  Free Software  Foundation,  59 Temple Place  -  Suite 330,  Boston, MA
 *  02111-1307, USA.
 *
 *  As a  special exception, if you  link this  unit  with other  files to
 *  produce an   executable,   this unit  does  not  by  itself cause  the
 *  resulting executable to be covered by the  GNU General Public License.
 *  This exception does  not however invalidate  any other reasons why the
 *  executable file might be covered by the GNU Public License.
 *
 *----------------------------------------------------------------------*/

#include <ros/ros.h>
#include <sstream>
#include <boost/thread.hpp>
#include <vector>
#include <ostream>
#include "argon.h"
#include <ros_profiling_msgs/packet.h>


static ros::Publisher message_publisher;
static int delay, size;

int main(int argc, char** argv) {
	int ans, delay, max, power, rate;
	argo_setCommentId(argo_addInt(&delay, STR("delay"), 50, 1),
			STR("Specify packet delay"));
	argo_setCommentId(argo_addInt(&size, STR("size"), 10, 1),
			STR("Specify packet size"));
	argo_setCommentId(argo_addInt(&max, STR("max-packets"), -1, 1),
			STR("Specify maximum number of packets"));
	argo_setCommentId(argo_addInt(&power, STR("power"), -1, 1),
			STR("Specify power"));
	argo_setCommentId(argo_addInt(&rate, STR("rate"), -1, 1),
			STR("Specify rate"));


	argo_doProcess(argc, argv, 0);

	ros::init(argc, argv, "pdr_pub");
	ros::NodeHandle n;

    message_publisher = n.advertise<ros_profiling_msgs::packet>("packet",1,true);
    ros_profiling_msgs::packet pack;
	pack.serial = 0;
	pack.data.resize(size);
	pack.rate = rate;
	pack.power = power;
	pack.size = size;
	pack.num = max;

	fprintf(stderr, "Delay: %d, size: %d, max-packets: %d, power: %d, rate: %d\n", delay, size, max, power, rate);
	
	int sent = 0;
	while(ros::ok() && max){
		pack.header.stamp = ros::Time::now();
		pack.serial++;
		message_publisher.publish(pack);
		ros::spinOnce();
		usleep(delay*1000);
		if (max > 0) max --;
		fprintf(stderr, "Sent: %d\r", ++sent);
	}
	return 0;

}

