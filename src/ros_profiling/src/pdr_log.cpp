/*------------------------------------------------------------------------
 *---------------------                         --------------------
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
#include <ros_profiling_msgs/rssi.h>
#include <nav_msgs/Odometry.h>

#include <sstream>

static nav_msgs::Odometry odom;
static ros::Publisher message_publisher;
static int this_node, other_node, last_rssi, last_serial = 0, total = 0, received = 0;
void packet_cb(const ros_profiling_msgs::packet::ConstPtr& msg){

    //if (last_serial == msg->serial) return;

    if (last_serial!=0){
        total += (msg->serial - last_serial);
        received ++;
        fprintf(stderr,"%f %d %d %d %d %f %f\n", msg->header.stamp.toSec(),msg->serial, last_rssi-98, msg->serial - last_serial,received*100/(total+1), odom.pose.pose.position.x, odom.pose.pose.position.y);
        fprintf(stdout,"%f %d %d %d %f %f\n", msg->header.stamp.toSec(),msg->serial, last_rssi-98, msg->serial - last_serial, odom.pose.pose.position.x, odom.pose.pose.position.y);
            }
    last_serial = msg->serial;

}

void rssi_cb(const ros_profiling_msgs::rssi::ConstPtr& msg){
    last_rssi = msg->rssi;
}
void odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
    odom = *msg;
//	        fprintf(stderr,"%f %f %d\n", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->header.seq);

}

int main(int argc, char** argv) {
    int ans, delay=1;
    argo_setCommentId(argo_addIntMandatory(&this_node, STR("this-node"), 0, 1),
            STR("Specify local node"));
    argo_setCommentId(argo_addIntMandatory(&other_node, STR("other-node"), 2, 1),
            STR("Specify away node"));
	argo_doProcess(argc, argv, 0);

    ros::init(argc, argv, "pdr_log");
	ros::NodeHandle n;

    std::ostringstream oss1, oss2, oss3;
    oss1 << "/R" << this_node << "/rx/R" << other_node << "/packet";
    oss2 << "/R" << this_node << "/rssi";
    oss3 << "/R" << this_node << "/rx/R" << other_node << "/odom_loc";

    std::cerr << "Subscribing to " << oss1.str() << std::endl;
    std::cerr << "Subscribing to " << oss2.str() << std::endl;
    std::cerr << "Subscribing to " << oss3.str() << std::endl;

    ros::Subscriber sub1 = n.subscribe(oss1.str(), 10, packet_cb);
    ros::Subscriber sub2 = n.subscribe(oss2.str(), 10, rssi_cb);
    ros::Subscriber sub3 = n.subscribe(oss3.str(), 10, odom_cb);

	while(ros::ok()){
        ros::spinOnce();
        usleep(10);
	}
	return 0;

}

