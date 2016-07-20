/*  ----------------------------------------------------------------------
 *  Copyright (C) 2000-2016, Universidad de Zaragoza, SPAIN
 *
 *  Contact Addresses: Danilo Tardioli                   dantard@unizar.es
 *
 *  This is free software; you can  redistribute it and/or  modify it
 *  under the terms of the GNU General Public License  as published by the
 *  Free Software Foundation;  either  version 2, or (at  your option) any
 *  later version.
 *
 *  This software is distributed in the hope that it will be   useful, but
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

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros_profiling_msgs/packet.h"
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <yaml-cpp/yaml.h>
#include <iomanip>
#include <iostream>
#include "../../utils/Argo.h"
#include "dds.h"

int txsockfd;
int num_packets, send_back;
struct sockaddr_in serv;
char buffer[1024];
bool has_server = false;

class RXTopic{
public:
    std::string name;
    FILE * fd;
    ros::Subscriber sub;
    int count;
    int lost_rx, last_rx;
    int mode;

    RXTopic(){
        name = "topic";
        count = 0;
        lost_rx = 0;
        last_rx = 0;
        mode = -1;
    }
};

std::vector<RXTopic> vec ;


int last_rx = 0;

void chatterCallback(const ros_profiling_msgs::packet::ConstPtr& msg)
{

    double secs =ros::Time::now().toSec();
    //sprintf(buffer,"%d %f %d %d %d %f", msg->power, secs, msg->serial, (int) msg->data.size(), msg->rate, msg->header.stamp.toSec());
    sprintf(buffer,"%d %f %d %d %d %f", msg->power, secs, msg->serial, (int) msg->data.size(), msg->rate, msg->ts);
    if (msg->serial < vec[msg->power].last_rx){
        vec[msg->power].last_rx = 0;
        vec[msg->power].lost_rx = 0;
        vec[msg->power].count = 0;
    }
    if (vec[msg->power].last_rx > 0){
        vec[msg->power].lost_rx += (msg->serial - vec[msg->power].last_rx -1);
    }
    vec[msg->power].last_rx = msg->serial;

    if (has_server){
        int size = strlen(buffer) + 1;
        if (sendto(txsockfd, buffer, size, 0, (struct sockaddr *) &serv, sizeof(serv)) != size){
            perror("Unable to send packet of this size...");
        }
        vec[msg->power].count++;
    }else{
        if (vec[msg->power].count > 0){
            vec[msg->power].count += -1;
            fprintf(vec[msg->power].fd, "%s\n", buffer);
        }
    }
}

int main(int argc, char *argv[])
{

    Argo a;
    bool quiet;
    int mode = 0;
    int queue_size = 1;
    std::string config_file, server_ip;

    a.addString('f',"config", config_file, std::string(getenv("HOME")) + std::string("/.rospacket/config.yaml"), "Specify config file");
    a.addString('a',"ip", server_ip, "localhost", "Specify Server ip");
    a.addSwitch('s', "server", has_server, "Specify if server mode or standalone");
    a.addSwitch('Q', "quiet", quiet, "Do not write on screen");
    a.addInt('m', "mode", mode, 0, "Specify mode of connection (0:TCP 1:NOD 2:UDP)");
    a.addInt('q', "queue-size", queue_size, 0, "Specify size of the queue");
    a.addDependency('a','s', false);
    a.process(argc, argv);
    //a.processYAML(config_file);
    a.showValues();

    ros::init(argc, argv, "rx", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    std::cout << "Reading config file " << config_file << std::endl;
    try{

        YAML::Node config = YAML::LoadFile(config_file);
        if (YAML::Node parameter = config["server_ip"]){
            server_ip = parameter.as<std::string>();
            has_server = true;
        }

        if (YAML::Node parameter = config["mode"]){
            mode = parameter.as<int>();
        }

        if (YAML::Node parameter = config["queue"]){
            queue_size = parameter.as<int>();
        }

        if (YAML::Node topics = config["topics"]){
            for (int i = 0; i<topics.size(); i++ ){
                RXTopic rxp;
                YAML::Node topic = topics[i];
                if (YAML::Node parameter = topic["name"]){
                    rxp.name = parameter.as<std::string>();
                }                
                if (YAML::Node parameter = topic["mode"]){
                    rxp.mode = parameter.as<int>();
                }
                if (!has_server){
                    if (YAML::Node parameter = topic["count"]){
                        rxp.count = parameter.as<int>();
                    }
                    rxp.fd = fopen(rxp.name.c_str(), "w+");
                }
                vec.push_back(rxp);
            }
        }
        std::cout << "Done." << std::endl;
    }catch(std::exception e){
        std::cout << "Bad file or file not found..." << std::endl;
        exit(1);
    }

    /* FEEDBACK */

    if (has_server){
        fprintf(stderr,"*** Server mode - IP: %s\n", server_ip.c_str());
        txsockfd = socket(AF_INET,SOCK_DGRAM,0);
        serv.sin_family = AF_INET;
        serv.sin_port = htons(56561);
        serv.sin_addr.s_addr = inet_addr(server_ip.c_str());
    }else{
        fprintf(stderr,"*** Standalone mode\n");
    }
#ifdef DDS
    std::cerr << "Using DDS " <<std::endl;
#endif
    for (int i = 0; i<vec.size(); i++ ){

        int themode;
        if (vec[i].mode >= 0){
            themode = vec[i].mode;
        }else{
            themode = mode;
        }

        if (themode == 0){
            vec[i].sub = n.subscribe(vec[i].name, queue_size, chatterCallback, ros::TransportHints().tcp());
            std::cerr << "Topic "<< vec[i].name.c_str() << " is using TCP" <<std::endl;
        }else if (themode == 1){
            vec[i].sub = n.subscribe(vec[i].name, queue_size, chatterCallback, ros::TransportHints().tcpNoDelay());
            std::cerr << "Topic "<< vec[i].name.c_str() << " is using NOD" <<std::endl;
        }else if (themode == 2){
            vec[i].sub = n.subscribe(vec[i].name, queue_size, chatterCallback, ros::TransportHints().udp());
            std::cerr << "Topic "<< vec[i].name.c_str() << " is using UDP" <<std::endl;
        }else if (themode == 3){
#ifdef DDS
            ros::SubscribeQoSOptions qos;
            qos.using_best_effort_protocol= false;
//            ros::Duration d(0,20000000);
//            qos.time_filter_duration = d;
            vec[i].sub = n.subscribeWithQoS(vec[i].name, queue_size, qos, chatterCallback);
            std::cerr << "Topic "<< vec[i].name.c_str() << " is using QOS" <<std::endl;
#else
            std::cerr << "DDS UNSUPPORTED " <<std::endl;
            exit(0);
#endif
        }
    }

    int filter = 0;
    while (ros::ok()){
        ros::spinOnce();
        if (filter++ % 500 == 0){
            int sum = 0;
            for (int i = 0; i < vec.size(); i++){
                //fprintf(stderr,"%s: %5d  (%d)  ", vec[i].name.c_str(), vec[i].count, vec[i].lost_rx);
                if (!quiet) {
                    fprintf(stderr,"topic #%d: %5d  (%d)  ", i, vec[i].count, vec[i].lost_rx);
                }
                sum+=vec[i].count;
            }
            if ((!has_server) && sum == 0){
                ros::shutdown();
            }
        }
        if (!quiet){
            fprintf(stderr,"\r");
        }
        usleep(100);
    }
    fprintf(stderr,"\n");

    return 0;
}
