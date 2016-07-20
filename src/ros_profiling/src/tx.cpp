#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros_profiling_msgs/packet.h"
#include <arpa/inet.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <thread>
#include "../../utils/Argo.h"
#include "dds.h"

#include <sstream>
#include <yaml-cpp/yaml.h>
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */


int rx_count = 0;
bool server, dry;

class TXTopic{
public:
    std::string name, friendly;
    int period;
    int size;
    int count;
    FILE * fd;
    ros::Publisher pub;
    std::thread * th;

    TXTopic(){
        name = "topic";
        period = 1000;
        size = 1024;
        count  = 0;
    }
};

std::vector<TXTopic> vec;

void tx_loop(int i){
    int count = 0;
    TXTopic & txp = vec[i];
    ros_profiling_msgs::packet msg;

    msg.data.resize(txp.size);
    for (int i = 0; i< txp.size; i++){
        msg.data[i] = i;
    }
    msg.power = i;
    msg.rate = txp.period;

    while(ros::ok()){
        msg.serial = count ++;
        msg.header.stamp = ros::Time::now();
        msg.ts = ros::Time::now().toSec();
        txp.pub.publish(msg);
        usleep(txp.period*1000);
        if (!server){
            txp.count += 1;
        }
    }
}

void rx_loop(){
    int rxsockfd;
    struct sockaddr_in rxservaddr, client;

    rxsockfd= socket(AF_INET, SOCK_DGRAM, 0);
    bzero(&rxservaddr, sizeof(rxservaddr));
    rxservaddr.sin_family = AF_INET;
    rxservaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    rxservaddr.sin_port = htons(56561);
    int res = bind(rxsockfd, (struct sockaddr *) &rxservaddr, sizeof(rxservaddr));
    if (res != 0) {
        fprintf(stderr, "*** ABORTING *** Bind error: do IP address and node-id are coherent in this machine?\n");
        exit(0);
    }

    char buffer[1024], aux[25];
    socklen_t l = sizeof(client);
    while (ros::ok()){

        recvfrom(rxsockfd,buffer,1024,0,(struct sockaddr *)&client,&l);
        sprintf(aux, "%.*s", 2, buffer);
        int flow = atoi(aux);

        if (server){
            if (vec[flow].count > 0){
                vec[flow].count += -1;
                if (!dry){
                    fprintf(vec[flow].fd,"%s %f\n", buffer, ros::Time::now().toSec());
                }
            }
        }
    }
}


int main(int argc, char **argv)
{

    Argo a;
    int queue_size, p[3], s[3], c[3], disable, prio = 0;
    std::string config_file;
    std::string prefix;
    unsigned short o;

    a.addString('f',"config", config_file, std::string(getenv("HOME")) + std::string("/.rospacket/config.yaml"), "Specify config file");
    a.addString('p',"prefix", prefix, "","Specify prefix of output files name");
    a.addInt('q', "queue-size",queue_size, 1, "Specify size of the queue");

    a.addInt('a', "p1", p[0], -1, "Specify period for 1st flow in ms (override config file)");
    a.addInt('b', "p2", p[1], -1, "Specify period for 1st flow in ms (override config file)");
    a.addInt('c', "p3", p[2], -1, "Specify period for 1st flow in ms (override config file)");

    a.addInt('h',"s1", s[0],  -1,"Specify size for 1st flow in ms (override config file)");
    a.addInt('i',"s2", s[1],  -1,"Specify size for 1st flow in ms (override config file)");
    a.addInt('j',"s3", s[2],  -1,"Specify size for 1st flow in ms (override config file)");

    a.addInt('k', "c1", c[0], -1, "Specify count for 1st flow in ms (override config file)");
    a.addInt('l', "c2", c[1], -1, "Specify count for 1st flow in ms (override config file)");
    a.addInt('m', "c3", c[2], -1, "Specify count for 1st flow in ms (override config file)");

    a.addInt('d', "disable", disable, -1, "Disable flow number (override config file)");
    a.addSwitch('s', "server",server, "Server mode");
    a.addSwitch('n', "dry", dry, "Dry run");
    a.process(argc, argv);

    ros::init(argc, argv, "tx", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    std::cout << "Reading config file " << config_file << std::endl;
    try{

        YAML::Node config = YAML::LoadFile(config_file);

        if (YAML::Node parameter = config["server"]){
            server = parameter.as<bool>();
        }

        int queue_size = 1;
        if (YAML::Node parameter = config["queue"]){
            queue_size = parameter.as<int>();
        }

        if (YAML::Node topics = config["topics"]){
            for (int i = 0; i<topics.size(); i++ ){
                YAML::Node topic = topics[i];

                if (YAML::Node enabled = topic["enabled"]){
                    if (!enabled.as<bool>()){
                        continue;
                    }
                }
                if (disable == i+1){
                    continue;
                }

                TXTopic txp;
                if (YAML::Node parameter = topic["period"]){
                    txp.period = parameter.as<int>();
                }
                if (YAML::Node parameter = topic["name"]){
                    txp.name = parameter.as<std::string>();
                    txp.friendly = txp.name;
                }
                if (YAML::Node parameter = topic["friendly"]){
                    txp.friendly = parameter.as<std::string>();
                }
                if (YAML::Node parameter = topic["size"]){
                    txp.size = parameter.as<int>();
                }
                if (YAML::Node parameter = topic["priority"]){
                    prio = parameter.as<int>();
                }
                if (server) {
                    std::string filename = (prefix + txp.friendly + ".dat");
                    std::replace(filename.begin(), filename.end(), '/', '_');

                    if (!dry){
                        txp.fd = fopen(filename.c_str(), "w+");
                        if (txp.fd == 0){
                            fprintf(stderr,"Unable to open file %s for writing, exiting...\n", filename.c_str());
                            exit(1);
                        }
                    }
                    if (YAML::Node parameter = topic["count"]){
                        txp.count = parameter.as<int>();
                    }
                }
                if (prio == 0){
                    txp.pub = n.advertise<ros_profiling_msgs::packet>(txp.name, queue_size);
                     std::cout << "Prio 0" << std::endl;
                }
#ifdef DDS
                else if(prio == 1){
                    txp.pub = n.advertiseWithQoS<ros_profiling_msgs::packet>(txp.name, queue_size, ros::Low);
                     std::cout << "Prio 1" << std::endl;
                }else{
                    txp.pub = n.advertiseWithQoS<ros_profiling_msgs::packet>(txp.name, queue_size, ros::High);
                     std::cout << "Prio 2" << std::endl;
                }
#else
                else{
                    std::cerr << "Priorities UNSUPPORTED without DDS support" <<std::endl;
                    exit(0);
                }
#endif
                vec.push_back(txp);
            }
        }
        std::cout << "Done." << std::endl;
    }catch(std::exception e){
        std::cout << "Bad file or file not found..." << std::endl;
        exit(1);
    }

#ifdef DDS
    std::cerr << "Using DDS " <<std::endl;
#endif

    /*OVERRIDE*/
    int cnt = vec.size() > 3 ? 3 : vec.size();
    for (int i = 0; i< cnt; i++){
        if (p[i]>0){
            vec[i].period = p[i];
        }
    }
    for (int i = 0; i< cnt; i++){
        if (s[i]>0){
            vec[i].size = s[i];
        }
    }
    for (int i = 0; i< cnt; i++){
        if (c[i]>0){
            vec[i].count = c[i];
        }
    }

    std::thread * rx_th;
    if (server){
        rx_th = new std::thread(rx_loop);
        fprintf(stderr,"*** Server mode\n");
    }else{
        fprintf(stderr,"*** Standalone mode\n");
    }

    for (int i = 0; i<vec.size(); i++ ){
        vec[i].th = new std::thread(tx_loop,i);
    }

    int filter = 0;
    while (ros::ok()){
        ros::spinOnce();
        if (filter++ % 500 == 0){
            int sum = 0;
            for (int i = 0; i < vec.size(); i++){
                fprintf(stderr,"%s: %5d (%dms, %dB) ", vec[i].friendly.c_str(), vec[i].count, vec[i].period, vec[i].size);
                sum+=vec[i].count;
            }
            if (server && sum == 0){
                ros::shutdown();
            }
        }
        fprintf(stderr,"\r");
        usleep(100);
    }
    fprintf(stderr,"\n");

    if (!dry){
        for (int i = 0; i< vec.size(); i++){
            fclose(vec[i].fd);
        }
    }

    return 0;
}
