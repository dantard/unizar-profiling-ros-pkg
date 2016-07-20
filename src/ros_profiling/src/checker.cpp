#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros_profiling_msgs/packet.h"
#include "ros_profiling_msgs/RFframe.h"
#include <std_msgs/Int32.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <yaml-cpp/yaml.h>
#include <netpacket/packet.h>
#include <net/ethernet.h>
#include <asm/types.h>
#include <time.h>
#include <netinet/if_ether.h>
#include <linux/if.h>
#include <sys/ioctl.h>
#include <topic_tools/shape_shifter.h>
#include <boost/bind/bind.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <curses.h>
#include <ros/master.h>
#include <ros/transport_hints.h>

struct Unit{
    std::string name;
    std::string topic;
    int valid, failures;
    double ts, ts2, freq;
    ros::Subscriber sub;
    ros::Publisher pub;
    int id, div;
    double ticks[10];
    Unit(){
        freq = 0;
        div = 2;
        ts2 = 0;
    }
    void tick(){
        ts = ros::Time::now().toSec();
        if (id++ % div == 0){
            freq = double(div)/(ts - ts2);
            ts2 = ts;
            div = freq > 10? 20: div;
            div = freq > 50? 100: div;
        }
    }
    double getFreq(){
        return freq;
    }
};

struct ByStatus{
    bool operator () (Unit  i,Unit  j) {
        return (i.valid<j.valid);
    }
};

struct ByName{
    bool operator () (Unit  i,Unit  j) {
        return (i.name.compare(j.name)<0);
    }
};

int slave = 0;
std::vector<Unit> vec;

void callback(const topic_tools::ShapeShifter::ConstPtr& msg, int i) {
    if (slave){
        std_msgs::Int32 msg;
        vec[i].pub.publish(msg);
        return;
    }
    vec[i].tick();
}

void dynamic_subscribe(ros::NodeHandle & n){
    ros::master::V_TopicInfo vti;
    ros::master::getTopics(vti);

    for (int i = 0; i<vti.size(); i++){
        ros::master::TopicInfo ti = vti.at(i);
        if (int(ti.name.find("_alive"))<0){
            continue;
        }

        /* Already added? */
        bool discard = false;
        for (int j = 0; j<vec.size(); j++){
            std::cerr << int(ti.name.find("_alive")) << " " << ti.name << " " << vec[j].topic << std::endl;
            if (ti.name.compare(vec[j].topic) == 0){
                discard = true;
                break;
            }
        }
        if (discard){
            continue;
        }

        Unit u;
        u.name =  ti.name;
        int name_begin = u.name.find_last_of("/");
        int name_end = u.name.find("_alive");
        u.name = u.name.substr(name_begin+1, name_end-name_begin-1);
        u.topic = ti.name;
        u.failures = 0;
        u.sub = n.subscribe<topic_tools::ShapeShifter>(u.topic, 1000, boost::bind(callback, _1, vec.size()));
        vec.push_back(u);
    }
}


int main(int argc, char **argv){
    ros::init(argc, argv, "checker", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    std::ostringstream oss1;

    oss1 << getenv("HOME") << "/.rospacket/checker.yaml";

    std::cout << "Reading config file " << oss1.str() << "..." << std::endl;
    int style = 1, delay = 100;
    std::string sort = "status";

    try{
        YAML::Node config = YAML::LoadFile(oss1.str());
        if (YAML::Node parameter = config["topics"]){
            for (int i = 0; i<parameter.size(); i++){
                Unit u;
                YAML::Node unit = parameter[i];
                u.name =  unit["name"].as<std::string>();
                u.topic = unit["topic"].as<std::string>();
                u.failures = 0;
                vec.push_back(u);
            }
        }
        if (YAML::Node parameter = config["style"]){
            style = parameter.as<int>();
        }
        if (YAML::Node parameter = config["delay"]){
            delay = parameter.as<int>();
        }
        if (YAML::Node parameter = config["slave"]){
            slave = parameter.as<bool>();
        }
        if (YAML::Node parameter = config["sort"]){
            sort = parameter.as<std::string>();
        }
        std::cout << "Done." << std::endl;
    }catch(YAML::ParserException &e){
        std::cout << "Bad file..."<< std::endl;
        exit(0);
    }catch(std::exception){
        std::cout << "No configuration file...done." << std::endl;
        sleep(2);
    }

    for (int i = 0; i<vec.size(); i++){
        vec[i].sub = n.subscribe<topic_tools::ShapeShifter>(vec[i].topic, 1000, boost::bind(callback, _1, i));
    }

    if (slave){
        for (int i = 0; i<vec.size(); i++){
            vec[i].pub = n.advertise<std_msgs::Int32>(vec[i].topic + "_alive",1);
        }
        ros::spin();
        exit(0);
    }

    WINDOW * mainwin;


    /*  Initialize ncurses  */

    if ( (mainwin = initscr()) == NULL ) {
        fprintf(stderr, "Error initialising ncurses.\n");
        exit(EXIT_FAILURE);
    }
    cbreak();
    nodelay(mainwin, TRUE);
    noecho();

    if (has_colors()){
        start_color();
        if (style == 0){
            init_pair(1,  COLOR_RED,     COLOR_BLACK);
            init_pair(2,  COLOR_GREEN,     COLOR_BLACK);
            init_pair(3,  COLOR_WHITE,     COLOR_BLACK);
        }else if(style == 1){
            init_pair(1,  COLOR_WHITE,     COLOR_RED);
            init_pair(2,  COLOR_WHITE,     COLOR_GREEN);
            init_pair(3,  COLOR_WHITE,     COLOR_BLACK);
        }
    }

    char buf[256];
    double last = 0.0;
    while (ros::ok()){

        double now = ros::Time::now().toSec();
        if (now - last > 1.0){

            dynamic_subscribe(n);

            for (int i = 0; i<vec.size(); i++){
                if (now - vec[i].ts < 2){
                    vec[i].valid = 1;
                }else{
                    vec[i].failures ++;
                    vec[i].valid = 0;
                }
            }
            last = now;

            clear();
            color_set(3,NULL);
            sprintf(buf, "[ id] STATUS RATE  FAILS  NAME             TOPIC\n---------------------------------------------------------------------------\n");
            waddstr(mainwin,buf);

            std::vector<Unit> gui = vec;
            if (sort.compare("status") == 0){
                ByStatus bs;
                std::sort(gui.begin(), gui.end(), bs);
            }else if (sort.compare("name") == 0){
                ByName bn;
                std::sort(gui.begin(), gui.end(), bn);
            }

            for (int i = 0; i<gui.size(); i++){
                float rate = gui[i].getFreq();
                if (gui[i].valid){
                    color_set(2,NULL);
                    if (rate < 100){
                        sprintf(buf, "[%3d] OK    %5.1f  %5d  %-16s %-32s\n",i, rate, gui[i].failures, gui[i].name.c_str(), gui[i].topic.c_str());
                    }else{
                        sprintf(buf, "[%3d] OK     >100  %5d  %-16s %-32s\n",i,  gui[i].failures, gui[i].name.c_str(), gui[i].topic.c_str());
                    }

                    waddstr(mainwin,buf);
                }else{
                    sprintf(buf, "[%3d] ERR   %5.1f  %5d  %-16s %-32s\n",i, 1.0/0.0 ,gui[i].failures, gui[i].name.c_str(), gui[i].topic.c_str());
                    color_set(1,NULL);
                    waddstr(mainwin,buf);
                }
            }
            refresh();
        }

        char c = getch();
        if (c=='r'){
            for (int i= 0; i< vec.size(); i++){
                vec[i].failures = 0;
            }
        }

        usleep(delay*1000);
        ros::spinOnce();
    }
    delwin(mainwin);
    endwin();
    refresh();

    return 0;
}
