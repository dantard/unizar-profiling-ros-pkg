#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros_profiling_msgs/Controller.h"

#include <arpa/inet.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <thread>
#include <fstream>
#include <gnuplot-iostream.h>
#include <sstream>
#include <yaml-cpp/yaml.h>
#include "../../utils/Argo.h"
#include <stdio.h>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int feedback = 0;
double R = 1, L = 1, C = 1, K = 1, Ts = 0.02, base = 0;
ros::Subscriber sub;
ros::Publisher pub;
std::fstream out;
FILE * fd;
double i_k = 0, i_k_minus_1 = 0, vc_minus_1 = 0, cnt = 0, last = 0;
int queue_size = 100, size = 1000, higher_serial = 0;
bool verbose;

std::vector<boost::tuple<double, double, double, double> > data1;
std::vector<boost::tuple<double, double, double, double> > data2;
std::vector<boost::tuple<double, double, double, double> > data3;

ros_profiling_msgs::Controller last_published;
ros_profiling_msgs::Controller last_ctrl;

void command_callback(ros_profiling_msgs::Controller ctrl)
{
    if (ctrl.serial <= higher_serial){
        std::cerr << "discarding repeat " << higher_serial << " " << ctrl.serial << std::endl;
        return;
    }

    higher_serial = ctrl.serial;
    ctrl.serial ++;

    last_ctrl = ctrl;
    double now = ros::Time::now().toSec();
    static bool first = true;
    if (first){
        base = now;
        last = now-Ts;
        first = false;
    }
    double T = now-last;
    data1.push_back(boost::make_tuple(cnt*Ts, vc_minus_1, 0, 0));
    data2.push_back(boost::make_tuple(now-base, vc_minus_1, 0, 0));


    double i_k = (ctrl.u - vc_minus_1 + L*i_k_minus_1/T - T*i_k_minus_1/2*C)  /(R + L/T + T/C/2);
    vc_minus_1 = vc_minus_1 + (i_k + i_k_minus_1)/2*T/C;
    i_k_minus_1 = i_k;

    fprintf(fd, "%f %f %f\n", now, now-last, vc_minus_1);
    if (verbose){
        fprintf(stdout, "%f %f %f %d\n", now, now-last, vc_minus_1, ctrl.serial);
    }
    cnt++;

    ctrl.y = vc_minus_1;
    pub.publish(ctrl);
    last_published = ctrl;
    last = now;
    //usleep(Ts*1000000*(rand()%3));
    usleep(Ts*1000000);
    feedback = 1;
}

void output_callback(ros_profiling_msgs::Controller ctrl)
{
    ctrl.u = 1 - ctrl.y;
    pub.publish(ctrl);
}
double now(){
    return ros::Time::now().toSec();
}

double elapsed(double time){
    return now() - time;
}

void simulate(double duration){
    double now = base = ros::Time::now().toSec();
    i_k = 0, i_k_minus_1 = 0, vc_minus_1 = 0, cnt = 0, last = 0;
    while ((now - base) < duration){
        now = ros::Time::now().toSec();
        data3.push_back(boost::make_tuple(now-base, vc_minus_1, 0, 0));
        if (verbose){
            fprintf(stdout, "%f %f %f %d\n", now, 0.0, vc_minus_1, 0);
        }
        double i_k = (1 - vc_minus_1 - vc_minus_1 + L*i_k_minus_1/Ts - Ts*i_k_minus_1/2*C)  /(R + L/Ts + Ts/C/2);
        vc_minus_1 = vc_minus_1 + (i_k + i_k_minus_1)/2*Ts/C;
        i_k_minus_1 = i_k;
        usleep(Ts*1000000);
    }
}

int main(int argc, char **argv)
{

    bool sys = 0, plot = 0;
    int mode = 0;
    double duration = 5, republish = 2;
    std::string config_file, out_file;

    Argo a;
    a.addSwitch('s', "system", sys,"Act as system");
    a.addSwitch('p',"plot",plot,"Plot results");
    a.addSwitch('V', "Verbose", verbose,"Verbose");
    a.addDouble('d', "duration", duration,5.0,"Duration of the simulation");
    a.addDouble('T',"period", Ts,0.02,"Period");
    a.addDouble('r', "republish",republish, 0.0,"Republish output after this number of periods");
    a.addInt('m',"mode",mode, 0,"Connection mode (0:TCP, 1:NOD, 2:UDP)");
    a.addString('f', "config", config_file,std::string(getenv("HOME")) + std::string("/.rospacket/controller.yaml"),"Specify config file");
    a.addString('o', "out-file",out_file, "rlc.dat","Specify output file");

    a.process(argc, argv);


    ros::init(argc, argv, "controller", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    std::cerr << "Reading config file " << config_file << std::endl;
    try{

        YAML::Node config = YAML::LoadFile(config_file);

        if (YAML::Node parameter = config["T"]){
            Ts = parameter.as<double>();
        }
        if (YAML::Node parameter = config["R"]){
            R = parameter.as<double>();
        }
        if (YAML::Node parameter = config["L"]){
            L = parameter.as<double>();
        }
        if (YAML::Node parameter = config["C"]){
            C = parameter.as<double>();
        }
        if (YAML::Node parameter = config["K"]){
            K = parameter.as<double>();
        }
        if (YAML::Node parameter = config["size"]){
            size = parameter.as<int>();
        }
        if (YAML::Node parameter = config["mode"]){
            mode = parameter.as<int>();
        }
        std::cerr << "Done." << std::endl;
    }catch(YAML::ParserException &e){
        std::cerr << "Bad file..."<< std::endl;
        exit(0);
    }catch(std::exception){
        std::cerr << "No configuration file...done." << std::endl;
    }

    ros::TransportHints ti;

    if (mode == 2){
        ti = ros::TransportHints().udp();
        std::cerr << "Using UDP" << std::endl;
    }else if(mode == 1){
        ti = ros::TransportHints().tcpNoDelay();
        std::cerr << "Using NOD" << std::endl;
    }else{
        ti = ros::TransportHints().tcp();
        std::cerr << "Using TCP" << std::endl;
    }

    if (sys){
        std::cerr << "Acting as system" << std::endl;
        sub = n.subscribe("u", queue_size, command_callback, ti);
        pub = n.advertise<ros_profiling_msgs::Controller>("y", queue_size);
        fd = fopen(out_file.c_str(),"w+");
    }else{
        std::cerr << "Acting as controller" << std::endl;
        sub = n.subscribe("y", queue_size, output_callback, ti);
        pub = n.advertise<ros_profiling_msgs::Controller>("u", queue_size);
    }

    ros_profiling_msgs::Controller ctrl;
    ctrl.data.resize(size);
    ctrl.u = ctrl.y = 0;
    ctrl.serial = 1;
    double begin = now();
    bool finished = false;
    while (ros::ok()){

        if (sys){
            if (feedback){
                if (republish>1.0 && now() - last > republish*Ts){
                    std::cerr << "Got Stuck -> republishing 'y'" << std::endl;
                    pub.publish(last_published);
                    last = now();
//                    command_callback(last_ctrl);
                }
            }else{
                if (now() - begin > 1.0){
                    pub.publish(ctrl);
                    begin = now();
                }
            }
            if (elapsed(begin) > duration) {
                finished = true;
                break;
            }
        }
        ros::spinOnce();
        usleep(1000);
    }

    if (sys){
        fclose(fd);
    }

    if (!plot || !finished){
        exit(0);
    }

    std::cerr << "Simulating ("<< duration << " seconds)" << std::endl;
    simulate(duration);
    std::cerr << "Done." << std::endl;

    if (sys){
        Gnuplot gp;
        gp << "plot '-' with lines title 'Raw','-' with lines title 'Adjusted','-' with lines title 'Local'\n";
        gp.send1d(data1);
        gp.send1d(data2);
        gp.send1d(data3);
    }


    return 0;
}
