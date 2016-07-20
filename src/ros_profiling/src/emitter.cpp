#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros_profiling_msgs/packet.h"
#include "ros_profiling_msgs/RFframe.h"

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


int txsockfd, rxsockfd;

typedef struct {
    int rate;
    int size;
    int txpower;
    int serial;
    int freq;
    int node_id;
} data_t ;


//void raw_send(int dest, char * data, int len){

//    static char txb[2342];
//    static struct ethhdr * eh = (struct ethhdr *) txb;
//    static struct sockaddr_ll broadcast;
//    static unsigned char bcast_mac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
//    static unsigned char tx_mac[6];

//    memcpy((void *) eh->h_dest, (void*) bcast_mac, ETH_ALEN);
//    broadcast.sll_pkttype = PACKET_BROADCAST;


//    broadcast.sll_family = PF_PACKET;
//    broadcast.sll_protocol = htons(0x6464);
//    broadcast.sll_ifindex = txi;
//    broadcast.sll_halen = ETH_ALEN;

//    eh->h_proto = htons(0x6464);
//    memcpy((void *) eh->h_source, (void*) tx_mac, ETH_ALEN);
//    memcpy(txb + ETH_HLEN, data, len);

//    sendto(txsockfd, txb, len + ETHER_HDR_LEN, 0, (struct sockaddr*) &broadcast, sizeof(broadcast));
//}


int raw_receive(char * buf){

    int rlen = recvfrom(rxsockfd, buf, 2342, 0, 0, 0);
    struct ethhdr * eh = (struct ethhdr *) buf;

    if (eh->h_proto == 0x6969){
        rlen = rlen - ETH_HLEN;
    }else{
        rlen = -1;
    }
    return rlen;
}



static int raw_sock_init(const char * DEVICE, int protocol, int * sock, int * if_idx, unsigned char src_mac[6]) {
    int s, ifindex, i;
    struct ifreq ifr;

    s = socket(PF_PACKET, SOCK_RAW, htons(protocol));
    if (s == -1) {
        perror("socket():");
        return 1;
    }

    strncpy(ifr.ifr_name, DEVICE, IFNAMSIZ);
    if (ioctl(s, SIOCGIFINDEX, &ifr) == -1) {
        perror("SIOCGIFINDEX");
        return 1;
    }
    ifindex = ifr.ifr_ifindex;

    if (ioctl(s, SIOCGIFHWADDR, &ifr) == -1) {
        perror("SIOCGIFINDEX");
        return 1;
    }

    for (i = 0; i < 6; i++) {
        src_mac[i] = ifr.ifr_hwaddr.sa_data[i];
    }

    /* bind socket to interface to receive frames ONLY from that interface */
    struct sockaddr_ll sll;
    sll.sll_family = AF_PACKET;
    sll.sll_ifindex = ifindex;
    sll.sll_protocol = htons(protocol);
    if ((bind(s, (struct sockaddr *) &sll, sizeof(sll))) == -1) {
        perror("bind: ");
        return 1;
    }

    (*sock) = s;
    (*if_idx) = ifindex;
    return 0;
}


int main(int argc, char **argv)
{


    ros::init(argc, argv, "talker", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    std::ostringstream oss1;

    oss1 << getenv("HOME") << "/.rospacket/emitter-"<< atoi(argv[1]) << ".yaml";

    std::string device = "wlan0", topic = "rfframe";
    std::cout << "Reading config file " << oss1.str() << "..." << std::endl;
    try{           
        YAML::Node config = YAML::LoadFile(oss1.str());

        if (YAML::Node parameter = config["device"]){
           device = parameter.as<std::string>();
        }
        int protocol = 0x6969;
        if (YAML::Node parameter = config["protocol"]){
           protocol = parameter.as<int>();
        }
        if (YAML::Node parameter = config["topic"]){
            topic = parameter.as<std::string>();
        }
        std::cout << "Done." << std::endl;

    }catch(std::exception e){
        std::cout << "Bad file or file not found..." << std::endl;
        exit(1);
    }

    int if_idx;
    unsigned char source_mac[6];
    raw_sock_init(device.c_str(), 0x6969, &rxsockfd, &if_idx, source_mac);

    char rxbuf[2342];
    char * raw_data = rxbuf + sizeof(struct ethhdr);
    data_t * data = (data_t *) raw_data;

    n.advertise<ros_profiling_msgs::RFframe>(topic,1);

    while (ros::ok()){
        int res = raw_receive(rxbuf);
        if (res < 0){
            std::cerr << "Received failed: " << res << std::endl;
        }
        std::cerr  <<int(data->node_id) << "  "<< int(raw_data[0]) << std::endl;
    }

    return 0;
}
