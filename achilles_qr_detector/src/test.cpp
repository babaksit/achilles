#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <achilles_qr/qrPos.h>
#include <stdio.h>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<achilles_qr::qrPos>("Qr_String", 1000);

  int count = 0;
  achilles_qr::qrPos msg;


    msg.id=1;
    msg.pos_x= 0;
    msg.pos_y= 0;
    msg.context="kghcf";
    while(ros::ok())
    {
        scanf("%d",&count);
    chatter_pub.publish(msg);

    ros::spinOnce();
    }



  return 0;
}
