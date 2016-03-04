#include "sensor_msgs/LaserScan.h"
#include "ros/ros.h"




class Scan_macher
{
public:

    sensor_msgs::LaserScan downlaser;
    ros::Publisher pub;
    bool new_downlaser;

    ros::NodeHandle nh;



    ros::Subscriber sub;
    ros::Subscriber sub1;

    explicit Scan_macher()
    {
        new_downlaser=false;
        ros::NodeHandle nh;
        pub=nh.advertise<sensor_msgs::LaserScan>("scan",10);
        sub = nh.subscribe("upscan", 10, &Scan_macher::up_laser,this);
        sub1 = nh.subscribe("downscan", 10,&Scan_macher:: down_laser,this);
    }





private:

    void down_laser(const sensor_msgs::LaserScan& msg)
    {
        new_downlaser=true;
        downlaser=msg;
    }




    void up_laser(const sensor_msgs::LaserScan& msg)
    {
        sensor_msgs::LaserScan scan;
        scan=downlaser;
        if(new_downlaser)
        {
            for(int i=0;i<msg.ranges.size();i++)
            {
                if(downlaser.ranges[i]<msg.ranges[i] )
                    scan.ranges.at(i)=downlaser.ranges[i];
                else
                    scan.ranges.at(i)=msg.ranges[i];
               if(downlaser.ranges[i]  < 0.02)
                    scan.ranges.at(i)=msg.ranges[i];
            }
        }
        pub.publish(scan);
    }


};








int main(int argc, char** argv)
{

    ros::init(argc, argv, "scan_macher");

    Scan_macher scanmacher;

    ros::spin();
    return 0;


}
