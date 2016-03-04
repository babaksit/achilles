#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include "achilles_qr_detector/qrPos.h"
#include "QList"

using namespace std;

class QR_filtering
{
public:

    vector<string> QR_list;
    geometry_msgs::PoseStamped now_pos;
    sensor_msgs::LaserScan laser;

    ros::Subscriber *qr_sub;
    ros::Subscriber *laser_sub;
    ros::Subscriber *slam_sub;
    ros::Publisher *qr_pub;

    explicit QR_filtering();

private:


    double quternionToEuler(double x,double y,double z,double w);
    double laser_data_[682];
    void qrMessageRecieved(const achilles_qr_detector::qrPos &msg);
    void laserMessageReceived(const sensor_msgs::LaserScan& msg);
    void slamPoseMessageReceived(const geometry_msgs::PoseStamped& msg);
    achilles_qr_detector::qrPos setPose(achilles_qr_detector::qrPos msg);

};
