#ifndef LEFT_HAND_H
#define LEFT_HAND_H
#include "QObject"
#include "ros/ros.h"
#include <boost/thread.hpp>
#include "qdebug.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "QMutex"



#define laser_msg_size_ 682

class LeftHand : public QObject
{
    Q_OBJECT
public:
    explicit LeftHand(ros::NodeHandle *nh);
    bool findPath(const sensor_msgs::LaserScan& laser_msg,std_msgs::String &command);
    ~LeftHand();
private:
    ros::NodeHandle *nh_;

    bool checkTrapState();

    void calculateMovingDirection();
    void filterMovingDirection();
    void fillPath(std_msgs::String &command);


    QMutex mutex_;
    double laser_data_[laser_msg_size_];
    int moving_direction_;

    int force_turn_counter_;
    bool force_turn_active_;
};

#endif
