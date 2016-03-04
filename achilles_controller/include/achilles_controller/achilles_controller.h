#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "QObject"
#include "ros/ros.h"
#include <boost/thread.hpp>
#include "qdebug.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "QtExtSerialPort/qextserialport.h"
#include "geometry_msgs/PoseStamped.h"
#include "QTimer"
#include "sensor_msgs/LaserScan.h"
#include "QMutex"
#include <QUdpSocket>
#include <QElapsedTimer>
#include <QFuture>
#include <QtConcurrentRun>

#define HOST_ADDRESS "192.168.1.100"
#define CMD_RECV_PORT 7007

#define MOTOR_SPEED (char) 84
#define VICTIM_MOTOR_SPEED (char) 70
#define forward_side_limit_ 500

class Controller : public QObject
{
    Q_OBJECT
public:
    explicit Controller(ros::NodeHandle *nh);
    ~Controller();
private slots:
    void turnToVictim();
    void moveToVictim();
    void commandPacketCB();
    void victimElpasedTimeout();
private:
    ros::NodeHandle *nh_;
    ros::Subscriber nav_motor_cmd_sub_;
    ros::Subscriber victim_motor_cmd_sub_;
    ros::Subscriber victim_cmd_sub_;

    ros::Publisher tpa_pose_pub_;
    ros::Publisher tpa_command_pub_;

    ros::Publisher victim_image_pub_;
    ros::Publisher mark_victim_pub_;
//    ros::Subscriber nav_motor_cmd_sub_;

    ros::Publisher nav_cmd_pub_;
    ros::Subscriber robot_pose_sub_;
    ros::Subscriber laser_data_sub_;

    QElapsedTimer victim_elpased_time_;
    bool victim_elpased_started_;
    QTimer victim_elpased_check_timer_;


    QTimer *robot_turning_timer_;
    QTimer *robot_moving_victim_timer_;

    QUdpSocket command_packet_sub_;

    double robot_pose_x_;
    double robot_pose_y_;
    double robot_theta_;


    double victim_local_theta_;
    double victim_theta_;



    bool robot_is_turning_;
    bool stop_mode_;
    bool robot_moving_victim_;
    bool victim_mode_;
    bool ask_operator_victim_;
    bool operator_answered_;
    bool waitForOperatorAnswer();
    bool resetTPA();
    void wakeUpNavigation();
    QMutex mutex_;

    double laser_data_[682];


    void laserDataCB(const sensor_msgs::LaserScan& laser_msg);
    void navMotorCommandCB(std_msgs::String command_msg);
    void victimMotorCommandCB(std_msgs::Int16 command_msg);
    void victimCommnadCB(std_msgs::String command_msg);
    void robotPoseCB(const geometry_msgs::PoseStamped &msg);
    double quternionToEuler(double x,double y,double z,double w);
    void writeToPort(QByteArray bytes);


    QextSerialPort *port_;
    void initializePort();

    void spin();

    boost::thread *thread_;
    bool shutdown_required_;

};

#endif
