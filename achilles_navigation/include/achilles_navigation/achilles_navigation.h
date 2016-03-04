#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "QObject"
#include "ros/ros.h"
#include <boost/thread.hpp>
#include "qdebug.h"
#include "std_msgs/String.h"
#include "left_hand.h"
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <achilles_navigation/grid_navigation.h>
#include <nav_msgs/OccupancyGrid.h>
#include <achilles_navigation/frontier_search.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include "achilles_navigation/defines.h"
#include "QElapsedTimer"

#define ROBOT_POSE_LIST_SIZE 600
#define ROBOT_TRAP_THRESH_DIST 0.2

#define TURN_BACK_SPEED (char)255
#define MAX_TURN_BACK 50



class Navigation : public QObject
{
    Q_OBJECT
public:
    explicit Navigation(ros::NodeHandle *nh);
    ~Navigation();
private:
    FrontierSearch *frontierSearch;
    ros::Publisher frontier_cloud_pub;

    GridNavigation *grid_nav_;
    bool grid_nav_initilized_;

    bool nextFrontierGoal(const sensor_msgs::LaserScan& laser_msg,std_msgs::String &command);
    std::vector<geometry_msgs::PoseStamped> last_plan_;
    int plan_num_pointer_;
    QElapsedTimer plan_timer_;

    tf::TransformListener tf;
    costmap_2d::Costmap2DROS *cost_map_ros_;

    ros::Subscriber costmap_2d_sub_;
    void costmapCB(costmap_2d::Costmap2D *costmap);


    ros::Subscriber robot_pose_sub_;
    void robotPoseCB(const geometry_msgs::PoseStamped &pose);
    geometry_msgs::PoseStamped  robot_pose_stamp_;

    ros::Subscriber goal_sub_;
    void goalCB(const geometry_msgs::PoseStamped &goal);
    geometry_msgs::PoseStamped goal_pose_;

    double laser_data_[laser_msg_size_];

    ros::NodeHandle *nh_;
    LeftHand *left_hand_;

    bool allow_navigate_;

    boost::thread *thread_;
    bool shutdown_required_;

    void spin();

    ros::Publisher nav_motor_cmd_pub_;
    ros::Subscriber laser_data_sub_;
    ros::Subscriber nav_cmd_sub_;


    void laserDataCB(const sensor_msgs::LaserScan& laser_msg);
    void navigationCommandCB(const std_msgs::String& command);

    bool robotIsInTrap();
    void addTrapPose(geometry_msgs::PoseStamped pose);
    QList<geometry_msgs::PoseStamped> trap_poses_;
    void turnBack(std_msgs::String &command);
    void goForward(const sensor_msgs::LaserScan &laser_msg,std_msgs::String &command);
    int turn_back_counter_;

    QList<geometry_msgs::PoseStamped> robot_pose_list_;

};

#endif
