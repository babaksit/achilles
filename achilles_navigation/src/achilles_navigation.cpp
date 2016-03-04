#include "achilles_navigation/achilles_navigation.h"

Navigation::Navigation(ros::NodeHandle *nh):
    shutdown_required_(false),nh_(nh),allow_navigate_(true)
  ,grid_nav_initilized_(false),tf(ros::Duration(10))
  ,turn_back_counter_(0),plan_num_pointer_(0)
{
    left_hand_ = new LeftHand(nh_);
    nav_motor_cmd_pub_= nh_->advertise<std_msgs::String>("navigation_to_motor_command",10);
    nav_cmd_sub_ = nh_->subscribe("navigation_command",10,&Navigation::navigationCommandCB,this);
    robot_pose_sub_= nh_->subscribe("slam_out_pose",10,&Navigation::robotPoseCB,this);
    goal_sub_ = nh_->subscribe("exploration/goal",10,&Navigation::goalCB,this);
    cost_map_ros_ = new costmap_2d::Costmap2DROS("", tf);
    grid_nav_ = new GridNavigation("grid_nav",cost_map_ros_->getCostmap(),cost_map_ros_->getGlobalFrameID());
    frontierSearch = new FrontierSearch(nh_,(cost_map_ros_->getCostmap()));
    frontier_cloud_pub = nh_->advertise<sensor_msgs::PointCloud2>("frontierss",5);
    laser_data_sub_= nh_->subscribe("scan",10,&Navigation::laserDataCB,this);
    thread_ = new boost::thread(boost::bind( &Navigation::spin, this ));
    plan_timer_.start();
}
void Navigation::laserDataCB(const sensor_msgs::LaserScan& laser_msg)
{

    if(allow_navigate_==false)
        return;
    std_msgs::String command;
    //    if(robotIsInTrap() == false)
    if(true)
    {
        if(!nextFrontierGoal(laser_msg,command))
        {
            left_hand_->findPath(laser_msg,command);
            ROS_INFO("USING LEFT HAND NAVIGATION");
        }
    }
    else
    {
        goForward(laser_msg,command);
        left_hand_->findPath(laser_msg,command);
        ROS_INFO("USING FORWARD HAND NAVIGATION");
    }
    nav_motor_cmd_pub_.publish(command);
}

void Navigation::navigationCommandCB(const std_msgs::String& command)
{
    if(command.data=="shut_down")
    {
        allow_navigate_=false;
    }
    if(command.data=="wake_up")
    {
        allow_navigate_=true;
    }

}

bool Navigation::robotIsInTrap()
{
    if(robot_pose_list_.size() < ROBOT_POSE_LIST_SIZE )
        return false;
    int diff=0;
    int step_size=5;
    try
    {
        for(int i=0;i<robot_pose_list_.size();i+=step_size)
        {
            if(QLineF(robot_pose_stamp_.pose.position.x,robot_pose_stamp_.pose.position.y,robot_pose_list_[i].pose.position.x,robot_pose_list_[i].pose.position.y).length()>ROBOT_TRAP_THRESH_DIST)
                return false;
        }
    }
    catch (...)
    {
        ROS_ERROR("Exception in Robot In trap function");
        return false;
    }
    ROS_ERROR("ROBOT IS IN TRAP");
    return true;
}

void Navigation::addTrapPose(geometry_msgs::PoseStamped pose)
{
    ROS_WARN("trap pose size %d", trap_poses_.size());
    for(int i=0; i<trap_poses_.size() ; i++)
    {
        if(QLineF(pose.pose.position.x,pose.pose.position.y,trap_poses_[i].pose.position.x,trap_poses_[i].pose.position.y).length() < 0.2)
            return ;
    }
    trap_poses_.push_back(pose);
}

void Navigation::turnBack(std_msgs::String &command)
{
    std::stringstream bytes;
    bytes << "*~";
    bytes << 'r' << TURN_BACK_SPEED << 'r' << TURN_BACK_SPEED;
    bytes << '#';
    command.data=bytes.str();
}

void Navigation::goForward(const sensor_msgs::LaserScan &laser_msg,std_msgs::String &command)
{
    if(laser_msg.ranges.size()==726)
    {
        for(int i=44;i<laser_msg_size_+44;i++)
        {
            laser_data_[i-44]=laser_msg.ranges[i]*1000;
        }
    }
    else
    {
        return ;
    }

    int moving_direction_ = 90;
    bool forward_side_open = true;
    bool right_side_open = true;
    bool left_side_open = true;
    moving_direction_=fabs(moving_direction_);

    //------------------------------------------------------------------------------------------------------------------
    for (int i = FORWARD_SIDE_LIMIT_I_L; i <= FORWARD_SIDE_LIMIT_I_R; i++)
    {
        if ((laser_data_[i] >= 20) && (laser_data_[i] < FORWARD_SIDE_LIMIT))
        {
            forward_side_open = false;
            break;
        }
    }
    //------------------------------------------------------------------------------------------------------------------
    for (int i = RIGHT_SIDE_LIMIT_I_L; i <= RIGHT_SIDE_LIMIT_I_R; i++)
    {
        if ((laser_data_[i] >= 20) && (laser_data_[i] < RIGHT_SIDE_LIMIT))
        {
            right_side_open = false;
            break;
        }
    }
    //------------------------------------------------------------------------------------------------------------------
    for (int i = LEFT_SIDE_LIMIT_I_L; i <= LEFT_SIDE_LIMIT_I_R; i++)
    {
        if ((laser_data_[i] >= 20) && (laser_data_[i] < LEFT_SIDE_LIMIT))
        {
            left_side_open = false;
            break;
        }
    }


    qDebug()<< left_side_open << forward_side_open << right_side_open;
    //------------------------------------------------------------------------------------------------------------------
    if ((forward_side_open) && (right_side_open) && (left_side_open))
    {

    }
    //------------------------------------------------------------------------------------------------------------------
    if ((forward_side_open) && (right_side_open) && (!left_side_open))
    {
        if (moving_direction_ >= MOVING_DIRECTION_LEFT_BOUNDARY)
        {
            moving_direction_ = FORWARD_MOVING_DIRECTION;
        }
    }
    //------------------------------------------------------------------------------------------------------------------
    if ((forward_side_open) && (!right_side_open) && (left_side_open))
    {
        if (moving_direction_ <= MOVING_DIRECTION_RIGHT_BOUNDARY)
        {
            moving_direction_ = FORWARD_MOVING_DIRECTION;
        }
    }
    //------------------------------------------------------------------------------------------------------------------
    if ((forward_side_open) && (!right_side_open) && (!left_side_open))
    {
        moving_direction_ = FORWARD_MOVING_DIRECTION;
    }
    //------------------------------------------------------------------------------------------------------------------
    if ((!forward_side_open) && (right_side_open) && (left_side_open))
    {
        if (moving_direction_ <= FORWARD_MOVING_DIRECTION)
        {
            moving_direction_ = RIGHT_MOVING_DIRECTION;
        }
        else
        {
            moving_direction_ = LEFT_MOVING_DIRECTION;
        }
    }
    //------------------------------------------------------------------------------------------------------------------
    if ((!forward_side_open) && (right_side_open) && (!left_side_open))
    {
        moving_direction_ = RIGHT_MOVING_DIRECTION;
    }
    //------------------------------------------------------------------------------------------------------------------
    if ((!forward_side_open) && (!right_side_open) && (left_side_open))
    {
        moving_direction_ = LEFT_MOVING_DIRECTION;
    }
    //------------------------------------------------------------------------------------------------------------------
    if ((!forward_side_open) && (!right_side_open) && (!left_side_open))
    {
        if (moving_direction_ <= FORWARD_MOVING_DIRECTION)
        {
            moving_direction_ = RIGHT_MOVING_DIRECTION;
        }
        else
        {
            moving_direction_ = LEFT_MOVING_DIRECTION;
        }
    }

    std::stringstream bytes;
    bytes << "*~";
    if(moving_direction_<MOVING_DIRECTION_RIGHT_BOUNDARY )
    {
        bytes << 'f' << ROBOT_TURNING_SPEED << 'r' << ROBOT_TURNING_SPEED;
    }
    else if(moving_direction_>=MOVING_DIRECTION_RIGHT_BOUNDARY && moving_direction_<=MOVING_DIRECTION_LEFT_BOUNDARY)
    {
        bytes << 'f' << ROBOT_SPEED << 'f' << ROBOT_SPEED;
    }
    else if(moving_direction_>MOVING_DIRECTION_LEFT_BOUNDARY )
    {
        bytes << 'r' << ROBOT_TURNING_SPEED << 'f' << ROBOT_TURNING_SPEED;
    }
    bytes << '#';
    command.data=bytes.str();
}

void Navigation::spin()
{

    ros::Rate loop(10);
    //    sleep(1);
    while ( ros::ok() && !shutdown_required_)
    {
        ros::spinOnce();
        loop.sleep();
    }

}
void Navigation::robotPoseCB(const geometry_msgs::PoseStamped &pose)
{
    robot_pose_stamp_=pose;
    if(robot_pose_list_.size() >= ROBOT_POSE_LIST_SIZE)
    {
        robot_pose_list_.push_back(robot_pose_stamp_);
        robot_pose_list_.pop_front();
    }
    else
    {
        robot_pose_list_.push_back(robot_pose_stamp_);
    }
}

void Navigation::goalCB(const geometry_msgs::PoseStamped &goal)
{
    //    ROS_INFO("Goal call back");
    goal_pose_=goal;
}

Navigation::~Navigation()
{
    delete left_hand_;
}

bool Navigation::nextFrontierGoal(const sensor_msgs::LaserScan& laser_msg,std_msgs::String &command)
{
    last_plan_ = grid_nav_->getLastPlan();
    int last_plan_size = last_plan_.size();
    if(plan_timer_.elapsed() < PLAN_TIMEOUT)
    {
        if(last_plan_size > PLAN_MIN_SIZE)
        {
            ROS_INFO("last plan size : %d", last_plan_size);
            ROS_INFO("plan_num_pointer_ : %d",plan_num_pointer_);
            if(QLineF(robot_pose_stamp_.pose.position.x,robot_pose_stamp_.pose.position.y,last_plan_[last_plan_size-1].pose.position.x,last_plan_[last_plan_size-1].pose.position.y).length() > GOAL_REACH_DIST)
            {
                ROS_INFO("plan length : %f", QLineF(robot_pose_stamp_.pose.position.x,robot_pose_stamp_.pose.position.y,last_plan_[last_plan_size-1].pose.position.x,last_plan_[last_plan_size-1].pose.position.y).length());
                if(plan_num_pointer_ < last_plan_size)
                {
                    if(QLineF(robot_pose_stamp_.pose.position.x,robot_pose_stamp_.pose.position.y,last_plan_[plan_num_pointer_].pose.position.x,last_plan_[plan_num_pointer_].pose.position.y).length() < GOAL_REACH_DIST)
                    {
                        if((plan_num_pointer_ + PLAN_MIN_SIZE) < last_plan_size)
                        {
                            plan_num_pointer_ += PLAN_MIN_SIZE;
                        }
                    }
                }
                if(plan_num_pointer_ < last_plan_size)
                {
                    tf::Quaternion q(robot_pose_stamp_.pose.orientation.x, robot_pose_stamp_.pose.orientation.y, robot_pose_stamp_.pose.orientation.z, robot_pose_stamp_.pose.orientation.w);
                    tf::Matrix3x3 m(q);
                    double roll, pitch, yaw;
                    m.getRPY(roll, pitch, yaw);
                    double robot_oriention=yaw;
                    grid_nav_->fillCommand(laser_msg,QLineF(robot_pose_stamp_.pose.position.x,robot_pose_stamp_.pose.position.y,last_plan_[plan_num_pointer_].pose.position.x,last_plan_[plan_num_pointer_].pose.position.y),robot_oriention,command);
                    return true;
                }
            }
        }
    }
    else
    {
        plan_timer_.restart();
        ROS_ERROR("TIMEOUT");
    }
    plan_num_pointer_ = 0;
    std::list<frontier_exploration::Frontier> frontier_list = frontierSearch->searchFrom(robot_pose_stamp_.pose.position);

    if(frontier_list.size() == 0){
        ROS_INFO("No frontiers found");
        return false;
    }
    else
    {
        //        ROS_INFO("frontiers size %d",frontier_list.size());
    }
    frontier_exploration::Frontier selected;
    selected.min_distance = std::numeric_limits<double>::infinity();

    BOOST_FOREACH(frontier_exploration::Frontier frontier, frontier_list){
        //check if this frontier is the nearest to robot
        if (frontier.min_distance < selected.min_distance){
            selected = frontier;
        }
    }

    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.frame_id = robot_pose_stamp_.header.frame_id;
    goal_pose.header.stamp = ros::Time::now();
    goal_pose.pose.position = selected.initial;
    if(grid_nav_->findPath(laser_msg,robot_pose_stamp_,goal_pose,command))
        return true;

    return false;
}



