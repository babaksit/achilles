#include "achilles_navigation/left_hand.h"
#include "achilles_navigation/defines.h"

LeftHand::LeftHand(ros::NodeHandle *nh):force_turn_counter_(0)
  ,force_turn_active_(false)
{
    nh_=nh;
}

bool LeftHand::findPath(const sensor_msgs::LaserScan &laser_msg,std_msgs::String &command)
{


    if(laser_msg.ranges.size()==726)
    {
        for(int i=44;i<laser_msg_size_+44;i++)
        {
            laser_data_[i-44]=laser_msg.ranges[i]*1000;
        }

        calculateMovingDirection();
        filterMovingDirection();
        fillPath(command);
        return true;
    }
    else
    {
        command.data="bad laser data";
//        qDebug()<<"laser size != 726 : the size is ~>"<<laser_msg.ranges.size();
//        mutex_.unlock();
        return false;
    }
}

void LeftHand::filterMovingDirection()
{
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
//        if (moving_direction_ <= FORWARD_MOVING_DIRECTION)
//        {
//            moving_direction_ = RIGHT_MOVING_DIRECTION;
//        }
//        else
//        {
//            moving_direction_ = LEFT_MOVING_DIRECTION;
//        }
        moving_direction_ = -90;
        force_turn_counter_ ++;
    }

    double sum_laser_data=0;

    for (int i = AVG_LASER_DATA_LIMIT_I_L; i <= AVG_LASER_DATA_LIMIT_I_R; i++)
    {
        if ((laser_data_[i] >= LASER_DATA_MIN_DIST))
        {
            sum_laser_data+=laser_data_[i];
        }

    }

    double avg_laser_data=sum_laser_data/(AVG_LASER_DATA_LIMIT_I_R-AVG_LASER_DATA_LIMIT_I_L+1.0);

    if(avg_laser_data < AVG_LASER_DATA_LIMIT || ((!forward_side_open) && (!right_side_open) && (!left_side_open)))
    {
        force_turn_counter_++;
    }
    if(force_turn_counter_> FORCE_TURN_COUNTER && (!forward_side_open) && (!right_side_open) && (!left_side_open) )
    {
        force_turn_active_=true;
    }


    //    else if(lastest_position_counter>=300)
    //    {
    //        force_turn_active_=true;
    //    }

    if(force_turn_active_ && forward_side_open)
    {
        force_turn_active_=false;
        force_turn_counter_=0;
    }
    if(force_turn_active_)
    {
        moving_direction_=LEFT_MOVING_DIRECTION;
        ROS_WARN("Force Turn Force Turn");
    }


}

void LeftHand::fillPath(std_msgs::String &command)
{
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
    if((moving_direction_ == -90) && force_turn_active_==false)
    {
        bytes << 'f' << 0 << 'f' << 0;
    }
    bytes << '#';
    command.data=bytes.str();
}

void LeftHand::calculateMovingDirection()
{

    bool forward_wall_reached_bool = false;
    bool right_wall_reached_bool = false;
    bool left_wall_reached_bool = false;
    //------------------------------------------------------------------------------------------------------------------
    for (int i = FORWARD_SIDE_LIMIT_I_L; i <= FORWARD_SIDE_LIMIT_I_R; i++)
    {
        if ((laser_data_[i] >= LASER_DATA_MIN_DIST) && (laser_data_[i] < FORWARD_WALL_REACHED_LIMIT))
        {
            forward_wall_reached_bool = true;
            break;
        }
    }
    //------------------------------------------------------------------------------------------------------------------
    for (int i = RIGHT_SIDE_LIMIT_I_L; i <= RIGHT_SIDE_LIMIT_I_R; i++)
    {
        if ((laser_data_[i] >= LASER_DATA_MIN_DIST) && (laser_data_[i] < RIGHT_WALL_REACHED_LIMIT))
        {
            right_wall_reached_bool = true;
            break;
        }
    }
    //------------------------------------------------------------------------------------------------------------------
    for (int i = LEFT_SIDE_LIMIT_I_L; i <= LEFT_SIDE_LIMIT_I_R; i++)
    {
        if ((laser_data_[i] >= LASER_DATA_MIN_DIST) && (laser_data_[i] < LEFT_WALL_REACHED_LIMIT))
        {
            left_wall_reached_bool = true;
            break;
        }
    }
    //------------------------------------------------------------------------------------------------------------------
    if (forward_wall_reached_bool)
    {
        moving_direction_ = 0;
    }
    else
    {
        if (left_wall_reached_bool)
        {
            moving_direction_ = 90;
        }
        else
        {
            moving_direction_ = 180;
        }
    }

}

LeftHand::~LeftHand()
{

}

bool LeftHand::checkTrapState()
{

}

