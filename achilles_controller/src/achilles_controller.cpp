#include "achilles_controller/achilles_controller.h"

Controller::Controller(ros::NodeHandle *nh):shutdown_required_(false),nh_(nh)
  ,victim_mode_(false),robot_is_turning_(false)
  ,victim_local_theta_(0),robot_moving_victim_(false)
  ,ask_operator_victim_(false),command_packet_sub_(this)
  ,stop_mode_(false),operator_answered_(false)
  ,victim_elpased_check_timer_(this)
  ,victim_elpased_started_(false)
{
    initializePort();
    nav_motor_cmd_sub_= nh_->subscribe("navigation_to_motor_command",10,&Controller::navMotorCommandCB,this);
    victim_motor_cmd_sub_= nh_->subscribe("victim_to_motor_command",10,&Controller::victimMotorCommandCB,this);
    tpa_pose_pub_ = nh_->advertise<std_msgs::Int16>("tpa_position",10);
    victim_cmd_sub_ = nh_->subscribe("victim_mode",10,&Controller::victimCommnadCB,this);
    nav_cmd_pub_ = nh_->advertise<std_msgs::String>("navigation_command",10);
    robot_pose_sub_ = nh_->subscribe("slam_out_pose", 10, &Controller::robotPoseCB,this);
    laser_data_sub_= nh_->subscribe("scan",10,&Controller::laserDataCB,this);
    victim_image_pub_ = nh_->advertise<std_msgs::String>("victim_image_transport",10);
    tpa_command_pub_ = nh_->advertise<std_msgs::String>("tpa_command",10);
    mark_victim_pub_ = nh_->advertise<std_msgs::String>("mark_victim",10);
    //Timer Signals MUST !!! CHANGE !!!!!!!!
    robot_turning_timer_ = new QTimer(this);
    robot_moving_victim_timer_ = new QTimer(this);
    connect(robot_turning_timer_,SIGNAL(timeout()),this,SLOT(turnToVictim()));
    robot_turning_timer_->start(30);
    connect(robot_moving_victim_timer_,SIGNAL(timeout()),this,SLOT(moveToVictim()));
    robot_moving_victim_timer_->start(30);

    connect(&victim_elpased_check_timer_,SIGNAL(timeout()),this,SLOT(victimElpasedTimeout()));
    victim_elpased_check_timer_.start(30);

    command_packet_sub_.bind(CMD_RECV_PORT);
    connect(&command_packet_sub_, SIGNAL(readyRead()),this, SLOT(commandPacketCB()));

    QByteArray bytes;
    bytes[0]=13;
    bytes[1]=48;
    port_->write(bytes);

    thread_ =  new boost::thread(boost::bind( &Controller::spin, this ));
//    ROS_ERROR("my boy");
}

Controller::~Controller()
{

}

void Controller::commandPacketCB()
{
    ROS_ERROR("heavennnn");
    QByteArray datagram;
    do {
        datagram.resize(command_packet_sub_.pendingDatagramSize());
        command_packet_sub_.readDatagram(datagram.data(), datagram.size());
    } while (command_packet_sub_.hasPendingDatagrams());

    QDataStream in(&datagram, QIODevice::ReadOnly);
    in.setVersion(QDataStream::Qt_4_8);
    QString command;
    in >> command;

    if(command=="START ROBOT")
    {
        stop_mode_=false;
    }
    if(command=="STOP ROBOT")
    {
        stop_mode_=true;
        QByteArray bytes;
        bytes[0]='*';bytes[1]='~';bytes[2]='f';bytes[3]=0;bytes[4]='f';bytes[5]=0;bytes[6]='#';
    }
    if(command=="VERIFY VICTIM")
    {
        operator_answered_=true;
        std_msgs::String command;
        command.data="correct victim";
        mark_victim_pub_.publish(command);
        ROS_ERROR("helllllllllllll");
    }
    if(command=="NOT VICTIM")
    {
        operator_answered_=true;
        std_msgs::String command;
        command.data="incorrect victim";
        mark_victim_pub_.publish(command);
    }

}

void Controller::victimElpasedTimeout()
{

}

void Controller::navMotorCommandCB(std_msgs::String command_msg)
{
    if(victim_mode_==false)
    {
//        qDebug() << "motor command received" << command_msg.data.c_str();
        QByteArray bytes;
        bytes.setRawData(command_msg.data.c_str(),7);
        writeToPort(bytes);
    }
}

void Controller::victimMotorCommandCB(std_msgs::Int16 command_msg)
{
    if(robot_is_turning_==false && robot_moving_victim_ == false && ask_operator_victim_==false)
    {

        victim_local_theta_=(command_msg.data*6);
        ROS_INFO("victim first theta: %lf ",victim_local_theta_);
        if(victim_local_theta_<84)
        {
            victim_theta_=robot_theta_-(90-victim_local_theta_);
            while ((victim_theta_<-90))
            {
                victim_theta_+=360;
            }
        }
        else if( victim_local_theta_ >96)
        {
            victim_theta_=robot_theta_+victim_local_theta_-90;
            while (victim_theta_>270)
            {
                victim_theta_-=360;
            }
        }
        else
        {
            victim_theta_=robot_theta_;
            //            robot_is_turning_=false;
            std_msgs::Int16 pose_msg;
            pose_msg.data=15;
            tpa_pose_pub_.publish(pose_msg);
        }
        robot_is_turning_=true;
    }
}



void Controller::victimCommnadCB(std_msgs::String command_msg)
{
    //    ask_operator_victim_=false;
    //    operator_answered_=false;
    //    robot_moving_victim_=false;
    //    robot_is_turning_=false;
    std_msgs::String command;
    if(command_msg.data=="shut_down_navigation")
    {
        command.data="shut_down";
        nav_cmd_pub_.publish(command);
        victim_mode_=true;
    }
    if(command_msg.data=="wake_up_navigation")
    {
        command.data="wake_up";
        nav_cmd_pub_.publish(command);
        victim_mode_=false;
    }
}

void Controller::robotPoseCB(const geometry_msgs::PoseStamped &pose)
{
    robot_pose_x_=pose.pose.position.x*1000;
    robot_pose_y_=pose.pose.position.y*1000;
    robot_theta_=quternionToEuler(pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w)*180/M_PI +90;
    //    ROS_INFO("robot_theta_ :: %lf ",robot_theta_ );
}
double Controller::quternionToEuler(double x,double y,double z,double w)
{
    double R3_2=2*(z*w-x*y);
    double R3_3=2*pow(x,2)-1+2*pow(w,2);
    return atan2(R3_2,R3_3);
}

void Controller::turnToVictim()
{
    if(robot_is_turning_==false || robot_moving_victim_==true || ask_operator_victim_==true )
        return;
//    ROS_INFO("victim local theta: %lf ",victim_local_theta_);
//    ROS_INFO("victim theta: %lf ",victim_theta_);
//    ROS_INFO("robot theta: %lf ",robot_theta_);
    {
        double diff=victim_theta_-robot_theta_;
        if((victim_theta_ > 0 && robot_theta_ < 0) || (victim_theta_ < 0 && robot_theta_ > 0))
        {
            if(diff>180)
            {
                diff=360-diff;
            }
        }
        ROS_INFO("victim_theta_-robot_theta_ :%lf",(diff));

        if(fabs(diff)<5)
        {
            robot_is_turning_=false;
            robot_moving_victim_=true;
            std_msgs::Int16 pose_msg;
            pose_msg.data=15;
            tpa_pose_pub_.publish(pose_msg);
        }
        QByteArray bytes;
        bytes[0]='*';bytes[1]='~';
        if(victim_local_theta_<84)
        {
            bytes[2]='f';bytes[3]=VICTIM_MOTOR_SPEED;bytes[4]='r';bytes[5]=VICTIM_MOTOR_SPEED;
        }
        else if(victim_local_theta_>96)
        {
            bytes[2]='r';bytes[3]=VICTIM_MOTOR_SPEED;bytes[4]='f';bytes[5]=VICTIM_MOTOR_SPEED;
        }
        else
        {
            bytes[2]='f';bytes[3]=0;bytes[4]='f';bytes[5]=0;
            robot_is_turning_=false;
            robot_moving_victim_=true;
            std_msgs::Int16 pose_msg;
            pose_msg.data=15;
            tpa_pose_pub_.publish(pose_msg);
        }
        bytes[6]='#';
        writeToPort(bytes);
    }
}

void Controller::laserDataCB(const sensor_msgs::LaserScan& laser_msg)
{
    //    mutex_.lock();
    if(laser_msg.ranges.size()==726)
    {
        for(int i=44;i<682+44;i++)
        {
            laser_data_[i-44]=laser_msg.ranges[i]*1000;
        }

        //        ROS_INFO("%lf , robot theta %lf",laser_data_[60],robot_theta_);
        //        mutex_.unlock();
        return ;
    }
    else
    {
        qDebug()<<"laser size != 726 : the size is ~>"<<laser_msg.ranges.size();
        //        mutex_.unlock();
        return ;
    }
}
void Controller::moveToVictim()
{
    if(robot_is_turning_==true || robot_moving_victim_==false || ask_operator_victim_==true)
        return ;
    ROS_INFO("robot is moving to victim");
    QByteArray bytes;
    bytes[0]='*';bytes[1]='~';
    bool forward_side_open=true;

    for (int i = 256; i <= 427; i++)
    {
        if ((laser_data_[i] >= 20) && (laser_data_[i] < forward_side_limit_))
        {
            forward_side_open = false;
            break;
        }
    }
    if(forward_side_open==false)
    {
        ROS_INFO("robot in 1 meter");
        ask_operator_victim_=true;
        robot_moving_victim_=false;
        waitForOperatorAnswer();
        command_packet_sub_.flush();
        resetTPA();
        wakeUpNavigation();
        return;
    }
    bytes[2]='f';bytes[3]=VICTIM_MOTOR_SPEED;bytes[4]='f';bytes[5]=VICTIM_MOTOR_SPEED;
    bytes[6]='#';
    writeToPort(bytes);
}
bool Controller::waitForOperatorAnswer()
{
    QElapsedTimer timer;
    timer.start();
    while (timer.hasExpired(19000)==false)
    {
        if(operator_answered_ == true)
        {
            operator_answered_ = false;
            return true;
        }
        ROS_INFO("waiting for operator answer");
        std_msgs::String command;
        command.data="send victim image";
        victim_image_pub_.publish(command);
        commandPacketCB();
        sleep(1);
    }
    return false;
}

bool Controller::resetTPA()
{
    ask_operator_victim_=false;
    operator_answered_=false;
    robot_moving_victim_=false;
    robot_is_turning_=false;
    victim_mode_=false;
    std_msgs::String command;
    command.data="reset tpa";
    tpa_command_pub_.publish(command);
}

void Controller::wakeUpNavigation()
{
    std_msgs::String command;
    command.data="wake_up";
    nav_cmd_pub_.publish(command);
}

void Controller::writeToPort(QByteArray bytes)
{

    ROS_WARN("%s",QString(bytes).toStdString().c_str());
    if(stop_mode_==true)
    {
        ROS_WARN("stop mode is active ");
        return;
    }
    if((bytes[0]=='*')&&(bytes[1]=='~')&&(bytes[6]=='#'))
    {
        port_->write(bytes);
//        ROS_WARN("writing to ports %d",(int)bytes[3]);
    }
    else
    {
        bytes[0]='*';bytes[1]='~';bytes[2]='f';
        bytes[3]=0;bytes[4]='f';bytes[5]=0;bytes[6]='#';
        port_->write(bytes);
    }

}

void Controller::initializePort()
{
    QByteArray bytes;
    QString port_name = QLatin1String("emma_motor_usb");
    port_ = new QextSerialPort(QString(port_name), QextSerialPort::EventDriven);

    port_->setBaudRate(BAUD19200);
    port_->setFlowControl(FLOW_OFF);
    port_->setParity(PAR_NONE);
    port_->setDataBits(DATA_8);
    port_->setStopBits(STOP_1);

    if (port_->open(QIODevice::ReadWrite) == true)
    {
        qDebug() << "listening for data on" << port_->portName();
    }
    else
    {
        qDebug() << "device failed to open:" << port_->errorString();
    }

    bytes[0]=8;

    port_->write(bytes,bytes.size());

}
void Controller::spin()
{

    ros::Rate loop(10);
    //    sleep(1);
    while ( ros::ok() && !shutdown_required_)
    {
        ros::spinOnce();
        loop.sleep();
    }

}
