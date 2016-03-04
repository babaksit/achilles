#include"QR_filtering.h"

QR_filtering::QR_filtering()
{

    ros::NodeHandle nh;

    qr_sub=new ros::Subscriber(nh.subscribe("Qr_String",10,&QR_filtering::qrMessageRecieved,this));
    laser_sub=new ros::Subscriber(nh.subscribe("scan",10,&QR_filtering::laserMessageReceived,this));
    slam_sub=new ros::Subscriber(nh.subscribe("slam_out_pose",10,&QR_filtering::slamPoseMessageReceived,this));
    qr_pub=new ros::Publisher(nh.advertise<achilles_qr_detector::qrPos>("QR_position",1));

}


void QR_filtering::qrMessageRecieved(const achilles_qr_detector::qrPos& msg)
{

    achilles_qr_detector::qrPos set_msg;
    bool new_qr = true;

    if(QR_list.size() == 0)
    {
        set_msg = this->setPose(msg);
        QR_list.push_back(msg.context);
        qr_pub->publish(set_msg);

    }
    else
    {
        for(int i=0; i < QR_list.size() ; i++)
        {
            if(QR_list[i] == msg.context)
            {
                new_qr = false;
            }
        }

        if(new_qr && msg.context.size() > 0)
        {
            set_msg = this->setPose(msg);
            QR_list.push_back(msg.context);
            qr_pub->publish(set_msg);
        }

    }

}

achilles_qr_detector::qrPos QR_filtering::setPose(achilles_qr_detector::qrPos msg)
{

    achilles_qr_detector::qrPos setting=msg;
    double distance=0;

    int counter=0;

    if(msg.id==1)
    {
        ROS_ERROR("%d",msg.id);
        for (int i = 256; i <= 427; i++)
        {
            if ((laser_data_[i] >= 0.02))
            {
                distance+=laser_data_[i];
                counter++;
            }
        }
        if(counter>0)
        distance/=counter;
        ROS_ERROR("%lf",distance);
        double theta=quternionToEuler(now_pos.pose.orientation.x,now_pos.pose.orientation.y,now_pos.pose.orientation.z,now_pos.pose.orientation.w);
        setting.pos_x = now_pos.pose.position.x + distance * cos(theta);
        setting.pos_y = now_pos.pose.position.y+ distance * sin(theta);//427-256
        return setting;
    }
    if(msg.id==2)
    {
        for (int i = 640; i <= 682; i++)
        {
            if ((laser_data_[i] >= 0.02))
            {
                distance+=laser_data_[i];
                counter++;
            }
        }
        if(counter>0)
        distance/=counter;
        double theta=quternionToEuler(now_pos.pose.orientation.x,now_pos.pose.orientation.y,now_pos.pose.orientation.z,now_pos.pose.orientation.w)+M_PI_2;
        setting.pos_x = now_pos.pose.position.x + distance * cos(theta);
        setting.pos_y = now_pos.pose.position.y+ distance * sin(theta);//427-256
        return setting;
    }

    if(msg.id==3)
    {
        for (int i = 171; i <= 256; i++)
        {
            if ((laser_data_[i] >= 0.02))
            {
                distance+=laser_data_[i];
                counter++;
            }
        }
        if(counter>0)
        distance/=counter;
        double theta=quternionToEuler(now_pos.pose.orientation.x,now_pos.pose.orientation.y,now_pos.pose.orientation.z,now_pos.pose.orientation.w)-M_PI_2;
        setting.pos_x = now_pos.pose.position.x + distance * cos(theta);
        setting.pos_y = now_pos.pose.position.y+ distance * sin(theta);//427-256
        return setting;
    }

    return setting;
}


void QR_filtering::laserMessageReceived(const sensor_msgs::LaserScan& msg)
{
    //    mutex_.lock();
    if(msg.ranges.size()==726)
    {
        for(int i=44;i<682+44;i++)
        {
            laser_data_[i-44]=msg.ranges[i];

        }
        //        mutex_.unlock();
        return ;
    }
    else
    {
        ROS_ERROR("laser size != 726 : the size is ~> %d ",msg.ranges.size());
        //        mutex_.unlock();
        return ;
    }
}
double QR_filtering::quternionToEuler(double x,double y,double z,double w)
{
    double R3_2=2*(z*w-x*y);
    double R3_3=2*pow(x,2)-1+2*pow(w,2);
    return atan2(R3_2,R3_3);
}
void QR_filtering::slamPoseMessageReceived(const geometry_msgs::PoseStamped& msg)
{
    this->now_pos=msg;
}
