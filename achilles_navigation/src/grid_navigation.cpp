#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

#include <achilles_navigation/dijkstra.h>
#include <achilles_navigation/grid_path.h>
#include <achilles_navigation/gradient_path.h>
#include <achilles_navigation/quadratic_calculator.h>
#include <achilles_navigation/grid_navigation.h>
#include <achilles_navigation/defines.h>

void GridNavigation::outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value) {

    unsigned char* pc = costarr;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr + (ny - 1) * nx;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
    pc = costarr + nx - 1;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;

}

GridNavigation::GridNavigation(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) :
    costmap_(NULL), initialized_(false), allow_unknown_(true)
  ,force_turn_active_(false),force_turn_counter_(0){
    initialize(name, costmap, frame_id);
}


GridNavigation::~GridNavigation() {
    if (p_calc_)
        delete p_calc_;
    if (planner_)
        delete planner_;
    if (path_maker_)
        delete path_maker_;
}

void GridNavigation::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void GridNavigation::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) {
    if (!initialized_) {
        ros::NodeHandle private_nh;
        costmap_ = costmap;
        frame_id_ = frame_id;

        unsigned int cx = costmap->getSizeInCellsX(), cy = costmap->getSizeInCellsY();

        private_nh.param("old_navfn_behavior", old_navfn_behavior_, false);
        if(!old_navfn_behavior_)
            convert_offset_ = 0.5;
        else
            convert_offset_ = 0.0;

        bool use_quadratic;
        private_nh.param("use_quadratic", use_quadratic, true);
        if (use_quadratic)
            p_calc_ = new QuadraticCalculator(cx, cy);
        else
            p_calc_ = new PotentialCalculator(cx, cy);

        //        bool use_dijkstra;
        //        private_nh.param("use_dijkstra", use_dijkstra, true);
        //        if (use_dijkstra)
        //        {
        DijkstraExpansion* de = new DijkstraExpansion(p_calc_, cx, cy);
        if(!old_navfn_behavior_)
            de->setPreciseStart(true);
        planner_ = de;
        //        }


        bool use_grid_path;
        private_nh.param("use_grid_path", use_grid_path, false);
        if (use_grid_path)
            path_maker_ = new GridPath(p_calc_);
        else
            path_maker_ = new GradientPath(p_calc_);

        orientation_filter_ = new OrientationFilter();


        plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
        potential_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("potential", 1);

        private_nh.param("allow_unknown", allow_unknown_, true);
        allow_unknown_ = true;
        planner_->setHasUnknown(allow_unknown_);
        private_nh.param("planner_window_x", planner_window_x_, 0.0);
        private_nh.param("planner_window_y", planner_window_y_, 0.0);
        private_nh.param("default_tolerance", default_tolerance_, 0.0);
        private_nh.param("publish_scale", publish_scale_, 100);

        double costmap_pub_freq;
        private_nh.param("planner_costmap_publish_frequency", costmap_pub_freq, 0.0);

        //get the tf prefix
        ros::NodeHandle prefix_nh;
        tf_prefix_ = tf::getPrefixParam(prefix_nh);



        reconfigureCB();

        initialized_ = true;
    } else
        ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");

}
void GridNavigation::reconfigureCB()
{
    planner_->setLethalCost(253);
    path_maker_->setLethalCost(253);
    planner_->setNeutralCost(50);
    planner_->setFactor(3.0);
    publish_potential_ = true;
}

void GridNavigation::clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my) {
    if (!initialized_) {
        ROS_ERROR(
                    "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }
    //set the associated costs in the cost map to be free
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
}



void GridNavigation::mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = costmap_->getOriginX() + (mx+convert_offset_) * costmap_->getResolution();
    wy = costmap_->getOriginY() + (my+convert_offset_) * costmap_->getResolution();
}

bool GridNavigation::worldToMap(double wx, double wy, double& mx, double& my) {
    double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
    double resolution = costmap_->getResolution();

    if (wx < origin_x || wy < origin_y)
        return false;

    mx = (wx - origin_x) / resolution - convert_offset_;
    my = (wy - origin_y) / resolution - convert_offset_;

    if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
        return true;

    return false;
}



void GridNavigation::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    if (!initialized_) {
        ROS_ERROR(
                    "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    if (!path.empty()) {
        gui_path.header.frame_id = path[0].header.frame_id;
        gui_path.header.stamp = path[0].header.stamp;
    }

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }
    plan_pub_.publish(gui_path);
}

bool GridNavigation::findPath(const sensor_msgs::LaserScan &laser_msg, const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                              std_msgs::String &command)
{
    if (!initialized_) {
        ROS_ERROR(
                    "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    std::vector<geometry_msgs::PoseStamped> plan;
    //    //clear the plan, just in case
    //    plan.clear();

    ros::NodeHandle n;
    std::string global_frame = frame_id_;

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if (tf::resolve(tf_prefix_, goal.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) {
        ROS_ERROR(
                    "The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, goal.header.frame_id).c_str());
        return false;
    }

    if (tf::resolve(tf_prefix_, start.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) {
        ROS_ERROR(
                    "The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, start.header.frame_id).c_str());
        return false;
    }

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
    double start_x, start_y, goal_x, goal_y;

    if (!costmap_->worldToMap(wx, wy, start_x_i, start_y_i)) {
        ROS_WARN(
                    "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
    }
    if(old_navfn_behavior_){
        start_x = start_x_i;
        start_y = start_y_i;
    }else{
        worldToMap(wx, wy, start_x, start_y);
    }

    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i)) {
        ROS_WARN(
                    "The goal sent to the navfn planner is off the global costmap. Planning will always fail to this goal.");
        return false;
    }
    if(old_navfn_behavior_){
        goal_x = goal_x_i;
        goal_y = goal_y_i;
    }else{
        worldToMap(wx, wy, goal_x, goal_y);
    }

    //clear the starting cell within the costmap because we know it can't be an obstacle
    tf::Stamped<tf::Pose> start_pose;
    tf::poseStampedMsgToTF(start, start_pose);
    clearRobotCell(start_pose, start_x_i, start_y_i);

    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();

    //make sure to resize the underlying array that Navfn uses
    p_calc_->setSize(nx, ny);
    planner_->setSize(nx, ny);
    path_maker_->setSize(nx, ny);
    potential_array_ = new float[nx * ny];

    outlineMap(costmap_->getCharMap(), nx, ny, costmap_2d::LETHAL_OBSTACLE);

    bool found_legal = planner_->calculatePotentials(costmap_->getCharMap(), start_x, start_y, goal_x, goal_y,
                                                     nx * ny * 2, potential_array_);

    if(!old_navfn_behavior_)
        planner_->clearEndpoint(costmap_->getCharMap(), potential_array_, goal_x_i, goal_y_i, 2);
    if(publish_potential_)
        publishPotential(potential_array_);

    if (found_legal) {
        //extract the plan
        if (getPlanFromPotential(start_x, start_y, goal_x, goal_y, goal, plan)) {
            //make sure the goal we push on has the same timestamp as the rest of the plan
            geometry_msgs::PoseStamped goal_copy = goal;
            goal_copy.header.stamp = ros::Time::now();
            plan.push_back(goal_copy);
        } else {
            ROS_ERROR("Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
        }
    }else{
        ROS_ERROR("Failed to get a plan.");
    }

    // add orientations if needed
    orientation_filter_->processPath(start, plan);


    tf::Quaternion q(start.pose.orientation.x, start.pose.orientation.y, start.pose.orientation.z, start.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    double robot_oriention=yaw;

    if(plan.size() > PLAN_MIN_SIZE)
    {
//        ROS_INFO("plan start %lf , %lf",plan[10].pose.position.x,plan[10].pose.position.y);
//        ROS_INFO("robot start %lf , %lf",start.pose.position.x,start.pose.position.y);
//        ROS_INFO("plan size : %d",plan.size());
        last_plan_ = plan;
        fillCommand(laser_msg,QLineF(start.pose.position.x,start.pose.position.y,plan[PLAN_MIN_SIZE].pose.position.x,plan[PLAN_MIN_SIZE].pose.position.y),robot_oriention,command);

    }
    else
    {
        moving_robot_ = false;
        turning_robot_ = false;

        publishPlan(plan);

        delete potential_array_;

        return false;
//        std::stringstream bytes;
//        bytes << "*~";
//        bytes << 'f' << 0 << 'f' << 0;
//        bytes << '#';
//        command.data=bytes.str();

    }


    //publish the plan for visualization purposes
    publishPlan(plan);

    delete potential_array_;
    return !plan.empty();
}

std::vector<geometry_msgs::PoseStamped> GridNavigation::getLastPlan()
{
    return last_plan_;
}

bool GridNavigation::getPlanFromPotential(double start_x, double start_y, double goal_x, double goal_y,
                                          const geometry_msgs::PoseStamped& goal,
                                          std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) {
        ROS_ERROR(
                    "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    std::string global_frame = frame_id_;

    //clear the plan, just in case
    plan.clear();

    std::vector<std::pair<float, float> > path;

    if (!path_maker_->getPath(potential_array_, start_x, start_y, goal_x, goal_y, path)) {
        ROS_ERROR("NO PATH!");
        return false;
    }

    ros::Time plan_time = ros::Time::now();
    for (int i = path.size() -1; i>=0; i--) {
        std::pair<float, float> point = path[i];
        //convert the plan to world coordinates
        double world_x, world_y;
        mapToWorld(point.first, point.second, world_x, world_y);

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = global_frame;
        pose.pose.position.x = world_x;
        pose.pose.position.y = world_y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        plan.push_back(pose);
    }
    if(old_navfn_behavior_){
        plan.push_back(goal);
    }
    return !plan.empty();
}

void GridNavigation::publishPotential(float* potential)
{
    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
    double resolution = costmap_->getResolution();
    nav_msgs::OccupancyGrid grid;
    // Publish Whole Grid
    grid.header.frame_id = frame_id_;
    grid.header.stamp = ros::Time::now();
    grid.info.resolution = resolution;

    grid.info.width = nx;
    grid.info.height = ny;

    double wx, wy;
    costmap_->mapToWorld(0, 0, wx, wy);
    grid.info.origin.position.x = wx - resolution / 2;
    grid.info.origin.position.y = wy - resolution / 2;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(nx * ny);

    float max = 0.0;
    for (unsigned int i = 0; i < grid.data.size(); i++) {
        float potential = potential_array_[i];
        if (potential < POT_HIGH) {
            if (potential > max) {
                max = potential;
            }
        }
    }

    for (unsigned int i = 0; i < grid.data.size(); i++) {
        if (potential_array_[i] >= POT_HIGH) {
            grid.data[i] = -1;
        } else
            grid.data[i] = potential_array_[i] * publish_scale_ / max;
    }
    potential_pub_.publish(grid);
}

void GridNavigation::fillCommand(const sensor_msgs::LaserScan &laser_msg,QLineF line,double oriention,std_msgs::String &command)
{

    if(laser_msg.ranges.size()==726)
    {
        for(int i=44;i<laser_msg_size_+44;i++)
        {
            laser_data_[i-44]=laser_msg.ranges[i]*1000;
        }
    }

    std::stringstream bytes;
    bytes << "*~";


    double y_diff=line.p2().y()-line.p1().y();
    double x_diff=line.p2().x()-line.p1().x();
    double angle_diff=atan2(y_diff,x_diff)-oriention;

    {
        if(fabs(angle_diff)<ANGLE_THRESHOLD)
        {
            turning_robot_=false;
            moving_robot_=true;
        }
        else
        {
            turning_robot_=true;
            moving_robot_=false;
        }

    }
    if(turning_robot_==true)
    {
        if (angle_diff<0) {
            if(fabs(angle_diff)<M_PI)
            {
                //                qDebug()<<"turning Right";
                bytes << 'f' << ROBOT_TURNING_SPEED << 'r' << ROBOT_TURNING_SPEED;
                moving_direction_ = 0;
            }
            else
            {
                bytes << 'r' << ROBOT_TURNING_SPEED << 'f' << ROBOT_TURNING_SPEED;
                //                qDebug()<<"turning Left";
                moving_direction_ = 180;
            }

        }
        else
        {
            if(fabs(angle_diff)<M_PI)
            {
                bytes << 'r' << ROBOT_TURNING_SPEED << 'f' << ROBOT_TURNING_SPEED;
                //                qDebug()<<"turning Left";
                moving_direction_ = 180;
            }
            else
            {
                bytes << 'f' << ROBOT_TURNING_SPEED << 'r' << ROBOT_TURNING_SPEED;
                //                qDebug()<<"turning Right";
                moving_direction_ = 0;
            }
        }
    }
    else if(moving_robot_==true)
    {
        bytes << 'f' << ROBOT_SPEED << 'f' << ROBOT_SPEED;
        //        qDebug()      <<"moving forward";
        moving_direction_ = 90;
    }

    bytes << '#';
//    command.data=bytes.str();
    filterMovingDirection(command);

}
void GridNavigation::filterMovingDirection(std_msgs::String &command)
{
    ROS_ERROR(" moving_direction_ moving_direction_:%d",moving_direction_);
    bool forward_side_open = true;
    bool right_side_open = true;
    bool left_side_open = true;

    //------------------------------------------------------------------------------------------------------------------
    for (int i = FORWARD_SIDE_LIMIT_I_L; i <= FORWARD_SIDE_LIMIT_I_R; i++)
    {
        if ((laser_data_[i] >= LASER_DATA_MIN_DIST) && (laser_data_[i] < FORWARD_SIDE_LIMIT))
        {
            forward_side_open = false;
            break;
        }
    }
    //------------------------------------------------------------------------------------------------------------------
    for (int i = RIGHT_SIDE_LIMIT_I_L; i <= RIGHT_SIDE_LIMIT_I_R; i++)
    {
        if ((laser_data_[i] >= LASER_DATA_MIN_DIST) && (laser_data_[i] < RIGHT_SIDE_LIMIT))
        {
            right_side_open = false;
            break;
        }
    }
    //------------------------------------------------------------------------------------------------------------------
    for (int i = LEFT_SIDE_LIMIT_I_L; i <= LEFT_SIDE_LIMIT_I_R; i++)
    {
        if ((laser_data_[i] >= LASER_DATA_MIN_DIST) && (laser_data_[i] < LEFT_SIDE_LIMIT))
        {
            left_side_open = false;
            break;
        }
    }

    ROS_WARN("%d , %d , %d",left_side_open,forward_side_open,right_side_open);
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
    if(force_turn_counter_>FORCE_TURN_COUNTER && (!forward_side_open) && (!right_side_open) && (!left_side_open) )
    {
        force_turn_active_=true;
    }

    if(force_turn_active_ && forward_side_open)
    {
        force_turn_active_=false;
        force_turn_counter_=0;
    }
    if(force_turn_active_)
    {
        moving_direction_=LEFT_MOVING_DIRECTION;
        ROS_INFO("\e[5;91mForce Turn Force Turn \e[0m");
    }

    ROS_ERROR(" moving_direction_ moving_direction_ moving_direction_:%d",moving_direction_);

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
    if(moving_direction_ == -90 && force_turn_active_== false)
    {
        bytes << 'f' << 0 << 'f' << 0;
    }
    bytes << '#';
    command.data=bytes.str();

}
