#ifndef GRID_NAVIGATION_H
#define GRID_NAVIGATION_H


#define POT_HIGH 1.0e10        // unassigned cell potential
#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <dynamic_reconfigure/server.h>
#include <achilles_navigation/potential_calculator.h>
#include <achilles_navigation/expander.h>
#include <achilles_navigation/traceback.h>
#include <achilles_navigation/orientation_filter.h>
#include <std_msgs/String.h>
#include <QObject>
#include <QLineF>
#include <sensor_msgs/LaserScan.h>





#define laser_msg_size_ 682


class Expander;
class GridPath;


class GridNavigation : public QObject{
    public:

        GridNavigation(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);

        ~GridNavigation();

        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);


        bool computePotential(const geometry_msgs::Point& world_point);
        bool getPlanFromPotential(double start_x, double start_y, double end_x, double end_y,
                                  const geometry_msgs::PoseStamped& goal,
                                  std::vector<geometry_msgs::PoseStamped>& plan);
        double getPointPotential(const geometry_msgs::Point& world_point);
        bool validPointPotential(const geometry_msgs::Point& world_point);
        bool validPointPotential(const geometry_msgs::Point& world_point, double tolerance);
        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
        bool findPath(const sensor_msgs::LaserScan &laser_msg,const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                      std_msgs::String &command);

//        bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);
        std::vector<geometry_msgs::PoseStamped> getLastPlan();
        void fillCommand(const sensor_msgs::LaserScan &laser_msg,QLineF line, double oriention,std_msgs::String &command);
protected:
        costmap_2d::Costmap2D* costmap_;
        std::string frame_id_;
        ros::Publisher plan_pub_;
        bool initialized_, allow_unknown_, visualize_potential_;

    private:
        std::vector<geometry_msgs::PoseStamped> last_plan_;

        bool moving_robot_;
        bool turning_robot_;
        void mapToWorld(double mx, double my, double& wx, double& wy);
        bool worldToMap(double wx, double wy, double& mx, double& my);
        void clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my);
        void publishPotential(float* potential);

        void filterMovingDirection(std_msgs::String &command);
        double laser_data_[laser_msg_size_];
        int moving_direction_;
        bool force_turn_active_;
        int force_turn_counter_;

        double planner_window_x_, planner_window_y_, default_tolerance_;
        std::string tf_prefix_;
        boost::mutex mutex_;
        ros::ServiceServer make_plan_srv_;

        PotentialCalculator* p_calc_;
        Expander* planner_;
        Traceback* path_maker_;
        OrientationFilter* orientation_filter_;

        bool publish_potential_;
        ros::Publisher potential_pub_;
        int publish_scale_;

        void outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value);
        unsigned char* cost_array_;
        float* potential_array_;
        unsigned int start_x_, start_y_, end_x_, end_y_;

        bool old_navfn_behavior_;
        float convert_offset_;


        void reconfigureCB();

};


#endif
