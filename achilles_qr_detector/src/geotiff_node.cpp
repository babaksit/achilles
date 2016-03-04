//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <cstdio>
#include <ros/ros.h>
#include <ros/console.h>
#include <pluginlib/class_loader.h>

#include <boost/algorithm/string.hpp>


#include "nav_msgs/GetMap.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Quaternion.h"


#include <eigen3/Eigen/Geometry>

#include <achilles_qr_detector//qrPos.h>
#include <achilles_qr_detector/victimPos.h>

#include <QtGui/QApplication>

#include <hector_map_tools/HectorMapTools.h>

#include "geotiff_writer.h"
#include "map_writer_plugin_interface.h"

#include "QFile"
#include <hector_nav_msgs/GetRobotTrajectory.h>
#include "QTextStream"

using namespace std;

struct position
{
    double x;
    double y;
    string context;
};

namespace hector_geotiff{

/**
 * @brief Map generation node.
 */
class MapGenerator
{
    int numberOfVictims;
    int numberOfQRs;
    struct position victim_position[1000];
    struct position qr_position[1000];
    ros::NodeHandle nh;
    ros::Subscriber victim_sub;
    ros::Subscriber qr_sub;

public:
    void victimPositionCallback(const achilles_qr_detector::victimPos& msg)
      {
        if(msg.true_victim)
        {
            victim_position[numberOfVictims].x=msg.pos_x;
            victim_position[numberOfVictims].y=msg.pos_y;
            victim_position[numberOfVictims].context="";
            numberOfVictims++;
        }
      }

      void QRPositionCallback(const achilles_qr_detector::qrPos& msg)
      {
          qr_position[numberOfQRs].x=msg.pos_x;
          qr_position[numberOfQRs].y=msg.pos_y;
          qr_position[numberOfQRs].context=msg.context;
          numberOfQRs++;
      }

    MapGenerator()
    : geotiff_writer_(false)
    , pn_("~")    
    , plugin_loader_(0)
    , running_saved_map_num_(0)
  {
    numberOfVictims=0;
    numberOfQRs=0;
    pn_.param("map_file_path", p_map_file_path_, std::string("."));
    geotiff_writer_.setMapFilePath(p_map_file_path_);
    geotiff_writer_.setUseUtcTimeSuffix(true);

    pn_.param("map_file_base_name", p_map_file_base_name_, std::string());

    pn_.param("draw_background_checkerboard", p_draw_background_checkerboard_, true);
    pn_.param("draw_free_space_grid", p_draw_free_space_grid_, true);

    sys_cmd_sub_ = n_.subscribe("syscommand", 1, &MapGenerator::sysCmdCallback, this);

    map_service_client_ = n_.serviceClient<nav_msgs::GetMap>("map");
    //object_service_client_ = n_.serviceClient<worldmodel_msgs::GetObjectModel>("worldmodel/get_object_model");
    path_service_client_ = n_.serviceClient<hector_nav_msgs::GetRobotTrajectory>("trajectory");


    double p_geotiff_save_period = 0.0;
    pn_.param("geotiff_save_period", p_geotiff_save_period, 0.0);

    if(p_geotiff_save_period > 0.0){
      //ros::Timer timer = pn_.createTimer(ros::Duration(p_geotiff_save_period), &MapGenerator::timerSaveGeotiffCallback, false);
      //publish_trajectory_timer_ = private_nh.createTimer(ros::Duration(1.0 / p_trajectory_publish_rate_), &PathContainer::publishTrajectoryTimerCallback, this, false);
      map_save_timer_ = pn_.createTimer(ros::Duration(p_geotiff_save_period), &MapGenerator::timerSaveGeotiffCallback, this, false );
    }


    pn_.param("plugins", p_plugin_list_, std::string(""));

    std::vector<std::string> plugin_list;
    boost::algorithm::split(plugin_list, p_plugin_list_, boost::is_any_of("\t "));

    victim_sub=nh.subscribe("victim_position",1,&MapGenerator::victimPositionCallback,this);
    qr_sub=nh.subscribe("QR_position",1,&MapGenerator::QRPositionCallback,this);

    //We always have at least one element containing "" in the string list
    if ((plugin_list.size() > 0) && (plugin_list[0].length() > 0)){
      plugin_loader_ = new pluginlib::ClassLoader<hector_geotiff::MapWriterPluginInterface>("hector_geotiff", "hector_geotiff::MapWriterPluginInterface");

      for (size_t i = 0; i < plugin_list.size(); ++i){
        try
        {
          boost::shared_ptr<hector_geotiff::MapWriterPluginInterface> tmp (plugin_loader_->createInstance(plugin_list[i]));
          tmp->initialize(plugin_loader_->getName(plugin_list[i]));
          plugin_vector_.push_back(tmp);
        }
        catch(pluginlib::PluginlibException& ex)
        {
          ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
        }
      }
    }else{
      ROS_INFO("No plugins loaded for geotiff node");
    }

    ROS_INFO("Geotiff node started");
  }

  ~MapGenerator()
  {
    if (plugin_loader_){
      delete plugin_loader_;
    }
  }

  void writeGeotiff()
  {
    ros::Time start_time (ros::Time::now());

    std::stringstream ssStream;

    nav_msgs::GetMap srv_map;
    if (map_service_client_.call(srv_map))
    {
      ROS_INFO("GeotiffNode: Map service called successfully");
      const nav_msgs::OccupancyGrid& map (srv_map.response.map);

      std::string map_file_name = p_map_file_base_name_;
      std::string competition_name;
      std::string team_name;
      std::string mission_name;
      std::string postfix;
      if (n_.getParamCached("/competition", competition_name) && !competition_name.empty()) map_file_name = map_file_name + "_" + competition_name;
      if (n_.getParamCached("/team", team_name)               && !team_name.empty())        map_file_name = map_file_name + "_" + team_name;
      if (n_.getParamCached("/mission", mission_name)         && !mission_name.empty())     map_file_name = map_file_name + "_" + mission_name;
      if (pn_.getParamCached("map_file_postfix", postfix)     && !postfix.empty())          map_file_name = map_file_name + "_" + postfix;
      if (map_file_name.substr(0, 1) == "_") map_file_name = map_file_name.substr(1);
      if (map_file_name.empty()) map_file_name = "GeoTiffMap";
      geotiff_writer_.setMapFileName(map_file_name);
      bool transformSuccess = geotiff_writer_.setupTransforms(map);

      if(!transformSuccess){
        ROS_INFO("Couldn't set map transform");
        return;
      }

      geotiff_writer_.setupImageSize();

      if (p_draw_background_checkerboard_){
        geotiff_writer_.drawBackgroundCheckerboard();
      }

      geotiff_writer_.drawMap(map, p_draw_free_space_grid_);
      geotiff_writer_.drawCoords();
//      geotiff_writer_.drawQR(Eigen::Vector2f(0,0),5);


      //ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    }
    else
    {
      ROS_ERROR("Failed to call map service");
      return;
    }

    for (size_t i = 0; i < plugin_vector_.size(); ++i){
      plugin_vector_[i]->draw(&geotiff_writer_);
    }

    /**
      * No Victims for now, first  agree on a common standard for representation
      */
    /*
    if (req_object_model_){
      worldmodel_msgs::GetObjectModel srv_objects;
      if (object_service_client_.call(srv_objects))
      {
        ROS_INFO("GeotiffNode: Object service called successfully");

        const worldmodel_msgs::ObjectModel& objects_model (srv_objects.response.model);

        size_t size = objects_model.objects.size();


        unsigned int victim_num  = 1;

        for (size_t i = 0; i < size; ++i){
          const worldmodel_msgs::Object& object (objects_model.objects[i]);

          if (object.state.state == worldmodel_msgs::ObjectState::CONFIRMED){
            geotiff_writer_.drawVictim(Eigen::Vector2f(object.pose.pose.position.x,object.pose.pose.position.y),victim_num);
            victim_num++;
          }
        }
      }
      else
      {
        ROS_ERROR("Failed to call objects service");
      }
    }
    */

    for(int i=0;i<numberOfVictims;i++)
    {
        geotiff_writer_.drawVictim(Eigen::Vector2f(victim_position[i].x,victim_position[i].y),i);
    }

    bool my_check=true;
    std::string add=p_map_file_path_+"/strings.txt";
    QFile file(QString::fromStdString(add));
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        my_check=false;
    }

    for(int i=0;i<numberOfQRs;i++)
    {
        geotiff_writer_.drawQR(Eigen::Vector2f(qr_position[i].x,qr_position[i].y),i);
        ROS_INFO_STREAM(qr_position[i].context);
        if (my_check)
        {
            QTextStream out(&file);
            stringstream ss;
            ss << i;
            string number = ss.str();
            stringstream ss_x;
            ss_x << qr_position[i].x;
            string x_str = ss_x.str();
            stringstream ss_y;
            ss_y << qr_position[i].y;
            string y_str = ss_y.str();
            QString t2(QString::fromStdString("QR"+number+"\tX:"+x_str+"\tY:"+y_str+"\tContext:"+qr_position[i].context+"\n"));
            out << t2;
        }
        else
        {
            ROS_FATAL_STREAM("Can't open text file for writing QR's contexts!");
        }
    }
    file.close();

    hector_nav_msgs::GetRobotTrajectory srv_path;

    /*if (path_service_client_.call(srv_path))
    {
      ROS_INFO("GeotiffNode: Path service called successfully");

      std::vector<geometry_msgs::PoseStamped>& traj_vector (srv_path.response.trajectory.poses);

      size_t size = traj_vector.size();

      std::vector<Eigen::Vector2f> pointVec;
      pointVec.resize(size);

      for (size_t i = 0; i < size; ++i){
        const geometry_msgs::PoseStamped& pose (traj_vector[i]);

        pointVec[i] = Eigen::Vector2f(pose.pose.position.x, pose.pose.position.y);
      }

      if (size > 0){
        //Eigen::Vector3f startVec(pose_vector[0].x,pose_vector[0].y,pose_vector[0].z);
        Eigen::Vector3f startVec(pointVec[0].x(),pointVec[0].y(),0.0f);
        geotiff_writer_.drawPath(startVec, pointVec);
      }
    }
    else
    {
      ROS_ERROR("Failed to call path service");
    }
*/


    geotiff_writer_.writeGeotiffImage();
    running_saved_map_num_++;

    ros::Duration elapsed_time (ros::Time::now() - start_time);

    ROS_INFO("GeoTiff created in %f seconds", elapsed_time.toSec());
  }

  void timerSaveGeotiffCallback(const ros::TimerEvent& e)
  {
    this->writeGeotiff();
  }

  void sysCmdCallback(const std_msgs::String& sys_cmd)
  {
    if ( !(sys_cmd.data == "savegeotiff")){
      return;
    }

    this->writeGeotiff();
  }

  std::string p_map_file_path_;
  std::string p_map_file_base_name_;
  std::string p_plugin_list_;
  bool p_draw_background_checkerboard_;
  bool p_draw_free_space_grid_;

  //double p_geotiff_save_period_;

  GeotiffWriter geotiff_writer_;

  ros::ServiceClient map_service_client_;// = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
  ros::ServiceClient object_service_client_;
  ros::ServiceClient path_service_client_;

  ros::Subscriber sys_cmd_sub_;

  ros::NodeHandle n_;
  ros::NodeHandle pn_;


  std::vector<boost::shared_ptr<hector_geotiff::MapWriterPluginInterface> > plugin_vector_;

  pluginlib::ClassLoader<hector_geotiff::MapWriterPluginInterface>* plugin_loader_;

  ros::Timer map_save_timer_;

  unsigned int running_saved_map_num_;

};

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kn2c_geotiff");

  hector_geotiff::MapGenerator mg;

  ros::spin();

  return 0;
}

