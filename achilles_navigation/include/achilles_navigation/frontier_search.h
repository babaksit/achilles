#ifndef FRONTIER_SEARCH_H
#define FRONTIER_SEARCH_H

#include "QObject"
#include "ros/ros.h"
#include <boost/thread.hpp>
#include "qdebug.h"
#include "std_msgs/String.h"
#include "costmap_2d/costmap_2d.h"
#include <costmap_2d/cost_values.h>
#include <boost/foreach.hpp>
#include <frontier_exploration/Frontier.h>




class FrontierSearch : public QObject
{
    Q_OBJECT
public:
    explicit FrontierSearch(ros::NodeHandle *nh, costmap_2d::Costmap2D* costmap);

    std::list<frontier_exploration::Frontier> searchFrom(geometry_msgs::Point position);
    ~FrontierSearch();

private:

    frontier_exploration::Frontier buildNewFrontier(unsigned int initial_cell, unsigned int reference, std::vector<bool>& frontier_flag);
    bool isNewFrontierCell(unsigned int idx, const std::vector<bool>& frontier_flag);
    unsigned char* map_;
    unsigned int size_x_ , size_y_;
    ros::NodeHandle *nh_;
    costmap_2d::Costmap2D* costmap_;

    std::vector<unsigned int> nhood4(unsigned int idx, const costmap_2d::Costmap2D& costmap){
        //get 4-connected neighbourhood indexes, check for edge of map
        std::vector<unsigned int> out;

        unsigned int size_x = costmap.getSizeInCellsX(), size_y = costmap.getSizeInCellsY();

        if (idx > size_x * size_y -1){
            ROS_WARN("Evaluating nhood for offmap point");
            return out;
        }

        if(idx % size_x > 0){
            out.push_back(idx - 1);
        }
        if(idx % size_x < size_x - 1){
            out.push_back(idx + 1);
        }
        if(idx >= size_x){
            out.push_back(idx - size_x);
        }
        if(idx < size_x*(size_y-1)){
            out.push_back(idx + size_x);
        }
        return out;

    }

    std::vector<unsigned int> nhood8(unsigned int idx, const costmap_2d::Costmap2D& costmap){
        //get 8-connected neighbourhood indexes, check for edge of map
        std::vector<unsigned int> out = nhood4(idx, costmap);

        unsigned int size_x_ = costmap.getSizeInCellsX(), size_y_ = costmap.getSizeInCellsY();

        if (idx > size_x_ * size_y_ -1){
            return out;
        }

        if(idx % size_x_ > 0 && idx >= size_x_){
            out.push_back(idx - 1 - size_x_);
        }
        if(idx % size_x_ > 0 && idx < size_x_*(size_y_-1)){
            out.push_back(idx - 1 + size_x_);
        }
        if(idx % size_x_ < size_x_ - 1 && idx >= size_x_){
            out.push_back(idx + 1 - size_x_);
        }
        if(idx % size_x_ < size_x_ - 1 && idx < size_x_*(size_y_-1)){
            out.push_back(idx + 1 + size_x_);
        }

        return out;

    }

    bool nearestCell(unsigned int &result, unsigned int start, unsigned char val, const costmap_2d::Costmap2D& costmap){

        const unsigned char* map = costmap.getCharMap();
        const unsigned int size_x = costmap.getSizeInCellsX(), size_y = costmap.getSizeInCellsY();

        if(start >= size_x * size_y){
            return false;
        }

        //initialize breadth first search
        std::queue<unsigned int> bfs;
        std::vector<bool> visited_flag(size_x * size_y, false);

        //push initial cell
        bfs.push(start);
        visited_flag[start] = true;

        //search for neighbouring cell matching value
        while(!bfs.empty()){
            unsigned int idx = bfs.front();
            bfs.pop();

            //return if cell of correct value is found
            if(map[idx] == val){
                result = idx;
                return true;
            }

            //iterate over all adjacent unvisited cells
            BOOST_FOREACH(unsigned nbr, nhood8(idx, costmap)){
                if(!visited_flag[nbr]){
                    bfs.push(nbr);
                    visited_flag[nbr] = true;
                }
            }
        }

        return false;
    }


};

#endif
