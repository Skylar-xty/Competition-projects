/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
// #include<global_planner/astar.h>
// #include<costmap_2d/cost_values.h>

// namespace global_planner {

// AStarExpansion::AStarExpansion(PotentialCalculator* p_calc, int xs, int ys) :
//         Expander(p_calc, xs, ys) {
// }

// bool AStarExpansion::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
//                                         int cycles, float* potential) {
//     queue_.clear();
//     int start_i = toIndex(start_x, start_y);
//     queue_.push_back(Index(start_i, 0));

//     std::fill(potential, potential + ns_, POT_HIGH);
//     potential[start_i] = 0;

//     int goal_i = toIndex(end_x, end_y);
//     int cycle = 0;

//     while (queue_.size() > 0 && cycle < cycles) {
//         Index top = queue_[0];
//         std::pop_heap(queue_.begin(), queue_.end(), greater1());
//         queue_.pop_back();

//         int i = top.i;
//         if (i == goal_i)
//             return true;

//         add(costs, potential, potential[i], i + 1, end_x, end_y);
//         add(costs, potential, potential[i], i - 1, end_x, end_y);
//         add(costs, potential, potential[i], i + nx_, end_x, end_y);
//         add(costs, potential, potential[i], i - nx_, end_x, end_y);

//         cycle++;
//     }

//     return false;
// }

// void AStarExpansion::add(unsigned char* costs, float* potential, float prev_potential, int next_i, int end_x,
//                          int end_y) {
//     if (next_i < 0 || next_i >= ns_)
//         return;

//     if (potential[next_i] < POT_HIGH)
//         return;

//     if(costs[next_i]>=lethal_cost_ && !(unknown_ && costs[next_i]==costmap_2d::NO_INFORMATION))
//         return;

//     potential[next_i] = p_calc_->calculatePotential(potential, costs[next_i] + neutral_cost_, next_i, prev_potential);
//     int x = next_i % nx_, y = next_i / nx_;
//     float distance = abs(end_x - x) + abs(end_y - y);

//     queue_.push_back(Index(next_i, potential[next_i] + distance * neutral_cost_));
//     std::push_heap(queue_.begin(), queue_.end(), greater1());
// }

// } //end namespace global_planner

/////////////////////////平移起点版//////////////////////////////
// #include <stdio.h>
// #include <fcntl.h>
// #include <global_planner/astar.h>
// #include <costmap_2d/cost_values.h>

// namespace global_planner {

// int cmap[2333][2333];//暂定大小，具体要看图的尺寸 
// bool is_init=false;

// AStarExpansion::AStarExpansion(PotentialCalculator* p_calc, int xs, int ys) :
//         Expander(p_calc, xs, ys) {
// }

// bool AStarExpansion::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
//                                         int cycles, float* potential) {
    
//     queue_.clear();
//     ros::param::set("end_xxx",end_x);
//     ros::param::set("end_yyy",end_y);
//     int start_i;
//     if(end_x>150&&end_x<160)
//     {
//         start_i = toIndex(start_x, (start_y-2.0));
//     }
//     else
//         start_i = toIndex(start_x, start_y);
//     queue_.push_back(Index(start_i, 0));

//     std::fill(potential, potential + ns_, POT_HIGH);
//     potential[start_i] = 0;

//     int goal_i = toIndex(end_x, end_y);
//     int cycle = 0;

//     while (queue_.size() > 0 && cycle < cycles) {
//         Index top = queue_[0];
//         std::pop_heap(queue_.begin(), queue_.end(), greater1());
//         queue_.pop_back();

//         int i = top.i;
//         if (i == goal_i)
//             return true;

//         add(costs, potential, potential[i], i + 1, end_x, end_y);
//         //if(!(end_x>-2.1&&end_x<-1.98)){
//         add(costs, potential, potential[i], i - 1, end_x, end_y);//}
//         add(costs, potential, potential[i], i + nx_, end_x, end_y);
//         add(costs, potential, potential[i], i - nx_, end_x, end_y);

//         cycle++;
//     }
	
//     return false;
// }

// void AStarExpansion::add(unsigned char* costs, float* potential, float prev_potential, int next_i, int end_x,
//                          int end_y) {
//     if (next_i < 0 || next_i >= ns_)
//         return;

//     if (potential[next_i] < POT_HIGH)
//         return;

//     if(costs[next_i]>=lethal_cost_ && !(unknown_ && costs[next_i]==costmap_2d::NO_INFORMATION))
//         return;

//     potential[next_i] = p_calc_->calculatePotential(potential, costs[next_i] + neutral_cost_, next_i, prev_potential);
//     int x = next_i % nx_, y = next_i / nx_;
// 	float distance;
// 	// if(end_x>-2.1&&end_x<-1.98)
// 	// {
// 	// 	distance = abs(end_x - x) + abs(end_y - y) +60*(abs(2.0974-x) + abs(-6.8396-y));
// 	// }
//     //else 
//     distance = abs(end_x - x) + abs(end_y - y);

	 
//     queue_.push_back(Index(next_i, potential[next_i] + distance * neutral_cost_));
//     std::push_heap(queue_.begin(), queue_.end(), greater1());
// }

// } //end namespace global_planner


/////////////长方形障碍版/////////////////
#include <stdio.h>
#include <fcntl.h>
#include <global_planner/astar.h>
#include <costmap_2d/cost_values.h>

namespace global_planner {

bool is_init=false;
int start_i;
bool is2=false;
AStarExpansion::AStarExpansion(PotentialCalculator* p_calc, int xs, int ys) :
        Expander(p_calc, xs, ys) {
}

bool AStarExpansion::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                        int cycles, float* potential) {
    
    queue_.clear();
    start_i = toIndex(start_x, start_y);
    queue_.push_back(Index(start_i, 0));

    std::fill(potential, potential + ns_, POT_HIGH);
    potential[start_i] = 0;
    ros::param::set("end_xxx",end_x);
    ros::param::set("end_yyy",end_y);
    int goal_i = toIndex(end_x, end_y);
    int cycle = 0;
    if(end_x>150&&end_x<160)
        is2=true;
    else
        is2=false;

    while (queue_.size() > 0 && cycle < cycles) {
        Index top = queue_[0];
        std::pop_heap(queue_.begin(), queue_.end(), greater1());
        queue_.pop_back();

        int i = top.i;
        if (i == goal_i)
            return true;

        add(costs, potential, potential[i], i + 1, end_x, end_y);
        add(costs, potential, potential[i], i - 1, end_x, end_y);
        add(costs, potential, potential[i], i + nx_, end_x, end_y);
        add(costs, potential, potential[i], i - nx_, end_x, end_y);

        cycle++;
    }
	
    return false;
}

void AStarExpansion::add(unsigned char* costs, float* potential, float prev_potential, int next_i, int end_x,
                         int end_y) {
    if (next_i < 0 || next_i >= ns_)
        return;
    if(next_i/nx_>=start_i/nx_+2  && \
       next_i/nx_<=start_i/nx_+10 && \
       next_i%nx_>=start_i%nx_-10 && \
       next_i%nx_<=start_i%nx_+10 && is2)
        return;
    if (potential[next_i] < POT_HIGH)
        return;

    if(costs[next_i]>=lethal_cost_ && !(unknown_ && costs[next_i]==costmap_2d::NO_INFORMATION))
        return;

    potential[next_i] = p_calc_->calculatePotential(potential, costs[next_i] + neutral_cost_, next_i, prev_potential);
    int x = next_i % nx_, y = next_i / nx_;
	float distance;
	// if(end_x>-2.1&&end_x<-1.98)
	// {
	// 	distance = abs(end_x - x) + abs(end_y - y) +60*(abs(2.0974-x) + abs(-6.8396-y));
	// }
    //else 
    distance = abs(end_x - x) + abs(end_y - y);

	 
    queue_.push_back(Index(next_i, potential[next_i] + distance * neutral_cost_));
    std::push_heap(queue_.begin(), queue_.end(), greater1());
}

} //end namespace global_planner