/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
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
 *   * Neither the name of the institute nor the names of its
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
 * Notes:
 * The following class is derived from a class defined by the
 * g2o-framework. g2o is licensed under the terms of the BSD License.
 * Refer to the base class source for detailed licensing information.
 *
 * Author: Christoph Rösmann, Franz Albers
 *********************************************************************/

#ifndef EDGE_DYNAMICOBSTACLE_H
#define EDGE_DYNAMICOBSTACLE_H

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/vertex_timediff.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/obstacles.h>
#include <teb_local_planner/teb_config.h>
#include <teb_local_planner/robot_footprint_model.h>

namespace teb_local_planner
{
  
/**
 * @class EdgeDynamicObstacle
 * @brief Edge defining the cost function for keeping a distance from dynamic (moving) obstacles.
 * 
 * The edge depends on two vertices \f$ \mathbf{s}_i, \Delta T_i \f$ and minimizes: \n
 * \f$ \min \textrm{penaltyBelow}( dist2obstacle) \cdot weight \f$. \n
 * \e dist2obstacle denotes the minimum distance to the obstacle trajectory (spatial and temporal). \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyBelow denotes the penalty function, see penaltyBoundFromBelow(). \n
 * @see TebOptimalPlanner::AddEdgesDynamicObstacles
 * @remarks Do not forget to call setTebConfig(), setVertexIdx() and 
 * @warning Experimental
 */  
class EdgeDynamicObstacle : public BaseTebUnaryEdge<2, const Obstacle*, VertexPose>
{
public:
  
  /**
   * @brief Construct edge.
   */    
  EdgeDynamicObstacle() : t_(0)
  {
  }
  
  /**
   * @brief Construct edge and specify the time for its associated pose (neccessary for computeError).
   * @param t_ Estimated time until current pose is reached
   */      
  EdgeDynamicObstacle(double t,double shrink_ratio,int lag_optimal) : t_(t),shrink_ratio_(shrink_ratio),lag_optimal_(lag_optimal)
  {
  }
  
  /**
   * @brief Actual cost function
   */   
  void computeError()
  {
    setlocale(LC_ALL,"");
    ROS_ASSERT_MSG(cfg_ && _measurement && robot_model_, "You must call setTebConfig(), setObstacle() and setRobotModel() on EdgeDynamicObstacle()");
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);
    
    double dist = robot_model_->estimateSpatioTemporalDistance(bandpt->pose(), _measurement, t_,shrink_ratio_) ;
    normalvectorOA = robot_model_->estimateNormalVector(bandpt->pose(), _measurement, t_,shrink_ratio_);//求向外推的法向量
    double future_distance = robot_model_->estimateSpatioTemporalDistance(bandpt->pose(), _measurement, t_,2);//未来的位置*2
    double obstacle_distance = robot_model_->estimateObstacleFutureDistance(bandpt->pose(), _measurement, t_,shrink_ratio_);

    if (lag_optimal_ == 0)
    {
      lag = 0;
      // ROS_INFO("777");
    }
    else
    {
      information_obs_pose = bandpt->pose().position() + normalvectorOA * future_distance;
      future_distance = robot_model_->estimateSpatioTemporalDistance(bandpt->pose(), _measurement, t_, 2, information_obs_pose);
      obstacle_distance = robot_model_->estimateObstacleFutureDistance(bandpt->pose(), _measurement, t_, shrink_ratio_, information_obs_pose);
      lag = 1;
      // ROS_INFO("666");
    }

    // //得到障碍物前面一系列点
    // if (shrink_ratio_ == 0)
    // {
    //   lag = 0;
    // }
    // else
    // {
    //   if (dist < 0)
    //   {
    //     information_obs_pose = bandpt->pose().position() + normalvectorOA * future_distance;
    //     future_distance = robot_model_->estimateSpatioTemporalDistance(bandpt->pose(), _measurement, t_, 2, information_obs_pose);
    //     obstacle_distance = robot_model_->estimateObstacleFutureDistance(bandpt->pose(), _measurement, t_, shrink_ratio_, information_obs_pose);
    //     lag = 1;
    //     ROS_INFO("666");
    //   }
    //   else
    //   {
    //     lag = 0;
    //   }
    // }
    
    // ROS_INFO("距离为:%f",information_obs_pose.x());

    // double dist = robot_model_->estimateSpatioTemporalDistance(bandpt->pose(), _measurement, t_,shrink_ratio_);
    // ROS_INFO("距离为:%f",dist);
    // double future_distance = robot_model_->estimateSpatioTemporalDistance(bandpt->pose(), _measurement, t_,1);//没有放缩未来的位置
    // double obstacle_distance = robot_model_->estimateObstacleFutureDistance(bandpt->pose(), _measurement, t_,shrink_ratio_);
    
    // if (dist <= 0.6)//机器人轨迹点位于行人内
    // {
    //   lag = 1;//标志变量为1，将轨迹向行人前进方向推离
    // }
    // else
    // {
    //   lag = 0;
    // }  
    // if (dist < 0)//机器人轨迹点位于行人内
    // {
    //   lag = 1;//标志变量为1，将轨迹向行人前进方向推离
    // }
    // else
    // {
    //   lag = 0;
    // }  
    _error[0] = penaltyBoundFromBelow(dist, cfg_->obstacles.min_obstacle_dist, cfg_->optim.penalty_epsilon,future_distance,obstacle_distance,lag, bandpt->pose(),costmap2d_, obs_map_labeled_);
    _error[1] = penaltyBoundFromBelow(dist, cfg_->obstacles.dynamic_obstacle_inflation_dist, 0.0,future_distance, obstacle_distance,lag,bandpt->pose(),costmap2d_, obs_map_labeled_);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeDynamicObstacle::computeError() _error[0]=%f\n",_error[0]);
  }
  
  double future_distance()
  {
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);
    double dist = robot_model_->estimateSpatioTemporalDistance(bandpt->pose(), _measurement, t_,shrink_ratio_);
    return dist;
  }
  
  void setCostmap(costmap_2d::Costmap2D* costmap){costmap2d_=costmap;};

  void setObsMapLabeled(const std::vector<std::vector<int>>& obs_map_labeled){obs_map_labeled_=obs_map_labeled;};
  /**
   * @brief Set Obstacle for the underlying cost function
   * @param obstacle Const pointer to an Obstacle or derived Obstacle
   */     
  void setObstacle(const Obstacle* obstacle)
  {
    _measurement = obstacle;
  }
  
  /**
   * @brief Set pointer to the robot model
   * @param robot_model Robot model required for distance calculation
   */
  void setRobotModel(const BaseRobotFootprintModel* robot_model)
  {
    robot_model_ = robot_model;
  }

  /**
   * @brief Set all parameters at once
   * @param cfg TebConfig class
   * @param robot_model Robot model required for distance calculation
   * @param obstacle 2D position vector containing the position of the obstacle
   */
  void setParameters(const TebConfig& cfg, const BaseRobotFootprintModel* robot_model, const Obstacle* obstacle)
  {
    cfg_ = &cfg;
    robot_model_ = robot_model;
    _measurement = obstacle;
  }

protected:
  
  const BaseRobotFootprintModel* robot_model_; //!< Store pointer to robot_model
  double t_; //!< Estimated time until current pose is reached
  double shrink_ratio_;
  int lag_optimal_;
  costmap_2d::Costmap2D* costmap2d_;
  std::vector<std::vector<int>> obs_map_labeled_;
  Eigen::Vector2d normalvectorOA;//用于生成障碍物前面一系列点的单位向量
  Eigen::Vector2d information_obs_pose;//存储生成障碍物前面一系列点的向量

public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int lag;

};
    
 
    

} // end namespace

#endif
