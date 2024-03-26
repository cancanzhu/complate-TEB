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
 * Author: Christoph Rösmann
 *********************************************************************/

#include <teb_local_planner/optimal_planner.h>

// g2o custom edges and vertices for the TEB planner
#include <teb_local_planner/g2o_types/edge_velocity.h>
#include <teb_local_planner/g2o_types/edge_velocity_obstacle_ratio.h>
#include <teb_local_planner/g2o_types/edge_acceleration.h>
#include <teb_local_planner/g2o_types/edge_kinematics.h>
#include <teb_local_planner/g2o_types/edge_time_optimal.h>
#include <teb_local_planner/g2o_types/edge_shortest_path.h>
#include <teb_local_planner/g2o_types/edge_obstacle.h>
#include <teb_local_planner/g2o_types/edge_dynamic_obstacle.h>
#include <teb_local_planner/g2o_types/edge_via_point.h>
#include <teb_local_planner/g2o_types/edge_prefer_rotdir.h>

#include <memory>
#include <limits>


namespace teb_local_planner
{

// ============== Implementation ===================

TebOptimalPlanner::TebOptimalPlanner() : cfg_(NULL), obstacles_(NULL), via_points_(NULL), cost_(HUGE_VAL), prefer_rotdir_(RotType::none),
                                         robot_model_(new PointRobotFootprint()), initialized_(false), optimized_(false)
{    
}
  
TebOptimalPlanner::TebOptimalPlanner(const TebConfig& cfg, ObstContainer* obstacles, RobotFootprintModelPtr robot_model, TebVisualizationPtr visual, const ViaPointContainer* via_points)
{    
  initialize(cfg, obstacles, robot_model, visual, via_points);
}

TebOptimalPlanner::~TebOptimalPlanner()
{
  clearGraph();
  // free dynamically allocated memory
  //if (optimizer_) 
  //  g2o::Factory::destroy();
  //g2o::OptimizationAlgorithmFactory::destroy();
  //g2o::HyperGraphActionLibrary::destroy();
}

void TebOptimalPlanner::updateRobotModel(RobotFootprintModelPtr robot_model)
{
  robot_model_ = robot_model;
}

void TebOptimalPlanner::initialize(const TebConfig& cfg, ObstContainer* obstacles, RobotFootprintModelPtr robot_model, TebVisualizationPtr visual, const ViaPointContainer* via_points)
{    
  // init optimizer (set solver and block ordering settings)
  optimizer_ = initOptimizer();
  
  cfg_ = &cfg;
  obstacles_ = obstacles;
  robot_model_ = robot_model;
  via_points_ = via_points;
  cost_ = HUGE_VAL;
  prefer_rotdir_ = RotType::none;
  setVisualization(visual);
  
  vel_start_.first = true;
  vel_start_.second.linear.x = 0;
  vel_start_.second.linear.y = 0;
  vel_start_.second.angular.z = 0;

  vel_goal_.first = true;
  vel_goal_.second.linear.x = 0;
  vel_goal_.second.linear.y = 0;
  vel_goal_.second.angular.z = 0;
  initialized_ = true;
}


void TebOptimalPlanner::setVisualization(TebVisualizationPtr visualization)
{
  visualization_ = visualization;
}

void TebOptimalPlanner::visualize()
{
  if (!visualization_)
    return;
 
  visualization_->publishLocalPlanAndPoses(teb_, teb_);
  
  if (teb_.sizePoses() > 0)
    visualization_->publishRobotFootprintModel(teb_.Pose(0), *robot_model_);
  
  if (cfg_->trajectory.publish_feedback)
    visualization_->publishFeedbackMessage(*this, *obstacles_);
}


/*
 * registers custom vertices and edges in g2o framework
 */
void TebOptimalPlanner::registerG2OTypes()
{
  g2o::Factory* factory = g2o::Factory::instance();
  factory->registerType("VERTEX_POSE", new g2o::HyperGraphElementCreator<VertexPose>);
  factory->registerType("VERTEX_TIMEDIFF", new g2o::HyperGraphElementCreator<VertexTimeDiff>);

  factory->registerType("EDGE_TIME_OPTIMAL", new g2o::HyperGraphElementCreator<EdgeTimeOptimal>);
  factory->registerType("EDGE_SHORTEST_PATH", new g2o::HyperGraphElementCreator<EdgeShortestPath>);
  factory->registerType("EDGE_VELOCITY", new g2o::HyperGraphElementCreator<EdgeVelocity>);
  factory->registerType("EDGE_VELOCITY_HOLONOMIC", new g2o::HyperGraphElementCreator<EdgeVelocityHolonomic>);
  factory->registerType("EDGE_ACCELERATION", new g2o::HyperGraphElementCreator<EdgeAcceleration>);
  factory->registerType("EDGE_ACCELERATION_START", new g2o::HyperGraphElementCreator<EdgeAccelerationStart>);
  factory->registerType("EDGE_ACCELERATION_GOAL", new g2o::HyperGraphElementCreator<EdgeAccelerationGoal>);
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC", new g2o::HyperGraphElementCreator<EdgeAccelerationHolonomic>);
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC_START", new g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicStart>);
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC_GOAL", new g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicGoal>);
  factory->registerType("EDGE_KINEMATICS_DIFF_DRIVE", new g2o::HyperGraphElementCreator<EdgeKinematicsDiffDrive>);
  factory->registerType("EDGE_KINEMATICS_CARLIKE", new g2o::HyperGraphElementCreator<EdgeKinematicsCarlike>);
  factory->registerType("EDGE_OBSTACLE", new g2o::HyperGraphElementCreator<EdgeObstacle>);
  factory->registerType("EDGE_INFLATED_OBSTACLE", new g2o::HyperGraphElementCreator<EdgeInflatedObstacle>);
  factory->registerType("EDGE_DYNAMIC_OBSTACLE", new g2o::HyperGraphElementCreator<EdgeDynamicObstacle>);
  factory->registerType("EDGE_VIA_POINT", new g2o::HyperGraphElementCreator<EdgeViaPoint>);
  factory->registerType("EDGE_PREFER_ROTDIR", new g2o::HyperGraphElementCreator<EdgePreferRotDir>);
  return;
}

/*
 * initialize g2o optimizer. Set solver settings here.
 * Return: pointer to new SparseOptimizer Object.
 */
boost::shared_ptr<g2o::SparseOptimizer> TebOptimalPlanner::initOptimizer()
{
  // Call register_g2o_types once, even for multiple TebOptimalPlanner instances (thread-safe)
  static boost::once_flag flag = BOOST_ONCE_INIT;
  boost::call_once(&registerG2OTypes, flag);  

  // allocating the optimizer
  boost::shared_ptr<g2o::SparseOptimizer> optimizer = boost::make_shared<g2o::SparseOptimizer>();
  std::unique_ptr<TEBLinearSolver> linear_solver(new TEBLinearSolver()); // see typedef in optimization.h
  linear_solver->setBlockOrdering(true);
  std::unique_ptr<TEBBlockSolver> block_solver(new TEBBlockSolver(std::move(linear_solver)));
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

  optimizer->setAlgorithm(solver);
  
  optimizer->initMultiThreading(); // required for >Eigen 3.1
  
  return optimizer;
}

// 优化器的入口
bool TebOptimalPlanner::optimizeTEB(int iterations_innerloop, int iterations_outerloop, bool compute_cost_afterwards,
                                    double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost)
{
  // 发布数据
 static ros::Publisher local_plan_pub_ = nh_shrink.advertise<nav_msgs::Path>("local_shrink_plan", 10);
 static ros::Publisher teb_poses_pub_ = nh_shrink.advertise<geometry_msgs::PoseArray>("teb_shrink_poses", 100);
  // create path msg
  nav_msgs::Path teb_path;
  teb_path.header.frame_id = "odom";
  teb_path.header.stamp = ros::Time::now();
  // create pose_array (along trajectory)
  geometry_msgs::PoseArray teb_poses;
  teb_poses.header.frame_id = teb_path.header.frame_id;
  teb_poses.header.stamp = teb_path.header.stamp;
  if (cfg_->optim.optimization_activate==false) 
    return false;
  setlocale(LC_ALL,"");
  int success = 0;
  optimized_ = false;
  
  double weight_multiplier = 1.0;

  int STC_TEB_flag = 1;//是否开启逐次优化

  // TODO(roesmann): we introduced the non-fast mode with the support of dynamic obstacles
  //                (which leads to better results in terms of x-y-t homotopy planning).
  //                 however, we have not tested this mode intensively yet, so we keep
  //                 the legacy fast mode asvisualize default until we finish our tests.
  bool fast_mode = !cfg_->obstacles.include_dynamic_obstacles;
  // 外部循环优化iteractions_outerloop次
  double j = 1.0;
  int no_shrink_optimize = 0;
  no_optimize = 0;
  double shrink_ratio = 0.0;
  double height = 0.0;
  int judge_lag = 0;//判断轨迹在行人前面还是后面的标志变量
  shrink_ratio_lag = 0;
  lag_optimal = 0;
  // ROS_INFO("新的轨迹");

  //根据行人速度调整迭代次数
  int  vel_iterations_outerloop = 0;
  double max_centroid_velocity_ = 0.0;
  if (STC_TEB_flag)
  {
    for (ObstContainer::const_iterator obst = obstacles_->begin(); obst != obstacles_->end(); ++obst)
    {
      if (!(*obst)->isDynamic())
        continue;

      if (((*obst)->getCentroidVelocity()).norm() >= max_centroid_velocity_)
      {
        max_centroid_velocity_ = ((*obst)->getCentroidVelocity()).norm();
      }
    }

    // ROS_INFO("最大速度为:%f",max_centroid_velocity_);
    if (max_centroid_velocity_ == 0)
    {
      vel_iterations_outerloop = 4;
    }
    else
    {
      // 感觉还是跟判断条件有关，当dist小于0.5时，都能优化到行人前面，而且没有往回优化的轨迹
      vel_iterations_outerloop = int(iterations_outerloop * max_centroid_velocity_ * 10); // 跟innerloop也有关系，不应该是线性的，应该是指数性的,因为随着优化的进行，路经点之间的时间间隔会越来越大，行人速度较快时，未来位置会更远
    }
    // ROS_INFO("迭代次数为:%d",vel_iterations_outerloop);
  }
  else
  {
    vel_iterations_outerloop = iterations_outerloop;
  }
  for(int i = 0; i <= vel_iterations_outerloop; i += 1)//j)
  {
    // for (int j = 0; j < iterations_outerloop; j++)
    // {   
    // int visualsignature = 0;
    //添加循环 将速度降低 通过j这个放缩因子
    //  for(double shrink_ratio=0;shrink_ratio < 1;shrink_ratio += 0.01)//系数为0.01
    //  {
    // for(double j = 0;j<1;j+=0.1)
    // {
    //double shrink_ratio =((double)i)*((double)i)/(iterations_outerloop*iterations_outerloop);
    //double shrink_ratio =-((double)(i-iterations_outerloop))*((double)(i-iterations_outerloop))/(iterations_outerloop*iterations_outerloop)+1;
     
    //  if (no_shrink_optimize == 2)
    //  {
    //   shrink_ratio = 1.000001;
    //  }
    //  if (no_shrink_optimize == 1)
    //  {
    //   shrink_ratio = 1.0;
    //  }
    if (STC_TEB_flag)
    {
      if (no_optimize == 0)
      {
        shrink_ratio = (double)i / vel_iterations_outerloop;
        // shrink_ratio =pow((((double)i)/vel_iterations_outerloop),0.5);
        // shrink_ratio =4*atan((double)i/vel_iterations_outerloop)/M_PI;
        // shrink_ratio = 1;
        // vel_iterations_outerloop = 4;
      }
      if (no_optimize == 1)
      {
        shrink_ratio = 1;
        vel_iterations_outerloop = 4;
      }
    }
    else
    {
      shrink_ratio = 1;
    }

    //  double shrink_ratio =1;
    //  shrink_ratio*=j;
    //double shrink_ratio = 100;
    // if(i == 0)
    // {
    //   iterations_outerloop = 100;
    // } 

    if (cfg_->trajectory.teb_autosize)
    {
      // 根据相邻点的时间间隔（距离），对路径进行remove和insert
      //teb_.autoResize(cfg_->trajectory.dt_ref, cfg_->trajectory.dt_hysteresis, cfg_->trajectory.min_samples, cfg_->trajectory.max_samples);
      teb_.autoResize(cfg_->trajectory.dt_ref, cfg_->trajectory.dt_hysteresis, cfg_->trajectory.min_samples, cfg_->trajectory.max_samples, fast_mode);

    }
    success = buildGraph(weight_multiplier,shrink_ratio);
    if (success == 0) 
    {
        clearGraph();
        return false;
    }
    // if (success == 3)
    // {
    //   //if距离过远 不进行此次优化 j/=2，放缩
    //   i-=j;//把i值还原
    //   j/=2;
    //   no_shrink_optimize = 0;
    // }
    // if (success == 1)//无需放缩
    // {
     success = optimizeGraph(iterations_innerloop, false);
    //  no_shrink_optimize = 1;
    // }
    // if (success ==2)//初始轨迹距离较远
    // {
    // //  success = optimizeGraph(iterations_innerloop, false);
    //  no_shrink_optimize = 2;
    // }
    // {
    //   no_shrink_optimize = 0;
    // }
    if (success == 0) 
    {
        clearGraph();
        return false;
    }

    // ros::Duration(0.0001).sleep();
    for (int k=0; k < teb_.sizePoses(); k++)
    {
      // ROS_INFO("标志:%d",visualsignature);
      // ROS_INFO("teb的x:%f",teb_.Pose(i).x());
      // ROS_INFO("teb的y:%f",teb_.Pose(i).y());
      // visualsignature = i;
      // teb_x.push_back(teb_.Pose(i).x());
      // teb_y.push_back(teb_.Pose(i).y());
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = teb_path.header.frame_id;
      pose.header.stamp = teb_path.header.stamp;
      pose.pose.position.x = teb_.Pose(k).x();
      pose.pose.position.y = teb_.Pose(k).y();
      // pose.pose.position.z = cfg_->hcp.visualize_with_time_as_z_axis_scale*teb.getSumOfTimeDiffsUpToIdx(i);
      pose.pose.position.z =height;
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(teb_.Pose(k).theta());
      teb_path.poses.push_back(pose);
      teb_poses.poses.push_back(pose.pose);
    }
    height+=0.1;
    local_plan_pub_.publish(teb_path);
    teb_poses_pub_.publish(teb_poses);
    //结束发布
    
    // tebs_shrink.push_back(teb_);

    optimized_ = true;

    // visualization_->publishLocalPlanAndPoses(teb_, teb_);
    //这里最后的计算有大大的问题，待修改，改了
    if (compute_cost_afterwards && i == iterations_outerloop) // compute cost vec only in the last iteration
    {
      computeCurrentCost(obst_cost_scale, viapoint_cost_scale, alternative_time_cost, shrink_ratio);
      shrink_ratio_lag = 0;
      lag_optimal = 0;
      // ROS_INFO("啦啦啦");
    }
    clearGraph();
    // }
    weight_multiplier *= cfg_->optim.weight_adapt_factor; //yaml里默认为2
    // }
    
  }

  // //正常建图优化（除去动态行人）
  // optimized_ = false;
  // weight_multiplier = 1.0;
  // for (int i = 0; i < iterations_outerloop; i += 1) // j)
  // {
  //    if (cfg_->trajectory.teb_autosize)
  //   {
  //     // 根据相邻点的时间间隔（距离），对路径进行remove和insert
  //     //teb_.autoResize(cfg_->trajectory.dt_ref, cfg_->trajectory.dt_hysteresis, cfg_->trajectory.min_samples, cfg_->trajectory.max_samples);
  //     teb_.autoResize(cfg_->trajectory.dt_ref, cfg_->trajectory.dt_hysteresis, cfg_->trajectory.min_samples, cfg_->trajectory.max_samples, fast_mode);

  //   }
  //   success = buildGraph(weight_multiplier,shrink_ratio);
  //   if (success == 0) 
  //   {
  //       clearGraph();
  //       return false;
  //   }

  //    success = optimizeGraph(iterations_innerloop, false);

  //   if (success == 0) 
  //   {
  //       clearGraph();
  //       return false;
  //   }

  //   optimized_ = true;
  //   //这里最后的计算有大大的问题，待修改，改了
  //   if (compute_cost_afterwards && i == iterations_outerloop - 1) // compute cost vec only in the last iteration
  //   {
  //     computeCurrentCost(obst_cost_scale, viapoint_cost_scale, alternative_time_cost, shrink_ratio);
  //     shrink_ratio_lag = 0;
  //     lag_optimal = 0;
  //     // ROS_INFO("啦啦啦");
  //   }
  //   clearGraph();
  //   weight_multiplier *= cfg_->optim.weight_adapt_factor; //yaml里默认为2

  // }
  return true;
}

void TebOptimalPlanner::setVelocityStart(const geometry_msgs::Twist& vel_start)
{
  vel_start_.first = true;
  vel_start_.second.linear.x = vel_start.linear.x;
  vel_start_.second.linear.y = vel_start.linear.y;
  vel_start_.second.angular.z = vel_start.angular.z;
}

void TebOptimalPlanner::setVelocityGoal(const geometry_msgs::Twist& vel_goal)
{
  vel_goal_.first = true;
  vel_goal_.second = vel_goal;
}

bool TebOptimalPlanner::plan(const std::vector<geometry_msgs::PoseStamped>& initial_plan, const geometry_msgs::Twist* start_vel, bool free_goal_vel)
{    
  ROS_ASSERT_MSG(initialized_, "Call initialize() first.");
  if (!teb_.isInit())
  {
    teb_.initTrajectoryToGoal(initial_plan, cfg_->robot.max_vel_x, cfg_->robot.max_vel_theta, cfg_->trajectory.global_plan_overwrite_orientation,
      cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);
  }
  else // warm start
  {
    PoseSE2 start_(initial_plan.front().pose);
    PoseSE2 goal_(initial_plan.back().pose);
    // 更新起点，prune teb内存的轨迹的机器人已经走过的部分
    if (teb_.sizePoses()>0
        && (goal_.position() - teb_.BackPose().position()).norm() < cfg_->trajectory.force_reinit_new_goal_dist
        && fabs(g2o::normalize_theta(goal_.theta() - teb_.BackPose().theta())) < cfg_->trajectory.force_reinit_new_goal_angular) // actual warm start!
      teb_.updateAndPruneTEB(start_, goal_, cfg_->trajectory.min_samples); // update TEB
    // 已经初始化过了，但目标和当前位置离得太远了，重新初始化个目标点~
    else // goal too far away -> reinit
    {
      ROS_DEBUG("New goal: distance to existing goal is higher than the specified threshold. Reinitalizing trajectories.");
      teb_.clearTimedElasticBand();
      teb_.initTrajectoryToGoal(initial_plan, cfg_->robot.max_vel_x, cfg_->robot.max_vel_theta, cfg_->trajectory.global_plan_overwrite_orientation,
        cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);
    }
  }
  if (start_vel)
    setVelocityStart(*start_vel);
  if (free_goal_vel)
    setVelocityGoalFree();
  else
    vel_goal_.first = true; // we just reactivate and use the previously set velocity (should be zero if nothing was modified)
  
  // now optimize
  return optimizeTEB(cfg_->optim.no_inner_iterations, cfg_->optim.no_outer_iterations);
}


bool TebOptimalPlanner::plan(const tf::Pose& start, const tf::Pose& goal, const geometry_msgs::Twist* start_vel, bool free_goal_vel)
{
  PoseSE2 start_(start);
  PoseSE2 goal_(goal);
  return plan(start_, goal_, start_vel);
}

bool TebOptimalPlanner::plan(const PoseSE2& start, const PoseSE2& goal, const geometry_msgs::Twist* start_vel, bool free_goal_vel)
{	
  ROS_ASSERT_MSG(initialized_, "Call initialize() first.");
  if (!teb_.isInit())
  {
    // init trajectory
    teb_.initTrajectoryToGoal(start, goal, 0, cfg_->robot.max_vel_x, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion); // 0 intermediate samples, but dt=1 -> autoResize will add more samples before calling first optimization
  }
  else // warm start
  {
    if (teb_.sizePoses() > 0
        && (goal.position() - teb_.BackPose().position()).norm() < cfg_->trajectory.force_reinit_new_goal_dist
        && fabs(g2o::normalize_theta(goal.theta() - teb_.BackPose().theta())) < cfg_->trajectory.force_reinit_new_goal_angular) // actual warm start!
      teb_.updateAndPruneTEB(start, goal, cfg_->trajectory.min_samples);
    else // goal too far away -> reinit
    {
      ROS_DEBUG("New goal: distance to existing goal is higher than the specified threshold. Reinitalizing trajectories.");
      teb_.clearTimedElasticBand();
      teb_.initTrajectoryToGoal(start, goal, 0, cfg_->robot.max_vel_x, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);
    }
  }
  if (start_vel)
    setVelocityStart(*start_vel);
  if (free_goal_vel)
    setVelocityGoalFree();
  else
    vel_goal_.first = true; // we just reactivate and use the previously set velocity (should be zero if nothing was modified)
      
  // now optimize
  return optimizeTEB(cfg_->optim.no_inner_iterations, cfg_->optim.no_outer_iterations);
}


bool TebOptimalPlanner::buildGraph(double weight_multiplier,double shrink_ratio)
{
  if (!optimizer_->edges().empty() || !optimizer_->vertices().empty())
  {
    ROS_WARN("Cannot build graph, because it is not empty. Call graphClear()!");
    return false;
  }

  optimizer_->setComputeBatchStatistics(cfg_->recovery.divergence_detection_enable);
  
  // add TEB vertices。即把所有的点和dt添加到优化器中
  AddTEBVertices();
  // add Edges (local cost functions)
  if (cfg_->obstacles.legacy_obstacle_association)    // 默认false
    AddEdgesObstaclesLegacy(weight_multiplier);
  else
    AddEdgesObstacles(weight_multiplier);   // 对teb中的每个pose点，找到离它左侧和右侧最近的障碍物点，然后把他们添加到需要考虑的obs中

  if (cfg_->obstacles.include_dynamic_obstacles) // 对动态障碍物，添加time戳，在xyt空间下，teb考虑同一time时的避障
    AddEdgesDynamicObstacles(1.0, shrink_ratio);

  // 实际上，这个viapoints没有被调用。。
  AddEdgesViaPoints();

  AddEdgesVelocity();
  
  AddEdgesAcceleration();

  AddEdgesTimeOptimal();	

  AddEdgesShortestPath();
  if (cfg_->robot.min_turning_radius == 0 || cfg_->optim.weight_kinematics_turning_radius == 0)
    AddEdgesKinematicsDiffDrive(); // we have a differential drive robot
  else
    AddEdgesKinematicsCarlike(); // we have a carlike robot since the turning radius is bounded from below.

  AddEdgesPreferRotDir();//机器人偏好的转弯方向，未调用

  if (cfg_->optim.weight_velocity_obstacle_ratio > 0)
  {
    AddEdgesVelocityObstacleRatio();//速度障碍物比率，考虑动态障碍物，没有调用
    // sleep(10);
  }

  return true;  
}

// 仅对障碍物目标函数建图优化
bool TebOptimalPlanner::buildGraphObstacle(double weight_multiplier,double shrink_ratio)
{
  if (!optimizer_->edges().empty() || !optimizer_->vertices().empty())
  {
    ROS_WARN("Cannot build graph, because it is not empty. Call graphClear()!");
    return false;
  }

  optimizer_->setComputeBatchStatistics(cfg_->recovery.divergence_detection_enable);
  
  // add TEB vertices。即把所有的点和dt添加到优化器中
  AddTEBVertices();
  // add Edges (local cost functions)

  if (cfg_->obstacles.include_dynamic_obstacles) // 对动态障碍物，添加time戳，在xyt空间下，teb考虑同一time时的避障
    AddEdgesDynamicObstacles(1.0, shrink_ratio);

  return true;  
}

bool TebOptimalPlanner::optimizeGraph(int no_iterations,bool clear_after)
{
  if (cfg_->robot.max_vel_x<0.01)
  {
    ROS_WARN("optimizeGraph(): Robot Max Velocity is smaller than 0.01m/s. Optimizing aborted...");
    if (clear_after) clearGraph();
    return false;	
  }
  
  if (!teb_.isInit() || teb_.sizePoses() < cfg_->trajectory.min_samples)
  {
    ROS_WARN("optimizeGraph(): TEB is empty or has too less elements. Skipping optimization.");
    if (clear_after) clearGraph();
    return false;	
  }
  optimizer_->setVerbose(cfg_->optim.optimization_verbose);
  optimizer_->initializeOptimization();
  int iter = optimizer_->optimize(no_iterations);
  // Save Hessian for visualization
  //  g2o::OptimizationAlgorithmLevenberg* lm = dynamic_cast<g2o::OptimizationAlgorithmLevenberg*> (optimizer_->solver());
  //  lm->solver()->saveHessian("~/MasterThesis/Matlab/Hessian.txt");

  if(!iter)
  {
	ROS_ERROR("optimizeGraph(): Optimization failed! iter=%i", iter);
	return false;
  }

  if (clear_after) clearGraph();	
    
  return true;
}

void TebOptimalPlanner::clearGraph()
{
  // clear optimizer states
  if (optimizer_)
  {
    // we will delete all edges but keep the vertices.
    // before doing so, we will delete the link from the vertices to the edges.
    auto& vertices = optimizer_->vertices();
    for(auto& v : vertices)
      v.second->edges().clear();

    optimizer_->vertices().clear();  // necessary, because optimizer->clear deletes pointer-targets (therefore it deletes TEB states!)
    optimizer_->clear();
  }
}



void TebOptimalPlanner::AddTEBVertices()
{
  // add vertices to graph
  ROS_DEBUG_COND(cfg_->optim.optimization_verbose, "Adding TEB vertices ...");
  unsigned int id_counter = 0; // used for vertices ids
  obstacles_per_vertex_.resize(teb_.sizePoses());
  auto iter_obstacle = obstacles_per_vertex_.begin();
  for (int i=0; i<teb_.sizePoses(); ++i)
  {
    teb_.PoseVertex(i)->setId(id_counter++);
    optimizer_->addVertex(teb_.PoseVertex(i));  //poseVertex是获得index为i对应的pose点，在initTraj时被建立
    if (teb_.sizeTimeDiffs()!=0 && i<teb_.sizeTimeDiffs())
    {
      teb_.TimeDiffVertex(i)->setId(id_counter++);
      optimizer_->addVertex(teb_.TimeDiffVertex(i));  //timeDiffVertex是获得index为i对应的dt点，在initTraj时被建立
    }
    iter_obstacle->clear();
    (iter_obstacle++)->reserve(obstacles_->size());
  }
}

// 先遍历teb_中的所有路径点；对每个路径点，再遍历所有的障碍物点，找到离该路径点最近的左侧和右侧的障碍物
void TebOptimalPlanner::AddEdgesObstacles(double weight_multiplier)
{
  if (cfg_->optim.weight_obstacle==0 || weight_multiplier==0 || obstacles_==nullptr )
    return; // if weight equals zero skip adding edges!
    
  
  bool inflated = cfg_->obstacles.inflation_dist > cfg_->obstacles.min_obstacle_dist;   // default: 0.6>0.25

  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_obstacle * weight_multiplier);
  
  Eigen::Matrix<double,2,2> information_inflated;
  information_inflated(0,0) = cfg_->optim.weight_obstacle * weight_multiplier;  // 默认：100x1
  information_inflated(1,1) = cfg_->optim.weight_inflation; // 默认0.2
  information_inflated(0,1) = information_inflated(1,0) = 0;

  auto iter_obstacle = obstacles_per_vertex_.begin();

  // 这里定义了一个临时函数，默认已用的谁是[ ]里面，外界要给的是()里的
  auto create_edge = [inflated, &information, &information_inflated, this] (int index, const Obstacle* obstacle) {
    // 这里的区别是，inflated是二维的error
    if (inflated)
    {
      EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
      dist_bandpt_obst->setVertex(0,teb_.PoseVertex(index));
      dist_bandpt_obst->setInformation(information_inflated);
      dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obstacle);
      optimizer_->addEdge(dist_bandpt_obst);
    }
    else
    {
      EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
      dist_bandpt_obst->setVertex(0,teb_.PoseVertex(index));
      dist_bandpt_obst->setInformation(information);
      dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obstacle);
      optimizer_->addEdge(dist_bandpt_obst);
    };
  };

  // 下面这个是新加的，只针对graphicTEB使用
  auto create_edge_Graphic = [inflated, &information, &information_inflated, this] (int index, const Obstacle* obstacle, costmap_2d::Costmap2D* costmap2d, const std::vector<std::vector<int>> & obs_map_labeled) {
    // 这里的区别是，inflated是二维的error
    if (inflated)
    {
      EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
      dist_bandpt_obst->setCostmap(costmap2d_);
      dist_bandpt_obst->setObsMapLabeled(obs_map_labeled_);
      dist_bandpt_obst->setVertex(0,teb_.PoseVertex(index));
      dist_bandpt_obst->setInformation(information_inflated);
      dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obstacle);
      optimizer_->addEdge(dist_bandpt_obst);
    }
    else
    {
      EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
      dist_bandpt_obst->setCostmap(costmap2d_);
      dist_bandpt_obst->setObsMapLabeled(obs_map_labeled_);
      dist_bandpt_obst->setVertex(0,teb_.PoseVertex(index));
      dist_bandpt_obst->setInformation(information);
      dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obstacle);
      optimizer_->addEdge(dist_bandpt_obst);
    };
  };
    
  // iterate all teb points, skipping the last and, if the EdgeVelocityObstacleRatio edges should not be created, the first one too
  const int first_vertex = cfg_->optim.weight_velocity_obstacle_ratio == 0 ? 1 : 0;
  // 遍历所有路径点
  for (int i = first_vertex; i < teb_.sizePoses() - 1; ++i)
  {    
      double left_min_dist = std::numeric_limits<double>::max();
      double right_min_dist = std::numeric_limits<double>::max();
      ObstaclePtr left_obstacle;
      ObstaclePtr right_obstacle;
      
      const Eigen::Vector2d pose_orient = teb_.Pose(i).orientationUnitVec();
      
      // iterate obstacles
      for (const ObstaclePtr& obst : *obstacles_)
      {
        // we handle dynamic obstacles differently below
        if(cfg_->obstacles.include_dynamic_obstacles && obst->isDynamic())
          continue;

          // calculate distance to robot model
          // 在obst这个障碍物簇中，找到离teb_.Pose(i)这个点最近的障碍物点
          double dist = robot_model_->calculateDistance(teb_.Pose(i), obst.get());
          
          // force considering obstacle if really close to the current pose
        if (dist < cfg_->obstacles.min_obstacle_dist*cfg_->obstacles.obstacle_association_force_inclusion_factor) //default: 0.25*1.5，0.25是机器人的半径（自己在yaml里设置）
          {
              iter_obstacle->push_back(obst);
              continue;
          }
          // cut-off distance
          if (dist > cfg_->obstacles.min_obstacle_dist*cfg_->obstacles.obstacle_association_cutoff_factor)  //default：0.25*5
            continue;
          
          // determine side (left or right) and assign obstacle if closer than the previous one
          // 找到路径点 左侧和最侧 分别的 最近的障碍物和对应的距离
          if (cross2d(pose_orient, obst->getCentroid()) > 0) // left
          {
              if (dist < left_min_dist)
              {
                  left_min_dist = dist;
                  left_obstacle = obst;
              }
          }
          else
          {
              if (dist < right_min_dist)
              {
                  right_min_dist = dist;
                  right_obstacle = obst;
              }
          }
      }   
      
      // 把最近障碍物添加到需要考虑的obs中
      if (left_obstacle)
        iter_obstacle->push_back(left_obstacle);
      if (right_obstacle)
        iter_obstacle->push_back(right_obstacle);

      // continue here to ignore obstacles for the first pose, but use them later to create the EdgeVelocityObstacleRatio edges
      if (i == 0)
      {
        ++iter_obstacle;
        continue;
      }

      // create obstacle edges
      for (const ObstaclePtr obst : *iter_obstacle){
        if (cfg_->hcp.graphic_exploration){
          create_edge_Graphic(i, obst.get(),costmap2d_,obs_map_labeled_);
        }
        else{
          create_edge(i, obst.get());
        }
      }
        
      ++iter_obstacle;
  }
}


void TebOptimalPlanner::AddEdgesObstaclesLegacy(double weight_multiplier)
{
  if (cfg_->optim.weight_obstacle==0 || weight_multiplier==0 || obstacles_==nullptr)
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,1,1> information; 
  information.fill(cfg_->optim.weight_obstacle * weight_multiplier);
    
  Eigen::Matrix<double,2,2> information_inflated;
  information_inflated(0,0) = cfg_->optim.weight_obstacle * weight_multiplier;
  information_inflated(1,1) = cfg_->optim.weight_inflation;
  information_inflated(0,1) = information_inflated(1,0) = 0;
  
  bool inflated = cfg_->obstacles.inflation_dist > cfg_->obstacles.min_obstacle_dist;
    
  for (ObstContainer::const_iterator obst = obstacles_->begin(); obst != obstacles_->end(); ++obst)
  {
    if (cfg_->obstacles.include_dynamic_obstacles && (*obst)->isDynamic()) // we handle dynamic obstacles differently below
      continue; 
    
    int index;
    
    if (cfg_->obstacles.obstacle_poses_affected >= teb_.sizePoses())
      index =  teb_.sizePoses() / 2;
    else
      index = teb_.findClosestTrajectoryPose(*(obst->get()));
     
    
    // check if obstacle is outside index-range between start and goal
    if ( (index <= 1) || (index > teb_.sizePoses()-2) ) // start and goal are fixed and findNearestBandpoint finds first or last conf if intersection point is outside the range
	    continue; 
        
    if (inflated)
    {
        EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
        dist_bandpt_obst->setVertex(0,teb_.PoseVertex(index));
        dist_bandpt_obst->setInformation(information_inflated);
        dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obst->get());
        optimizer_->addEdge(dist_bandpt_obst);
    }
    else
    {
        EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
        dist_bandpt_obst->setVertex(0,teb_.PoseVertex(index));
        dist_bandpt_obst->setInformation(information);
        dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obst->get());
        optimizer_->addEdge(dist_bandpt_obst);
    }

    for (int neighbourIdx=0; neighbourIdx < floor(cfg_->obstacles.obstacle_poses_affected/2); neighbourIdx++)
    {
      if (index+neighbourIdx < teb_.sizePoses())
      {
            if (inflated)
            {
                EdgeInflatedObstacle* dist_bandpt_obst_n_r = new EdgeInflatedObstacle;
                dist_bandpt_obst_n_r->setVertex(0,teb_.PoseVertex(index+neighbourIdx));
                dist_bandpt_obst_n_r->setInformation(information_inflated);
                dist_bandpt_obst_n_r->setParameters(*cfg_, robot_model_.get(), obst->get());
                optimizer_->addEdge(dist_bandpt_obst_n_r);
            }
            else
            {
                EdgeObstacle* dist_bandpt_obst_n_r = new EdgeObstacle;
                dist_bandpt_obst_n_r->setVertex(0,teb_.PoseVertex(index+neighbourIdx));
                dist_bandpt_obst_n_r->setInformation(information);
                dist_bandpt_obst_n_r->setParameters(*cfg_, robot_model_.get(), obst->get());
                optimizer_->addEdge(dist_bandpt_obst_n_r);
            }
      }
      if ( index - neighbourIdx >= 0) // needs to be casted to int to allow negative values
      {
            if (inflated)
            {
                EdgeInflatedObstacle* dist_bandpt_obst_n_l = new EdgeInflatedObstacle;
                dist_bandpt_obst_n_l->setVertex(0,teb_.PoseVertex(index-neighbourIdx));
                dist_bandpt_obst_n_l->setInformation(information_inflated);
                dist_bandpt_obst_n_l->setParameters(*cfg_, robot_model_.get(), obst->get());
                optimizer_->addEdge(dist_bandpt_obst_n_l);
            }
            else
            {
                EdgeObstacle* dist_bandpt_obst_n_l = new EdgeObstacle;
                dist_bandpt_obst_n_l->setVertex(0,teb_.PoseVertex(index-neighbourIdx));
                dist_bandpt_obst_n_l->setInformation(information);
                dist_bandpt_obst_n_l->setParameters(*cfg_, robot_model_.get(), obst->get());
                optimizer_->addEdge(dist_bandpt_obst_n_l);
            }
      }
    } 
    
  }
}

// 动态障碍物就没有静态那么麻烦了，不用酸左右或者是不是最近了，统统添加就好了。。
// 这里也体现了一个点是，只要把障碍物设置成dynamic，那么teb会在xyt空间，针对同一个time，进行距离的计算了。。
int TebOptimalPlanner::AddEdgesDynamicObstacles(double weight_multiplier,double shrink_ratio)
{
  if (cfg_->optim.weight_obstacle==0 || weight_multiplier==0 || obstacles_==NULL )
    return 0; // if weight equals zero skip adding edges!

  setlocale(LC_ALL,"");
  Eigen::Matrix<double,2,2> information;
  information(0,0) = cfg_->optim.weight_dynamic_obstacle * weight_multiplier;
  information(1,1) = cfg_->optim.weight_dynamic_obstacle_inflation;
  information(0,1) = information(1,0) = 0;

  double min=10000;
  BaseRobotFootprintModel* robot_model_min;
  int j;
  Obstacle* obstacle_min;
  double time_min;
  Eigen::Vector2d obs_every_pos;//存储逐步优化的障碍物位置，用于可视化
  int count = 0;//记录处于行人内控制点的个数

  //注册发布行人坐标的消息
  static ros::Publisher pedestrain_poses_pub_ = nh_shrink.advertise<geometry_msgs::PoseArray>("pedestrain_poses", 100);
  geometry_msgs::PoseArray pedestrain_poses;
  pedestrain_poses.header.frame_id = "odom";
  pedestrain_poses.header.stamp = ros::Time::now();

  for (ObstContainer::const_iterator obst = obstacles_->begin(); obst != obstacles_->end(); ++obst)
  {
    if (!(*obst)->isDynamic())
      continue;
    // Skip first and last pose, as they are fixed
    /*
      找到距离障碍物最近TEB点，如果距离很远则不放缩，优化次数仍由no_optimize决定（因为不知道另外一个障碍物可能距离TEB点很近）
    */
    // double time_lab = teb_.TimeDiff(0);
    // double min_lab = 1000;
    // for (int i=1; i < teb_.sizePoses() - 1; ++i)
    // {
    //   double dist = robot_model_.get()->estimateSpatioTemporalDistance(teb_.PoseVertex(i)->pose(), obst->get(), time_lab,0);
    //   time_lab += teb_.TimeDiff(i);
    //   if(min_lab >= dist) //寻找距离障碍物未来位置最近点
    //   {
    //     min_lab = dist;
    //   }
    // }
    // if (min_lab > 0.5)
    // {
    //   shrink_ratio = 1;
    // }
    /*
  找到距离障碍物最近TEB点，判断轨迹是在行人前还是行人后面
  如果控制点比较稀疏,咋办
  如何精准判断轨迹是在行人前面还是后面
  例如平行着走的时候
  判断交角累计和，并归一化
  先判断行人速度是否为0
  要在shrink_ratio不为0的时候，先优化一轮
*/
    if (((*obst)->getCentroidVelocity()).norm() == 0)
    {
      lag_optimal = 0;
    }
    else
    {
      if (shrink_ratio == 0)
      {
        lag_optimal = 0;
      }
      else
      {
        if (shrink_ratio_lag == 0)
        {
          double time_lab = teb_.TimeDiff(0);
          double min_lab = 1000;
          Eigen::Vector2d normal_vector;
          Eigen::Vector2d teb_min;
          Eigen::Vector2d obs_now;
          Eigen::Vector2d judge_ctrl_point;
          for (int i = 1; i < teb_.sizePoses() - 1; ++i)
          {
            // 根据最近控制点判断轨迹前后 如果稀疏，则不合理
            double dist = robot_model_.get()->estimateSpatioTemporalDistance(teb_.PoseVertex(i)->pose(), obst->get(), time_lab, shrink_ratio);
            // time_lab += teb_.TimeDiff(i);
            if (min_lab >= dist) // 寻找距离障碍物未来位置最近点
            {
              min_lab = dist;
              teb_min.x() = teb_.Pose(i).x();
              teb_min.y() = teb_.Pose(i).y();
              normal_vector = robot_model_.get()->estimateNormalVector(teb_.PoseVertex(i)->pose(), obst->get(), time_lab, shrink_ratio);
              (*obst)->predictEveryCentroidConstantVelocity(time_lab, obs_now, shrink_ratio);
              judge_ctrl_point = teb_min - obs_now;
            }
            // 根据锐角控制点的个数判断轨迹前后 不合理
            // normal_vector = robot_model_.get()->estimateNormalVector(teb_.PoseVertex(i)->pose(), obst->get(), time_lab, shrink_ratio);
            // (*obst)->predictEveryCentroidConstantVelocity(time_lab, obs_now, shrink_ratio);
            // judge_ctrl_point = teb_min - obs_now;
          }
          // 判断点积，如果大于0，则轨迹在前面，此时lag_optimal为1，全部向外推
          if (normal_vector.dot(judge_ctrl_point) > 0)
          {
            lag_optimal = 1;
            // ROS_INFO("YES");
          }
          else
          {
            lag_optimal = 0;
            // ROS_INFO("NO");
          }
          shrink_ratio_lag = 1;
        }
      }
    }

    double time = teb_.TimeDiff(0);
    // ros::Duration(0.0001).sleep();
    for (int i=1; i < teb_.sizePoses() - 1; ++i)
    {
      (*obst)->predictEveryCentroidConstantVelocity(time, obs_every_pos, shrink_ratio) ;
      // ROS_INFO("行人的x:%f",obs_every_pos[0]);
      geometry_msgs::PoseStamped pedestrain_pose;
      pedestrain_pose.header.frame_id = "odom";
      pedestrain_pose.header.stamp = ros::Time::now();
      pedestrain_pose.pose.position.x = obs_every_pos.x();
      pedestrain_pose.pose.position.y = obs_every_pos.y();
      // pose.pose.position.z = cfg_->hcp.visualize_with_time_as_z_axis_scale*teb.getSumOfTimeDiffsUpToIdx(i);
      // pose.pose.position.z =height;
      pedestrain_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
      // tf2::Quaternion qtn;
      // qtn.setRPY(0, 0, 0);
      // pose.pose.transform.rotation.x = qtn.getX();
      // ts.transform.rotation.y = qtn.getY();
      // ts.transform.rotation.z = qtn.getZ();
      // ts.transform.rotation.w = qtn.getW();
      pedestrain_poses.poses.push_back(pedestrain_pose.pose);
      pedestrain_poses_pub_.publish(pedestrain_poses);

      EdgeDynamicObstacle* dynobst_edge = new EdgeDynamicObstacle(time,shrink_ratio,lag_optimal);
      dynobst_edge->setCostmap(costmap2d_);
      dynobst_edge->setObsMapLabeled(obs_map_labeled_);
      dynobst_edge->setVertex(0,teb_.PoseVertex(i));
      dynobst_edge->setInformation(information);
      dynobst_edge->setParameters(*cfg_, robot_model_.get(), obst->get());
      optimizer_->addEdge(dynobst_edge);
      time += teb_.TimeDiff(i); // we do not need to check the time diff bounds, since we iterate to "< sizePoses()-1".

      // if (dynobst_edge->future_distance() < 0)
      // {
      //   count ++;
      // }
      if(min >= dynobst_edge->future_distance()) //寻找距离障碍物未来位置最近点
      {
        min = dynobst_edge->future_distance();
        //指针记录此时的i值、robot_model_.get()和obst->get()
        //  robot_model_min = robot_model_.get();
        //  j = i;
        // obstacle_min = obst->get();
        // time_min = time;
      }
    }
  }
  //大于阈值则认为障碍物距离推进的轨迹过远，增加迭代次数，降低障碍物速度
  //全局路径，距离障碍物较远，无需多次迭代优化
  // if(shrink_ratio == 0 && min > 0.5)
  // {
  //   //存在问题，障碍物不在局部地图，全局路径最小值还是大于阈值的，无法优化，咋办？
  //   //利用shrink_ratio，为0如果全局路径，则min>阈值，正常优化
  //   // return 2;
  //   no_optimize = 1;
  // }
  if (shrink_ratio != 0 && min <= 0)
  {
    // judge_lag = 1;
  }
  
  // if (shrink_ratio == 1.000001 )
  // {
  //   return 2;
  // }
  // if (shrink_ratio != 1.000001 && min > 2)//放缩数值不为1.000001(初始轨迹符合)，但是最小值大于阈值，则认为优化的轨迹过远，返回true调整shrink_ratio
  // {
  //   EdgeDynamicObstacle* dynobst_edge = new EdgeDynamicObstacle(time_min,shrink_ratio);
  //   dynobst_edge->setVertex(0,teb_.PoseVertex(j));
  //   dynobst_edge->setInformation(information);
  //   dynobst_edge->setParameters(*cfg_, robot_model_min, obstacle_min);
  //   //存在问题，如果轨迹位于障碍物后面，障碍物速度较快，未来障碍物位置距离优化轨迹过远，则一直返回true
  //   //利用向量方向判断，位于障碍物运动方向后的轨迹：
  //   //未来障碍物（设定一个较大速度：单位化向量乘阈值）与teb路经点构成的向量和当前障碍物与teb路经点构成的向量是同向的，前面的轨迹是反向的
  //   // if (/* condition */)
  //   // {
  //   //   /* code */
  //   // }
  //   return 1;
  // }
    return 0;
}

void TebOptimalPlanner::AddEdgesViaPoints()
{
  if (cfg_->optim.weight_viapoint==0 || via_points_==NULL || via_points_->empty() )
    return; // if weight equals zero skip adding edges!

  int start_pose_idx = 0;
  
  int n = teb_.sizePoses();
  if (n<3) // we do not have any degrees of freedom for reaching via-points
    return;
  
  for (ViaPointContainer::const_iterator vp_it = via_points_->begin(); vp_it != via_points_->end(); ++vp_it)
  {
    int index = teb_.findClosestTrajectoryPose(*vp_it, NULL, start_pose_idx);
    if (cfg_->trajectory.via_points_ordered)
      start_pose_idx = index+2; // skip a point to have a DOF inbetween for further via-points
     
    // check if point conicides with goal or is located behind it
    if ( index > n-2 ) 
      index = n-2; // set to a pose before the goal, since we can move it away!
    // check if point coincides with start or is located before it
    if ( index < 1)
    {
      if (cfg_->trajectory.via_points_ordered)
      {
        index = 1; // try to connect the via point with the second (and non-fixed) pose. It is likely that autoresize adds new poses inbetween later.
      }
      else
      {
        ROS_DEBUG("TebOptimalPlanner::AddEdgesViaPoints(): skipping a via-point that is close or behind the current robot pose.");
        continue; // skip via points really close or behind the current robot pose
      }
    }
    Eigen::Matrix<double,1,1> information;
    information.fill(cfg_->optim.weight_viapoint);    // 默认：1
    
    EdgeViaPoint* edge_viapoint = new EdgeViaPoint;
    edge_viapoint->setVertex(0,teb_.PoseVertex(index));
    edge_viapoint->setInformation(information);
    edge_viapoint->setParameters(*cfg_, &(*vp_it));
    optimizer_->addEdge(edge_viapoint);   
  }
}

void TebOptimalPlanner::AddEdgesVelocity()
{
  if (cfg_->robot.max_vel_y == 0) // non-holonomic robot
  {
    if ( cfg_->optim.weight_max_vel_x==0 && cfg_->optim.weight_max_vel_theta==0)
      return; // if weight equals zero skip adding edges!

    int n = teb_.sizePoses();
    Eigen::Matrix<double,2,2> information;
    information(0,0) = cfg_->optim.weight_max_vel_x;
    information(1,1) = cfg_->optim.weight_max_vel_theta;
    information(0,1) = 0.0;
    information(1,0) = 0.0;

    for (int i=0; i < n - 1; ++i)
    {
      EdgeVelocity* velocity_edge = new EdgeVelocity;
      velocity_edge->setVertex(0,teb_.PoseVertex(i));
      velocity_edge->setVertex(1,teb_.PoseVertex(i+1));
      velocity_edge->setVertex(2,teb_.TimeDiffVertex(i));
      velocity_edge->setInformation(information);
      velocity_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(velocity_edge);
    }
  }
  else // holonomic-robot
  {
    if ( cfg_->optim.weight_max_vel_x==0 && cfg_->optim.weight_max_vel_y==0 && cfg_->optim.weight_max_vel_theta==0)
      return; // if weight equals zero skip adding edges!
      
    int n = teb_.sizePoses();
    Eigen::Matrix<double,3,3> information;
    information.fill(0);
    information(0,0) = cfg_->optim.weight_max_vel_x;
    information(1,1) = cfg_->optim.weight_max_vel_y;
    information(2,2) = cfg_->optim.weight_max_vel_theta;

    for (int i=0; i < n - 1; ++i)
    {
      EdgeVelocityHolonomic* velocity_edge = new EdgeVelocityHolonomic;
      velocity_edge->setVertex(0,teb_.PoseVertex(i));
      velocity_edge->setVertex(1,teb_.PoseVertex(i+1));
      velocity_edge->setVertex(2,teb_.TimeDiffVertex(i));
      velocity_edge->setInformation(information);
      velocity_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(velocity_edge);
    } 
    
  }
}

void TebOptimalPlanner::AddEdgesAcceleration()
{
  if (cfg_->optim.weight_acc_lim_x==0  && cfg_->optim.weight_acc_lim_theta==0) 
    return; // if weight equals zero skip adding edges!

  int n = teb_.sizePoses();  
    
  if (cfg_->robot.max_vel_y == 0 || cfg_->robot.acc_lim_y == 0) // non-holonomic robot
  {
    Eigen::Matrix<double,2,2> information;
    information.fill(0);
    information(0,0) = cfg_->optim.weight_acc_lim_x;
    information(1,1) = cfg_->optim.weight_acc_lim_theta;
    
    // check if an initial velocity should be taken into accound
    if (vel_start_.first)
    {
      EdgeAccelerationStart* acceleration_edge = new EdgeAccelerationStart;
      acceleration_edge->setVertex(0,teb_.PoseVertex(0));
      acceleration_edge->setVertex(1,teb_.PoseVertex(1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex(0));
      acceleration_edge->setInitialVelocity(vel_start_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // now add the usual acceleration edge for each tuple of three teb poses
    for (int i=0; i < n - 2; ++i)
    {
      EdgeAcceleration* acceleration_edge = new EdgeAcceleration;
      acceleration_edge->setVertex(0,teb_.PoseVertex(i));
      acceleration_edge->setVertex(1,teb_.PoseVertex(i+1));
      acceleration_edge->setVertex(2,teb_.PoseVertex(i+2));
      acceleration_edge->setVertex(3,teb_.TimeDiffVertex(i));
      acceleration_edge->setVertex(4,teb_.TimeDiffVertex(i+1));
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }
    
    // check if a goal velocity should be taken into accound
    if (vel_goal_.first)
    {
      EdgeAccelerationGoal* acceleration_edge = new EdgeAccelerationGoal;
      acceleration_edge->setVertex(0,teb_.PoseVertex(n-2));
      acceleration_edge->setVertex(1,teb_.PoseVertex(n-1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex( teb_.sizeTimeDiffs()-1 ));
      acceleration_edge->setGoalVelocity(vel_goal_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }  
  }
  else // holonomic robot
  {
    Eigen::Matrix<double,3,3> information;
    information.fill(0);
    information(0,0) = cfg_->optim.weight_acc_lim_x;
    information(1,1) = cfg_->optim.weight_acc_lim_y;
    information(2,2) = cfg_->optim.weight_acc_lim_theta;
    
    // check if an initial velocity should be taken into accound
    if (vel_start_.first)
    {
      EdgeAccelerationHolonomicStart* acceleration_edge = new EdgeAccelerationHolonomicStart;
      acceleration_edge->setVertex(0,teb_.PoseVertex(0));
      acceleration_edge->setVertex(1,teb_.PoseVertex(1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex(0));
      acceleration_edge->setInitialVelocity(vel_start_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // now add the usual acceleration edge for each tuple of three teb poses
    for (int i=0; i < n - 2; ++i)
    {
      EdgeAccelerationHolonomic* acceleration_edge = new EdgeAccelerationHolonomic;
      acceleration_edge->setVertex(0,teb_.PoseVertex(i));
      acceleration_edge->setVertex(1,teb_.PoseVertex(i+1));
      acceleration_edge->setVertex(2,teb_.PoseVertex(i+2));
      acceleration_edge->setVertex(3,teb_.TimeDiffVertex(i));
      acceleration_edge->setVertex(4,teb_.TimeDiffVertex(i+1));
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }
    
    // check if a goal velocity should be taken into accound
    if (vel_goal_.first)
    {
      EdgeAccelerationHolonomicGoal* acceleration_edge = new EdgeAccelerationHolonomicGoal;
      acceleration_edge->setVertex(0,teb_.PoseVertex(n-2));
      acceleration_edge->setVertex(1,teb_.PoseVertex(n-1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex( teb_.sizeTimeDiffs()-1 ));
      acceleration_edge->setGoalVelocity(vel_goal_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }  
  }
}



void TebOptimalPlanner::AddEdgesTimeOptimal()
{
  if (cfg_->optim.weight_optimaltime==0)  // default: 1
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_optimaltime);

  for (int i=0; i < teb_.sizeTimeDiffs(); ++i)
  {
    EdgeTimeOptimal* timeoptimal_edge = new EdgeTimeOptimal;
    timeoptimal_edge->setVertex(0,teb_.TimeDiffVertex(i));
    timeoptimal_edge->setInformation(information);
    timeoptimal_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(timeoptimal_edge);
  }
}

void TebOptimalPlanner::AddEdgesShortestPath()
{
  if (cfg_->optim.weight_shortest_path==0)
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_shortest_path);

  for (int i=0; i < teb_.sizePoses()-1; ++i)
  {
    EdgeShortestPath* shortest_path_edge = new EdgeShortestPath;
    shortest_path_edge->setVertex(0,teb_.PoseVertex(i));
    shortest_path_edge->setVertex(1,teb_.PoseVertex(i+1));
    shortest_path_edge->setInformation(information);
    shortest_path_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(shortest_path_edge);
  }
}



void TebOptimalPlanner::AddEdgesKinematicsDiffDrive()
{
  if (cfg_->optim.weight_kinematics_nh==0 && cfg_->optim.weight_kinematics_forward_drive==0)
    return; // if weight equals zero skip adding edges!
  
  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,2,2> information_kinematics;
  information_kinematics.fill(0.0);
  information_kinematics(0, 0) = cfg_->optim.weight_kinematics_nh;
  information_kinematics(1, 1) = cfg_->optim.weight_kinematics_forward_drive;
  
  for (int i=0; i < teb_.sizePoses()-1; i++) // ignore twiced start only
  {
    EdgeKinematicsDiffDrive* kinematics_edge = new EdgeKinematicsDiffDrive;
    kinematics_edge->setVertex(0,teb_.PoseVertex(i));
    kinematics_edge->setVertex(1,teb_.PoseVertex(i+1));      
    kinematics_edge->setInformation(information_kinematics);
    kinematics_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(kinematics_edge);
  }	 
}

void TebOptimalPlanner::AddEdgesKinematicsCarlike()
{
  if (cfg_->optim.weight_kinematics_nh==0 && cfg_->optim.weight_kinematics_turning_radius==0)
    return; // if weight equals zero skip adding edges!

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,2,2> information_kinematics;
  information_kinematics.fill(0.0);
  information_kinematics(0, 0) = cfg_->optim.weight_kinematics_nh;
  information_kinematics(1, 1) = cfg_->optim.weight_kinematics_turning_radius;
  
  for (int i=0; i < teb_.sizePoses()-1; i++) // ignore twiced start only
  {
    EdgeKinematicsCarlike* kinematics_edge = new EdgeKinematicsCarlike;
    kinematics_edge->setVertex(0,teb_.PoseVertex(i));
    kinematics_edge->setVertex(1,teb_.PoseVertex(i+1));      
    kinematics_edge->setInformation(information_kinematics);
    kinematics_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(kinematics_edge);
  }  
}


void TebOptimalPlanner::AddEdgesPreferRotDir()
{
  //TODO(roesmann): Note, these edges can result in odd predictions, in particular
  //                we can observe a substantional mismatch between open- and closed-loop planning
  //                leading to a poor control performance.
  //                At the moment, we keep these functionality for oscillation recovery:
  //                Activating the edge for a short time period might not be crucial and
  //                could move the robot to a new oscillation-free state.
  //                This needs to be analyzed in more detail!
  if (prefer_rotdir_ == RotType::none || cfg_->optim.weight_prefer_rotdir==0)
    return; // if weight equals zero skip adding edges!

  if (prefer_rotdir_ != RotType::right && prefer_rotdir_ != RotType::left)
  {
    ROS_WARN("TebOptimalPlanner::AddEdgesPreferRotDir(): unsupported RotType selected. Skipping edge creation.");
    return;
  }
  // sleep(10);
  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,1,1> information_rotdir;
  information_rotdir.fill(cfg_->optim.weight_prefer_rotdir);
  
  for (int i=0; i < teb_.sizePoses()-1 && i < 3; ++i) // currently: apply to first 3 rotations
  {
    EdgePreferRotDir* rotdir_edge = new EdgePreferRotDir;
    rotdir_edge->setVertex(0,teb_.PoseVertex(i));
    rotdir_edge->setVertex(1,teb_.PoseVertex(i+1));      
    rotdir_edge->setInformation(information_rotdir);
    
    if (prefer_rotdir_ == RotType::left)
        rotdir_edge->preferLeft();
    else if (prefer_rotdir_ == RotType::right)
        rotdir_edge->preferRight();
    
    optimizer_->addEdge(rotdir_edge);
  }
}

void TebOptimalPlanner::AddEdgesVelocityObstacleRatio()
{
  Eigen::Matrix<double,2,2> information;
  information(0,0) = cfg_->optim.weight_velocity_obstacle_ratio;
  information(1,1) = cfg_->optim.weight_velocity_obstacle_ratio;
  information(0,1) = information(1,0) = 0;

  auto iter_obstacle = obstacles_per_vertex_.begin();

  for (int index = 0; index < teb_.sizePoses() - 1; ++index)
  {
    for (const ObstaclePtr obstacle : (*iter_obstacle++))
    {
      EdgeVelocityObstacleRatio* edge = new EdgeVelocityObstacleRatio;
      edge->setVertex(0,teb_.PoseVertex(index));
      edge->setVertex(1,teb_.PoseVertex(index + 1));
      edge->setVertex(2,teb_.TimeDiffVertex(index));
      edge->setInformation(information);
      edge->setParameters(*cfg_, robot_model_.get(), obstacle.get());
      optimizer_->addEdge(edge);
    }
  }
}

bool TebOptimalPlanner::hasDiverged() const
{
  // Early returns if divergence detection is not active
  if (!cfg_->recovery.divergence_detection_enable)
    return false;

  auto stats_vector = optimizer_->batchStatistics();

  // No statistics yet
  if (stats_vector.empty())
    return false;

  // Grab the statistics of the final iteration
  const auto last_iter_stats = stats_vector.back();

  return last_iter_stats.chi2 > cfg_->recovery.divergence_detection_max_chi_squared;
}

void TebOptimalPlanner::computeCurrentCost(double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost,double shrink_ratio)
{ 
  // check if graph is empty/exist  -> important if function is called between buildGraph and optimizeGraph/clearGraph
  bool graph_exist_flag(false);
  if (optimizer_->edges().empty() && optimizer_->vertices().empty())
  {
    // here the graph is build again, for time efficiency make sure to call this function 
    // between buildGraph and Optimize (deleted), but it depends on the application
    buildGraph(1.0,shrink_ratio);	
    // buildGraphObstacle(1.0,shrink_ratio);
    optimizer_->initializeOptimization();
  }
  else
  {
    graph_exist_flag = true;
  }
  
  optimizer_->computeInitialGuess();
  
  cost_ = 0;
  cost_origin_ = 0;

  if (alternative_time_cost)
  {
    //执行，计算了时间
    // sleep(10);
    cost_ += teb_.getSumOfAllTimeDiffs();
    cost_origin_ += teb_.getSumOfAllTimeDiffs();
    // TEST we use SumOfAllTimeDiffs() here, because edge cost depends on number of samples, which is not always the same for similar TEBs,
    // since we are using an AutoResize Function with hysteresis.
  }
  
  // now we need pointers to all edges -> calculate error for each edge-type
  // since we aren't storing edge pointers, we need to check every edge
  //成本调整
  for (std::vector<g2o::OptimizableGraph::Edge*>::const_iterator it = optimizer_->activeEdges().begin(); it!= optimizer_->activeEdges().end(); it++)
  {
    double cur_cost = (*it)->chi2();//获取误差的平方

    if (dynamic_cast<EdgeObstacle*>(*it) != nullptr
        || dynamic_cast<EdgeInflatedObstacle*>(*it) != nullptr
        || dynamic_cast<EdgeDynamicObstacle*>(*it) != nullptr)
    {
      cur_cost *= obst_cost_scale;
    }
    else if (dynamic_cast<EdgeViaPoint*>(*it) != nullptr)
    {
      cur_cost *= viapoint_cost_scale;
    }
    else if (dynamic_cast<EdgeTimeOptimal*>(*it) != nullptr && alternative_time_cost)
    {
      continue; // skip these edges if alternative_time_cost is active
    }
    // cost_ += cur_cost;
  }

//计算高斯代价
  double time = 0;
  double cost_gauss = 0;
  for (int i=0; i<teb_.sizePoses()-1; i++){
    auto p = teb_.PoseVertex(i);
    // std::cout<<"("<<p->x()<<","<<p->y()<<","<<time<<") ";
    double current_cost_gauss = 0;
    for (auto dynamic_obs:*obstacles_){
      if (!dynamic_obs->isDynamic())
        continue;
      auto po = dynamic_obs->getCentroid() + time*dynamic_obs->getCentroidVelocity();
      auto vo = dynamic_obs->getCentroidVelocity();
      // std::cout<<"relative: ("<<po.x()<<","<<po.y()<<"), ("<<vo.x()<<","<<vo.y()<<"), ("<<p->x()<<","<<p->y()<<")"<<std::endl;
      current_cost_gauss += get2DAsyGaussValue(po.x(), po.y(), vo.x(), vo.y(), p->x(), p->y());

    }
    cost_gauss = std::max(cost_gauss,current_cost_gauss);
    time += teb_.TimeDiff(i);
  }
  
  // 累计航向变化
  double cost_yaw = 0;
  for (int i = 0; i < teb_.sizePoses() - 1; i++)
  {
    cost_yaw += std::abs(teb_.PoseVertex(i + 1)->theta() - teb_.PoseVertex(i)->theta());
  }
  cost_yaw /= teb_.sizePoses();
  // cost_ += cost_yaw;

  // 累计到障碍物的距离
  time = 0;
  double cost_obs_dist = 0;
  for (int i = 0; i < teb_.sizePoses() - 1; i++)
  {
    auto p = teb_.PoseVertex(i)->pose().position();
    // std::cout<<"("<<p->x()<<","<<p->y()<<","<<time<<") ";
    for (auto dynamic_obs : *obstacles_)
    {
      if (!dynamic_obs->isDynamic())
        continue;
      auto po = dynamic_obs->getCentroid() + time * dynamic_obs->getCentroidVelocity();
      // std::cout<<"relative: ("<<po.x()<<","<<po.y()<<"), ("<<vo.x()<<","<<vo.y()<<"), ("<<p->x()<<","<<p->y()<<")"<<std::endl;
      cost_obs_dist += (p - po).norm();
    }
    time += teb_.TimeDiff(i);
  }
  cost_obs_dist /= teb_.sizePoses();//路径长度不一致因此需要归一化处理
  // cost_ += 1/cost_obs_dist;//取倒数，从远离行人的地方通过
  
  //区分前面轨迹和后面轨迹，前面轨迹代价大, 行人得有速度
  //但是如果机器人通过了行人但是没有到达终点，会认为反向轨迹是在行人前面，无法到达终点 depthfirst函数修改
  time = 0;
  double cost_distinguish = 0;
  for (int i = 0; i < teb_.sizePoses() - 1; i++)
  {
    auto p = teb_.PoseVertex(i)->pose().position();
    for(auto dynamic_obs : *obstacles_)
    {
      if (!dynamic_obs->isDynamic())
      {
        continue;
      }
    auto po = (dynamic_obs->getCentroidVelocity()).normalized();
    auto p_obs = (p - dynamic_obs->getCentroid()).normalized();
    cost_distinguish += po.dot(p_obs);
    }
  }
  cost_distinguish /= teb_.sizePoses();
  cost_ += -cost_distinguish*100000;
  

  // std::cout<<"cost: "<<cost_<<", "<<cost_gauss<<std::endl;
  // cost_ += cfg_->optim.weight_gauss *cost_gauss;
  //std::cout<<"cost: "<<cost_<<std::endl;
  // delete temporary created graph
  if (!graph_exist_flag) 
    clearGraph();
}

//高斯函数，从人前面的轨迹代价大，wxo、wyo为障碍物未来位置的x和y坐标，vxo、vyo是障碍物速度的xy分量，wx和wy是控制点的xy坐标
double TebOptimalPlanner::get2DAsyGaussValue(double wxo, double wyo, double vxo, double vyo, double wx, double wy){
  double rob_radius = 0.2;
  double obs_radius = 0.3;
  double rob_orientation = atan2(vyo,vxo);
  double human_direction = atan2(wy-wyo, wx-wxo);
  double delta_theta = human_direction - rob_orientation;
  if (delta_theta<0)
    delta_theta += 3.1415;

  double sigma_lr = 0.05+0.2*hypot(vxo,vyo), sigma_f = 0.5+1.0*hypot(vxo,vyo), sigma_b = 0.05+0.2*hypot(vxo,vyo);
  double A = 2.0;
  double dis = hypot(wx-wxo, wy-wyo)-rob_radius-obs_radius;

  double fb_part;
  //otherwise
  if (delta_theta>=1.67 && delta_theta<=1.67+3.14)
    fb_part = (dis*cos(delta_theta))*(dis*cos(delta_theta))/(2*sigma_b*sigma_b);
  else  //[-pi/2,pi/2]
    fb_part = (dis*cos(delta_theta))*(dis*cos(delta_theta))/(2*sigma_f*sigma_f);
  double lr_part = (dis*sin(delta_theta))*(dis*sin(delta_theta))/(2*sigma_lr*sigma_lr);
  double res = A*pow(2.718,(-lr_part-fb_part));
  return res>0.05?res:0;
}

void TebOptimalPlanner::extractVelocity(const PoseSE2& pose1, const PoseSE2& pose2, double dt, double& vx, double& vy, double& omega) const
{
  if (dt == 0)
  {
    vx = 0;
    vy = 0;
    omega = 0;
    return;
  }
  
  Eigen::Vector2d deltaS = pose2.position() - pose1.position();
  
  if (cfg_->robot.max_vel_y == 0) // nonholonomic robot
  {
    Eigen::Vector2d conf1dir( cos(pose1.theta()), sin(pose1.theta()) );
    // translational velocity
    double dir = deltaS.dot(conf1dir);
    vx = (double) g2o::sign(dir) * deltaS.norm()/dt;
    vy = 0;
  }
  else // holonomic robot
  {
    // transform pose 2 into the current robot frame (pose1)
    // for velocities only the rotation of the direction vector is necessary.
    // (map->pose1-frame: inverse 2d rotation matrix)
    double cos_theta1 = std::cos(pose1.theta());
    double sin_theta1 = std::sin(pose1.theta());
    double p1_dx =  cos_theta1*deltaS.x() + sin_theta1*deltaS.y();
    double p1_dy = -sin_theta1*deltaS.x() + cos_theta1*deltaS.y();
    vx = p1_dx / dt;
    vy = p1_dy / dt;    
  }
  
  // rotational velocity
  double orientdiff = g2o::normalize_theta(pose2.theta() - pose1.theta());
  omega = orientdiff/dt;
}

bool TebOptimalPlanner::getVelocityCommand(double& vx, double& vy, double& omega, int look_ahead_poses) const
{
  if (teb_.sizePoses()<2)
  {
    ROS_ERROR("TebOptimalPlanner::getVelocityCommand(): The trajectory contains less than 2 poses. Make sure to init and optimize/plan the trajectory fist.");
    vx = 0;
    vy = 0;
    omega = 0;
    return false;
  }
  look_ahead_poses = std::max(1, std::min(look_ahead_poses, teb_.sizePoses() - 1));
  double dt = 0.0;
  for(int counter = 0; counter < look_ahead_poses; ++counter)
  {
    dt += teb_.TimeDiff(counter);
    if(dt >= cfg_->trajectory.dt_ref * look_ahead_poses)  // TODO: change to look-ahead time? Refine trajectory?
    {
        look_ahead_poses = counter + 1;
        break;
    }
  }
  if (dt<=0)
  {	
    ROS_ERROR("TebOptimalPlanner::getVelocityCommand() - timediff<=0 is invalid!");
    vx = 0;
    vy = 0;
    omega = 0;
    return false;
  }
	  
  // Get velocity from the first two configurations
  extractVelocity(teb_.Pose(0), teb_.Pose(look_ahead_poses), dt, vx, vy, omega);
  return true;
}

void TebOptimalPlanner::getVelocityProfile(std::vector<geometry_msgs::Twist>& velocity_profile) const
{
  int n = teb_.sizePoses();
  velocity_profile.resize( n+1 );

  // start velocity 
  velocity_profile.front().linear.z = 0;
  velocity_profile.front().angular.x = velocity_profile.front().angular.y = 0;  
  velocity_profile.front().linear.x = vel_start_.second.linear.x;
  velocity_profile.front().linear.y = vel_start_.second.linear.y;
  velocity_profile.front().angular.z = vel_start_.second.angular.z;
  
  for (int i=1; i<n; ++i)
  {
    velocity_profile[i].linear.z = 0;
    velocity_profile[i].angular.x = velocity_profile[i].angular.y = 0;
    extractVelocity(teb_.Pose(i-1), teb_.Pose(i), teb_.TimeDiff(i-1), velocity_profile[i].linear.x, velocity_profile[i].linear.y, velocity_profile[i].angular.z);
  }
  
  // goal velocity
  velocity_profile.back().linear.z = 0;
  velocity_profile.back().angular.x = velocity_profile.back().angular.y = 0;  
  velocity_profile.back().linear.x = vel_goal_.second.linear.x;
  velocity_profile.back().linear.y = vel_goal_.second.linear.y;
  velocity_profile.back().angular.z = vel_goal_.second.angular.z;
}

void TebOptimalPlanner::getFullTrajectory(std::vector<TrajectoryPointMsg>& trajectory) const
{
  int n = teb_.sizePoses();
  
  trajectory.resize(n);
  
  if (n == 0)
    return;
     
  double curr_time = 0;
  
  // start
  TrajectoryPointMsg& start = trajectory.front();
  teb_.Pose(0).toPoseMsg(start.pose);
  start.velocity.linear.z = 0;
  start.velocity.angular.x = start.velocity.angular.y = 0;
  start.velocity.linear.x = vel_start_.second.linear.x;
  start.velocity.linear.y = vel_start_.second.linear.y;
  start.velocity.angular.z = vel_start_.second.angular.z;
  start.time_from_start.fromSec(curr_time);
  
  curr_time += teb_.TimeDiff(0);
  
  // intermediate points
  for (int i=1; i < n-1; ++i)
  {
    TrajectoryPointMsg& point = trajectory[i];
    teb_.Pose(i).toPoseMsg(point.pose);
    point.velocity.linear.z = 0;
    point.velocity.angular.x = point.velocity.angular.y = 0;
    double vel1_x, vel1_y, vel2_x, vel2_y, omega1, omega2;
    extractVelocity(teb_.Pose(i-1), teb_.Pose(i), teb_.TimeDiff(i-1), vel1_x, vel1_y, omega1);
    extractVelocity(teb_.Pose(i), teb_.Pose(i+1), teb_.TimeDiff(i), vel2_x, vel2_y, omega2);
    point.velocity.linear.x = 0.5*(vel1_x+vel2_x);
    point.velocity.linear.y = 0.5*(vel1_y+vel2_y);
    point.velocity.angular.z = 0.5*(omega1+omega2);    
    point.time_from_start.fromSec(curr_time);
    
    curr_time += teb_.TimeDiff(i);
  }
  
  // goal
  TrajectoryPointMsg& goal = trajectory.back();
  teb_.BackPose().toPoseMsg(goal.pose);
  goal.velocity.linear.z = 0;
  goal.velocity.angular.x = goal.velocity.angular.y = 0;
  goal.velocity.linear.x = vel_goal_.second.linear.x;
  goal.velocity.linear.y = vel_goal_.second.linear.y;
  goal.velocity.angular.z = vel_goal_.second.angular.z;
  goal.time_from_start.fromSec(curr_time);
}


bool TebOptimalPlanner::isTrajectoryFeasible(base_local_planner::CostmapModel* costmap_model, const std::vector<geometry_msgs::Point>& footprint_spec,
                                             double inscribed_radius, double circumscribed_radius, int look_ahead_idx)
{
  if (look_ahead_idx < 0 || look_ahead_idx >= teb().sizePoses())
    look_ahead_idx = teb().sizePoses() - 1;
  
  for (int i=0; i <= look_ahead_idx; ++i)
  {           
    if ( costmap_model->footprintCost(teb().Pose(i).x(), teb().Pose(i).y(), teb().Pose(i).theta(), footprint_spec, inscribed_radius, circumscribed_radius) == -1 )
    {
      if (visualization_)
      {
        visualization_->publishInfeasibleRobotPose(teb().Pose(i), *robot_model_);
      }
      return false;
    }
    // Checks if the distance between two poses is higher than the robot radius or the orientation diff is bigger than the specified threshold
    // and interpolates in that case.
    // (if obstacles are pushing two consecutive poses away, the center between two consecutive poses might coincide with the obstacle ;-)!
    if (i<look_ahead_idx)
    {
      double delta_rot = g2o::normalize_theta(g2o::normalize_theta(teb().Pose(i+1).theta()) -
                                              g2o::normalize_theta(teb().Pose(i).theta()));
      Eigen::Vector2d delta_dist = teb().Pose(i+1).position()-teb().Pose(i).position();
      if(fabs(delta_rot) > cfg_->trajectory.min_resolution_collision_check_angular || delta_dist.norm() > inscribed_radius)
      {
        int n_additional_samples = std::max(std::ceil(fabs(delta_rot) / cfg_->trajectory.min_resolution_collision_check_angular), 
                                            std::ceil(delta_dist.norm() / inscribed_radius)) - 1;
        PoseSE2 intermediate_pose = teb().Pose(i);
        for(int step = 0; step < n_additional_samples; ++step)
        {
          intermediate_pose.position() = intermediate_pose.position() + delta_dist / (n_additional_samples + 1.0);
          intermediate_pose.theta() = g2o::normalize_theta(intermediate_pose.theta() + 
                                                           delta_rot / (n_additional_samples + 1.0));
          if ( costmap_model->footprintCost(intermediate_pose.x(), intermediate_pose.y(), intermediate_pose.theta(),
            footprint_spec, inscribed_radius, circumscribed_radius) == -1 )
          {
            if (visualization_) 
            {
              visualization_->publishInfeasibleRobotPose(intermediate_pose, *robot_model_);
            }
            return false;
          }
        }
      }
    }
  }
  return true;
}

} // namespace teb_local_planner
