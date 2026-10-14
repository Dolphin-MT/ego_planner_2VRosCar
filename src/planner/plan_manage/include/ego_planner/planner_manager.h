#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <traj_utils/msg/data_disp.hpp>
#include <plan_env/grid_map.h>
#include <plan_env/obj_predictor.h>
#include <traj_utils/plan_container.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traj_utils/planning_visualization.h>

namespace ego_planner
{

  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  class EGOPlannerManager//计划管理器类，主要用于管理规划器的各个模块，包括地图、目标点、轨迹优化等
  {
    // SECTION stable
  public:
    EGOPlannerManager();
    ~EGOPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    //重规划函数：EGO-Planner 的核心算法入口，执行完整的轨迹重规划流程
    // flag_polyInit: 是否使用随机多项式 初始化标志
    // flag_randomPolyTraj: 是否使用随机多项式 轨迹标志
    bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                       Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit, bool flag_randomPolyTraj);
    //紧急停机函数：生成一条从当前位置减速到零速度的紧急制动轨迹
    bool EmergencyStop(Eigen::Vector3d stop_pos);
    // 规划一条从起点到终点的全局参考轨迹
    bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    // 规划一条通过多个航路点的全局参考轨迹
    //waypoints：中间航路点列表
    bool planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                 const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    //初始化规划模块 vis：可视化模块指针，默认为空 用于rviz显示 EGOReplanFSM::init() 中调用
    void initPlanModules(rclcpp::Node::SharedPtr &node, PlanningVisualization::Ptr vis = NULL);
    // 传递集群轨迹给优化器
    /*
      将 swarm_trajs_buf_（存储的其他无人机轨迹缓冲区）传递给优化器
      优化器在计算代价函数时，需要查询其他无人机的轨迹来评估集群碰撞代价
    */
    void deliverTrajToOptimizer(void) { bspline_optimizer_->setSwarmTrajs(&swarm_trajs_buf_); };

    //设置本级ID
    void setDroneIdtoOpt(void) { bspline_optimizer_->setDroneId(pp_.drone_id); }

    //获取集群安全间距 返回最小安全间距
    double getSwarmClearance(void) { return bspline_optimizer_->getSwarmClearance(); }
    //检查本机轨迹与指定其他无人机的轨迹是否在时间和空间上发生冲突（碰撞）。
    bool checkCollision(int drone_id);
    

    PlanParameters pp_; //存储所有规划算法的配置参数
    LocalTrajData local_data_; //存储本机正在执行的 B-spline 轨迹的所有信息
    GlobalTrajData global_data_;  //存储从起点到终点的全局参考路径
    GridMap::Ptr grid_map_; //存储环境地图的指针，GridMap 类用于管理占据栅格地图和欧几里得距离信息
    fast_planner::ObjPredictor::Ptr obj_predictor_; //预测动态障碍物的未来轨迹（如行人、其他移动物体）。
    SwarmTrajData swarm_trajs_buf_; //存储集群中所有其他无人机的轨迹信息，是实现多机避障的核心数据结构。

  private:
    /* main planning algorithms & modules */
    PlanningVisualization::Ptr visualization_;

    // ros::Publisher obj_pub_; //zx-todo 

    //原因：EGOPlannerManager 内部原来没有保存 node，
    //所以它无法使用节点的 ROS/仿真时钟，只能用 rclcpp::Clock().now() 获取系统时间
    rclcpp::Node::SharedPtr node_;//增加一个节点指针 ROS小车改动

    BsplineOptimizer::Ptr bspline_optimizer_;


    int continous_failures_count_{0};

    void updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now);

    void reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
                        double &time_inc);

    bool refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);

    // !SECTION stable

    // SECTION developing

  public:
    typedef unique_ptr<EGOPlannerManager> Ptr;

    // !SECTION
  };
} // namespace ego_planner

#endif