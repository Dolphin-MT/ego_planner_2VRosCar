// #include <fstream>
#include <ego_planner/planner_manager.h>
#include <thread>
#include "visualization_msgs/msg/marker.hpp" // zx-todo

namespace ego_planner
{

  EGOPlannerManager::EGOPlannerManager() {}

  EGOPlannerManager::~EGOPlannerManager() {}

  void EGOPlannerManager::initPlanModules(rclcpp::Node::SharedPtr &node, PlanningVisualization::Ptr vis)
  {
    node->declare_parameter("manager/max_vel", -1.0);
    node->declare_parameter("manager/max_acc", -1.0);
    node->declare_parameter("manager/max_jerk", -1.0);
    node->declare_parameter("manager/feasibility_tolerance", 0.0);
    node->declare_parameter("manager/control_points_distance", -1.0);
    node->declare_parameter("manager/planning_horizon", 5.0);
    node->declare_parameter("manager/use_distinctive_trajs", false);
    node->declare_parameter("manager/drone_id", -1);

    node->get_parameter("manager/max_vel", pp_.max_vel_);
    node->get_parameter("manager/max_acc", pp_.max_acc_);
    node->get_parameter("manager/max_jerk", pp_.max_jerk_);
    node->get_parameter("manager/feasibility_tolerance", pp_.feasibility_tolerance_);
    node->get_parameter("manager/control_points_distance", pp_.ctrl_pt_dist);
    node->get_parameter("manager/planning_horizon", pp_.planning_horizen_);
    node->get_parameter("manager/use_distinctive_trajs", pp_.use_distinctive_trajs);
    node->get_parameter("manager/drone_id", pp_.drone_id);

    local_data_.traj_id_ = 0;
    grid_map_.reset(new GridMap);
    // grid_map_->initMap(nh);
    grid_map_->initMap(node);
    node_ = node;//ROS小车更改

    bspline_optimizer_.reset(new BsplineOptimizer);
    // bspline_optimizer_->setParam(nh);
    bspline_optimizer_->setParam(node);
    bspline_optimizer_->setEnvironment(grid_map_, obj_predictor_);
    bspline_optimizer_->a_star_.reset(new AStar);
    bspline_optimizer_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 1));

    visualization_ = vis;
  }

  /*一个重规划函数
  * 1.前置检查
    2.INT 初始化 生成初始路径和控制点
    3.Optimze B样条轨迹优化
    4.Refine 时间重分配 让轨迹满足动力学约束
  传入变量解释：
    flag_polyInit：是否强制用多项式生成初始路径
    flag_randomPolyTraj：是否使用随机扰动的多项式（用于避障/探索）
  */
  bool EGOPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
  {
    static int count = 0;//记录规划的次数
    //重要的消息会在终端打印出来，方便调试，绿色
    printf("\033[47;30m\n[drone %d replan %d]==============================================\033[0m\n", pp_.drone_id, count++);

    //起点接近目标点检测 太近就认为已经到达目标点，不再进行规划
    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      cout << "Close to goal" << endl;
      continous_failures_count_++;
      return false;
    }

    bspline_optimizer_->setLocalTargetPt(local_target_pt);//将局部目标点传给优化器

    rclcpp::Time t_start = node_->now();
    rclcpp::Duration t_init(0, 0), t_opt(0, 0), t_refine(0, 0);//初始化消耗时间，优化消耗时间，重参数化消耗时间

    /*** STEP 1: INIT 初始化 刚起步走不动，需要在热身时给一个初值，讲解怎么生成一个较好的初值
    根据起始点和目标点的距离计算首个时间步长ts,向量的模大于0.1则用1.5倍否则用5倍
    ts：B 样条的时间步长
    ***/
   //ctrl_pt_dist / max_vel 是理论最小时间步长（控制点间距 / 最大速度），乘以系数保证可行性。
    double ts = (start_pt - local_target_pt).norm() > 0.1 
    ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.5 //距离远 较紧的时间步长
    : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; //距离近 较松的时间步长
    // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    vector<Eigen::Vector3d> point_set, start_end_derivatives;//起点和终点的速度 加速度
    static bool flag_first_call = true, flag_force_polynomial = false;
    bool flag_regenerate = false; //是否重新生成轨迹标志位
    do
    {
      point_set.clear(); //point_set 采样得到的控制点位置序列
      start_end_derivatives.clear();
      flag_regenerate = false;
      //分支A：多项式初始化
      // 这里如果正常进入if（通常为初次生成），则do部分只进行一次，即只清空一次点集；若进入else则有可能对异常情况重置flag_regenerate并再do一次
      if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated from a min-snap traj by order.
      {//首次 强制多项式 上次失败强制
        flag_first_call = false;
        flag_force_polynomial = false;
        // 用于存储生成的轨迹
        PolynomialTraj gl_traj;

        double dist = (start_pt - local_target_pt).norm();
        // 梯形速度曲线时间计算：
        // 如果能加速到 max_vel 再减速：时间 = 加速段 + 匀速段 + 减速段
        // 如果距离太短，无法达到 max_vel：时间 = 2 * sqrt(dist / acc)
        double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist 
        ? 2*sqrt(dist / pp_.max_acc_) 
        : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

        if (!flag_randomPolyTraj) //确定性多项式 分支A1
        {
          gl_traj = PolynomialTraj::one_segment_traj_gen(
            start_pt, start_vel, start_acc, //起点状态
            local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), //终点状态
            time);//轨迹时长
        }
        else//随机多项式轨迹 分支A2
        {
          // normalized():单位化；cross():差乘
          // 计算水平方向 start_pt - local_target_pt 与(0,0,1)的叉积，得到垂直于两者的向量，并单位化
          Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
          //计算垂直方向(垂直于起点-目标连线和水平方向）
          Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
          //随机插入点 = 中点+水平随机偏移+垂直随机偏移
          // continous_failures_count_ 连续失败次数 水平偏移系数 0.8 > 垂直偏移系数 0.4→ 优先在水平面内绕道（更符合无人机运动特性）
          Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +//中点
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +//水平偏移
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);//垂直偏移
          
                                               Eigen::MatrixXd pos(3, 3);
          // pos.col(0)—— 作用是“获取矩阵的第 0 列，并把它当作一个可读写的向量”（引用）。
          pos.col(0) = start_pt;
          pos.col(1) = random_inserted_pt;
          pos.col(2) = local_target_pt;
          Eigen::VectorXd t(2);
          t(0) = t(1) = time / 2;
          //生成的是两段光滑连接的 5 次多项式，在 P 点位置、速度、加速度都连续。
          gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
        }

        // t：沿多项式采样的时间，[s]；循环结束后用于取得最后一个有效样本状态。
        // flag_too_far：只要存在相邻点距大于 1.5*ctrl_pt_dist，就需要加密采样。
        double t;
        bool flag_too_far;
        // 预乘 1.5 是为了抵消下面第一次循环开头的除法，使第一次尝试仍使用
        // 前面计算的原始 ts；后续失败才依次变为 ts/1.5、ts/1.5^2……。
        ts *= 1.5; // ts will be divided by 1.5 in the next
        // 得到 7 个点，保证三次 B 样条参数化有足够控制点。
        do
        {
          ts /= 1.5;//逐步减小步长 增加采样密度
          point_set.clear();
          flag_too_far = false;
          Eigen::Vector3d last_pt = gl_traj.evaluate(0); //evaluate(t):求多项式在时刻 t 的位置
          for (t = 0; t < time; t += ts)
          {
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            //检查相邻采样点距离是否过大
            if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
            {
              flag_too_far = true;
              break;
            }
            last_pt = pt;
            point_set.push_back(pt);
          }
        } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
        t -= ts;//回退到最后一个有效时刻
        //记录起点和终点的导数 速度 加速度
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));
        start_end_derivatives.push_back(local_target_vel);      //终点速度
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));//终点加速度
      }
      else // Initial path generated from previous trajectory. 分支B：多项式初始化失败，使用上次轨迹的终点作为起点，重新生成轨迹
      {
        flag_regenerate = false;
        // 滚动重规划：采样旧轨迹尚未执行的部分，再用一段多项式连接到新局部目标，最后按近似弧长重新均匀采样。
        // t 是旧轨迹/连接多项式的采样时间；t_cur 是旧轨迹已经执行的时间，[s]。
        double t;
        double t_cur = (node_->now() - local_data_.start_time_).seconds();
        //计算当前轨迹的伪弧长
        vector<double> pseudo_arc_length;//累积距离 pseudo_arc_length 记录了从起点到每个点的累计路径长度
        // segment_point 保存“旧轨迹剩余部分 + 通往新局部目标的连接段”的稠密样本。
        vector<Eigen::Vector3d> segment_point;
        pseudo_arc_length.push_back(0.0); //push_back()意为：比如：往pseudo_arc_length的末尾追加一个数据0.0
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
        {
          //evaluateDeBoorT(t)：B 样条轨迹在时刻 t 的位置求值（De Boor 算法）
          segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
          if (t > t_cur)
          {
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        t -= ts;
        //生成延伸段多项式 计算从上次轨迹终点到局部目标的延伸段时间
        double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
        if (poly_time > ts)
        {
          //用 one_segment_traj_gen 生成一段多项式连接过去
          PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                        local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                        local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);
          // 将延伸段加入采样
          for (t = ts; t < poly_time; t += ts)
          {
            if (!pseudo_arc_length.empty())
            {
              segment_point.push_back(gl_traj.evaluate(t));
              pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
            }
            else
            {
              RCLCPP_ERROR(rclcpp::get_logger("ego_planner"), "pseudo_arc_length is empty, return!");
              continous_failures_count_++;
              return false;
            }
          }
        }
        //按照弧长间隔 cps_dist 重采样控制点
        double sample_length = 0;
        double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
        size_t id = 0;
        do
        {
          cps_dist /= 1.5;// 第一次就除回来，变成 ctrl_pt_dist
          point_set.clear();
          sample_length = 0;
          id = 0;
          //back():返回容器中最后一个元素  pseudo_arc_length.back()等价于 pseudo_arc_length[pseudo_arc_length.size() - 1]
          //pseudo_arc_length 记录了从起点到每个点的累计路径长度
          while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
          {
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
            {
              //一维线性插值公式P(interp)=a*P(id+1)+(1-a)P(id)
              //其中    (sample_length - pseudo_arc_length[id]) 
              //  a= —————————————————————————————————————————————————— 
              //     (pseudo_arc_length[id + 1] - pseudo_arc_length[id])
              point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                  (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
              sample_length += cps_dist;
            }
            else
              id++;
          }
          point_set.push_back(local_target_pt);
        } while (point_set.size() < 7); // If the start point is very close to end point, this will help
        //收集边界导数：上次轨迹在当前时刻的速度/加速度，以及目标速度、零加速度
        start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());

        if (point_set.size() > pp_.planning_horizen_ / pp_.ctrl_pt_dist * 3) // The initial path is unnormally too long!
        {//如果控制点数量异常多
          flag_force_polynomial = true;
          flag_regenerate = true;
        }
      }
    } while (flag_regenerate);

    // 将轨迹变为B样条轨迹
    /*
    ctrl_pts：当前这条轨迹的 B 样条控制点矩阵
      是后续非线性优化的初始猜想 尺寸是3*N 3就是三个轴 N=路径点+2
      最后会储存B样条轨迹的最终控制点
    ctrl_pts_temp：临时工作区
      多机优化时会循环被调用
    */
      
    Eigen::MatrixXd ctrl_pts, ctrl_pts_temp; 
    /*parameterizeToBspline()：桥梁作用 
      将离散控制点point_set + 边界导数约束start_end_derivatives，参数化为均匀 B 样条控制点矩阵
    * ctrl_pts 是 (3, N) 的矩阵，每列是一个 3D 控制点
    * 使用了B样条的端点插值性质，通过求解线性方程组确保 B 样条经过给定控制点并满足边界速度/加速度
    */
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

    vector<std::pair<int, int>> segments;
    /*initControlPoints: 检测 ctrl_pts 中哪些控制点位于障碍物内，返回碰撞段的索引区间 [start_idx, end_idx]
      检测控制点碰撞，初始化梯度计算所需的 base_point/direction，返回碰撞段区间[start_idx, end_idx]。
      这里貌似还有AstarSearch的内容
    */
    segments = bspline_optimizer_->initControlPoints(ctrl_pts, true); //碰撞检测
    // 计算时间差并更新时间
    auto now = node_->now();
    t_init = now - t_start;
    t_start = now;

    /*** STEP 2: OPTIMIZE 
     * 开始做优化，优化的目标是让轨迹在满足约束条件下尽量平滑，约束条件包括速度、加速度、jerk、碰撞等
     * 1. 优化目标函数：最小化轨迹的jerk（三阶导数）平方和，保证轨迹平滑
     * 2. 约束条件：
     *    a. 速度约束：轨迹的速度不能超过最大速度限制
     *    b. 加速度约束：轨迹的加速度不能超过最大加速度限制
     *    c. jerk约束：轨迹的jerk不能超过最大jerk限制
     *    d. 碰撞约束：轨迹不能与障碍物发生碰撞，保证安全性
     * 3. 优化方法：使用非线性优化算法（如SQP、IPOPT等）对控制点进行优化，得到满足约束条件的最优轨迹
     * 4. 输出结果：优化后的控制点和轨迹，以及优化所需的时间    
     * ***/
    bool flag_step_1_success = false;
    vector<vector<Eigen::Vector3d>> vis_trajs; //多条轨迹的集合 暂存所有优化成功的候选轨迹，供可视化用
    if (pp_.use_distinctive_trajs)//多机集群
    {
      // cout << "enter" << endl;
      //生成多条差异化的初始轨迹 为了让多台无人机能够分散开
      std::vector<ControlPoints> trajs = bspline_optimizer_->distinctiveTrajs(segments);
      cout << "\033[1;33m"
           << "multi-trajs=" << trajs.size() << "\033[1;0m" << endl;

      double final_cost, min_cost = 999999.0;
      /*BsplineOptimizeTrajRebound() 轨迹优化与避障的核心代码
        核心优化函数，使用梯度下降/L-BFGS优化 B 样条控制点
        目标函数：平滑项（最小化 jerk）+ 碰撞惩罚项 + 可行性惩罚项 + 目标牵引项
        约束：通过软约束（惩罚函数）处理，而非硬约束
      */
      for (int i = trajs.size() - 1; i >= 0; i--) //对每一条轨迹进行优化
      {
        if (bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts_temp, final_cost, trajs[i], ts))
        {
          cout << "traj " << trajs.size() - i << " success." << endl;

          flag_step_1_success = true;
          if (final_cost < min_cost)
          {
            min_cost = final_cost;
            ctrl_pts = ctrl_pts_temp;
          }

          // visualization 可视化
          point_set.clear(); //point_set会被循环使用
          for (int j = 0; j < ctrl_pts_temp.cols(); j++)//cols():返回矩阵的列数
          {
            //col(j):返回矩阵的第j列，得到一个烈向量表达式
            // // 依次把每列(每个控制点)放进点集
            point_set.push_back(ctrl_pts_temp.col(j));
          }
          vis_trajs.push_back(point_set);
        }
        else
        {
          cout << "traj " << trajs.size() - i << " failed." << endl;
        }
      }

      t_opt = node_->now() - t_start;

      visualization_->displayMultiInitPathList(vis_trajs, 0.2);
    }
    else//单机模式
    {
      //BsplineOptimizeTrajRebound():检测是否会穿过障碍物；
      //并在rebound_optimize中重新规划在障碍物中的轨迹 直接优化初始控制点，不生成多条候选轨迹
      flag_step_1_success = bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
      t_opt = node_->now() - t_start;
      // static int vis_id = 0;
      visualization_->displayInitPathList(point_set, 0.2, 0);
    }

    cout << "plan_success=" << flag_step_1_success << endl;
    if (!flag_step_1_success) //如果优化失败，说明轨迹不可行，可能是因为障碍物太多或者参数设置不合理，需要重新规划
    {
      visualization_->displayOptimalList(ctrl_pts, 0);
      continous_failures_count_++;
      return false;
    }
    //精细化 时间重分配
    t_start = node_->now();

    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);//tolerance 容差

    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
    // Note: Only adjust time in single drone mode. But we still allow drone_0 to adjust its time profile.
    if (pp_.drone_id <= 0) //单机或多机中 drone_0
    {

      double ratio;
      bool flag_step_2_success = true;
      if (!pos.checkFeasibility(ratio, false)) //检查是否满足速度和加速度约束
      {
        cout << "Need to reallocate time." << endl;

        Eigen::MatrixXd optimal_control_points;
        //refineTrajAlgo() 时间重分配算法 将ts调整为ts*ratio 再次检查是否有碰撞
        flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
        if (flag_step_2_success)
          pos = UniformBspline(optimal_control_points, 3, ts);
      }

      if (!flag_step_2_success)
      {
        printf("\033[34mThis refined trajectory hits obstacles. It doesn't matter if appeares occasionally. But if continously appearing, Increase parameter \"lambda_fitness\".\n\033[0m");
        continous_failures_count_++;
        return false;
      }
    }
    else//禁用精细化 他们时间必须同步 多机模式
    {
      static bool print_once = true;
      if (print_once)
      {
        print_once = false;
        RCLCPP_ERROR(rclcpp::get_logger("ego_planner"), "IN SWARM MODE, REFINE DISABLED!");
      }
    }
    //保存结果与统计
    // t_refine = ros::Time::now() - t_start;
    t_refine = node_->now() - t_start;

    // save planned results 将优化后的 B 样条 pos 保存到 local_data_ 中
    updateTrajInfo(pos, node_->now());

    static double sum_time = 0;
    static int count_success = 0;

    sum_time += (t_init + t_opt + t_refine).seconds();

    count_success++;

    // cout << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec() << "\033[0m,optimize:" << (t_init + t_opt).toSec() << ",refine:" << t_refine.toSec() << ",avg_time=" << sum_time / count_success << endl;
    cout << "total time:\033[42m" << (t_init + t_opt + t_refine).seconds() << "\033[0m,optimize:" << (t_init + t_opt).seconds() << ",refine:" << t_refine.seconds() << ",avg_time=" << sum_time / count_success << endl;

    // success. YoY
    continous_failures_count_ = 0;
    return true;
  }

  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), node_->now());

    return true;
  }

  bool EGOPlannerManager::checkCollision(int drone_id)
  {
    // if (local_data_.start_time_.toSec() < 1e9) // It means my first planning has not started
    if (local_data_.start_time_.seconds() < 1e9)
      return false;

    // double my_traj_start_time = local_data_.start_time_.toSec();
    // double other_traj_start_time = swarm_trajs_buf_[drone_id].start_time_.toSec();
    //确定时间对齐参数
    double my_traj_start_time = local_data_.start_time_.seconds();
    double other_traj_start_time = swarm_trajs_buf_[drone_id].start_time_.seconds();
    // 取较晚的起始时间 → 两者都有轨迹的最早时刻
    double t_start = max(my_traj_start_time, other_traj_start_time);
    // 取较早的结束时间 → 两者都有轨迹的最晚时刻
    double t_end = min(my_traj_start_time + local_data_.duration_ * 2 / 3, other_traj_start_time + swarm_trajs_buf_[drone_id].duration_);

    for (double t = t_start; t < t_end; t += 0.03)
    {
      /*
        t:绝对时间（ROS2 时间戳），在重叠窗口内采样
        t-my_traj_start_time:   本机轨迹的相对时间
        t-other_traj_start_time:用 De Boor 算法计算 B-spline 在该时刻的位置
        evaluateDeBoorT:        计算B样条轨迹在给定时间的点
        (A-B).norm():           计算两点之间的欧几里得距离
        < getSwarmClearance():  如果距离小于安全间距，则返回 true，表示发生碰撞
      */
      if ((local_data_.position_traj_.evaluateDeBoorT(t - my_traj_start_time) - swarm_trajs_buf_[drone_id].position_traj_.evaluateDeBoorT(t - other_traj_start_time)).norm() < bspline_optimizer_->getSwarmClearance())
      {
        return true;
      }
    }

    return false;
  }

  bool EGOPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    double total_len = 0;
    total_len += (start_pos - waypoints[0]).norm();
    for (size_t i = 0; i < waypoints.size() - 1; i++)
    {
      total_len += (waypoints[i + 1] - waypoints[i]).norm();
    }

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = node_->now();

    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool EGOPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    /*挨个读取点并计算点距判断是否需要插点，随后计算插点并写入矩阵，最后根据插点数量生成全局轨迹
      最终返回值为是否规划成功的布尔值 */
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = node_->now();

    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool EGOPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    // std::cout << "ratio: " << ratio << std::endl;
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    bool success = bspline_optimizer_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  void EGOPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now)
  {
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
  }

  void EGOPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    double time_origin = bspline.getTimeSum();
    int seg_num = bspline.getControlPoint().cols() - 3;

    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  }

} // namespace ego_planner
