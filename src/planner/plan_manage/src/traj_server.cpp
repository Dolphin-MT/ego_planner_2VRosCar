#include "bspline_opt/uniform_bspline.h"//Bsppline 类优化器
#include "nav_msgs/msg/odometry.hpp"    // 里程计消息类型
#include "traj_utils/msg/bspline.hpp"   // B样条轨迹消息类型
#include "quadrotor_msgs/msg/position_command.hpp"// 无人机位置控制命令消息类型
#include "std_msgs/msg/empty.hpp"       // 空消息类型
#include "visualization_msgs/msg/marker.hpp"   // 可视化标记消息类型
#include <rclcpp/rclcpp.hpp>  // ROS2 C++ 客户端库

/*traj_server.cpp
 *  作为轨迹服务器，接收bspline轨迹消息，解析后发布无人机位置控制命令，是规划层与控制层的桥梁

*/
//发布者 发布位置消息给无人机控制器 pos_cmd_pub
rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub;

quadrotor_msgs::msg::PositionCommand cmd;//位置指令消息模板
//PID增益
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

using ego_planner::UniformBspline;

bool receive_traj_ = false;//轨迹接收标志位
vector<UniformBspline> traj_;// 轨迹存储：位置、速度、加速度（B-spline 及其导数）
double traj_duration_;//轨迹总时长
rclcpp::Time start_time_;// 轨迹起始时间
int traj_id_;//轨迹ID

// yaw control  偏航角控制相关
double last_yaw_, last_yaw_dot_;// 上一次的偏航角和角速度
double time_forward_;           // 前视时间（用于偏航角计算）

//功能：接收ego_planner 发布的B_spline轨迹 解析并储存
/*疑问：
  1.msg->knots.size()与knots(i) = msg->knots[i];中的msg有什么区别？
  2.pos_pts(0, i) = msg->pos_pts[i].x;中的x是怎么来的？如何变换的，包括后面的y z
*/
void bsplineCallback(traj_utils::msg::Bspline::ConstPtr msg)
{
  // parse pos traj 解析位置轨迹控制点 pos_pts是3*N的矩阵 每列是一个3D控制点
  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
  //解析knots vector knots：时间节点向量 定义B_spline的参数区间
  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;//填充控制点矩阵
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }
  //构建位置 B_spline 
  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  // parse yaw traj

  // Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  // for (int i = 0; i < msg->yaw_pts.size(); ++i) {
  //   yaw_pts(i, 0) = msg->yaw_pts[i];
  // }

  // UniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);//traj_[0] = 位置 B_spline 
  traj_.push_back(traj_[0].getDerivative()); //traj_[1] = 速度 B_spline 
  traj_.push_back(traj_[1].getDerivative());  //traj_[2] = 加速度 B_spline 
  //利用B_spline的导数性质，直接对控制点求导得到速度和加速度 
  // getDerivative()内部实现：P'_i = (P_{i+1} - P_i)/Δt (控制点差分)
  traj_duration_ = traj_[0].getTimeSum();//计算轨迹总时长 getTimeSum()：返回B_spline的总时间跨度

  receive_traj_ = true;//标记已经收到轨迹
}
/*计算无人机的偏航角yaw和偏航角速度yaw_dot,使机头指向飞行方向
* t_cur:当前时间
* &pos:当前位置
* &time_now:当前时刻
* &time_last:上次时刻
*/
std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, rclcpp::Time &time_now, rclcpp::Time &time_last)
{
  constexpr double PI = 3.1415926;
  constexpr double YAW_DOT_MAX_PER_SEC = PI; //最大偏航角速度180度/s
  // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
  std::pair<double, double> yaw_yawdot(0, 0);//储存偏航角和角速度
  double yaw = 0;
  double yawdot = 0;
  //计算前视方向 计算(t + time_forward_)时刻的位置，减去当前位置，得到飞行方向
  //evaluateDeBoorT(t)：让用户用直观的秒数操作
  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? 
  traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos : //前视点 - 当前点
  traj_[0].evaluateDeBoorT(traj_duration_) - pos; //终点 - 当前点

  // 偏航角 yaw = atan2(dir_y, dir_x)  （指向飞行方向）计算目标偏航角
  double yaw_temp = dir.norm() > 0.1 ?
   atan2(dir(1), dir(0)) : //大于0.1指 有方向->计算角度
    last_yaw_;  //方向太小：保持上次角度
    //计算最大允许偏航角变化 限制偏航角速度，避免电机过载
  double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).seconds();
  /*下述，较长if条件判断说明
  总结：三个分支先判断"最短路径往哪边转、要不要穿过 ±π 边界"，再判断"这个周期能不能转完"，能转完就直接给目标角，转不完就按最大角速度走一步并回绕到合法区间。
  注意点：
    1. 分支一、二里的 else 是死代码。以分支一为例，yaw 已被赋值为 yaw_temp，外层条件 yaw_temp - last_yaw_ > PI 又保证成立，
    所以 if (yaw - last_yaw_ > PI) 恒为真，else 里按原始差值算 yawdot 的那段永远执行不到。跨界
    周期里只要一步到位，yawdot 一律输出 ±最大速率，方向正确但数值偏大，靠后面的 50% 滤波平滑。
    2. 分支三内层的 > PI / < -PI 判断同样是死代码。能进到正常分支已经保证 |Δ| ≤ π，所以那两个条件永远不会成立，
    实际只有 yawdot = (yaw_temp - last_yaw_) / dt 会被执行。
    3. 末尾低通滤波对结果有实际影响。if (fabs(yaw - last_yaw_) <= max_yaw_change) 用的是回绕后的差值：正常小角度转动时满足条件，
    yaw 会变成新旧各 50% 的混合（比如例子 3c 实际输出是 0.5×0° + 0.5×20° = 10°）；而跨边界时 yaw 差接近 340°，远超 max，
    会自动跳过混合，避免把 340° 这种"表象跳变"当成小角度去平滑。yawdot 则无条件做 50/50 滤波。

  yawdot： 为+ yaw增大 逆时针
          为- yaw减小 顺时针
  
  */
  if (yaw_temp - last_yaw_ > PI)//处理正向跨界
  {    
    // 直接差值：170 - (-170) = +340° ❌（错误，应该 -20°）
    // 正确处理：340 - 360 = -20° ✓
    if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).seconds();
    }
  }
  else if (yaw_temp - last_yaw_ < -PI)//处理反响跨界
  {  
    // 目标偏航角比当前大超过 π，说明跨过了 -π/π 边界
    // 例如：当前 yaw = 170°，目标 yaw = -170°，差值 = -340°（实际应 +20°）
    if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).seconds();
    }
  }
  else // 正常范围，直接限制变化率
  {
    if (yaw_temp - last_yaw_ < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else if (yaw_temp - last_yaw_ > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).seconds();
    }
  }
  // 低通滤波（LPF），平滑偏航角 一阶低通滤波：减少高频抖动。
  if (fabs(yaw - last_yaw_) <= max_yaw_change)
    yaw = 0.5 * last_yaw_ + 0.5 * yaw; // nieve LPF
  yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
  last_yaw_ = yaw;
  last_yaw_dot_ = yawdot;

  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  return yaw_yawdot;// 返回偏航角和角速度
}

//指令发布回调函数：实时计算并发布位置指令
void cmdCallback()
{
  /* no publishing before receive traj_ */
  if (!receive_traj_)
    return;

  // 统一时间源
  rclcpp::Clock clock(RCL_ROS_TIME);  
  rclcpp::Time time_now = clock.now();
  double t_cur = (time_now - start_time_).seconds();//相对轨迹起始时间
  //初始化
  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);

  static rclcpp::Time time_last = clock.now();
  if (t_cur < traj_duration_ && t_cur >= 0.0)
  { // ========== 正常执行轨迹 ==========
    //B_spline插值：计算当前时刻的位置、速度、加速度
    pos = traj_[0].evaluateDeBoorT(t_cur);  //位置
    vel = traj_[1].evaluateDeBoorT(t_cur);  //速度
    acc = traj_[2].evaluateDeBoorT(t_cur);  //加速度

    /*** calculate yaw 获取当前偏航角和角速度***/
    yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);
    /*** calculate yaw ***/
    // 计算前视点（用于可视化或控制）
    double tf = min(traj_duration_, t_cur + 2.0);
    pos_f = traj_[0].evaluateDeBoorT(tf);
  }
  else if (t_cur >= traj_duration_)
  {
    // ========== 轨迹执行完毕，悬停 ==========
    /* hover when finish traj_ */
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();

    yaw_yawdot.first = last_yaw_; // 保持当前偏航角
    yaw_yawdot.second = 0;

    pos_f = pos;
  }
  else
  {// 时间异常（小于0）
    cout << "[Traj server]: invalid time." << endl;
  }
  time_last = time_now;
  
  // 填充位置指令消息
  cmd.header.stamp = time_now;
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);

  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  cmd.yaw = yaw_yawdot.first;
  cmd.yaw_dot = yaw_yawdot.second;

  last_yaw_ = cmd.yaw;

  pos_cmd_pub->publish(cmd);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("traj_server");

  auto bspline_sub = node->create_subscription<traj_utils::msg::Bspline>(//创建订阅者，订阅bspline轨迹消息
      "planning/bspline",//话题名
      10, //队列长度
      bsplineCallback);//回调函数

  pos_cmd_pub = node->create_publisher<quadrotor_msgs::msg::PositionCommand>(//创建发布者，发布位置控制命令
      "/position_cmd",
      50);

  auto cmd_timer = node->create_wall_timer(//100Hz的定时器 0.01s触发一次
      std::chrono::milliseconds(10),
      cmdCallback);

  /* control parameter  控制参数*/
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];
  //读取参数
  node->declare_parameter("traj_server/time_forward", -1.0);
  node->get_parameter("traj_server/time_forward", time_forward_);

  last_yaw_ = 0.0;
  last_yaw_dot_ = 0.0;

  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_WARN(node->get_logger(), "[Traj server]: ready.");

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}