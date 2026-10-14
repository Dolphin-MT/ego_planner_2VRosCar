#pragma once

#include <Eigen/Dense>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace fast_planner
{
    /*
    * 这是一个最小化版本的 ObjPredictor。
    *
    * 原来的 swarm 版本支持动态障碍物预测。
    * 我们现在只做单车 2D 导航，不使用动态障碍物预测。
    *
    * 但是 bspline_optimizer.cpp 里仍然会调用：
    *
    *     getObjNums()
    *     evaluateConstVel()
    *
    * 所以这里需要保留这两个接口，让代码能够编译通过。
    */
class ObjPredictor
{

private:
    int obj_num_{0};
public:
    using Ptr = std::shared_ptr<ObjPredictor>;
    ObjPredictor() = default;
    ~ObjPredictor() = default;

    int getObjNums() { return obj_num_; }

    //没有启用动态避障 返回一个很远的点就好了
     Eigen::Vector3d evaluateConstVel(int obj_id, double time) const
    {
        return Eigen::Vector3d(1.0e6, 1.0e6, 0.0);
    }
};

}  // namespace fast_planner