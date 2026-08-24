#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include "erobot_controller/CubicPlanner.hpp"

namespace erobot_controller {

// 单轴三次多项式轨迹规划器实现
SingleAxisCubicPlanner::SingleAxisCubicPlanner(const MotionConstraints& constraints)
    : current_pos_(0.0), current_vel_(0.0), current_acc_(0.0),
      target_pos_(0.0), target_vel_(0.0), constraints_(constraints),
      duration_(1.0), t_start_(0.0), is_moving_(false), is_first_update_(true), is_planned_(false) {}

void SingleAxisCubicPlanner::plan(
    double start_pos,       // 起始位置
    double start_vel,       // 起始速度
    double end_pos,         // 目标位置
    double end_vel,         // 目标速度
    double duration         // 运动时间
) {
    current_pos_ = start_pos;
    current_vel_ = start_vel;
    
    // 应用位置限制
    target_pos_ = std::clamp(end_pos, constraints_.min_position, constraints_.max_position);
    if (std::abs(target_pos_ - end_pos) > 1e-6) {
        std::cout << "Warning: Target position " << end_pos << " clamped to " << target_pos_ 
                  << " (Range: [" << constraints_.min_position << ", " << constraints_.max_position << "])" << std::endl;
    }

    target_vel_ = end_vel;

    // 设置持续时间
    if (duration > 0) {
        duration_ = duration;
    } else {
        // 自动计算时间：基于距离和最大速度
        double distance = std::abs(target_pos_ - current_pos_);
        duration_ = std::max(0.5, distance / constraints_.max_velocity);
    }

    // 计算时间缩放
    calculate_time_scaling();

    // 重置标志
    is_first_update_ = true;
    is_moving_ = true;
    is_planned_ = true;
}

TrajectoryPoint SingleAxisCubicPlanner::update(double current_time) {
    if (!is_moving_ || !is_planned_) {
        return {current_pos_, 0.0, 0.0};
    }
    // 第一次调用时设置起始时间
    if (is_first_update_) {
        t_start_ = current_time;
        is_first_update_ = false;
        // 计算五次多项式系数
        T = duration_;
        a0 = current_pos_;
        a1 = current_vel_;
        a2 = 0.0; // 假设起始加速度为0
        
        double delta_pos = target_pos_ - current_pos_;
        a3 = (10 * delta_pos - (6 * current_vel_ + 4 * target_vel_) * T) / (T * T * T);
        a4 = (-15 * delta_pos + (8 * current_vel_ + 7 * target_vel_) * T) / (T * T * T * T);
        a5 = (6 * delta_pos - (3 * current_vel_ + 3 * target_vel_) * T) / (T * T * T * T * T);
    }

    double t = current_time - t_start_;

    // 计算当前时刻状态
    double tau = std::clamp(t, 0.0, T);
    double tau2 = tau * tau;
    double tau3 = tau2 * tau;
    double tau4 = tau3 * tau;
    double tau5 = tau4 * tau;

    double pos = a0 + a1*tau + a2*tau2 + a3*tau3 + a4*tau4 + a5*tau5;
    double vel = a1 + 2*a2*tau + 3*a3*tau2 + 4*a4*tau3 + 5*a5*tau4;
    double acc = 2*a2 + 6*a3*tau + 12*a4*tau2 + 20*a5*tau3;


    // 更新当前状态
    current_pos_ = pos;
    current_vel_ = vel;
    current_acc_ = acc;

    // 检查是否到达目标
    if (tau >= T) {
        is_moving_ = false;
        current_pos_ = target_pos_;
        current_vel_ = target_vel_;
        current_acc_ = 0.0;
    }

    return {current_pos_, current_vel_, current_acc_};
}

void SingleAxisCubicPlanner::replan(
    double new_target_pos,     // 新的目标位置
    double new_target_vel,     // 新的目标速度
    double new_duration        // 新的运动时间
) {
    // 以当前状态作为新的起点
    double start_pos = current_pos_;
    double start_vel = current_vel_;

    // 设置新的目标并应用位置限制
    target_pos_ = std::clamp(new_target_pos, constraints_.min_position, constraints_.max_position);
    if (std::abs(target_pos_ - new_target_pos) > 1e-6) {
        std::cout << "Warning: Target position " << new_target_pos << " clamped to " << target_pos_ 
                  << " (Range: [" << constraints_.min_position << ", " << constraints_.max_position << "])" << std::endl;
    }
    
    target_vel_ = new_target_vel;

    // 设置新的持续时间
    if (new_duration > 0) {
        duration_ = new_duration;
    } else {
        // 自动计算时间：基于距离和最大速度
        double distance = std::abs(target_pos_ - start_pos);
        duration_ = std::max(0.5, distance / constraints_.max_velocity);
    }

    // 重新计算时间缩放
    calculate_time_scaling();

    // 重置起始时间标志
    is_first_update_ = true;
    is_moving_ = true;
}

void SingleAxisCubicPlanner::set_constraints(const MotionConstraints& constraints) {
    constraints_ = constraints;
    if (is_planned_)
        calculate_time_scaling();
}

TrajectoryPoint SingleAxisCubicPlanner::get_current_state() const {
    return {current_pos_, current_vel_, current_acc_};
}

bool SingleAxisCubicPlanner::is_moving() const {
    return is_moving_;
}

double SingleAxisCubicPlanner::get_duration() const {
    return duration_;
}

// 计算时间缩放以满足速度和加速度约束
void SingleAxisCubicPlanner::calculate_time_scaling() {
    double T = duration_;
    double delta_pos = target_pos_ - current_pos_;

    // 计算初始系数 (5次多项式)
    double a0 = current_pos_;
    double a1 = current_vel_;
    double a2 = 0.0; // 假设起始加速度为0
    
    double a3 = (10 * delta_pos - (6 * current_vel_ + 4 * target_vel_) * T) / (T * T * T);
    double a4 = (-15 * delta_pos + (8 * current_vel_ + 7 * target_vel_) * T) / (T * T * T * T);
    double a5 = (6 * delta_pos - (3 * current_vel_ + 3 * target_vel_) * T) / (T * T * T * T * T);

    // 采样寻找最大速度和加速度
    double max_vel = 0.0;
    double max_accel = 0.0;
    int samples = 100;
    
    for (int i = 0; i <= samples; ++i) {
        double t = T * i / samples;
        double t2 = t * t;
        double t3 = t2 * t;
        double t4 = t3 * t;
        
        double vel = std::abs(a1 + 2*a2*t + 3*a3*t2 + 4*a4*t3 + 5*a5*t4);
        double acc = std::abs(2*a2 + 6*a3*t + 12*a4*t2 + 20*a5*t3);
        
        if (vel > max_vel) max_vel = vel;
        if (acc > max_accel) max_accel = acc;
    }

    // 3. 计算需要的时间缩放因子
    double scale_factor_accel = 1.0;
    if (max_accel > constraints_.max_acceleration) {
        scale_factor_accel = std::sqrt(max_accel / constraints_.max_acceleration);
    }

    double scale_factor_vel = 1.0;
    if (max_vel > constraints_.max_velocity) {
        // 速度与时间成反比，所以缩放因子直接等于速度比
        scale_factor_vel = max_vel / constraints_.max_velocity;
    }

    // 取较大的缩放因子
    double scale_factor = std::max(scale_factor_accel, scale_factor_vel);

    // 应用时间缩放
    if (scale_factor > 1.0) {
        duration_ *= scale_factor;
        // 增加一点余量以确保安全
        duration_ *= 1.05;
    }
}

// 多轴轨迹规划器实现
MultiAxisCubicPlanner::MultiAxisCubicPlanner(size_t num_axes, const std::vector<MotionConstraints>& constraints_list)
{
    if (num_axes != constraints_list.size()) {
        throw std::invalid_argument("轴数与约束数量不匹配");
    }

    // 为每个轴创建规划器
    for (size_t i = 0; i < num_axes; i++) {
        planners_.emplace_back(constraints_list[i]);
    }
    start_pos.reserve(num_axes);
    start_vel.reserve(num_axes);
    end_pos.reserve(num_axes);
    end_vel.reserve(num_axes);
}

double MultiAxisCubicPlanner::plan(
    const std::vector<double>& start_positions,  // 所有轴的起始位置
    const std::vector<double>& start_vels,       // 所有轴的起始速度
    const std::vector<double>& end_positions,    // 所有轴的目标位置
    const std::vector<double>& end_vels,         // 所有轴的目标速度
    double duration                              // 运动时间
) {
    if (start_positions.size() != planners_.size() ||
        start_vels.size() != planners_.size() ||
        end_positions.size() != planners_.size() ||
        end_vels.size() != planners_.size()) {
        std::cout << "输入参数数量与轴数不匹配" << "start_positions.size():" << start_positions.size() << "planners_.size():" << planners_.size()   << "start_vels.size():" << start_vels.size() << "end_positions.size():" << end_positions.size() << "end_vels.size():" << end_vels.size() << std::endl;
        throw std::invalid_argument("输入参数数量与轴数不匹配");
        }

    // 首先计算每个轴所需的时间
    double max_duration = duration; // 初始化为用户提供的时间
    for (size_t i = 0; i < planners_.size(); i++) {
        // 为每个轴规划轨迹以获取所需时间
        planners_[i].plan(
            start_positions[i],
            start_vels[i],
            end_positions[i],
            end_vels[i],
            duration
        );
        // 更新最大时间
        max_duration = std::max(max_duration, planners_[i].get_duration());
    }

    // 如果最大时间大于原始时间，则使用最大时间重新规划所有轴
    if (max_duration > duration) {
        for (size_t i = 0; i < planners_.size(); i++) {
            planners_[i].plan(
                start_positions[i],
                start_vels[i],
                end_positions[i],
                end_vels[i],
                max_duration
            );
        }
    }

    // std::cout << "所有轴同步后的最大运动时间: " << max_duration << "s" << std::endl;
    return max_duration;
}

void MultiAxisCubicPlanner::sync_all_axes_duration() {
    // 找出最长的运动时间
    double max_duration = 0.0;
    for (const auto& planner : planners_) {
        max_duration = std::max(max_duration, planner.get_duration());
        std::cout << "Planned trajectory with max_duration: " << planner.get_duration() << "s" << std::endl;
    }

    // 为每个轴重新规划，设置新的持续时间
    for (size_t i = 0; i < planners_.size(); i++) {
        // 获取当前位置和速度
        TrajectoryPoint current_state = planners_[i].get_current_state();
        // 重新规划，保持目标位置和速度不变，但更新持续时间
        planners_[i].replan(
            planners_[i].get_current_state().position,  // 保持当前目标位置
            planners_[i].get_current_state().velocity,  // 保持当前目标速度
            max_duration                               // 新的持续时间
        );
    
    }
}

std::vector<TrajectoryPoint> MultiAxisCubicPlanner::update(double current_time) {
    std::vector<TrajectoryPoint> results;
    for (auto& planner : planners_) {
        results.push_back(planner.update(current_time));
    }
    return results;
}

void MultiAxisCubicPlanner::replan_axis(
    size_t axis_index,        // 轴索引
    double new_target_pos,    // 新目标位置
    double new_target_vel,    // 新目标速度
    double new_duration       // 新持续时间
) {
    if (axis_index >= planners_.size()) {
        throw std::invalid_argument("轴索引超出范围");
    }
    planners_[axis_index].replan(new_target_pos, new_target_vel, new_duration);

    // 重新同步所有轴的运动时间
    sync_all_axes_duration();
}

void MultiAxisCubicPlanner::replan_all_axes(
    const std::vector<double>& new_target_positions,  // 所有轴的新目标位置
    const std::vector<double>& new_target_vels,       // 所有轴的新目标速度
    double new_duration                              // 新持续时间
) {
    if (new_target_positions.size() != planners_.size() ||
        new_target_vels.size() != planners_.size()) {
        throw std::invalid_argument("目标位置或速度数量与轴数不匹配");
    }

    // 为每个轴重新规划
    for (size_t i = 0; i < planners_.size(); i++) {
        planners_[i].replan(new_target_positions[i], new_target_vels[i], new_duration);
    }

    // 重新同步所有轴的运动时间
    sync_all_axes_duration();
}

void MultiAxisCubicPlanner::set_new_targets(
    const std::vector<double>& new_target_positions,  // 所有轴的新目标位置
    const std::vector<double>& new_target_vels        // 所有轴的新目标速度
) {
    if (new_target_positions.size() != planners_.size() ||
        new_target_vels.size() != planners_.size()) {
        throw std::invalid_argument("目标位置或速度数量与轴数不匹配");
    }

    // 为每个轴设置新目标
    for (size_t i = 0; i < planners_.size(); i++) {
        // 获取当前速度作为起始速度
        double current_vel = planners_[i].get_current_state().velocity;
        planners_[i].replan(new_target_positions[i], new_target_vels[i], -1.0);
    }

    // 重新同步所有轴的运动时间
    sync_all_axes_duration();
}

void MultiAxisCubicPlanner::set_axis_constraints(
    size_t axis_index, 
    const MotionConstraints& constraints
) {
    if (axis_index >= planners_.size()) {
        throw std::invalid_argument("轴索引超出范围");
    }
    planners_[axis_index].set_constraints(constraints);
}

TrajectoryPoint MultiAxisCubicPlanner::get_axis_state(size_t axis_index) const {
    if (axis_index >= planners_.size()) {
        throw std::invalid_argument("轴索引超出范围");
    }
    return planners_[axis_index].get_current_state();
}

bool MultiAxisCubicPlanner::all_axes_stopped() const {
    for (const auto& planner : planners_) {
        if (planner.is_moving()) {
            return false;
        }
    }
    return true;
}

} // namespace erobot_controller

