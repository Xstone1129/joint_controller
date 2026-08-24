#ifndef CUBIC_PLANNER_HPP
#define CUBIC_PLANNER_HPP

#include <vector>
#include <cmath>

namespace erobot_controller {

// 轨迹点结构体（位置/速度/加速度）
struct TrajectoryPoint {
    double position;    // 位置 (rad)
    double velocity;    // 速度 (rad/s)
    double acceleration;// 加速度 (rad/s²)
};

// 运动约束结构体
struct MotionConstraints {
    double max_velocity;    // 最大速度 (rad/s)
    double max_acceleration; // 最大加速度 (rad/s²)
    double min_position;    // 最小位置 (rad)
    double max_position;    // 最大位置 (rad)
};

// 单轴三次多项式轨迹规划器
class SingleAxisCubicPlanner {
public:
    // 简化构造函数，只需要运动约束
    explicit SingleAxisCubicPlanner(const MotionConstraints& constraints);

    // 规划轨迹（传入初始状态和目标状态）
    void plan(
        double start_pos,       // 起始位置
        double start_vel,       // 起始速度
        double end_pos,         // 目标位置
        double end_vel,         // 目标速度
        double duration = -1.0  // 运动时间，-1表示自动计算
    );

    // 更新当前时间，返回新的轨迹点
    TrajectoryPoint update(double current_time);

    // 重新规划轨迹（添加新点）
    void replan(
        double new_target_pos,     // 新的目标位置
        double new_target_vel,     // 新的目标速度
        double new_duration = -1.0 // 新的运动时间，-1表示自动计算
    );

    // 设置运动约束
    void set_constraints(const MotionConstraints& constraints);

    // 获取当前状态
    TrajectoryPoint get_current_state() const;

    // 检查是否正在运动
    bool is_moving() const;

    // 获取运动持续时间
    double get_duration() const;
    
    double current_pos() const { return current_pos_; }
    double current_vel() const { return current_vel_; }

private:
    // 计算时间缩放以满足速度和加速度约束
    void calculate_time_scaling();

    double current_pos_;      // 当前位置
    double current_vel_;      // 当前速度
    double current_acc_;      // 当前加速度
    double target_pos_;       // 目标位置
    double target_vel_;       // 目标速度
    MotionConstraints constraints_; // 运动约束
    double duration_;         // 运动持续时间
    double t_start_;          // 开始时间
    bool is_moving_;          // 是否正在运动
    bool is_first_update_;    // 是否是第一次更新
    bool is_planned_;         // 是否已经规划
    double a0,a1,a2,a3,a4,a5,T;
};

// 多轴轨迹规划器（组合多个单轴规划器）
class MultiAxisCubicPlanner {
public:
    // 构造函数，传入轴数和约束
    MultiAxisCubicPlanner(size_t num_axes, const std::vector<MotionConstraints>& constraints_list);

    // 规划所有轴的轨迹
    double plan(
        const std::vector<double>& start_positions,  // 所有轴的起始位置
        const std::vector<double>& start_vels,       // 所有轴的起始速度
        const std::vector<double>& end_positions,    // 所有轴的目标位置
        const std::vector<double>& end_vels,         // 所有轴的目标速度
        double duration = -1.0                       // 运动时间，-1表示自动计算
    );

    // 同步所有轴的运动时间，以最长的轴为准
    void sync_all_axes_duration();

    // 更新所有轴的状态
    std::vector<TrajectoryPoint> update(double current_time);

    // 为特定轴重新规划
    void replan_axis(
        size_t axis_index,        // 轴索引
        double new_target_pos,    // 新目标位置
        double new_target_vel,    // 新目标速度
        double new_duration = -1.0 // 新持续时间
    );

    // 为所有轴重新规划
    void replan_all_axes(
        const std::vector<double>& new_target_positions,  // 所有轴的新目标位置
        const std::vector<double>& new_target_vels,       // 所有轴的新目标速度
        double new_duration = -1.0                       // 新持续时间（对所有轴相同）
    );

    // 快速设置所有轴的新目标位置和速度（保持当前约束）
    void set_new_targets(
        const std::vector<double>& new_target_positions,  // 所有轴的新目标位置
        const std::vector<double>& new_target_vels        // 所有轴的新目标速度
    );

    // 设置特定轴的约束
    void set_axis_constraints(
        size_t axis_index, 
        const MotionConstraints& constraints
    );

    // 获取特定轴的当前状态
    TrajectoryPoint get_axis_state(size_t axis_index) const;

    // 检查所有轴是否都停止运动
    bool all_axes_stopped() const;
    
    std::vector<double> start_pos;
    std::vector<double> start_vel;
    std::vector<double> end_pos;
    std::vector<double> end_vel;
private:
    std::vector<SingleAxisCubicPlanner> planners_; // 每个轴的规划器
};

} // namespace erobot_controller

#endif // CUBIC_PLANNER_HPP