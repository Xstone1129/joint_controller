#ifndef TRAJ_H
#define TRAJ_H

// Required headers
#include <ruckig/ruckig.hpp>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/StdVector>
#include "globaldefine.h"
// #include "dynamics.h"

using namespace Eigen;
using namespace std;
using namespace ruckig;

/**
 * @class ruckigtrajectory
 * @brief Trajectory planning and generation using Ruckig library
 * @brief 使用Ruckig库进行轨迹规划与生成
 */
class ruckigtrajectory
{
public:
    /**
     * @brief Default constructor
     * @brief 默认构造函数
     */
    ruckigtrajectory();

    /**
     * @brief Calculate position-based trajectory (6-axis version)
     * @brief 计算基于位置的轨迹 (6轴版本)
     * @param cp Current position / 当前位置
     * @param tp Target position / 目标位置
     * @param maxVel Maximum velocity / 最大速度
     * @param maxAccel Maximum acceleration / 最大加速度
     * @param cv Current velocity (default: zero) / 当前速度(默认: 零)
     * @param tv Target velocity (default: zero) / 目标速度(默认: 零)
     * @param ca Current acceleration (default: zero) / 当前加速度(默认: 零)
     * @param ta Target acceleration (default: zero) / 目标加速度(默认: 零)
     */
    void rmlPos(Eigen::VectorXd cp, Eigen::VectorXd tp,
                float maxVel, float maxAccel,
                Eigen::VectorXd cv = Eigen::VectorXd::Zero(6),
                Eigen::VectorXd tv = Eigen::VectorXd::Zero(6),
                Eigen::VectorXd ca = Eigen::VectorXd::Zero(6),
                Eigen::VectorXd ta = Eigen::VectorXd::Zero(6));

    /**
     * @brief Calculate position-based trajectory with default limits (6-axis)
     * @brief 使用默认限制计算基于位置的轨迹 (6轴)
     * @param cp Current position / 当前位置
     * @param tp Target position / 目标位置
     * @param cv Current velocity (default: zero) / 当前速度(默认: 零)
     * @param tv Target velocity (default: zero) / 目标速度(默认: 零)
     */
    void rmlPos(Eigen::VectorXd cp, Eigen::VectorXd tp,
                Eigen::VectorXd cv = Eigen::VectorXd::Zero(6),
                Eigen::VectorXd tv = Eigen::VectorXd::Zero(6),
                Eigen::VectorXd ca = Eigen::VectorXd::Zero(6));

    /**
     * @brief Calculate position-based trajectory (single-axis version)
     * @brief 计算基于位置的轨迹 (单轴版本)
     * @param cp Current position / 当前位置
     * @param tp Target position / 目标位置
     * @param vm Maximum velocity / 最大速度
     * @param am Maximum acceleration / 最大加速度
     */
    void rmlPos(double cp, double tp, double vm, double am);

    /**
     * @brief Execute a single trajectory step (single-axis)
     * @brief 执行单步轨迹生成 (单轴)
     * @param[out] p Resulting position / 生成的位置
     * @param[out] v Resulting velocity / 生成的速度
     * @param[out] a Resulting acceleration / 生成的加速度
     * @return True if trajectory is ongoing / 轨迹仍在进行返回true
     */
    bool rmlStep(double &p, double &v, double &a);

    /**
     * @brief Execute a single trajectory step (multi-axis)
     * @brief 执行单步轨迹生成 (多轴)
     * @param[out] jp Resulting joint positions / 生成的关节位置
     * @param[out] jv Resulting joint velocities / 生成的关节速度
     * @param[out] ja Resulting joint accelerations / 生成的关节加速度
     * @return True if trajectory is ongoing / 轨迹仍在进行返回true
     */
    bool rmlStep(Eigen::VectorXd& jp, Eigen::VectorXd& jv, Eigen::VectorXd& ja);

    /**
     * @brief Reset trajectory generator
     * @brief 重置轨迹生成器
     */
    void rmlRemove();

    /**
     * @brief Initialize trajectory generator
     * @brief 初始化轨迹生成器
     * @param dofs Degrees of freedom / 自由度数量
     * @param T Cycle time (s) / 周期时间(秒)
     * @param maxv Maximum velocity (default: 1) / 最大速度(默认: 1)
     * @param maxa Maximum acceleration (default: 1) / 最大加速度(默认: 1)
     * @param maxj Maximum jerk (default: 10) / 最大加加速度(默认: 10)
     */
    void init(size_t dofs, float T, float maxv = 1, float maxa = 1, float maxj =10);

    /**
     * @brief Get total trajectory duration
     * @brief 获取轨迹总时长
     * @return Trajectory duration in seconds / 轨迹时长(秒)
     */
    double getduration();

    /**
     * @brief Get trajectory length
     * @brief 获取轨迹长度
     * @return Trajectory path length / 轨迹路径长度
     */
    double getlength();
    
    /**
     * @brief Generate trajectory through multiple waypoints
     * @brief 生成通过多个路径点的轨迹
     * @param waypoints Vector of waypoints / 路径点向量
     * @param tcycle Cycle time (s) / 周期时间(秒)
     * @param vellim Velocity limit / 速度限制
     * @param acclim Acceleration limit / 加速度限制
     * @param jerklim Jerk limit / 加加速度限制
     * @return Vector of trajectory points / 轨迹点向量
     */
    vector<Eigen::VectorXd> movewaypoints(vector<Eigen::VectorXd> &waypoints,  
                                          double tcycle, 
                                          double vellim, 
                                          double acclim, 
                                          double jerklim);

    /**
     * @brief Generate joint-space trajectory
     * @brief 生成关节空间轨迹
     * @param startjp Start joint positions / 起始关节位置
     * @param goaljp Goal joint positions / 目标关节位置
     * @return True if trajectory generation succeeded / 轨迹生成成功返回true
     */
    bool movej(Eigen::VectorXd startjp, Eigen::VectorXd goaljp,Eigen::VectorXd startVel,Eigen::VectorXd goalVel,Eigen::VectorXd startAccel);

    /**
     * @brief Generate Cartesian-space trajectory
     * @brief 生成笛卡尔空间轨迹
     * @param leftright Arm selection (1:left, 2:right) / 手臂选择(1:左臂, 2:右臂)
     * @param cpose Current end-effector pose (xyz + rxryrz) / 当前末端执行器位姿(xyz + rxryrz)
     * @param tpose Target end-effector pose (xyz + rxryrz) / 目标末端执行器位姿(xyz + rxryrz)
     * @param vellim Velocity limit / 速度限制
     * @param acclim Acceleration limit / 加速度限制
     * @return True if trajectory generation succeeded / 轨迹生成成功返回true
     */
    bool movel(int leftright, 
               Eigen::VectorXd cpose, 
               Eigen::VectorXd tpose, 
               double vellim, 
               double acclim);
               
    bool movel_dual_arm(Eigen::VectorXd lcpose, Eigen::VectorXd ltpose,Eigen::VectorXd rcpose, Eigen::VectorXd rtpose, 
               double vellim, 
               double acclim);

    int trajlength; /**< Length of generated trajectory / 生成的轨迹长度 */

    // Generated trajectory data
    vector<Eigen::VectorXd> trajpositions; /**< Joint positions along trajectory / 轨迹关节位置 */
    vector<Eigen::VectorXd> trajvelotities; /**< Joint velocities along trajectory / 轨迹关节速度 */
    vector<Eigen::VectorXd> trajtorques; /**< Joint torques along trajectory / 轨迹关节力矩 */

    // dynamics traj_leftdyn; /**< Dynamics model for left arm / 左臂动力学模型 */
    // dynamics traj_rightdyn; /**< Dynamics model for right arm / 右臂动力学模型 */

    // Ruckig trajectory generator components
    Ruckig<DynamicDOFs> *m_otg; /**< Ruckig OTG instance / Ruckig在线轨迹生成器实例 */
    InputParameter<DynamicDOFs> *m_IP; /**< Input parameters / 输入参数 */
    OutputParameter<DynamicDOFs> *m_OP; /**< Output parameters / 输出参数 */
    Trajectory<DynamicDOFs> *m_traj; /**< Generated trajectory / 生成的轨迹 */

    // Joint limits
    float m_JointMaxVel; /**< Maximum joint velocity / 最大关节速度 */
    float m_JointMaxAccel; /**< Maximum joint acceleration / 最大关节加速度 */
    float m_JointMaxJerk; /**< Maximum joint jerk / 最大关节加加速度 */

    // End-effector limits
    float m_EndMaxVel; /**< Maximum end-effector velocity / 最大末端执行器速度 */
    float m_EndMaxAccel; /**< Maximum end-effector acceleration / 最大末端执行器加速度 */
    float m_EndMaxJerk; /**< Maximum end-effector jerk / 最大末端执行器加加速度 */

    size_t m_Dofs; /**< Number of degrees of freedom / 自由度数量 */
    float m_T; /**< Cycle time (s) / 周期时间(秒) */
};

#endif