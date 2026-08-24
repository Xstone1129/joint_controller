#include "erobot_controller/traj.h"

// 构造函数：初始化轨迹生成器
ruckigtrajectory::ruckigtrajectory()
{
    m_otg = NULL;          // Ruckig轨迹生成器对象
    m_IP = NULL;           // 输入参数对象
    m_OP = NULL;           // 输出参数对象
    m_traj = NULL;         // 轨迹对象

    // 默认关节限制参数
    m_JointMaxVel = 3.1415926f;    // 关节最大速度
    m_JointMaxAccel = 10.0f;  // 关节最大加速度
    m_JointMaxJerk = 10.0f;   // 关节最大加加速度（急动度）

    // 末端执行器限制参数（初始化为0）
    m_EndMaxJerk = 0;
    m_EndMaxVel = 0;
    m_EndMaxAccel = 0;

    // 初始化左右臂动力学模型
    // traj_leftdyn.init(1);   // 左臂动力学模型
    // traj_rightdyn.init(2);  // 右臂动力学模型
}

// 初始化轨迹生成器参数
void ruckigtrajectory::init(size_t dofs, float T, float maxv, float maxa, float maxj)
{
    m_Dofs = dofs;          // 自由度数量
    m_T = T;                // 控制周期（秒）
    m_JointMaxVel = maxv;   // 关节最大速度
    m_JointMaxAccel = maxa; // 关节最大加速度
    m_JointMaxJerk = maxj;  // 关节最大急动度
}

// 执行一步轨迹生成（多自由度）
bool ruckigtrajectory::rmlStep(Eigen::VectorXd& jp, Eigen::VectorXd& jv, Eigen::VectorXd& ja)
{
    // 更新轨迹生成器
    Result res = m_otg->update(*m_IP, *m_OP);

    if(res == Result::Working) {
        // 获取当前步的位置、速度、加速度
        for(int i=0; i<m_Dofs; ++i) {
            jp[i] = m_OP->new_position.at(i);
            jv[i] = m_OP->new_velocity.at(i);
            ja[i] = m_OP->new_acceleration.at(i);
        }
        // 将输出传递给下一步的输入
        m_OP->pass_to_input(*m_IP);
        return false; // 轨迹未完成
    } else if (res == Result::Finished) {
        return true; // 轨迹完成
    } else {
        // 错误处理
        IERROR("An error occurred (%d).\n", res);
        assert(false && "ruckig rml step error");
        return false;
    }
}

// 设置位置轨迹生成参数（自定义限制）
void ruckigtrajectory::rmlPos(Eigen::VectorXd cp, Eigen::VectorXd tp,
                            float maxVel, float maxAccel,
                            Eigen::VectorXd cv, Eigen::VectorXd tv,
                            Eigen::VectorXd ca, Eigen::VectorXd ta)
{
    // 清理旧对象
    delete m_otg;
    delete m_IP;
    delete m_OP;

    // 创建新对象
    m_otg = new Ruckig<DynamicDOFs>{m_Dofs, m_T}; // 轨迹生成器
    m_IP = new InputParameter<DynamicDOFs>{m_Dofs}; // 输入参数
    m_OP = new OutputParameter<DynamicDOFs>{m_Dofs}; // 输出参数

    // 设置每个自由度的参数
    for (int i = 0; i < m_Dofs; i++) {
        m_IP->current_position.at(i) = cp[i];      // 当前位置
        m_IP->current_velocity.at(i) = cv[i];      // 当前速度
        m_IP->current_acceleration.at(i) = ca[i];  // 当前加速度
        
        // 设置限制
        m_IP->max_velocity.at(i) = maxVel;         // 最大速度
        m_IP->max_acceleration.at(i) = maxAccel;   // 最大加速度
        m_IP->max_jerk.at(i) = m_JointMaxJerk;     // 最大急动度
        
        // 目标状态
        m_IP->target_position.at(i) = tp[i];       // 目标位置
        m_IP->target_velocity.at(i) = tv[i];       // 目标速度
        m_IP->target_acceleration.at(i) = ta[i];   // 目标加速度
    }
    
    // 计算轨迹
    m_traj = new Trajectory<DynamicDOFs>{m_Dofs};
    m_otg->calculate(*m_IP, *m_traj);
    
    // 打印轨迹持续时间
    double ttime = getduration();
    cout << "轨迹持续时间: " << ttime << endl;
}

// 设置位置轨迹生成参数（使用类默认限制）
void ruckigtrajectory::rmlPos(Eigen::VectorXd cp, Eigen::VectorXd tp,
            Eigen::VectorXd cv, Eigen::VectorXd tv,
            Eigen::VectorXd ca)
{
    // 清理旧对象
    delete m_otg;
    delete m_IP;
    delete m_OP;
    
    // 创建新对象
    m_otg = new Ruckig<DynamicDOFs>{m_Dofs, m_T};
    m_IP = new InputParameter<DynamicDOFs>{m_Dofs};
    m_OP = new OutputParameter<DynamicDOFs>{m_Dofs};

    // 设置参数
    for (int i = 0; i < m_Dofs; i++) {
        m_IP->current_position.at(i) = cp[i];     // 当前位置
        m_IP->current_velocity.at(i) = cv[i];     // 当前速度
        m_IP->current_acceleration.at(i) = ca[i];     // 当前加速度（设为0）
        
        // 使用类默认限制
        m_IP->max_velocity.at(i) = m_JointMaxVel;     // 最大速度
        m_IP->max_acceleration.at(i) = m_JointMaxAccel; // 最大加速度
        m_IP->max_jerk.at(i) = m_JointMaxJerk;        // 最大急动度
        
        // 目标状态
        m_IP->target_position.at(i) = tp[i];      // 目标位置
        m_IP->target_velocity.at(i) = tv[i];      // 目标速度
        m_IP->target_acceleration.at(i) = 0;      // 目标加速度（设为0）
    }
    
    // 计算轨迹
    m_traj = new Trajectory<DynamicDOFs>{m_Dofs};
    m_otg->calculate(*m_IP, *m_traj);
}

// 设置单自由度位置轨迹参数
void ruckigtrajectory::rmlPos(double cp, double tp, double vm, double am)
{
    // 创建单自由度轨迹生成器
    m_otg = new Ruckig<DynamicDOFs>{1, m_T};
    m_IP = new InputParameter<DynamicDOFs>{1};
    m_OP = new OutputParameter<DynamicDOFs>{1};

    // 设置参数
    m_IP->current_position.at(0) = cp;  // 当前位置
    m_IP->current_velocity.at(0) = 0;   // 当前速度
    m_IP->current_acceleration.at(0) = 0; // 当前加速度
    
    m_IP->max_velocity.at(0) = vm;      // 最大速度
    m_IP->max_acceleration.at(0) = am;  // 最大加速度
    m_IP->max_jerk.at(0) = m_JointMaxJerk; // 最大急动度
    
    m_IP->target_position.at(0) = tp;   // 目标位置
    m_IP->target_velocity.at(0) = 0;    // 目标速度
    m_IP->target_acceleration.at(0) = 0;// 目标加速度
    
    // 计算轨迹
    m_traj = new Trajectory<DynamicDOFs>{1};
    m_otg->calculate(*m_IP, *m_traj);
}

// 执行一步轨迹生成（单自由度）
bool ruckigtrajectory::rmlStep(double &p, double &v, double &a)
{
    // 更新轨迹生成器
    Result res = m_otg->update(*m_IP, *m_OP);

    if(res == Result::Working) {
        // 获取当前位置、速度、加速度
        p = m_OP->new_position.at(0);
        v = m_OP->new_velocity.at(0);
        a = m_OP->new_acceleration.at(0);
        
        // 将输出传递给下一步的输入
        m_OP->pass_to_input(*m_IP);
        return false; // 轨迹未完成
    } else if (res == Result::Finished) {
        return true; // 轨迹完成
    } else {
        // 错误处理
        IERROR("An error occurred (%d).\n", res);
        assert(false && "ruckig rml step error");
        return false;
    }
}

// 清理Ruckig对象
void ruckigtrajectory::rmlRemove()
{
    delete m_otg;
    delete m_IP;
    delete m_OP;
}

// 关节空间运动规划
bool ruckigtrajectory::movej(Eigen::VectorXd startjp, Eigen::VectorXd goaljp,Eigen::VectorXd startVel,Eigen::VectorXd goalVel,Eigen::VectorXd startAccel)
{
    // 初始化轨迹生成器
    // init(14,0.002f,3.1415926f,10,10);
    
    // 设置轨迹参数（使用默认速度/加速度限制）  
    rmlPos(startjp, goaljp,startVel,goalVel,startAccel);
    
    // 准备存储轨迹点
    Eigen::VectorXd tp = Eigen::VectorXd::Zero(m_Dofs);
    Eigen::VectorXd tv = Eigen::VectorXd::Zero(m_Dofs);
    Eigen::VectorXd ta = Eigen::VectorXd::Zero(m_Dofs);
    trajpositions.clear();
    trajvelotities.clear();
    trajtorques.clear();
    
    // 生成轨迹
    while(!rmlStep(tp, tv, ta)) {
        trajpositions.push_back(tp);   // 存储位置
        trajvelotities.push_back(tv);  // 存储速度
        trajtorques.push_back(ta);     // 存储加速度
    }
    
    // 计算并存储轨迹信息
    double trajtime = getduration();
    trajlength = (int)(trajtime * 1000);
    // cout << "轨迹保存完成，持续时间: " << trajtime << "秒 (" << trajlength << "毫秒)" << endl;
    
    return true;
}

bool ruckigtrajectory::movel_dual_arm(Eigen::VectorXd lcpose, Eigen::VectorXd ltpose,
                                      Eigen::VectorXd rcpose, Eigen::VectorXd rtpose,
                                      double vellim, double acclim)
{
    // // 初始化归一化轨迹生成器（1维）
    // init(1, CYCLETIME, vellim, acclim, jmax);
    // rmlPos(0, 1, vellim, acclim); // 从0到1的归一化轨迹

    // // 左臂起止位置和姿态
    // rbdlmath::Vector3d lcpos = lcpose.segment(0,3);
    // rbdlmath::Vector3d ltpos = ltpose.segment(0,3);
    // rbdlmath::Vector3d lcrpy = lcpose.segment(3,3);
    // rbdlmath::Vector3d ltrpy = ltpose.segment(3,3);

    // // 右臂起止位置和姿态
    // rbdlmath::Vector3d rcpos = rcpose.segment(0,3);
    // rbdlmath::Vector3d rtpos = rtpose.segment(0,3);
    // rbdlmath::Vector3d rcrpy = rcpose.segment(3,3);
    // rbdlmath::Vector3d rtrpy = rtpose.segment(3,3);

    // // 计算位置差
    // rbdlmath::Vector3d ldeltapos = ltpos - lcpos;
    // rbdlmath::Vector3d rdeltapos = rtpos - rcpos;

    // // 姿态插值参数
    // rbdlmath::Matrix3d lmat1 = traj_leftdyn.rpyToMatrix(lcrpy);
    // rbdlmath::Matrix3d lmat2 = traj_leftdyn.rpyToMatrix(ltrpy);
    // rbdlmath::Matrix3d lmat01 = lmat1.transpose();
    // rbdlmath::Matrix3d ldeltamat = lmat01 * lmat2;
    // double ltheta = acos((ldeltamat(0, 0) + ldeltamat(1, 1) + ldeltamat(2, 2) - 1.0) / 2);
    // rbdlmath::Vector3d lax;
    // if(ltheta < 0.001) {
    //     lax << 1, 1, 1;
    // } else {
    //     double x = (ldeltamat(2, 1) - ldeltamat(1, 2)) / (2 * sin(ltheta));
    //     double y = (ldeltamat(0, 2) - ldeltamat(2, 0)) / (2 * sin(ltheta));
    //     double z = (ldeltamat(1, 0) - ldeltamat(0, 1)) / (2 * sin(ltheta));
    //     double norm = sqrt(x*x + y*y + z*z);
    //     lax << x/norm, y/norm, z/norm;
    // }

    // rbdlmath::Matrix3d rmat1 = traj_rightdyn.rpyToMatrix(rcrpy);
    // rbdlmath::Matrix3d rmat2 = traj_rightdyn.rpyToMatrix(rtrpy);
    // rbdlmath::Matrix3d rmat01 = rmat1.transpose();
    // rbdlmath::Matrix3d rdeltamat = rmat01 * rmat2;
    // double rtheta = acos((rdeltamat(0, 0) + rdeltamat(1, 1) + rdeltamat(2, 2) - 1.0) / 2);
    // rbdlmath::Vector3d rax;
    // if(rtheta < 0.001) {
    //     rax << 1, 1, 1;
    // } else {
    //     double x = (rdeltamat(2, 1) - rdeltamat(1, 2)) / (2 * sin(rtheta));
    //     double y = (rdeltamat(0, 2) - rdeltamat(2, 0)) / (2 * sin(rtheta));
    //     double z = (rdeltamat(1, 0) - rdeltamat(0, 1)) / (2 * sin(rtheta));
    //     double norm = sqrt(x*x + y*y + z*z);
    //     rax << x/norm, y/norm, z/norm;
    // }

    // // 轨迹点存储
    // trajpositions.clear();

    // double tp, tv, ta;
    // while(!rmlStep(tp, tv, ta)) {
    //     // 左臂插值
    //     rbdlmath::Vector3d ltrajpos = lcpos + tp * ldeltapos;
    //     double ltrajtheta = tp * ltheta;
    //     rbdlmath::Matrix3d lmat_c;
    //     lmat_c(0, 0) = pow(lax[0], 2)*(1-cos(ltrajtheta)) + cos(ltrajtheta);
    //     lmat_c(0, 1) = lax[0]*lax[1]*(1-cos(ltrajtheta)) - lax[2]*sin(ltrajtheta);
    //     lmat_c(0, 2) = lax[0]*lax[2]*(1-cos(ltrajtheta)) + lax[1]*sin(ltrajtheta);
    //     lmat_c(1, 0) = lax[0]*lax[1]*(1-cos(ltrajtheta)) + lax[2]*sin(ltrajtheta);
    //     lmat_c(1, 1) = pow(lax[1], 2)*(1-cos(ltrajtheta)) + cos(ltrajtheta);
    //     lmat_c(1, 2) = lax[1]*lax[2]*(1-cos(ltrajtheta)) - lax[0]*sin(ltrajtheta);
    //     lmat_c(2, 0) = lax[0]*lax[2]*(1-cos(ltrajtheta)) - lax[1]*sin(ltrajtheta);
    //     lmat_c(2, 1) = lax[1]*lax[2]*(1-cos(ltrajtheta)) + lax[0]*sin(ltrajtheta);
    //     lmat_c(2, 2) = pow(lax[2], 2)*(1-cos(ltrajtheta)) + cos(ltrajtheta);
    //     rbdlmath::Matrix3d ltrajmat = (lmat1 * lmat_c).transpose();

    //     // 右臂插值
    //     rbdlmath::Vector3d rtrajpos = rcpos + tp * rdeltapos;
    //     double rtrajtheta = tp * rtheta;
    //     rbdlmath::Matrix3d rmat_c;
    //     rmat_c(0, 0) = pow(rax[0], 2)*(1-cos(rtrajtheta)) + cos(rtrajtheta);
    //     rmat_c(0, 1) = rax[0]*rax[1]*(1-cos(rtrajtheta)) - rax[2]*sin(rtrajtheta);
    //     rmat_c(0, 2) = rax[0]*rax[2]*(1-cos(rtrajtheta)) + rax[1]*sin(rtrajtheta);
    //     rmat_c(1, 0) = rax[0]*rax[1]*(1-cos(rtrajtheta)) + rax[2]*sin(rtrajtheta);
    //     rmat_c(1, 1) = pow(rax[1], 2)*(1-cos(rtrajtheta)) + cos(rtrajtheta);
    //     rmat_c(1, 2) = rax[1]*rax[2]*(1-cos(rtrajtheta)) - rax[0]*sin(rtrajtheta);
    //     rmat_c(2, 0) = rax[0]*rax[2]*(1-cos(rtrajtheta)) - rax[1]*sin(rtrajtheta);
    //     rmat_c(2, 1) = rax[1]*rax[2]*(1-cos(rtrajtheta)) + rax[0]*sin(rtrajtheta);
    //     rmat_c(2, 2) = pow(rax[2], 2)*(1-cos(rtrajtheta)) + cos(rtrajtheta);
    //     rbdlmath::Matrix3d rtrajmat = (rmat1 * rmat_c).transpose();

    //     // 逆运动学求解
    //     VectorXd lqres = VectorXd::Zero(DOFS);
    //     VectorXd rqres = VectorXd::Zero(DOFS);
    //     traj_leftdyn.inverseKinematics(ltrajpos, ltrajmat, lqres,3);
    //     traj_rightdyn.inverseKinematics(rtrajpos, rtrajmat, rqres,3);

    //     // 合并双臂关节角度
    //     VectorXd dualqres(DOFS*2);
    //     dualqres << lqres, rqres;
    //     trajpositions.push_back(dualqres);
    // }

    // // 保存轨迹
    // IPRINT("双臂直线运动轨迹规划完成，耗时: %f秒", getduration());
    // traj_outx("../data/movel_dual_arm_traj.txt", trajpositions, DOFS*2);


    // // 保存轨迹，文件名带时间戳
    // // time_t now = time(nullptr);
    // // struct tm* tm_info = localtime(&now);
    // // char buffer[128];
    // // strftime(buffer, sizeof(buffer), "../data/movel_dual_arm_traj_%Y%m%d_%H%M%S.txt", tm_info);
    // // traj_outx(buffer, trajpositions, DOFS*2);

    // // IPRINT("双臂直线运动轨迹规划完成，耗时: %f秒，文件: %s", getduration(), buffer);

    return true;
}

// 笛卡尔空间直线运动规划
bool ruckigtrajectory::movel(int leftright, Eigen::VectorXd cpose, Eigen::VectorXd tpose, 
                             double vellim, double acclim)
{
    // // 初始化归一化的轨迹生成器（1维）
    // init(1, CYCLETIME, vellim, acclim, jmax);
    // rmlPos(0, 1, vellim, acclim); // 从0到1的归一化轨迹
    
    // // 提取起始和结束位姿
    // rbdlmath::Vector3d cpos, tpos, crpy, trpy;
    // cpos = cpose.segment(0,3);   // 起始位置
    // tpos = tpose.segment(0,3);   // 目标位置
    // crpy = cpose.segment(3,3);   // 起始姿态（欧拉角）
    // trpy = tpose.segment(3,3);   // 目标姿态（欧拉角）
    
    // // 计算位置差
    // rbdlmath::Vector3d deltapos = tpos - cpos;
    
    // // 计算姿态差
    // rbdlmath::Matrix3d mat1 = traj_leftdyn.rpyToMatrix(crpy);
    // rbdlmath::Matrix3d mat2 = traj_leftdyn.rpyToMatrix(trpy);
    // rbdlmath::Matrix3d mat01 = mat1.transpose();
    // rbdlmath::Matrix3d deltamat = mat01 * mat2;
    
    // // 计算旋转角度和旋转轴
    // double theta = acos((deltamat(0, 0) + deltamat(1, 1) + deltamat(2, 2) - 1.0) / 2);
    // rbdlmath::Vector3d axs;
    
    // // 处理特殊情况（无需旋转）
    // if(theta < 0.001) {
    //     axs << 1, 1, 1;
    //     cout << "无需转动" << endl;
    // } else {
    //     // 计算旋转轴
    //     double axs_x = (deltamat(2, 1) - deltamat(1, 2)) / (2 * sin(theta));
    //     double axs_y = (deltamat(0, 2) - deltamat(2, 0)) / (2 * sin(theta));
    //     double axs_z = (deltamat(1, 0) - deltamat(0, 1)) / (2 * sin(theta));
    //     double axs_norm = sqrt(axs_x*axs_x + axs_y*axs_y + axs_z*axs_z);
    //     axs << axs_x/axs_norm, axs_y/axs_norm, axs_z/axs_norm;
    // }
    
    // // 生成笛卡尔空间轨迹
    // double tp, tv, ta;
    // VectorXd qres = VectorXd::Zero(DOFS); 
    // trajpositions.clear();
    
    // while(!rmlStep(tp, tv, ta)) {
    //     // 插值计算当前位置
    //     rbdlmath::Vector3d trajpos = cpos + tp * deltapos;
        
    //     // 插值计算当前姿态（使用旋转矩阵）
    //     double trajtheta = tp * theta;
    //     rbdlmath::Matrix3d mat_c;
    //     mat_c(0, 0) = pow(axs[0], 2)*(1-cos(trajtheta)) + cos(trajtheta);
    //     mat_c(0, 1) = axs[0]*axs[1]*(1-cos(trajtheta)) - axs[2]*sin(trajtheta);
    //     mat_c(0, 2) = axs[0]*axs[2]*(1-cos(trajtheta)) + axs[1]*sin(trajtheta);
        
    //     mat_c(1, 0) = axs[0]*axs[1]*(1-cos(trajtheta)) + axs[2]*sin(trajtheta);
    //     mat_c(1, 1) = pow(axs[1], 2)*(1-cos(trajtheta)) + cos(trajtheta);
    //     mat_c(1, 2) = axs[1]*axs[2]*(1-cos(trajtheta)) - axs[0]*sin(trajtheta);
        
    //     mat_c(2, 0) = axs[0]*axs[2]*(1-cos(trajtheta)) - axs[1]*sin(trajtheta);
    //     mat_c(2, 1) = axs[1]*axs[2]*(1-cos(trajtheta)) + axs[0]*sin(trajtheta);
    //     mat_c(2, 2) = pow(axs[2], 2)*(1-cos(trajtheta)) + cos(trajtheta);
        
    //     // 计算目标姿态矩阵
    //     rbdlmath::Matrix3d trajmat = (mat1 * mat_c).transpose();
        
    //     // 逆运动学求解关节角度
    //     if(leftright == 1) { // 左臂
    //         traj_leftdyn.inverseKinematics(trajpos, trajmat, qres);
    //         traj_leftdyn.setJointPositions(qres);
    //     } else if(leftright == 2) { // 右臂
    //         traj_rightdyn.inverseKinematics(trajpos, trajmat, qres);
    //         traj_rightdyn.setJointPositions(qres);
    //     }
        
    //     // 存储关节位置
    //     trajpositions.push_back(qres);
    // }
    
    // // 打印信息并保存轨迹
    // IPRINT("关节运动规划完成，耗时: %f秒", getduration());
    // // traj_outx("../data/moveltraj.txt", trajpositions, DOFS);
    
    return true;
}

// 获取轨迹长度（未实现）
double ruckigtrajectory::getlength()
{
    double len = 0;
    if(m_traj) m_traj->get_profiles();
    return len;
}

// 获取轨迹持续时间
double ruckigtrajectory::getduration()
{
    return m_traj ? m_traj->get_duration() : 0.0;
}

// 多路点轨迹规划
vector<Eigen::VectorXd> ruckigtrajectory::movewaypoints(
    vector<Eigen::VectorXd> & waypoints, 
    double tcycle, 
    double vellim, 
    double acclim, 
    double jerklim)
{
    const double control_cycle = tcycle; // 控制周期
    const size_t wDOFs = DOFS * 2;       // 总自由度（双臂）
    const size_t numall = waypoints.size(); // 路径点数量
    
    // 创建Ruckig对象和参数
    Ruckig<DynamicDOFs> otg(wDOFs, control_cycle, numall);
    InputParameter<DynamicDOFs> input(wDOFs);
    OutputParameter<DynamicDOFs> output(wDOFs, numall);
    cout << "轨迹初始化..." << endl;

    // 设置起始状态
    input.current_position = { 
        waypoints[0][0], waypoints[0][1], waypoints[0][2], waypoints[0][3],
        waypoints[0][4], waypoints[0][5], waypoints[0][6], waypoints[0][7],
        waypoints[0][8], waypoints[0][9], waypoints[0][10], waypoints[0][11],
        waypoints[0][12], waypoints[0][13] 
    };
    input.current_velocity = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    input.current_acceleration = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    
    // 设置目标状态
    input.target_position = {
        waypoints[numall-1][0], waypoints[numall-1][1], waypoints[numall-1][2], waypoints[numall-1][3],
        waypoints[numall-1][4], waypoints[numall-1][5], waypoints[numall-1][6], waypoints[numall-1][7],
        waypoints[numall-1][8], waypoints[numall-1][9], waypoints[numall-1][10], waypoints[numall-1][11],
        waypoints[numall-1][12], waypoints[numall-1][13]
    };
    input.target_velocity = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    input.target_acceleration = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    
    // 设置关节限制
    input.max_velocity = {vellim,vellim,vellim,vellim,vellim,vellim,vellim,
                          vellim,vellim,vellim,vellim,vellim,vellim,vellim};
    input.max_acceleration = {acclim,acclim,acclim,acclim,acclim,acclim,acclim,
                             acclim,acclim,acclim,acclim,acclim,acclim,acclim};
    input.max_jerk = {jerklim,jerklim,jerklim,jerklim,jerklim,jerklim,jerklim,
                      jerklim,jerklim,jerklim,jerklim,jerklim,jerklim,jerklim};
    
    cout << "轨迹起始点和目标点设置完成" << endl;
    
    // 添加中间路径点
    for(int m = 1; m < numall - 1; ++m) {
        std::vector<double> apoint = {
            waypoints[m][0], waypoints[m][1], waypoints[m][2], waypoints[m][3],
            waypoints[m][4], waypoints[m][5], waypoints[m][6], waypoints[m][7],
            waypoints[m][8], waypoints[m][9], waypoints[m][10], waypoints[m][11],
            waypoints[m][12], waypoints[m][13]
        };
        input.intermediate_positions.emplace_back(apoint);
    }
    cout << "中间路径点设置完成" << endl;
    
    // 生成轨迹
    vector<VectorXd> outputtrajectory;
    VectorXd trajpoints(wDOFs);
    
    double calculation_duration = 0.0;
    while (otg.update(input, output) == Result::Working) {
        // 获取当前轨迹点
        for (int n = 0; n < wDOFs; ++n) {
            trajpoints[n] = output.new_position[n];
        }
        outputtrajectory.push_back(trajpoints);
        
        // 传递参数给下一步
        output.pass_to_input(input);
        
        // 记录计算时间
        if (output.new_calculation) {
            calculation_duration = output.calculation_duration;
        }
    }
    
    // 打印轨迹信息
    std::cout << "到达目标位置耗时: " << output.trajectory.get_duration() << "秒" << std::endl;
    std::cout << "计算耗时: " << calculation_duration << "微秒" << std::endl;
    
    return outputtrajectory;
}