
#ifndef ROS2_CONTROL_HPP_
#define ROS2_CONTROL_HPP_

#include <memory>
#include <string>
#include <vector>
#include "erobot_hw_arm.hpp"

  // 多个电机的读取和设置的参数
  struct MotorState {
    double position;//rad
    double velocity;//rad/s
    double effort;//Nm
    double motor_encoder[2];  //电机端编码器0 19bit 输出端编码器1 21bit
    double status;             // 电机状态码
    double error_code;         // CiA402 0x603F error code
    double mode;               // 电机工作模式
    double power_enable; //电机上下使能
  };
//设置电机的运行模式
enum motor_mode
{
    CSP = 8,
    CSV = 9,
    CST = 10,
};

enum  RunMode {
    CUBIC_POLYNOMIAL = 0,  // 三次多项式运行模式
    RUCKIG = 1,            // Ruckig轨迹规划器运行模式
    VELOCITY = 2,          // 速度运行模式
    EFFORT = 3,            // 力矩运行模式
    CARTRSIAN = 4,
    EXTERNAL_POSITION_STREAM = 5
};

enum Motor_class
{
    ZERO = 1,
    HC = 2,
    FHDL = 3,
};

typedef struct {
    uint8_t ec_modestate; /**< Slave alias address. */
    uint8_t ec_ctrstate; /**< Slave position. */
    int32_t axis_position ; /**< Slave vendor ID. */
    int32_t axis_velocity; /**< Slave product code. */
    int32_t axis_effort; /**< PDO entry index. */
    int32_t axis_encoder_0; /**< Slave product code. */ 
    int32_t axis_encoder_1; /**< Slave product code. */ 
    uint16_t axis_error_code; /**< Latest 0x603F fault code. */
    uint8_t axis_counter; /**< PDO entry subindex. */
    uint8_t axis_checksum; /**< PDO entry subindex. */
} ec_app_axis_read_reg_t;

typedef struct {
    uint8_t ec_mode; /**< Slave alias address. */
    uint8_t ec_ctrword; /**< Slave position. */
    int32_t axis_position ; /**< Slave vendor ID. */
    int32_t axis_velocity; /**< Slave product code. */
    int32_t axis_effort; /**< PDO entry index. */
    uint8_t axis_counter; /**< PDO entry subindex. */
    uint8_t axis_checksum; /**< PDO entry subindex. */
} ec_app_axis_write_reg_t;

typedef struct {
    uint8_t ec_powerstate; /**< Slave alias address. */
    ec_app_axis_read_reg_t axis_state[23]; /**< Must match the IGH driver shared-memory ABI. */
} ec_app_real_reg_t;

typedef struct {
    uint8_t ec_poweron; /**< Slave alias address. */
    ec_app_axis_write_reg_t axis_ctr[23]; /**< Must match the IGH driver shared-memory ABI. */
} ec_app_desire_reg_t;

void get_joint_real(ec_app_axis_read_reg_t &real_reg, MotorState &motor_state, uint8_t motor_class,uint8_t motor_CW){
    motor_state.error_code = static_cast<double>(real_reg.axis_error_code);
    switch (motor_class)
    {
        case ZERO:
            if (motor_CW == 1){
                motor_state.position = ((double)real_reg.axis_position)* -1.1984224905356572107172559292906e-5;   
                motor_state.velocity = ((double)real_reg.axis_velocity)* -1.1984224905356572107172559292906e-5;
                motor_state.effort = ((double)real_reg.axis_effort)*0.001*-2.55;
                motor_state.power_enable = (double)real_reg.ec_modestate;
            }
            else{
                motor_state.position = ((double)real_reg.axis_position)* 1.1984224905356572107172559292906e-5;   
                motor_state.velocity = ((double)real_reg.axis_velocity)* 1.1984224905356572107172559292906e-5;
                motor_state.effort = ((double)real_reg.axis_effort)*0.001*2.55;
                motor_state.power_enable = (double)real_reg.ec_modestate;
            }
            break;
        case HC:
            motor_state.position = ((double)real_reg.axis_position)*-0.000047937/101.0f;   
            motor_state.velocity = ((double)real_reg.axis_velocity)*-0.000047937/101.0f;
            motor_state.effort = ((double)real_reg.axis_effort)*-0.001*2.55*101.0f;
            motor_state.power_enable = (double)real_reg.ec_modestate;
            break;
        case FHDL:
            if (motor_CW == 1){
                motor_state.position = ((double)real_reg.axis_position)* 0.000002996*16.0f;   
                motor_state.velocity = ((double)real_reg.axis_velocity)* 0.000002996*16.0f;
                motor_state.effort = ((double)real_reg.axis_effort)*0.001*2.55;
                motor_state.power_enable = (double)real_reg.ec_modestate;
            }
            else{
                motor_state.position = ((double)real_reg.axis_position)* -0.000002996*16.0f;   
                motor_state.velocity = ((double)real_reg.axis_velocity)* -0.000002996*16.0f;
                motor_state.effort = ((double)real_reg.axis_effort)*-0.001*2.55;
                motor_state.power_enable = (double)real_reg.ec_modestate;
            }
            break;
    }
}
void set_joint_desire(ec_app_axis_write_reg_t &desire_reg, MotorState &motor_desire,uint8_t motor_class,uint8_t motor_CW,uint8_t motor_mode_){

    switch (motor_class)
    {
        case ZERO:
            if(motor_mode_ == RunMode::CUBIC_POLYNOMIAL || motor_mode_ == RunMode::RUCKIG ||
              motor_mode_ == RunMode::EXTERNAL_POSITION_STREAM){
                desire_reg.ec_mode = motor_mode::CSP;
            }
            else if (motor_mode_ == RunMode::VELOCITY){desire_reg.ec_mode = motor_mode::CSV;}
            else if (motor_mode_ == RunMode::EFFORT){desire_reg.ec_mode = motor_mode::CST;}

            if (motor_CW == 1){
                desire_reg.axis_position = (int32_t)(motor_desire.position * -83443.026803763621f);   
                desire_reg.axis_velocity = (int32_t)(motor_desire.velocity * -83443.026803763621f);
                desire_reg.axis_effort = (int32_t)(motor_desire.effort * -1.0);
            }
            else{
                desire_reg.axis_position = (int32_t)(motor_desire.position * 83443.026803763621f);
                desire_reg.axis_velocity = (int32_t)(motor_desire.velocity * 83443.026803763621f);
                desire_reg.axis_effort = (int32_t)(motor_desire.effort * 1.0);
            }
            break;
        case HC:
            desire_reg.ec_mode = 8;
            desire_reg.axis_position = (int32_t)(motor_desire.position * -20860.713019171f*101.0f);   
            break;
        
        case FHDL:
            static uint8_t num = 0;
            if (motor_CW == 1){
                desire_reg.axis_velocity = (int32_t)(motor_desire.velocity * 333778.371161549f*0.0625f);   
            }
            else{
                desire_reg.axis_velocity = (int32_t)(motor_desire.velocity * -333778.371161549f*0.0625f);
            }
            desire_reg.axis_counter = num++;
            if (num > 250) num = 0;
            break;
    }
    

}

#endif  
