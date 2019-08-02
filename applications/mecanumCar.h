#ifndef __MECANUMCAR_H__
#define __MECANUMCAR_H__

#include <rtthread.h>
#include "chassis.h"
#include "inc_pid_controller.h"
#include "ab_phase_encoder.h"
#include "single_pwm_motor.h"
#include "board.h"
#include "command.h"

/************** car *****************/
chassis_t chas;
#define THREAD_PRIORITY 10
#define THREAD_STACK_SIZE 1024
#define THREAD_TIMESLICE 5

/************** encoder *************/
//FL TIM2 CH2  D
#define FL_ENCODER_A_PHASE_PIN GET_PIN(C, 0)
#define FL_ENCODER_B_PHASE_PIN GET_PIN(C, 1)
//FR TIM2 CH3  B
#define FR_ENCODER_A_PHASE_PIN GET_PIN(A, 1)
#define FR_ENCODER_B_PHASE_PIN GET_PIN(A, 0)
//BL TIM3 CH1  C
#define BL_ENCODER_A_PHASE_PIN GET_PIN(B, 0)
#define BL_ENCODER_B_PHASE_PIN GET_PIN(A, 4)
//BR TIM3 CH2  A
#define BR_ENCODER_A_PHASE_PIN GET_PIN(B, 9)
#define BR_ENCODER_B_PHASE_PIN GET_PIN(B, 8)
#define PULSE_PER_REVOL 390 // Real value 2000
#define SAMPLE_TIME 50
//#define FL_ENCODER_PIN            62     // GET_PIN(D, 14)
//#define FR_ENCODER_PIN           61     // GET_PIN(D, 13)

//#define BL_ENCODER_PIN            62     // GET_PIN(D, 14)
//#define BR_ENCODER_PIN           61     // GET_PIN(D, 13)

//#define PULSE_PER_REVOL             20     // Real value 20
//#define SAMPLE_TIME               1000

/************** motor ***************/
#define FL_motor_PWM "pwm2"   //D
#define FL_motor_PWM_CHANNEL 2
#define FL_motor_DIR_PIN GET_PIN(A, 10)
#define FR_motor_PWM "pwm2"   //B
#define FR_motor_PWM_CHANNEL 3
#define FR_motor_DIR_PIN GET_PIN(A, 8)

#define BL_motor_PWM "pwm3"  //C
#define BL_motor_PWM_CHANNEL 1
#define BL_motor_DIR_PIN GET_PIN(B, 5)
#define BR_motor_PWM "pwm3"  //A
#define BR_motor_PWM_CHANNEL 2
#define BR_motor_DIR_PIN GET_PIN(A, 9)
//Forward left
//extern int FL_motor_init(void);
//extern int FL_motor_enable(void);
//extern int FL_motor_disable(void);
//extern int FL_motor_set_speed(rt_int8_t percentage);
//Forward right
//extern int FR_motor_init(void);
//extern int FR_motor_enable(void);
//extern int FR_motor_disable(void);
//extern int FR_motor_set_speed(rt_int8_t percentage);
//back left
//extern int BL_motor_init(void);
//extern int BL_motor_enable(void);
//extern int BL_motor_disable(void);
//extern int BL_motor_set_speed(rt_int8_t percentage);
//back right
//extern int BR_motor_init(void);
//extern int BR_motor_enable(void);
//extern int BR_motor_disable(void);
//extern int BR_motor_set_speed(rt_int8_t percentage);
/**************wheel ****************/
#define WHEEL_RADIUS 0.060
#define GEAR_RATIO 1 //(1.0f/30.0f)

#define WHEEL_DIST_X 0.165 //轴距
#define WHEEL_DIST_Y 0.210 //轮距

/************controllr PID***********/
#define PID_SAMPLE_TIME 50
#define PID_PARAM_KP 6.6
#define PID_PARAM_KI 6.5
#define PID_PARAM_KD 7.6

#endif
