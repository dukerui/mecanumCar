#include "mecanumCar.h"

#define DBG_SECTION_NAME "mecanumCar"
#define DBG_LEVEL DBG_LOG
#include <rtdbg.h>

static rt_thread_t tid_car = RT_NULL;
static rt_err_t car_stop(rt_int8_t cmd, void *param);
static rt_err_t car_forward(rt_int8_t cmd, void *param);

void car_thread(void *param)
{
    // TODO

    struct velocity target_velocity;

    target_velocity.linear_x = 0.0f;
    target_velocity.linear_y = 0.0f;
    target_velocity.angular_z = 0.0f;
    chassis_set_velocity(chas, target_velocity);

    // Open control
    controller_disable(chas->c_wheels[0]->w_controller);
    controller_disable(chas->c_wheels[1]->w_controller);
    controller_disable(chas->c_wheels[2]->w_controller);
    controller_disable(chas->c_wheels[3]->w_controller);

    while (1)
    {
        rt_thread_mdelay(1000);
        chassis_update(chas);
        // ano_send_senser(chas->c_wheels[0]->rpm, chas->c_wheels[0]->w_controller->target, chas->c_wheels[1]->rpm, chas->c_wheels[1]->w_controller->target,0,0,0,0,0,0);
    }

    //    chassis_destroy(chas);
}

void car_init(void *parameter)
{
    // 1. Initialize four wheels
    wheel_t *c_wheels = (wheel_t *)rt_malloc(sizeof(wheel_t) * 4);
    if (c_wheels == RT_NULL)
    {
        LOG_D("Failed to malloc memory for wheels");
    }

    // 1.1 Create four motors
    single_pwm_motor_t FL_motor = single_pwm_motor_create(FL_motor_PWM, FL_motor_PWM_CHANNEL, FL_motor_DIR_PIN, NULL);
    single_pwm_motor_t FR_motor = single_pwm_motor_create(FR_motor_PWM, FR_motor_PWM_CHANNEL, FR_motor_DIR_PIN, NULL);
    single_pwm_motor_t BL_motor = single_pwm_motor_create(BL_motor_PWM, BL_motor_PWM_CHANNEL, BL_motor_DIR_PIN, NULL);
    single_pwm_motor_t BR_motor = single_pwm_motor_create(BR_motor_PWM, BR_motor_PWM_CHANNEL, BR_motor_DIR_PIN, NULL);
    // single_pwm_motor_enable(FL_motor);
    // single_pwm_motor_enable(FR_motor);
    // single_pwm_motor_enable(BL_motor);
    // single_pwm_motor_enable(BR_motor);

    // 1.2 Create four encoders
    ab_phase_encoder_t FL_encoder = ab_phase_encoder_create(FL_ENCODER_A_PHASE_PIN, FL_ENCODER_B_PHASE_PIN, PULSE_PER_REVOL);
    ab_phase_encoder_t FR_encoder = ab_phase_encoder_create(FR_ENCODER_A_PHASE_PIN, FR_ENCODER_B_PHASE_PIN, PULSE_PER_REVOL);
    ab_phase_encoder_t BL_encoder = ab_phase_encoder_create(BL_ENCODER_A_PHASE_PIN, BL_ENCODER_B_PHASE_PIN, PULSE_PER_REVOL);
    ab_phase_encoder_t BR_encoder = ab_phase_encoder_create(BR_ENCODER_A_PHASE_PIN, BR_ENCODER_B_PHASE_PIN, PULSE_PER_REVOL);
    // 1.3 Create four pid contollers
    inc_pid_controller_t FL_pid = inc_pid_controller_create(PID_PARAM_KP, PID_PARAM_KI, PID_PARAM_KD);
    inc_pid_controller_t FR_pid = inc_pid_controller_create(PID_PARAM_KP, PID_PARAM_KI, PID_PARAM_KD);
    inc_pid_controller_t BL_pid = inc_pid_controller_create(PID_PARAM_KP, PID_PARAM_KI, PID_PARAM_KD);
    inc_pid_controller_t BR_pid = inc_pid_controller_create(PID_PARAM_KP, PID_PARAM_KI, PID_PARAM_KD);
    // 1.4 Add four wheels
    c_wheels[0] = wheel_create((motor_t)FL_motor, (encoder_t)FL_encoder, (controller_t)FL_pid, WHEEL_RADIUS, GEAR_RATIO);
    c_wheels[1] = wheel_create((motor_t)FR_motor, (encoder_t)FR_encoder, (controller_t)FR_pid, WHEEL_RADIUS, GEAR_RATIO);
    c_wheels[2] = wheel_create((motor_t)BL_motor, (encoder_t)BL_encoder, (controller_t)BL_pid, WHEEL_RADIUS, GEAR_RATIO);
    c_wheels[3] = wheel_create((motor_t)BR_motor, (encoder_t)BR_encoder, (controller_t)BR_pid, WHEEL_RADIUS, GEAR_RATIO);

    // 2. Iinialize Kinematics - mecanum
    kinematics_t c_kinematics = kinematics_create(MECANUM, WHEEL_DIST_X, WHEEL_DIST_Y, WHEEL_RADIUS);

    // 3. Initialize Chassis
    chas = chassis_create(c_wheels, c_kinematics);
    // Set Sample time
    encoder_set_sample_time(chas->c_wheels[0]->w_encoder, SAMPLE_TIME);
    encoder_set_sample_time(chas->c_wheels[1]->w_encoder, SAMPLE_TIME);
    encoder_set_sample_time(chas->c_wheels[2]->w_encoder, SAMPLE_TIME);
    encoder_set_sample_time(chas->c_wheels[3]->w_encoder, SAMPLE_TIME);
    controller_set_sample_time(chas->c_wheels[0]->w_controller, PID_SAMPLE_TIME);
    controller_set_sample_time(chas->c_wheels[1]->w_controller, PID_SAMPLE_TIME);
    controller_set_sample_time(chas->c_wheels[2]->w_controller, PID_SAMPLE_TIME);
    controller_set_sample_time(chas->c_wheels[3]->w_controller, PID_SAMPLE_TIME);

    // 4. Enable Chassis
    chassis_enable(chas);

    // Register command
    command_register(COMMAND_CAR_STOP, car_stop);
    command_register(COMMAND_CAR_FORWARD, car_forward);
    //command_register(COMMAND_CAR_BACKWARD, car_backward);
    //command_register(COMMAND_CAR_TURNLEFT, car_turnleft);
    //command_register(COMMAND_CAR_TURNRIGHT, car_turnright);

    rt_kprintf("car command register complete\n");
    // struct velocity target_velocity;

    // target_velocity.linear_x = 0.20f;
    // target_velocity.linear_y = 0;
    // target_velocity.angular_z = 0;
    // chassis_set_velocity(chas, target_velocity);
    // Controller
    //ps2_init(28, 29, 4, 36);

    // thread
    tid_car = rt_thread_create("tcar",
                               car_thread, RT_NULL,
                               THREAD_STACK_SIZE,
                               THREAD_PRIORITY, THREAD_TIMESLICE);

    if (tid_car != RT_NULL)
    {
        rt_thread_startup(tid_car);
    }
}

static rt_err_t car_stop(rt_int8_t cmd, void *param)
{
    struct velocity target_velocity;

    target_velocity.linear_x = 0.00f;
    target_velocity.linear_y = 0;
    target_velocity.angular_z = 0;
    chassis_set_velocity(chas, target_velocity);

    LOG_D("stop cmd");

    return RT_EOK;
}

static rt_err_t car_forward(rt_int8_t cmd, void *param)
{
    struct velocity target_velocity;

    target_velocity.linear_x = 0.05f;
    target_velocity.linear_y = 0;
    target_velocity.angular_z = 0;
    chassis_set_velocity(chas, target_velocity);

    LOG_D("forward cmd");

    return RT_EOK;
}
