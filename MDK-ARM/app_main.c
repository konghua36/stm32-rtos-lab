/**
  ******************************************************************************
  * @file    app_main.c
  * @brief   培训任务2 全部代码（单文件版，约 200 行）
  *
  *   ① 参数区：要改就改这一段
  *   ② 电机：PWM 输出 + 编码器测速
  *   ③ PID
  *   ④ 串口发送（VoFA+ JustFloat 协议）
  *   ⑤ 三个 FreeRTOS 任务 + 创建
  *
  *   用法：main.c 里调用一次 App_Init(); 即可
  ******************************************************************************
  */

#include "main.h"
#include "tim.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ============================================================================
 * ① 参数区（只改这里）
 * ==========================================================================*/

/* --- 电机 / 编码器：JGB37-520 12V 333rpm，11 线，减速比 1:18 --- */
#define ENCODER_PPR         11.0f
#define ENCODER_MULT        4.0f
#define GEAR_RATIO          18.0f    /* 200rpm版填30，133rpm版填45，100rpm版填60 */
#define PWM_MAX_DUTY        1000.0f

/* --- 方向引脚（改你的接线）--- */
#define MOTOR_IN1_PORT      GPIOB
#define MOTOR_IN1_PIN       GPIO_PIN_0
#define MOTOR_IN2_PORT      GPIOB
#define MOTOR_IN2_PIN       GPIO_PIN_1
#define MOTOR_DIR_INVERT    0        /* 电机转向反了就改成 1 */

/* --- 周期 / 优先级 / 栈 --- */
#define MOTOR_PERIOD_MS     10
#define SIN_PERIOD_MS       20
#define VOFA_PERIOD_MS      50

#define TASK_PRIO_MOTOR     4
#define TASK_PRIO_SIN       3
#define TASK_PRIO_VOFA      2

#define TASK_STACK_MOTOR    256      /* 单位：字，256 字 = 1KB */
#define TASK_STACK_SIN      192
#define TASK_STACK_VOFA     256

/* --- 正弦目标：180±70 => 110~250rpm --- */
#define SINE_OFFSET_RPM     180.0f
#define SINE_AMP_RPM        70.0f
#define SINE_FREQ_HZ        0.2f

/* --- PID --- */
#define PID_KP              15.0f
#define PID_KI              45.0f
#define PID_KD              0.0f
#define PID_OUT_MAX         1000.0f  /* 输出限幅 */
#define PID_I_TERM_MAX      950.0f   /* 积分项最大贡献 */
#define PID_SEP_THRESHOLD   30.0f    /* 积分分离阈值 */

/* ============================================================================
 * ② 电机：PWM 输出 + 编码器测速
 * ==========================================================================*/
static int16_t s_last_cnt = 0;       /* 上次的编码器计数 */
static float   s_rpm      = 0.0f;    /* 滤波后的转速 */

static void Motor_SetDir(int8_t dir)
{
#if MOTOR_DIR_INVERT
    dir = (int8_t)(-dir);
#endif
    if (dir >= 0)
    {
        HAL_GPIO_WritePin(MOTOR_IN1_PORT, MOTOR_IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_IN2_PORT, MOTOR_IN2_PIN, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(MOTOR_IN1_PORT, MOTOR_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_IN2_PORT, MOTOR_IN2_PIN, GPIO_PIN_SET);
    }
}

static void Motor_Init(void)
{
    Motor_SetDir(1);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    s_last_cnt = 0;
    s_rpm = 0.0f;
}

/* duty: -1000 ~ +1000 */
static void Motor_SetDuty(float duty)
{
    uint32_t arr;
    uint32_t ccr;

    if (duty > PWM_MAX_DUTY)  { duty = PWM_MAX_DUTY; }
    if (duty < -PWM_MAX_DUTY) { duty = -PWM_MAX_DUTY; }

    if (duty >= 0.0f)
    {
        Motor_SetDir(1);
    }
    else
    {
        Motor_SetDir(-1);
        duty = -duty;
    }

    arr = __HAL_TIM_GET_AUTORELOAD(&htim3);
    ccr = (uint32_t)((duty / PWM_MAX_DUTY) * (float)arr);
    if (ccr > arr) { ccr = arr; }
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ccr);
}

/* 必须每 10ms 调一次 */
static void Motor_UpdateSpeed(void)
{
    int16_t now   = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    int16_t delta = (int16_t)(now - s_last_cnt);   /* 计数器回绕也正确 */
    float   rpm;

    s_last_cnt = now;

    /* 脉冲数 -> 每秒脉冲 -> 每分钟转数 */
    rpm = (float)delta * (1000.0f / (float)MOTOR_PERIOD_MS) * 60.0f
        / (ENCODER_PPR * ENCODER_MULT * GEAR_RATIO);

    s_rpm = s_rpm * 0.7f + rpm * 0.3f;   /* 一阶低通，滤掉量化抖动 */
}

/* ============================================================================
 * ③ PID
 * ==========================================================================*/
typedef struct
{
    float kp, ki, kd;
    float integral;   /* 误差积分 */
    float err_last;   /* 上次误差，用于微分 */
    float out;        /* 最近一次输出，调试用 */
} PID_t;

static PID_t s_pid;

static void PID_Init(PID_t *p, float kp, float ki, float kd)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->integral = 0.0f;
    p->err_last = 0.0f;
    p->out      = 0.0f;
}

static float PID_Calc(PID_t *p, float target, float actual, float dt)
{
    float err    = target - actual;
    float p_term = p->kp * err;
    float i_term;
    float out;
    float old_i  = p->integral;
    float i_lim;

    /* 积分分离：误差很大时先不积分，避免启动就把积分积爆 */
    if (fabs(err) < PID_SEP_THRESHOLD)
    {
        p->integral += err * dt;
    }

    /* 积分限幅：积分项最多贡献 PID_I_TERM_MAX */
    if (p->ki > 1.0e-4f)
    {
        i_lim = PID_I_TERM_MAX / p->ki;
        if (p->integral >  i_lim) { p->integral =  i_lim; }
        if (p->integral < -i_lim) { p->integral = -i_lim; }
    }
    i_term = p->ki * p->integral;

    out = p_term + i_term + (p->kd * (err - p->err_last) / dt);
    p->err_last = err;

    /* 输出限幅 + 抗饱和：已经顶住时撤回本次积分 */
    if (out > PID_OUT_MAX)
    {
        out = PID_OUT_MAX;
        if (err > 0.0f) { p->integral = old_i; }
    }
    else if (out < -PID_OUT_MAX)
    {
        out = -PID_OUT_MAX;
        if (err < 0.0f) { p->integral = old_i; }
    }

    p->out = out;
    return out;
}

/* ============================================================================
 * ④ 串口：给 VoFA+ 发 3 路 float（JustFloat 协议）
 *    一帧 = 12 字节数据 + 4 字节帧尾 00 00 80 7F
 * ==========================================================================*/
static void Vofa_Send3(float a, float b, float c)
{
    float          v[3];
    uint8_t        buf[16];
    const uint8_t *p;
    uint8_t        i, j;

    v[0] = a;
    v[1] = b;
    v[2] = c;

    for (i = 0; i < 3; i++)
    {
        p = (const uint8_t *)&v[i];
        for (j = 0; j < 4; j++)
        {
            buf[(i * 4u) + j] = p[j];
        }
    }
    buf[12] = 0x00u;
    buf[13] = 0x00u;
    buf[14] = 0x80u;
    buf[15] = 0x7Fu;

    (void)HAL_UART_Transmit(&huart1, buf, 16u, 100u);
}

/* ============================================================================
 * ⑤ 三个任务
 * ==========================================================================*/
static QueueHandle_t s_target_q = NULL;

/* 调试用：Keil 的 Watch 窗口里直接输入这几个名字 */
volatile float    g_target_rpm = 0.0f;
volatile float    g_actual_rpm = 0.0f;
volatile float    g_out_duty   = 0.0f;


/* --- 任务1：产生正弦目标值（优先级 3，20ms）--- */
static void TaskSin(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    float      t    = 0.0f;
    float      target;

    (void)arg;

    for (;;)
    {
        target = SINE_OFFSET_RPM +
                 (SINE_AMP_RPM * (float)sin(2.0 * M_PI * (double)SINE_FREQ_HZ * (double)t));

        (void)xQueueOverwrite(s_target_q, &target);   /* 只保留最新值 */

        t += (float)SIN_PERIOD_MS / 1000.0f;
        if (t > 1000.0f) { t -= 1000.0f; }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(SIN_PERIOD_MS));
    }
}

/* --- 任务2：电机控制（优先级 4，10ms）--- */
static void TaskMotor(void *arg)
{
    TickType_t last   = xTaskGetTickCount();
    float      dt     = (float)MOTOR_PERIOD_MS / 1000.0f;
    float      target = 0.0f;
    float      new_t, actual, out;

    (void)arg;

    for (;;)
    {
        if (xQueueReceive(s_target_q, &new_t, 0) == pdPASS)   /* 取最新目标 */
        {
            target = new_t;
        }

        Motor_UpdateSpeed();              /* 1. 读编码器 */
        actual = s_rpm;
        out    = PID_Calc(&s_pid, target, actual, dt);   /* 2. PID */
        Motor_SetDuty(out);               /* 3. 输出 PWM */

        g_target_rpm = target;
        g_actual_rpm = actual;
        g_out_duty   = out;
    

        vTaskDelayUntil(&last, pdMS_TO_TICKS(MOTOR_PERIOD_MS));
    }
}

/* --- 任务3：串口上传（优先级 2，50ms）--- */
static void TaskVofa(void *arg)
{
    TickType_t last = xTaskGetTickCount();

    (void)arg;

    for (;;)
    {
        Vofa_Send3(g_target_rpm, g_actual_rpm, g_out_duty / 10.0f);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(VOFA_PERIOD_MS));
    }
}

/* --- 初始化并创建任务：main.c 里调用一次 --- */
void App_Init(void)
{
    Motor_Init();
    PID_Init(&s_pid, PID_KP, PID_KI, PID_KD);

    s_target_q = xQueueCreate(1u, sizeof(float));
    if (s_target_q == NULL) { Error_Handler(); }

    /* 参数：函数, 名字, 栈(字), 参数, 优先级, 句柄 */
    if (xTaskCreate(TaskMotor, "motor", TASK_STACK_MOTOR, NULL, TASK_PRIO_MOTOR, NULL) != pdPASS)
    {
        Error_Handler();
    }
    if (xTaskCreate(TaskSin, "sin", TASK_STACK_SIN, NULL, TASK_PRIO_SIN, NULL) != pdPASS)
    {
        Error_Handler();
    }
    if (xTaskCreate(TaskVofa, "vofa", TASK_STACK_VOFA, NULL, TASK_PRIO_VOFA, NULL) != pdPASS)
    {
        Error_Handler();
    }
}

/************************ (C) COPYLEFT 培训示例代码 *****END OF FILE****/