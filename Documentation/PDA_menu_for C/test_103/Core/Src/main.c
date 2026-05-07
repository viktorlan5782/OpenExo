/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DrEmpower_can.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  Can_Config(); //Can配置信息
  HAL_CAN_ActivateNotification(&SERVO_CAN,CAN_IT_RX_FIFO0_MSG_PENDING);//使能中断
  /* USER CODE END 2 */

  /* Infinite loop */
  /////**************231130*************////////
      HAL_Delay(1000); // 延时 1s

      /////**************设置 ID 号*************////////
  //    uint8_t id_old = 0;
  //    uint8_t id_new = 7;
  //    set_id(id_old, id_new); // 将 1 号关节设置为 7 号

      /////**************设置零点位置*************////////
    //  uint8_t id_num = 7;
    //  set_zero_position(id_num); //给 1 号关节设置零点

      /////**************设置临时零点位置*************////////
    //  uint8_t id_num = 1;
    //  set_zero_position_temp(id_num); // 给 1 号关节设置临时零点

      /////**************单个绝对角度控制*************////////
      uint8_t id_num = 0;
      float angle = 180;
      float speed = 10;
      float param = 10;
      int mode = 1;
      set_angle(id_num, angle, speed, param, mode); // 控制 7 号关节转到 180°

      /////**************多个绝对角度控制*************////////
  //    uint8_t id_list[]={4,2,3,7,1};  // 6 OK
  //    float angle_list[]={60, 90, 160, 60, 160};
  //    float f_angle_list[]={-60, -90, -160, -60, -160};
  //    while(1)
  //    {
  //    	set_angles(id_list, angle_list, 10, 10, 1, 5);  //多个关节角度控制函数
  //    	positions_done(id_list, 5);
  //    	set_angles(id_list, f_angle_list, 10, 10, 1, 5);  //多个关节角度控制函数
  //    	positions_done(id_list, 5);
  //    }

      /////**************单个关节相对/步进角度控制*************////////
  //    uint8_t id_num = 7;
  //    float angle = 10;
  //    float speed = 10;
  //    float param = 10;
  //    int mode = 1;
  //    step_angle(id_num, angle, speed, param, mode);

      /////**************多个关节相对/步进角度控制*************////////
  //    uint8_t id_list[]={7,2,3,1,4};  // 6 OK
  //    float zeros[] = {0, 0, 0, 0, 0};
  //    set_angles(id_list, zeros, 10, 10, 1, 5);
  //    positions_done(id_list,5);
  //    float angle_list[]={60,60,60,60,90};
  //    float f_angle_list[]={-60,-60,-60,-60, -90};
  //    while(1)
  //    {
  //        step_angles(id_list,angle_list,10,10,1,5);  //多个关节相对角度控制函数
  //        positions_done(id_list, 5);
  //        step_angles(id_list,f_angle_list,10,10,1,5);  //多个关节相对角度控制函数
  //        positions_done(id_list, 5);
  //    }

      /////**************单个关节力位混合（自适应）绝对角度控制*************////////
  //    uint8_t id_num = 7;
  //    float angle = 180;
  //    float speed = 20;
  //    float torque = 1;
  //    set_angle_adaptive(id_num, angle, speed, torque); // 控制 7 号关节以 1N 力，20r/min 转速，转到 180°

      /////**************多个关节力位混合（自适应）绝对角度控制*************////////
  //    uint8_t id_list[]={1,2,3,4,7};  // 6 OK
  //    while (1)
  //    {
  //  	  float angle_list[]={65, 60, 60, 60, 60};
  //  	  float speed_list[]={10, 10, 10, 10, 10};
  //  	  float tff_list[]={1, 60, 22, 9, 1};
  //  	  set_angles_adaptive(id_list, angle_list, speed_list, tff_list, 5);  //多个关节自适应绝对角度控制
  //  	  positions_done(id_list, 5);
  //
  //  	  float angle_list1[]={-65, -60, -60, -60, -60};
  //  	  float speed_list1[]={10, 10, 10, 10, 10};
  //  	  float tff_list1[]={1, 60, 22, 9, 1};
  //  	  set_angles_adaptive(id_list, angle_list1, speed_list1, tff_list1, 5);  //多个关节自适应绝对角度控制
  //  	  positions_done(id_list, 5);
  //    }

      /////**************单关节阻抗控制*************////////
  //    uint8_t id_num = 7;
  //    float pos = 90;
  //    float vel = 0;
  //    float tff = 0;
  //    float kp = 0.1;
  //    float kd = 0.1;
  //    impedance_control(id_num, pos, vel, tff, kp, kd);

      /////**************多关节阻抗控制*************////////
  //    uint8_t id_list[]={1,2,3,4,7}; // 6 OK
  //    float angle_list[]={50, 60, 60, 60, 60};
  //    float speed_list[]={0, 0, 0, 0, 0};
  //    float tff_list[]={0, 0, 0, 0, 0};
  //    float kp_list[]={0.1, 0.5, 0.5, 0.5, 0.01};
  //    float kd_list[]={0.1, 0.5, 0.5, 0.5, 0.01};
  //    impedance_control_multi(id_list,angle_list,speed_list,tff_list,kp_list,kd_list,5);  //多个关节阻抗控制
  //    HAL_Delay(2500); // 延时2.5s
  //    while(1)
  //    {
  //        float angle_list1[]={0, 0, 0, 0, 0};
  //        float speed_list1[]={0, 0, 0, 0, 0};
  //        float tff_list1[]={0, 0, 0, 0, 0};
  //        float kp_list1[]={0.1, 0.5, 0.5, 0.5, 0.01};
  //        float kd_list1[]={0.1, 0.5, 0.5, 0.5, 0.01};
  //        impedance_control_multi(id_list, angle_list1, speed_list1, tff_list1, kp_list1, kd_list1, 5);  //多个关节阻抗控制
  //        HAL_Delay(2500); // 延时2.5s
  //
  //        float angle_list2[]={50, 50, 50, 50, 50};
  //        float speed_list2[]={0, 0, 0, 0, 0};
  //        float tff_list2[]={0, 0, 0, 0, 0};
  //        float kp_list2[]={0.1, 0.5, 0.5, 0.5, 0.01};
  //        float kd_list2[]={0.1, 0.5, 0.5, 0.5, 0.01};
  //        impedance_control_multi(id_list, angle_list2, speed_list2, tff_list2, kp_list2, kd_list2, 5);  //多个关节阻抗控制
  //        HAL_Delay(2500); // 延时2.5s
  //    }

      /////**************单个关节运动跟随与助力*************////////
  //    uint8_t id_num = 7;
  //    float angle = 90;
  //    float speed = 10;
  //    float angle_err = 0.1;
  //    float speed_err = 0.1;
  //    float torque = 1;
  //    motion_aid(id_num, angle, speed, angle_err, speed_err, torque);

      /////**************多个关节运动跟随与助力*************////////
  //    uint8_t id_list[]={1,2,3,4,7};  // 6 OK
  //    float angle_list[5] = {120, 120, 120, 120, 120};
  //    float speed_list[5] = {10,10,10, 10, 10};
  //    float angle_err_list[5] = {0.1,0.1,0.1, 0.1, 0.1};
  //    float speed_err_list[5] = {0.1,0.1,0.1, 0.1, 0.1};
  //    float torque_list[5] = {1, 5, 5, 3, 0.1};
  //    motion_aid_multi(id_list, angle_list, speed_list, angle_err_list, speed_err_list, torque_list, 5);

      /////**************检查并等待单个关节转动到目标角度*************////////
    //  uint8_t id_num = 1;
    //  position_done(id_num);

      /////**************检查并等待多个关节转动到目标角度*************////////
    //  uint8_t id_list[]={1,2,3,4,7};  // 6 OK
    //  positions_done(id_list);

      /////**************单个关节转速控制*************////////
  //    uint8_t id_num = 7;
  //    float speed = 60;
  //    float param = 20;
  //    int mode = 1;
  //    set_speed(id_num, speed, param, mode);
  //    HAL_Delay(1000);
  //    estop(id_num); // 急停

      /////**************多个关节转速控制*************////////
  //    uint8_t id_list[]={7,2,3,1,4}; // 6 OK
  //    float speed_list[]={-10,10,10,10,10}; //三个关节的正转角度
  //    float speed_list_2[]={10,-10,-10,-10,-10}; //三个关节的正转角度
  //    set_speeds(id_list, speed_list, 2, 1, 5);
  //    HAL_Delay(1000);
  //    set_speeds(id_list, speed_list_2, 20, 1, 5);
  //    HAL_Delay(1000);
  //    estop(0); // 所有关节急停

      /////**************单个关节力矩控制*************////////
  //      uint8_t id_num = 7;
  //      float torque = -1;
  //      float param = 1;
  //      int mode = 1;
  //      set_torque(id_num, torque, param, mode);
  //      HAL_Delay(1000);
  //      estop(id_num); // 急停

      /////**************多个关节力矩控制*************////////
  //    uint8_t id_list[]={7,2,3,1,4}; // 6 OK
  //    float torque_list[]={1, 17, 17, 1, 1}; //三个关节的正转角度
  //    float torque_list_2[]={-1, -17, -17, -1, -1}; //三个关节的正转角度
  //    set_torques(id_list,torque_list, 20, 1, 5);
  //    HAL_Delay(1000);
  //    set_torques(id_list, torque_list_2, 40, 1, 5);
  //    HAL_Delay(1000);
  //    estop(0);

      /////**************回读关节 ID 号 此时总线上只能有 1 个关节*************////////
    //  get_id(0);

      /////**************回读关节当前角度*************////////
    //  uint8_t id_num = 1;
    //  speed = get_angle(id_num);

      /////**************回读关节当前转速*************////////
    //  uint8_t id_num = 1;
    //  get_speed(id_num);

      /////**************回读关节当前输出力矩*************////////
    //  uint8_t id_num = 1;
    //  get_torque(id_num);

      /////**************开启角度、转速、力矩实时反馈*************////////
    //  uint8_t id_num = 1;
    //  enable_angle_speed_torque_state(id_num);

      /////**************设置角度、转速、力矩状态实时反馈时间间隔，单位 ms，默认为 2*************////////
    //  uint8_t id_num = 1;
    //  uint32_t n_ms = 10;
    //  set_state_feedback_rate_ms(id_num, n_ms);

      /////**************关节角度、转速、力矩实时反馈*************////////
  //    enable_angle_speed_torque_state(0); // 6 OK 设置所有关节开启状态反馈
  //    set_state_feedback_rate_ms(0, 10); // 设置状态反馈时间间隔 10ms, 改时间建议值为>2*N（N 为总线上关节数量）
  //    struct angle_speed_torque angle_speed_torque = {0, 0, 0}; // 定义结构体用于接收状态反馈结果
  //    uint8_t id = 1;
  //    angle_speed_torque = angle_speed_torque_state(id); // 读取状态，假设总线上只有一个关节
  //    set_speed(id, angle_speed_torque.angle*0.1+0.1, 0, 0);
  //    id = 2;
  //    angle_speed_torque = angle_speed_torque_state(id); // 读取状态，假设总线上只有一个关节
  //    set_speed(id, angle_speed_torque.angle*0.1+0.1, 0, 0);
  //    id = 3;
  //    angle_speed_torque = angle_speed_torque_state(id);
  //    set_speed(id, angle_speed_torque.angle*0.1+0.1, 0, 0);
  //    id = 4;
  //    angle_speed_torque = angle_speed_torque_state(id);
  //    set_speed(id, angle_speed_torque.angle*0.1+0.1, 0, 0);
  //    id = 7;
  //    angle_speed_torque = angle_speed_torque_state(id);
  //    set_speed(id, angle_speed_torque.angle*0.1+0.1, 0, 0);
  //    disable_angle_speed_torque_state(0);

      /////**************关闭角度、转速、力矩实时反馈*************////////
    //  uint8_t id_num = 7;
    //  disable_angle_speed_torque_state(id_num);

      /////**************回读控制环位置增益 P、积分增益 I函数、转速增益 D*************////////
    //  uint8_t id_num = 7;
    //  struct PID PID = {0, 0, 0};
    //  PID = get_pid(id_num);

      /////**************回读配置参数*************////////
    //  uint8_t id_num = 7;
    //  int param_address = 32102; // 参数键码，可查阅库函数说明书的附件：关节开放参数表
    //  int param_type = 0; // 数据类型代号 float:0; uint16_t:1; int16_t:2; uint32_t:3; int32_t:4
    //  float value = 0;
    //  value = read_property(id_num, param_address, param_type);

      /////**************设置本次运行期间关节角度限位*************////////
    //  uint8_t id_num = 7;
    //  float angle_min = -180;
    //  float angle_max = 180;
    //  set_angle_range(id_num, angle_min, angle_max);

      /////**************关闭本次运行期间的关节角度限位*************////////
    //  uint8_t id_num = 7;
    //  disable_angle_range(id_num);

      /////**************设置关节限位属性*************////////
    //  uint8_t id_num = 7;
    //  float angle_min = -180;
    //  float angle_max = 180;
    //  set_angle_range_config(id_num, angle_min, angle_max);

      /////**************关闭关节限位属性*************////////
    //  uint8_t id_num = 7;
    //  disable_angle_range_config(id_num);

      /////**************设置本次运行期间转速限制*************////////
    //  uint8_t id_num = 7;
    //  float speed_limit = 1;
    //  set_speed_limit(id_num, speed_limit);

      /////**************设置本次运行期间力矩限制*************////////
    //  uint8_t id_num = 7;
    //  float torque_limit = 0.3;
    //  set_torque_limit(id_num, torque_limit);

      /////**************设置本次运行期间自适应转速*************////////
  //    uint8_t id_num = 7;
  //    float speed_adaptive = 1;
  //    set_angle(id_num, 180, 60, 60, 1);
  //    set_speed_adaptive(id_num, speed_adaptive); // 需要放在运动指令之后

      /////**************设置本次运行期间自适应力矩*************////////
  //    uint8_t id_num = 7;
  //    float torque_adaptive = 0.08;
  //    set_angle(id_num, -1180, 60, 60, 1);
  //    set_torque_adaptive(id_num, torque_adaptive); // 需要放在运动指令之后


      /////**************设置本次运行期间位置增益 P、积分增益 I、转速增益 D*************////////
    //  uint8_t id_num = 7;
    //  float P = 1;
    //  float I = 1;
    //  float D = 1;
    //  set_pid(id_num, P, I, D);

      /////**************设置关节待机或闭环控制*************////////
    //  uint8_t id_num = 7;
    //  int mode = 2;
    //  set_mode(id_num, mode);

      /////**************写入关节配置参数*************////////
    //  uint8_t id_num = 7;
    //  int param_address = 32102; // 参数键码，可查阅库函数说明书的附件：关节开放参数表
    //  int param_type = 0; // 数据类型代号 float:0; uint16_t:1; int16_t:2; uint32_t:3; int32_t:4
    //  float value = 1;
    //  write_property(id_num, 32102, 0, value);

      /////**************保存关节配置参数*************////////
    //  uint8_t id_num = 7;
    //  save_config(id_num);

      /////**************重启一体化关节*************////////
    //  uint8_t id_num = 7;
    //  reboot(id_num);

      /////**************恢复出厂设置，不改变关节 ID 号*************////////
    //  uint8_t id_num = 7;
    //  init_config(id_num);

      /////**************格式化擦除配置参数*************////////
    //  uint8_t id_num = 7;
    //  erase_config(id_num);
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
