#include "imu.h"
#include "imu_data_decode.h"
#include "packet.h"

// 优化点1：加上 volatile，防止编译器优化中断变量
volatile uint8_t rxbuf; 
imu_t imu;

// 优化点2：致命参数修改！！！
// 原来的 0.03 会导致约 60ms 的信号延迟，平衡车会因为反应慢而必倒。
// 改为 0.5 (兼顾实时性和滤波)，或者 0.3。绝对不能太小。
float alpha = 0.5f; 

float low_pass_filter(float value);

void imu_init(void)
{
	imu_data_decode_init();
}

void imu_get(void)
{
	// 获取原始欧拉角
	imu.pit = id0x91.eul[0];
	imu.rol = id0x91.eul[1];
	imu.yaw = id0x91.eul[2];
	
	// 简单的角度归一化处理
	if(imu.yaw < 0) imu.yaw = id0x91.eul[2] + 360;
	
	// 获取原始角速度
	// 注意：平衡车最依赖的是 vx (roll轴角速度)
	imu.vx = id0x91.gyr[0];
	imu.vy = id0x91.gyr[1];
	imu.vz = id0x91.gyr[2];	
	
	// 一阶低通滤波
	// 现在 alpha 变大了，延迟会小很多，阻尼感会变强
	imu.vx = low_pass_filter(imu.vx);
}

void UART8_IRQHandler(void)
{
	if(LL_USART_IsActiveFlag_RXNE(UART8) && LL_USART_IsEnabledIT_RXNE(UART8))
	{
		rxbuf = LL_USART_ReceiveData8(UART8);
		packet_decode(rxbuf);
		// LL库通常不需要手动再次Enable，但保留你的原样
		LL_USART_EnableIT_RXNE(UART8); 
	}
}

// 滤波函数保持逻辑不变，只依靠全局 alpha 调整强度
float low_pass_filter(float value)
{
  static float out_last = 0;
  float out;

  // 第一次进入时初始化
  static char first_flag = 1; // 修正拼写错误 fisrt -> first (不改也不影响运行)
  if (first_flag == 1)
  {
    first_flag = 0;
    out_last = value;
  }

  // 滤波公式：Out = Last + alpha * (New - Last)
  out = out_last + alpha * (value - out_last);
  out_last = out;

  return out;
}
