## 开发日志

| 时间       | 内容                                   | 备注 |
| ---------- | -------------------------------------- | ---- |
| 2024.11.30 | 新建工程，移植freertos，增加监管机任务 |      |
| 2024.12.3  | 宇树电机多机通信，监管机任务基本完成   |      |
| 2024.12.7  | 达妙电机通信基本完成                   |      |

## 错误日志

| 时间       | 问题                                                         | 解决                                                         |
| ---------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 2024.11.30 | freertos使用sys时钟与hal库使用tim1时钟冲突，HAL_GetTick返回值不变 | hal库使用tim6时钟                                            |
|            | ws2812灯珠信号错误                                           | spi6时钟树配置错误导致波特率错误                             |
| 2024.12.2  | 宇树电机反馈数据包丢包头，高转速下电机卡顿严重               |                                                              |
| 2024.12.7  | 达妙电机FDCAN发送数据包错误，FDCAN数据包长度总为0            | HAL库函数FDCAN_CopyMessageToRAM问题。HAL库版本差异，替换FW_H7_V1.11.2版本HAL库后正常 |

