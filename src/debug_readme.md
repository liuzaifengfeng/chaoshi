# Debug相关函数实现总结

## 实现的功能

### 1. 串口命令解析函数 (`Task_Debug_Serial0_CMD`)
- 接收串口输入的命令
- 解析GOTOpose和GETpose命令
- 将解析后的命令通过队列传递给调试模式任务

### 2. 调试模式执行函数 (`Task_Debug_Mode`)
- 从队列中接收命令
- 执行GOTOpose命令：调用GotoPose函数移动机器人到指定位置
- 执行GETpose命令：输出当前机器人位置

## 实现细节

### 数据结构
- `DebugCommand_t`结构体：存储命令类型和参数
- `xDebugQueue`队列：用于在任务间传递命令

### 命令格式
- GOTOpose命令：`GOTOpose x y theta`
- GETpose命令：`GETpose`
- GETdist命令：`GETdist`
- EMMpos命令：`EMMpos addr dir clk`
- PWM命令：`PWM addr angle`
- reset命令：`reset`


### 实现步骤
1. 在main.cpp中添加队列定义和命令结构体
2. 在setup()函数中初始化调试队列
3. 实现Task_Debug_Serial0_CMD函数：读取和解析串口命令
4. 实现Task_Debug_Mode函数：执行队列中的命令

## 代码修改

### main.cpp
- 添加了`DebugCommand_t`结构体定义
- 添加了`xDebugQueue`队列定义
- 在setup()函数中初始化队列
- 实现了Task_Debug_Serial0_CMD函数
- 实现了Task_Debug_Mode函数

## 使用方法

1. 确保`isdebug`变量设置为`true`
2. 上传代码到设备
3. 打开串口监视器
4. 发送命令：
   - 移动到指定位置：`GOTOpose 100 200 0`
   - 获取当前位置坐标：`GETpose`
   - 获取原始距离：`GETdist`
   - 设置电机位置：`EMMpos 1 1 3200`
   - 重置机器人位置：`reset`
   - 设置PWM占空比：`PWM 1 50`

## 注意事项

- 命令格式必须正确，否则会提示"Invalid command format"
- 队列容量为20，超过会提示"Failed to send command to debug queue"
- GOTOpose命令的坐标单位为mm，角度单位为度
