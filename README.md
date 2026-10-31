# Hand truck

基于 STM32F407VGTx、STM32 HAL 和 FreeRTOS 的控制板工程，使用 Keil 构建。

## 当前版本

本次备份包含已完成的串口屏分批绘制修复：

- 上电后串口屏全白，等待数据。
- `321+123` 完整显示在屏幕上半部分。
- `321+123+456+654` 上半行显示 `321+123+`，下半行显示 `456+654`。
- 数字按字符分批绘制，每批等待处理时间，避免超长指令导致字符缺失。

重复数据跳过刷新、字体提速和局部刷新仍为待确认方案，尚未实现。

## 工程与固件

- `Hand truck.ioc`：STM32CubeMX 配置。
- `MDK-ARM/HAND TRUCK.uvprojx`：Keil 工程。
- `Core/`：应用与板级代码。
- `Drivers/`、`Middlewares/`：STM32 驱动、CMSIS 和 FreeRTOS，保留各目录中的许可证文件。
- `firmware/HAND_TRUCK.hex`：2026-10-31 编译的当前固件；本次已编译生成，尚未完成烧录后的屏幕实测。
- `DISPLAY_REQUIREMENTS.md`：已记录的显示要求。
- `docs/superpowers/`：修改计划、验证记录和待确认方案。

## 构建与验证

用 Keil 打开 `MDK-ARM/HAND TRUCK.uvprojx`，构建 `Hand truck` 目标。当前本机使用 ARMCLANG V6.16，构建结果为 0 错误、4 条已有警告；工程文件原先记录的编译器版本为 V6.24，请根据本机安装情况选择。

安装 Python 和 GCC 后，在工程根目录执行主机回归测试：

```sh
python tests/run_jc014_tests.py
```

当前 11 项检查全部通过，覆盖绘图内容、上下布局、指令长度、发送等待和 UART 错误恢复。主机测试不能代替烧录后的实物验证。

编译缓存、构建日志和本机调试配置不纳入版本控制。
