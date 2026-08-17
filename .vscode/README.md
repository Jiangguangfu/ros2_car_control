# VS Code / Cursor WSL 调试配置

当前工作区配置用于：

- 在 WSL/Ubuntu 中使用 `arm-none-eabi-gcc` 编译
- 启动 Windows 侧的 `JLinkGDBServerCL.exe`
- 让 WSL 中的 GDB 通过 `127.0.0.1` 连接 Windows 侧 J-Link GDB Server

## WSL 网络设置

打开 Windows 的 **适用于 Linux 的 Windows 子系统** 设置应用，进入 **网络** 页面。

推荐设置：

- 网络模式：`Mirrored`
- Hyper-V 防火墙：开启
- Localhost 转发：开启
- 主机地址环回：开启
- 自动代理：如果 Windows 网络环境需要代理，则开启

这套调试配置最关键的是 **Localhost 转发**。它允许 WSL 中的 GDB 连接 Windows 侧 J-Link GDB Server：

```text
127.0.0.1:2331
```

修改 WSL 网络设置后，重启 WSL：

```powershell
wsl --shutdown
```

然后重新打开 Ubuntu/WSL 终端和 Cursor 工作区。

## Ubuntu 软件包

安装 ARM 嵌入式工具链和 GDB：

```bash
sudo apt update
sudo apt install gcc-arm-none-eabi binutils-arm-none-eabi gdb-multiarch cmake ninja-build
```

检查工具是否可用：

```bash
arm-none-eabi-gcc --version
arm-none-eabi-objdump --version
arm-none-eabi-nm --version
gdb-multiarch --version
```

本项目默认使用 `gdb-multiarch`，因为部分 Ubuntu 版本不再提供 `arm-none-eabi-gdb`。

## 项目配置

工具路径集中配置在 `.vscode/settings.json`：

```jsonc
"bms.armToolchainBin": "/usr/bin",
"bms.gdbPath": "/usr/bin/gdb-multiarch",
"bms.jlinkGdbServerCli": "/mnt/c/Program Files/SEGGER/JLink_V910/JLinkGDBServerCL.exe",
"bms.jlinkDevice": "STM32U385CGUx",
"bms.jlinkRtosPlugin": "GDBServer/RTOSPlugin_FreeRTOS"
```

如果你的 SEGGER 安装路径不同，请修改 `bms.jlinkGdbServerCli`。

## 调试端口

当前工作区使用固定的 J-Link 端口：

```text
GDB:    2331
SWO:    2332
Telnet: 2333
```

如果这些端口被占用，请在 `.vscode/settings.json` 中修改。

## 调试配置

可用的调试配置：

- `J-Link Flash + GDB Debug (Inc Build)`：增量构建、烧录，然后进入调试
- `J-Link Flash + GDB Debug (Full Build)`：清理后完整重建、烧录，然后进入调试
- `J-Link Attach`：连接到正在运行的目标板，不执行构建

J-Link Server 启动脚本是 `.vscode/start-jlink-gdbserver.sh`。它会启动 Windows 侧 J-Link Server，并等待输出：

```text
Waiting for GDB connection
```

Server 日志写入：

```text
build/jlink-gdbserver.log
```

## FreeRTOS 线程调试

三个 Debug 配置都声明了：

```jsonc
"rtos": "FreeRTOS"
```

同时 J-Link Server 启动时会传入：

```text
-rtos GDBServer/RTOSPlugin_FreeRTOS
```

这样进入断点后可以在调试视图中查看 FreeRTOS 线程信息。

## 常见问题

### GDB 连接超时（Connection timed out）

1. 确认 WSL **Localhost 转发** 已开启（见上文）
2. 查看 `build/jlink-gdbserver.log`：
   - `Connecting to J-Link failed`：检查 J-Link 是否插好、目标板是否上电
   - `Could not connect to target`：检查 SWD 接线，或降低 `bms.jlinkSpeed`（如 `100`）
3. 确认 `bms.jlinkDevice` 为 `STM32U385CGUx`（与 CubeMX 中 MCU 名称一致）

### 手动测试 J-Link Server

```bash
bash .vscode/start-jlink-gdbserver.sh \
  "$(pwd)" \
  "/mnt/c/Program Files/SEGGER/JLink_V910/JLinkGDBServerCL.exe" \
  "STM32U385CGUx" "2331" "2332" "2333" \
  "GDBServer/RTOSPlugin_FreeRTOS" "1000"
```

成功时应看到 `J-Link GDB Server is ready on port 2331`。
