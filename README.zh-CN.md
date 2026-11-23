# T100 激光雷达协议工具包

[English](README.md) · [完整协议](docs/protocol.zh-CN.md)

依据公开协议独立重写的 Python 工具，包含控制帧编解码、点云和 IMU 解码、参数读写、采样启停、角度设置和固件写入模式入口指令。提供本地 UDP 模拟雷达与浏览器预览。

演示中的房间、点云及 IMU 数据均为合成数据。

## 打开预览

Mac 上双击 `Start Demo.command`，或者执行：

```sh
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -e .
.venv/bin/t100 demo --port 18766
```

浏览器打开 `http://127.0.0.1:18766`。程序需要 Python 3.9 及以上，运行时只使用 Python 标准库。Windows 对应命令在 `.venv\Scripts` 目录中。

预览支持采样启停、0–360° 单窗口设置、参数回读、3D 旋转和俯视、IMU、完整收发报文、CSV/JSON 导出和固件写入模式入口。模拟设备与网页均只绑定本地回环地址。

## 获取点云

```sh
t100 discover --seconds 5
t100 --ip 192.0.2.10 capture --bind 192.0.2.50 --seconds 10 --pcap capture.pcap
```

上面的地址仅为文档示例，实际使用时替换为设备和接收端地址。被动收包不会自动启动雷达，也不会自动修改网络配置。默认监听点云 `60001`、IMU `60003`，均可通过参数更改；设备的真实目的地址需另行核对。

```sh
t100 --ip 192.0.2.10 status
t100 --ip 192.0.2.10 read pcl_data_type
t100 --ip 192.0.2.10 --trace start
t100 --ip 192.0.2.10 stop
```

目标状态查询、实际状态推送和真实数据流分别观察。ACK 或参数读回本身不能证明电机、点云覆盖和重启结果。

## 更改角度

```sh
t100 --ip 192.0.2.10 roi
t100 --ip 192.0.2.10 roi --start 0 --end 180 --backup roi-before.json
t100 --ip 192.0.2.10 roi --disable --backup roi-before-disable.json
```

修改前保存 11 项 ROI 原值，先关闭窗口，逐项写入并回读，最后写入使能位。备份文件必须是新文件，避免覆盖先前记录。部分写入失败时，先核查状态，再决定如何恢复。底层 `write` 命令不代替调用方的配置备份。

## 进入固件写入模式

只生成完整请求帧：

```sh
t100 frame enter-write-mode --sequence 0x4810
```

结果：`55 0D 04 33 28 AD 10 48 40 00 07 76 F9`。

向实际设备发送进入固件写入模式的请求：

```sh
t100 --ip 192.0.2.10 enter-write-mode
```

网页的“进入固件写入模式”按钮向本地模拟设备发送 `00/07`。“恢复模拟采样”重置本地模型。上述 CLI 命令会向指定设备发送入口请求。

## 其他文件

- `extras/t100_protocol.h`：独立 C 控制帧及 CRC 辅助函数。
- `extras/wireshark/t100.lua`：Wireshark 字段解析插件，显示 CRC 字段，不执行 CRC 校验。
- `docs/`：公开协议快照与来源记录。
- `tests/`：公开报文样例、异常报文、UDP 模拟设备和本地服务测试。

PCAP 记录保留 UDP 载荷、接收时间和端点，IPv4/UDP 头由程序重建，不能当作完整链路层抓包。网页 CSV 是最近两秒内的有效模拟点。没有加入 ROS2、厂商 SDK 或未实现的平台支持描述。
