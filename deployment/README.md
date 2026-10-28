# 慧目智行

本仓库为 **2026年10月28日的板端版本**：原版 TwinLite 权重对应的纯 Pad 模型、交通灯与人车感知、灯复核、ATLAS v4、斑马线区域及预测、中文界面和 A/D 变道交互。

主程序保存已验收的板端 ARM64 ELF，完整 SHA256 为：

```text
3b1d686cb900673a92a36d9c6038da65c37c2c653e43cccb01784851319a3afd
```

本部署子目录提供程序、实际加载的模型、运行依赖、部署工具和参考驱动源码。主应用的完整C++源码与构建入口位于上一级的`app/`，整体目录见[仓库首页](../README.md)。

## 功能与默认配置

| 模块 | 当前配置 |
|---|---|
| 车道感知 | TwinLite，纯 Pad 模型；每2帧运行一次 |
| 交通灯检测 | LISA，7类；每2帧运行一次 |
| 人车检测 | COCO 320，80类模型；每帧运行一次 |
| 灯复核 | `light_classifier_cn.rknn` |
| 灯属性辅助 | ATLAS v4，128输入、FP16；保留当前决策保护逻辑 |
| 斑马线 | 左目图像几何检测、区域预测、横向提示带及行人区域关联；无额外NPU模型 |
| 变道交互 | 请求最长保留2000ms；临时目标定位证据不足时等待更新；模拟车道序号不直接充当实际道路边缘 |
| 判断约束 | 实线、近车、灯态、前方风险、数据有效期等保护保持当前验收配置 |
| 显示 | Qt主线程显示、中文界面；双目SBS输入 |
| 当前关闭 | RTL车道感知/绘线、UFLD、实验性CAAM替换、车速、路牌、四路回放 |

**不收录** RTL车道源码/工程、C2四路驱动和发送器、模型训练材料、未采用候选模型、诊断采集帧、大体积运行CSV及历史实验目录。FPGA目录只保留当前配对的已验证位流与校验文件。

## 目录

```text
bin/           当前默认主程序
models/        实际加载的5个RKNN模型
runtime/       原始默认入口、正常停止/回退控制器、日志监督器、键盘桥
lib/           固定版本的RKNN、RGA和当前ELF需要的HyperLPR动态库
assets/        中文界面使用的文泉驿字库
driver/        已加载build-id匹配的参考PCIe驱动ko及源码
common/        正式驱动所需的帧ABI公共头
firmware/      当前无硬件OSD位流和匹配smsk；仅作SRAM恢复
recovery/      上一正式软件版本及原入口，支持软件回退
desktop/       与板上一致的桌面入口
tools/         正常GUI退出工具及其源码
scripts/       安装、启动、正常停止和只读核验
licenses/      从板端保留的第三方声明
ARTIFACTS.json 文件来源、目标路径、权限、大小和SHA256
RELEASE.json   默认参数、版本身份、驱动身份和已验证性能范围
SHA256SUMS     仓库文件校验清单
```

历史路径中个别目录带有`caam`或`four_capture`字样，安装清单只使用其中的正常退出工具或键盘包装脚本。它们不表示对应实验功能已启用，仓库内没有复制这些实验工程。

## 运行前提

- Rockchip **RK3568 / 3568JHF**，ARM64，带已配置的FPGA/PCIe采集硬件。
- 板端已验证系统为 Debian 12、Linux **6.1.99**。预编译驱动仅与对应内核配套。
- 采集输入为1280×360 RGB565 SBS：左右各640×360，2个缓冲槽，921600字节/帧。
- X11显示会话`:0`和用户`linaro`存在，`/home/linaro/.Xauthority`有效。
- 正式PCIe驱动已加载，`/dev/pango_pci_driver`存在。安装脚本不会自动装卸驱动或烧录FPGA。

OpenCV 4.6、Qt5、Python3、X11等使用板端系统软件包；实际版本见[system-packages.tsv](system-packages.tsv)。在配套Debian镜像上可准备：

```bash
sudo apt-get update
sudo apt-get install python3 util-linux libopencv-dev libopencv-contrib-dev \
  libqt5widgets5 libqt5gui5 libqt5test5 libqt5opengl5 libgtk2.0-0 \
  libfreetype6 libx11-6
```

`libhyperlpr3.so`仍是当前封存ELF的动态链接依赖，因此保留该库；这不意味着车牌识别在当前默认模式下启用。操作系统、内核、Qt/OpenCV及厂商NPU驱动需要配套镜像，仓库不是整板系统镜像。

在配套内核的全新冷启动环境中，如尚未加载正式驱动，应先完成安装、确认没有遗留采集者，再手动加载随包模块：

```bash
sudo insmod /userdata/team666_default_20261028/driver/pango_pci_driver.ko
```

已加载且build-id匹配时无需重复装卸。驱动源码的公共头保存在`common/`，与`driver/`保持原相对位置；更换内核时需使用对应内核构建树重新编译和验收，不能直接复用此ko。

## 校验与安装

在板端克隆仓库，先核验文件：

```bash
git clone https://github.com/koalwijoing/team666.git
cd team666/deployment
python3 scripts/verify.py
sha256sum -c SHA256SUMS
```

安装前通过界面`Q`正常退出，或在已有本版本的板上使用正常停止脚本。安装器发现仍有采集FD或映射时会拒绝覆盖：

```bash
sudo bash scripts/stop.sh
sudo python3 scripts/install.py
```

全新板上没有既有APP时，直接运行安装器即可。若目标文件已存在且指纹不同，先检查差异，再执行：

```bash
sudo python3 scripts/install.py --replace
```

`--replace`把被替换文件备份到`/userdata/team666_install_backups/<时间>/`。安装只复制清单指定的文件，不清空旧目录，不改变采集参数，不启动APP，不自动设置开机自启，也不装卸驱动或加载位流。

需要先检查安装结果而不写真实系统路径时，可使用隔离目录：

```bash
python3 scripts/install.py --root /tmp/team666-stage
python3 scripts/verify.py --installed-root /tmp/team666-stage
```

软件目标路径保持当前板端路径，避免改变封存ELF的硬编码模型路径和原始默认入口。具体映射由`ARTIFACTS.json`逐文件列出。

## 启动、按键与停止

```bash
sudo bash scripts/start.sh
sudo python3 scripts/verify.py --board
```

启动脚本同时启动APP与原键盘桥。已存在的“P3智能驾驶（正式默认）”桌面入口保留原行为，只启动APP；键盘桥需已运行或通过上述脚本启动。只读核验检查唯一采集进程、帧推进、模型/程序指纹、默认参数、键盘、驱动build-id和当前启动的位流配对证明，不开启第二个采集者。

| 按键 | 功能 |
|---|---|
| W | 开始/停止模拟驾驶状态 |
| S | 循环模拟车道位置 |
| A / D | 左/右变道请求 |
| F | 识别为红灯时触发强行驶报警 |
| Q / Esc | APP正常退出；Q/Esc由APP处理，键盘桥不注入这两个键 |

正常停止APP和键盘桥：

```bash
sudo bash scripts/stop.sh
```

停止控制器核对PID与可执行文件，通过GUI请求退出，并检查采集FD/映射释放。若正常退出未确认会报错；不要用强杀进程代替采集停止确认。
## FPGA与掉电恢复

当前配对镜像为：

| 文件 | SHA256 |
|---|---|
| `firmware/current_noosd.sbit` | `a98fedea197cad365acd18af242dbe58bbcc1f082336b04b2239b8ad54e6e962` |
| `firmware/current_noosd.smsk` | `64489c656be5911510e7c83d4e380fd3cb88f7af02acbc651c0fa77220bc5f17` |

此镜像保留当前采集、PCIe及按键/LED接口，关闭硬件画面OSD。它只曾加载到**SRAM，未固化SPI**。软件安装不包含FPGA烧录操作；重新加载须使用匹配板卡和PDS/JTAG流程，先正常停止所有硬件访问进程，完成实际program/verify与PCIe恢复后再启动软件。

默认入口仅在`/run/codex_p3_noosd_a98fedea.verified`与**当前boot_id+完整位流SHA**匹配时设置`P3_HW_OSD=0`。证明失效时自动恢复Telea遮挡处理。不要复制旧`/run`文件或仅凭文件名制造配对证明；掉电后须重新核实实际加载结果。

## 性能与验收范围

- 已验证窗口：1280帧，136.3852秒，**9.3852 processed FPS**；单帧耗时p95为187.1923ms。
- 源侧计数约**20.6FPS**。这是源计数，与完整算法处理帧率不同。
- 变道交互的49项原版兼容检查和54项新交互检查在PC与ARM均通过，共103种检查。
- 发布包已进行Windows与ARM Linux隔离安装、冲突拒绝/替换备份、脚本语法及动态库解析核验；发布核验记录见[docs/VALIDATION.json](docs/VALIDATION.json)。
日志写到`/userdata/claude_qt_async_default_20261021/runs/`，逐次运行隔离；控制台日志轮转，逐帧CSV仍会增长。运行日志、PID、锁文件和本次启动的FPGA证明均不放入Git仓库。
