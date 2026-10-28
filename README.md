# 慧目智行

本仓库为**2026年10月28日的板端版本**，按完整代码提交包的目录组织，包含ARM应用源码、FPGA工程、PCIe驱动、实际使用的模型，以及部署与恢复文件。

代码、模型和配对固件与当日RAR代码包一致。RAR单独交付，仓库直接维护源码文件。对应关系和校验信息见[提交包记录](docs/SUBMISSION_PACKAGE.json)。

## 目录

| 目录 | 内容 |
|---                                           |---                                                    |
| `app/`                                       | ARM应用的22个编译单元、必要头文件、CMake构建入口和测试    |
| `fpga/source/`                               | 当前FPGA镜像对应的采集、DDR、PCIe、按键/LED及必要兼容源码 |
| `fpga/project/`                              | PDS工程、时序约束、管脚文件及当前配对位流                 |
| `deployment/models/`                         | 正式链实际加载的5个RKNN模型                             |
| `deployment/driver/`、`deployment/common/`   | 已验证参考PCIe驱动源码、公共ABI头和ko                    |
| `deployment/runtime/`、`deployment/scripts/` | 原始默认参数、启动、正常停止、键盘桥与部署工具             |
| `deployment/lib/`、`deployment/assets/`      | 运行库和中文字库                                        |
| `deployment/bin/`、`deployment/recovery/`    | 已验收程序和上一软件版文件                               |
| `scripts/`                                   | 源码构建、测试和整个仓库的完整性核验                      |
| `docs/`                                      | 源码来源、构建验证及文件指纹                             |

目录不纳入独立历史实验、C2四路驱动/发送器、未采用模型、训练数据集、采集素材、长时间运行日志和账户凭据。

## 当前功能

- 车道感知：原版TwinLite权重对应的纯Pad模型。
- 交通灯、人车感知：LISA与COCO 320；保留灯复核和ATLAS属性保护。
- 斑马线：几何检测、区域预测、横向提示带及行人区域关联。
- 交互与显示：中文界面、双目SBS显示、A/D变道请求与风险判断。

灯和Twin每2帧运行一次，人车每帧运行一次；A/D请求最长保留2秒。UFLD、实验性CAAM替换、车速、路牌和四路回放保持关闭。

F键沿用原逻辑：**识别为红灯时触发强行驶报警**。本次目录同步没有增加强制行驶状态切换。

### 必要兼容依赖

当前冻结FPGA工程仍实例化旧RTL车道模块并保留相应寄存器，**ARM正式车道判断不使用这些输出，画面车道由TwinLite生成**。为保证工程可重建，保留这些构建依赖；独立旧RTL车道实验未纳入。

ARM源码中的OCR、UFLD、车速等兼容单元也是现有ELF的链接依赖。它们随源码保留，默认不开启相关功能，也不附带未使用权重。

## ARM源码构建

已验证环境为RK3568 / 3568JHF、ARM64、Debian 12、Linux 6.1.99、GCC/G++ 12.2、CMake 3.25、OpenCV 4.6和GTK 2.24。

在配套板端系统准备依赖：

```bash
sudo apt-get update
sudo apt-get install build-essential cmake pkg-config python3 \
  libopencv-dev libopencv-contrib-dev libgtk2.0-dev libfreetype6-dev \
  libqt5widgets5 libqt5gui5 libqt5test5 libqt5opengl5 libx11-6
```

在仓库根目录运行：

```bash
bash scripts/build_arm.sh
```

或分步构建：

```bash
cmake -S app -B build/arm -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build/arm --parallel 1
ctest --test-dir build/arm --output-on-failure
```

输出为`build/arm/adas_app`。C和C++均使用`-O2 -ffp-contract=off`；脚本默认单编译任务，可通过`BUILD_JOBS`设置并行数。

构建和单元测试不自动启动重建APP、不装卸驱动或烧录FPGA，也不替换正式默认程序。重建ELF与原始封存ELF分别记录，不能仅凭源码编译成功视为完成新一轮实板运行验收。

## FPGA工程

- 工具：Pango Design Suite **2022.2-SP6.4**。
- 器件：**PG2L50H，FBG484，速度等级-6**。
- 工程入口：`fpga/project/project.pds`。
- 配套文件：`impl.pds`、`project.fdc`、`device_map/image_pcie_capture.pcf`。

源文件相对位置保持为`project/../source/`，生成目录由PDS重新产生。当前位流与校验文件见`fpga/project/generate_bitstream/`及`deployment/firmware/`。

当前板端位流只曾加载到SRAM，未固化SPI。重新加载与掉电恢复说明见[部署说明](deployment/README.md#fpga与掉电恢复)。

## 部署与运行

从本次版本开始，原仓库根目录的部署文件统一位于`deployment/`。先检查文件，再按[部署说明](deployment/README.md)操作：

```bash
python3 deployment/scripts/verify.py
sudo python3 deployment/scripts/install.py
sudo bash deployment/scripts/start.sh
```

已有程序正在采集时，应先正常退出；安装器会拒绝覆盖活动采集者。目标文件指纹不同时须检查差异，再按部署说明使用带备份的`--replace`。

正常停止：

```bash
sudo bash deployment/scripts/stop.sh
```

原始正式APP的SHA256保持为：

```text
3b1d686cb900673a92a36d9c6038da65c37c2c653e43cccb01784851319a3afd
```

## 校验与已验证结果

```bash
python3 scripts/verify_submission.py
sha256sum -c SHA256SUMS
```

- ARM源码构建通过；4组测试共177项检查通过。
- 22个编译单元的可执行代码段与冻结对象逐项一致；此范围不等同于完整ELF逐字节相同。
- 已验收运行窗口：1280帧、136.3852秒，约9.39 processed FPS；源侧计数约20.6FPS。
- FPGA部分核对了冻结源码、工程输入和配对位流，本次同步未重新综合或烧录。

详细结果见[构建验证](docs/BUILD_VALIDATION.json)、[对象代码比较](docs/OBJECT_CODE_COMPARISON.json)、[ARM源码清单](docs/ARM_SOURCE_MANIFEST.json)和[FPGA源码清单](docs/FPGA_SOURCE_MANIFEST.json)。`SHA256SUMS`对应本次发布的文件内容，修改文件后需相应更新清单。

第三方声明保留在`deployment/THIRD_PARTY_NOTICES.md`与`deployment/licenses/`；系统库版本见`deployment/system-packages.tsv`。
