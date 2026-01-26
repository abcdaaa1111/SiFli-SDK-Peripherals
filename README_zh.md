# SiFli SDK 外设驱动库 (SiFli SDK Peripherals)

[English](README.md) | [简体中文](README_zh.md)

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](./LICENSE)

## 📖 概述

**SiFli SDK Peripherals** 是专为 SiFli 系列芯片及 SDK 设计的扩展外设驱动仓库。本项目旨在为开发者提供一系列高质量、结构清晰且易于集成的外设驱动程序，涵盖摄像头、传感器、显示屏、音频设备及 Flash 等常见硬件。

本仓库中的驱动均经过精心设计，可无缝集成到基于 SCons 的构建系统中，也可通过 [SiFli 组件管理器](https://packages.sifli.com/sifli) 下载使用。

## ✨ 主要特性

- **广泛的硬件支持**：涵盖 Camera, Sensor, LCD, Touch Panel, Audio 等多种外设。
- **模块化设计**：每个驱动独立维护，低耦合，易于移植和裁剪。
- **开箱即用**：遵循 SiFli SDK 标准接口，提供标准化的配置 (Kconfig) 和构建脚本 (SConscript)。
- **包管理器支持**：支持打包发布至 SiFli 组件中心，一行命令即可安装。

## 📂 仓库结构

为了方便导航和管理，本仓库采用分层目录结构：

```text
SiFli-SDK-Peripherals/
├── camera/             # 摄像头驱动
│   ├── bf30a2/         # 示例: BF30A2
│   │   ├── src/        # 源代码
│   │   ├── inc/        # 头文件
│   │   ├── Kconfig     # 配置选项
│   │   └── SConscript  # 构建脚本
│   └── ...
├── sensor/             # 传感器驱动 (IMU, 温湿度等)
├── lcd/                # 显示屏驱动
├── touch/              # 触摸屏驱动
└── ...
```

## 🚀 安装与使用

### 方法一：通过 SiFli 组件管理器 (推荐)

如果是已发布的驱动包，你可以直接在你的项目中安装：

1.  访问 [SiFli Packages](https://packages.sifli.com/sifli) 查找所需驱动。
2.  在你的 SDK 工程目录中运行包管理命令（具体命令参考 SDK 文档）。


## 🤝 贡献指南

我们非常欢迎社区贡献新的驱动或优化现有代码！

1.  Fork 本仓库。
2.  基于 `main` 分支创建一个新的特性分支 (`git checkout -b feature/new-driver`).
3.  提交你的更改。请确保包含 `Kconfig` 和 `SConscript` 以支持构建系统。
4.  Push 到你的分支 (`git push origin feature/new-driver`).
5.  Create a Pull Request.

## 🐛 问题反馈

如果您在使用过程中遇到 Bug 或有新的功能建议，请通过 [GitHub Issues](https://github.com/OpenSiFli/SiFli-SDK-Periphierals/issues) 反馈。

反馈时请包含：
- 芯片型号 (例如 SF32LB55x, SF32LB52x)
- SDK 版本
- 驱动名称及版本
- 复现步骤及日志

## 📄 许可证

本项目采用 [Apache-2.0](./LICENSE) 许可证。

## 🔗 相关资源

- [SiFli SDK 文档](https://docs.sifli.com/)
- [SiFli 组件管理器](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/sf_pkg/index.html)
- [SiFli 社区](https://github.com/OpenSiFli)

