# SiFli SDK Peripherals

[English](README.md) | [简体中文](README_zh.md)

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](./LICENSE)

## 📖 Overview

**SiFli SDK Peripherals** is a repository dedicated to extending the SiFli SDK with support for various peripheral device drivers. This project aims to provide developers with a collection of high-quality, well-structured, and easy-to-integrate drivers, covering common hardware such as cameras, sensors, displays, audio devices, and Flash modules.

The drivers in this repository are designed to integrate seamlessly into SCons-based build systems and can also be downloaded via the [SiFli Component Manager](https://packages.sifli.com/sifli).

## ✨ Key Features

- **Extensive Hardware Support**: Covers Camera, Sensor, LCD, Touch Panel, Audio, and more.
- **Modular Design**: Independent maintenance for each driver, ensuring low coupling and easy portability.
- **Out-of-the-Box**: Adheres to SiFli SDK standard interfaces, providing standardized configuration (Kconfig) and build scripts (SConscript).
- **Package Manager Support**: Supports packaging and publishing to the SiFli Component Center, enabling one-line installation.

## 📂 Repository Structure

For easier navigation and management, this repository adopts a hierarchical directory structure:

```text
SiFli-SDK-Peripherals/
├── camera/             # Camera drivers
│   ├── bf30a2/         # Example: BF30A2
│   │   ├── src/        # Source code
│   │   ├── inc/        # Header files
│   │   ├── Kconfig     # Configuration options
│   │   └── SConscript  # Build script
│   └── ...
├── sensor/             # Sensor drivers (IMU, Temp/Hum, etc.)
├── lcd/                # Display drivers
├── touch/              # Touch panel drivers
└── ...
```

## 🚀 Installation & Usage

### Method 1: via SiFli Component Manager (Recommended)

For published driver packages, you can install them directly into your project:

1.  Visit [SiFli Packages](https://packages.sifli.com/sifli) to find the required driver.
2.  Run the package management command in your SDK project directory (refer to SDK documentation for specific commands).

### Method 2: Manual Integration

1.  Clone this repository as a submodule in your project or download the specific driver folder.
    ```bash
    git clone https://github.com/OpenSiFli/SiFli-SDK-Periphierals.git
    ```
2.  Place the driver folder into your project's `packages` or custom component directory.
3.  Include the driver path in your project's `SConscript` or enable it via Kconfig.

## 🤝 Contribution Guidelines

We warmly welcome community contributions for new drivers or optimizations!

1.  Fork this repository.
2.  Create a new feature branch based on `main` (`git checkout -b feature/new-driver`).
3.  Commit your changes. Please ensure `Kconfig` and `SConscript` are included to support the build system.
4.  Push to your branch (`git push origin feature/new-driver`).
5.  Create a Pull Request.

## 🐛 Issue Reporting

If you encounter bugs or have feature suggestions, please provide feedback via [GitHub Issues](https://github.com/OpenSiFli/SiFli-SDK-Periphierals/issues).

When reporting, please include:
- Chip Model (e.g., SF32LB55x, SF32LB52x)
- SDK Version
- Driver Name and Version
- Reproduction Steps and Logs

## 📄 License

This project is licensed under the [Apache-2.0](./LICENSE) license.

## 🔗 Resources

- [SiFli SDK Documentation](https://docs.sifli.com/)
- [SiFli Component Manager](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/sf_pkg/index.html)
- [SiFli Community](https://github.com/OpenSiFli)

