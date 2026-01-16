# InfMVS

[![CI](https://github.com/user/InfMVS/actions/workflows/ci.yml/badge.svg)](https://github.com/user/InfMVS/actions/workflows/ci.yml)
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

**InfMVS** 是一个模块化、高性能的 C++ MVS（Multi-View Stereo）重建工具库，采用现代 C++20 标准和函数式编程风格。

## 特性

- 🧩 **模块化架构** - 6 个独立 DLL 模块，按需加载
- ⚡ **高性能** - 零拷贝设计、RAII 资源管理
- 📦 **vcpkg 清单模式** - 可复现的依赖管理
- 🔧 **易于扩展** - 清晰的模块边界和接口

## 模块架构

```
┌─────────────────────────────────────────────────────────────┐
│                     InfMeshPipeline                         │
│                   (端到端重建流水线)                          │
├─────────────────┬─────────────────┬─────────────────────────┤
│   InfTexture    │     InfMesh     │      InfPointCloud      │
│  (纹理处理)      │   (网格处理)     │       (点云处理)         │
├─────────────────┴─────────────────┴─────────────────────────┤
│                         InfIO                               │
│                 (COLMAP/MVS 格式 I/O)                        │
├─────────────────────────────────────────────────────────────┤
│                        InfCore                              │
│              (数学类型、内存、日志、错误处理)                   │
└─────────────────────────────────────────────────────────────┘
```

### 模块说明

| 模块 | 描述 | 主要功能 |
|------|------|----------|
| **InfCore** | 基础设施库 | 数学类型（向量/矩阵）、内存池、日志系统、错误处理 |
| **InfIO** | I/O 库 | COLMAP Text/Binary 读写、图像加载、PLY/OBJ 导出 |
| **InfPointCloud** | 点云处理库 | 统计滤波、体素下采样、法向估计、ICP 配准 |
| **InfMesh** | 网格处理库 | 半边数据结构、网格简化、细分、孔洞修复 |
| **InfTexture** | 纹理处理库 | UV 展开、多视角纹理映射、Texture Atlas 生成 |
| **InfMeshPipeline** | 流水线库 | 端到端重建、批处理、参数配置 |

## 快速开始

### 依赖

- CMake 3.20+
- C++20 编译器 (MSVC 2022 / GCC 11+ / Clang 14+)
- vcpkg

### 构建

```bash
# 克隆仓库
git clone https://github.com/user/InfMVS.git
cd InfMVS

# 配置（vcpkg 自动安装依赖）
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake

# 构建
cmake --build build --config Release -j

# 测试
ctest --test-dir build --output-on-failure -C Release
```

### Windows (Visual Studio)

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 `
    -DCMAKE_TOOLCHAIN_FILE="$env:VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake"
cmake --build build --config Release
```

## 使用示例

```cpp
#include <InfIO/colmap_reader.hpp>
#include <InfPointCloud/filters.hpp>
#include <InfMesh/reconstruction.hpp>

int main() {
    // 读取 COLMAP 模型
    auto model = inf::io::read_colmap_text("path/to/model");
    
    // 点云滤波
    auto filtered = inf::pc::statistical_outlier_removal(model.points, 20, 2.0);
    
    // 网格重建
    auto mesh = inf::mesh::poisson_reconstruction(filtered);
    
    return 0;
}
```

## 目录结构

```
InfMVS/
├── modules/                    # 模块源码（内部库，hpp/cpp 放一起）
│   ├── InfCore/
│   │   ├── CMakeLists.txt
│   │   ├── core.hpp            # 内部实现
│   │   ├── core.cpp
│   │   └── InfCore.hpp         # 公开接口（唯一对外入口）
│   ├── InfIO/
│   ├── InfPointCloud/
│   ├── InfMesh/
│   ├── InfTexture/
│   └── InfMeshPipeline/
├── apps/                       # CLI 应用程序
├── tests/                      # 测试代码
├── cmake/                      # CMake 模块
├── vcpkg.json                  # vcpkg 依赖清单
├── CMakeLists.txt              # 根构建脚本
└── README.md
```

## 开发

### 编码规范

- 文件名: `snake_case`
- 类型名: `PascalCase`
- 函数名: `lower_snake_case`
- 私有成员: `snake_case_` (尾部下划线)

详见 [.cursor/rules](.cursor/rules)

### 构建选项

| 选项 | 默认值 | 描述 |
|------|--------|------|
| `INFMVS_BUILD_APPS` | ON | 构建 CLI 应用 |
| `INFMVS_BUILD_TESTS` | ON | 构建测试 |
| `INFMVS_BUILD_SHARED` | ON | 构建动态库 (DLL) |

## 路线图

- [x] 项目骨架搭建
- [ ] InfCore 基础设施
- [ ] InfIO COLMAP 读写
- [ ] InfPointCloud 基础滤波
- [ ] InfMesh 网格数据结构
- [ ] InfTexture UV 展开
- [ ] InfMeshPipeline 端到端流水线

## 许可证

[MIT License](LICENSE)

## 致谢

- [COLMAP](https://colmap.github.io/) - SfM/MVS 参考实现
- [OpenMVS](https://github.com/cdcseacave/openMVS) - 架构参考
- [vcpkg](https://vcpkg.io/) - 包管理
