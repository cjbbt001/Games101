# GAMES101 作业集

GAMES101 现代计算机图形学入门课程作业，C++17，CMake 构建。

## 用户画像

- 非计算机专业，有 C/Python 基础但不系统
- 学习目标：打牢图形学基础，后续学 TA / Shader / UE
- 讲解要求：不默认懂 C++，解释语法、数学直觉、图形学概念
- 每次只推进一个小任务，先讲目标再讲代码

## 目录结构

| 目录 | 主题 | 已完成 | 依赖 |
|------|------|--------|------|
| `pa0/` | 热身：Eigen 线性代数 | 是 | Eigen3 |
| `Assignment1/Code/` | 光栅化：MVP 变换、Rodrigues 旋转 | **是** | OpenCV, Eigen |
| `Assignment2/Code/` | 光栅化：深度缓冲、MSAA | 当前 | OpenCV, Eigen |
| `Assignment3/Code/` | 着色：Blinn-Phong、纹理、法线贴图 | 否 | OpenCV, Eigen |
| `Assignment4/Code/` | 几何：贝塞尔曲线（C++14） | 否 | OpenCV |
| `Assignment5/Code/` | 光线追踪：Whitted-style | 否 | 无第三方 |
| `Assignment6/Code/` | 光线追踪：BVH 加速结构 | 否 | 无第三方 |
| `Assignment7/Code/` | 路径追踪：Monte Carlo | 否 | 无第三方 |
| `Assignment8/Code/` | 绳子物理模拟，实时渲染 | 否 | OpenGL, Freetype, 内嵌 CGL/GLFW |

## 构建方式

```bash
cd AssignmentX/Code   # 或 pa0/
mkdir build && cd build
cmake ..
make -j4
```

如果 build 已存在：`cd build && make`

Assignment5-7 输出可执行文件 `RayTracing`，Assignment8 输出到源码根目录。

## 已知 Bug / 陷阱

- **pa0/CMakeLists.txt**：`include_directories(EIGEN3_INCLUDE_DIR)` 缺少 `${}` 展开，字符串字面量而非变量。能编译是因为系统 Eigen 在默认搜索路径。
- **Assignment5** 启用 `-fsanitize=undefined`（UBSan），运行时未定义行为会报错终止，是预期行为。
- **Assignment8** 内嵌 CGL/GLEW/GLFW，无需系统安装；`BUILD_LIBCGL=ON` 为默认值。
- Assignment1-3 的 CMakeLists 硬编码 `/usr/local/include`，非标准安装环境可能找不到 Eigen 头文件，需手动添加路径。

## 学生修改区域

各作业源码中用 `// Edit begin` / `// Edit end` 标注需填写的函数。

## C++ 标准

- pa0、Assignment1-3、Assignment5-7：C++17
- Assignment4：C++14
- Assignment8：C++11（`-std=gnu++11`）

## 已完成作业的代码要点

### pa0
- 齐次坐标 `(x,y,1)` 统一处理旋转+平移
- 矩阵乘法从右往左读（先旋转再平移）
- `pa0/build/` 已有编译产物

### Assignment1
- 已做：`get_model_matrix()`、`get_projection_matrix()`、`get_rotation()`（Rodrigues 提高题）
- `get_model_matrix`：绕 Z 轴旋转矩阵，角度转弧度后用 `cos/sin`
- `get_projection_matrix`：透视投影 = 透视挤压 → 正交平移 → 正交缩放
  - `zNear`/`zFar` 传入正数，转负数：`n=-zNear, f=-zFar`
  - `M_persp_to_ortho` 最后一行 `(0,0,1,0)`
- `get_rotation`：Rodrigues 公式 `R = cosθ I + (1-cosθ) aaᵀ + sinθ N`
- `build/` 已有编译产物，运行：`./Rasterizer -r 20 output.png`

## 环境

- OS: Windows 11 + WSL2 Ubuntu
- 用户名: cjbbt，机器名: LAPTOP-J8JH4MLG
- VS Code + Remote-WSL
- Obsidian vault: `D:\Obsidian\obsidian`，WSL 软链接指向 `~/obsidian` → `/mnt/d/Obsidian/obsidian`
- OpenCode TUI 快捷键：`Ctrl+P` 命令列表

## Obsidian 笔记路径

Games 101 笔记根目录：`~/obsidian/TA/Games 101/`

## 已学课程内容

- 第 2 课：线性代数（向量、点乘、叉乘、矩阵）
- 第 3-4 课：变换与观测（MVP 矩阵、齐次坐标、透视投影、Rodrigues）
- 第 5-6 课：光栅化与抗锯齿（采样、走样、MSAA、Z-Buffer）
- 第 7-9 课：着色与纹理映射（Blinn-Phong、重心坐标、Mipmap）
