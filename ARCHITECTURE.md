# ROS2 Chat Client - 动态库架构说明

## 架构概览

该项目已成功转换为模块化架构，核心的 `ChatNode` 类现在作为**共享动态库**发布，可被多个应用程序使用。

```
┌─────────────────────────────────────────────────────────┐
│         internet_chat_client Package                    │
├─────────────────────────────────────────────────────────┤
│                                                          │
│  动态库层 (libchat_node_lib.so)                         │
│  ┌──────────────────────────────────┐                   │
│  │  ChatNode 类                     │                   │
│  │  - ROS2 通信逻辑                  │                   │
│  │  - 消息发布/订阅                  │                   │
│  │  - 群组管理                       │                   │
│  └──────────────────────────────────┘                   │
│           ▲                    ▲                         │
│           │                    │                         │
│  ┌────────┴──────┐    ┌────────┴──────────┐             │
│  │  CLI 应用      │    │  GUI 应用 (wxWidgets) │          │
│  │  chat_node     │    │  chat_gui          │          │
│  │                │    │                    │          │
│  │ 命令行交互      │    │ 图形界面交互        │          │
│  └────────────────┘    └────────────────────┘          │
│                                                          │
└─────────────────────────────────────────────────────────┘
```

## 文件结构

```
internet_chat_client/
├── CMakeLists.txt              # 编译配置（支持动态库 + 两个应用）
├── package.xml
├── include/
│   └── internet_chat_client/
│       └── chat_node.hpp       # ChatNode 类声明
├── src/
│   ├── chat_node_impl.cpp      # ChatNode 实现（编译为动态库）
│   ├── chat_node.cpp           # CLI 应用（使用动态库）
│   └── chat_gui_main.cpp       # GUI 应用（使用动态库 + wxWidgets）
└── README.md
```

## 构建产物

编译后在 `install/` 目录中：

```
install/internet_chat_client/lib/
├── libchat_node_lib.so         # 动态库
├── chat_node                   # CLI 可执行文件
└── chat_gui                    # GUI 可执行文件
```

## 关键特性

### 1. 动态库模块化
- **chat_node_lib**: 包含所有 ROS2 聊天功能
- 可被多个应用程序独立链接和使用
- 便于代码复用和维护

### 2. 线程安全设计
```cpp
// 消息回调使用函数指针，支持任意接收处理
using MessageCallback = std::function<void(const std::string& sender, 
                                           const std::string& timestamp, 
                                           const std::string& text)>;

void set_message_callback(MessageCallback callback);
```

### 3. CLI 应用 (chat_node)
- 保持原有命令行界面
- 功能：
  - `/join <group>` - 切换群组
  - `/help` - 显示帮助
  - `/exit` 或 `/quit` - 退出

### 4. GUI 应用 (chat_gui)
- 基于 wxWidgets 3.0 开发
- 功能：
  - 图形化消息显示
  - 群组选择和切换
  - 实时消息接收和发送
  - 友好的用户界面

## 运行方式

### 编译
```bash
cd ~/ros2_ws
colcon build --packages-select internet_chat_client
source install/setup.bash
```

### 运行 CLI 版本
```bash
ros2 run internet_chat_client chat_node
```

### 运行 GUI 版本
```bash
ros2 run internet_chat_client chat_gui
```

## 代码使用示例

如果要在其他项目中使用 `ChatNode` 库：

```cpp
#include "internet_chat_client/chat_node.hpp"

// 创建 ChatNode
auto chat_node = std::make_shared<ChatNode>("MyUsername");

// 设置消息回调
chat_node->set_message_callback([](const std::string& sender, 
                                    const std::string& timestamp, 
                                    const std::string& text) {
    std::cout << "[" << timestamp << "] " << sender << ": " << text << std::endl;
});

// 加入群组
chat_node->join_group("my_group");

// 发送消息
chat_node->send_message("Hello, everyone!");

// 获取当前信息
std::cout << "User: " << chat_node->get_username() << std::endl;
std::cout << "Group: " << chat_node->get_current_group() << std::endl;

// 运行
rclcpp::spin(chat_node);
```

## CMakeLists.txt 集成示例

```cmake
# 在你的项目中
find_package(internet_chat_client REQUIRED)

add_executable(my_chat_app my_app.cpp)
target_link_libraries(my_chat_app chat_node_lib)
ament_target_dependencies(my_chat_app rclcpp std_msgs)
```

## 优势总结

✅ **代码复用**: ChatNode 逻辑独立，可被多个 UI 应用使用  
✅ **模块化设计**: 清晰的依赖关系和职责分离  
✅ **易于扩展**: 可轻松添加新的应用（Web UI、移动端等）  
✅ **线程安全**: 使用回调机制和互斥锁保证线程安全  
✅ **向后兼容**: 保留原有 CLI 应用，同时支持新的 GUI  
✅ **跨平台**: wxWidgets 支持 Windows、Linux、macOS

## 故障排除

### wxWidgets 未找到
```bash
# Ubuntu/Debian
sudo apt-get install libwxgtk3.0-gtk3-dev

# Fedora/CentOS
sudo dnf install wxGTK-devel

# macOS
brew install wxwidgets
```

### 动态库未找到错误
```bash
# 确保已 source setup.bash
source ~/ros2_ws/install/setup.bash

# 检查库路径
echo $LD_LIBRARY_PATH
```

### 运行时链接问题
```bash
# 使用完整路径运行
LD_LIBRARY_PATH=~/ros2_ws/install/internet_chat_client/lib:$LD_LIBRARY_PATH \
  ~/ros2_ws/install/internet_chat_client/lib/internet_chat_client/chat_gui
```

## 下一步改进

1. 📱 创建移动端应用（使用相同的 ChatNode 库）
2. 🌐 开发 Web 前端（通过 REST API 与 ChatNode 通信）
3. 🔐 添加加密功能
4. 💾 实现消息持久化存储
5. 🎨 增强 GUI 样式和主题支持
