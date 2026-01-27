# 快速使用指南

## ✅ 已完成的工作

### 1. 动态库模块化
✓ 将 `ChatNode` 类提取为独立的**共享动态库** (`libchat_node_lib.so`)  
✓ 创建了头文件 `chat_node.hpp` 用于公开 API  
✓ 分离实现到 `chat_node_impl.cpp`

### 2. GUI 应用开发
✓ 基于 wxWidgets 创建了完整的图形化聊天应用 (`chat_gui`)  
✓ 支持实时消息显示、群组切换、消息发送  
✓ 实现了线程安全的消息处理

### 3. 构建系统优化
✓ 更新 CMakeLists.txt 支持动态库和两个应用的编译  
✓ 自动检测 wxWidgets 依赖  
✓ 正确的头文件安装配置

---

## 📦 构建产物

编译成功！生成了：

```
libchat_node_lib.so  (5.3 MB)  ← 动态库，包含核心聊天逻辑
├─ chat_node        (77 KB)    ← CLI 应用（旧版）
└─ chat_gui         (383 KB)   ← GUI 应用（新版）✨
```

---

## 🚀 运行方式

### 编译
```bash
cd ~/ros2_ws
colcon build --packages-select internet_chat_client
source install/setup.bash
```

### 运行 CLI 版本（原有功能）
```bash
ros2 run internet_chat_client chat_node
```

### 运行 GUI 版本（新）✨
```bash
ros2 run internet_chat_client chat_gui
```

---

## 🏗️ 架构亮点

### 模块设计
```
libchat_node_lib.so
│
├── ChatNode 类（所有 ROS2 逻辑）
│   ├── join_group()
│   ├── send_message()
│   ├── set_message_callback()
│   └── ...
│
├─ 可被任何应用使用
│  ├─ chat_node (CLI)
│  ├─ chat_gui (wxWidgets GUI)
│  ├─ 你的自定义应用
│  └─ Web 服务 (未来)
```

### 线程安全
- ROS2 spin 运行在后台线程
- 消息通过线程安全的回调机制传递
- GUI 事件循环独立运行

---

## 📝 代码示例

### 在你的项目中使用 ChatNode

```cpp
#include "internet_chat_client/chat_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // 创建节点
    auto chat = std::make_shared<ChatNode>("MyApp");
    
    // 设置消息处理回调
    chat->set_message_callback([](const std::string& sender, 
                                   const std::string& time, 
                                   const std::string& msg) {
        std::cout << "[" << time << "] " << sender << ": " << msg << std::endl;
    });
    
    // 加入群组
    chat->join_group("tech_team");
    
    // 后台运行 ROS2
    std::thread spin_thread([&chat]() {
        rclcpp::spin(chat);
    });
    
    // 你的自定义逻辑...
    chat->send_message("Hello from custom app!");
    
    // 清理
    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
```

### CMakeLists.txt 配置

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# 你的应用
add_executable(my_app src/my_app.cpp)
target_link_libraries(my_app chat_node_lib)
ament_target_dependencies(my_app rclcpp)
```

---

## 🎯 可以实现的功能

现在基于这个动态库架构，你可以轻松实现：

1. **Web 前端** - Python Flask/FastAPI + libchat_node_lib
2. **移动应用** - 通过 ROS bridge 连接
3. **消息持久化** - 添加数据库支持
4. **加密通信** - 集成到库中
5. **高级 UI** - Qt/GTK 替代方案
6. **实时音频** - 在同一库基础上扩展

---

## ✨ 总结

| 方面 | 改进 |
|------|------|
| **代码复用** | ChatNode 逻辑集中在动态库中 |
| **易于集成** | 其他项目只需链接 libchat_node_lib.so |
| **模块化** | 清晰的公共接口和实现分离 |
| **扩展性** | 支持多种 UI 框架和应用类型 |
| **维护性** | 一处修改，所有应用受益 |

---

## 🔧 故障排除

```bash
# 编译不过？检查依赖
sudo apt-get install libwxgtk3.0-gtk3-dev

# 链接报错？
source ~/ros2_ws/install/setup.bash

# 详细编译信息
colcon build --packages-select internet_chat_client -v
```

---

**现在你可以像使用任何其他库一样使用 ChatNode 了！** 🎉
