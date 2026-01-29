#include <wx/wx.h>
#include <wx/treectrl.h>
#include <wx/dcbuffer.h>
#include <wx/imaglist.h>
#include <wx/richtext/richtextctrl.h>
#include <rclcpp/rclcpp.hpp>
#include "internet_chat_client/chat_node.hpp"
#include <thread>
#include <queue>
#include <mutex>
#include <memory>
#include <map>
#include <set>
#include <vector>
#include <string>
#include <tuple>
#include <chrono>

// 头像生成工具函数
wxBitmap GenerateAvatar(const wxString& username, int size = 48) {
    wxBitmap bitmap(size, size);
    wxMemoryDC dc(bitmap);
    
    // 根据用户名生成颜色
    unsigned int hash = 0;
    for(size_t i = 0; i < username.length(); i++) {
        hash = hash * 31 + username[i].GetValue();
    }
    
    int r = (hash & 0xFF0000) >> 16;
    int g = (hash & 0x00FF00) >> 8;
    int b = (hash & 0x0000FF);
    
    // 确保颜色不会太暗
    r = std::max(r, 100);
    g = std::max(g, 100);
    b = std::max(b, 100);
    
    // 绘制圆形背景
    dc.SetBackground(*wxWHITE_BRUSH);
    dc.Clear();
    
    wxBrush brush(wxColour(r, g, b));
    dc.SetBrush(brush);
    dc.SetPen(*wxTRANSPARENT_PEN);
    dc.DrawCircle(size/2, size/2, size/2);
    
    // 绘制首字母
    wxString initial = username.IsEmpty() ? "?" : username.Mid(0, 1).Upper();
    wxFont font(size/2, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD);
    dc.SetFont(font);
    dc.SetTextForeground(*wxWHITE);
    
    wxSize textSize = dc.GetTextExtent(initial);
    int x = (size - textSize.GetWidth()) / 2;
    int y = (size - textSize.GetHeight()) / 2;
    dc.DrawText(initial, x, y);
    
    dc.SelectObject(wxNullBitmap);
    return bitmap;
}

// 前向声明
class ChatWindow;

// 树节点数据
class TreeItemData : public wxTreeItemData {
public:
    TreeItemData(const wxString& username, const wxString& group, bool isGroup)
        : username_(username), group_(group), is_group_(isGroup) {}
    
    wxString GetUsername() const { return username_; }
    wxString GetGroup() const { return group_; }
    bool IsGroup() const { return is_group_; }

private:
    wxString username_;
    wxString group_;
    bool is_group_;
};

// 聊天窗口类
class ChatWindow : public wxFrame {
public:
    ChatWindow(const wxString& username, const wxString& group);
    ~ChatWindow();

private:
    wxStaticBitmap* avatar_bitmap_;
    wxRichTextCtrl* message_display_;
    wxTextCtrl* message_input_;
    wxButton* send_button_;
    wxStaticText* status_text_;

    std::shared_ptr<ChatNode> chat_node_;
    std::thread ros_spin_thread_;
    std::thread message_handler_thread_;
    std::queue<std::tuple<std::string, std::string, std::string>> message_queue_;
    std::mutex queue_mutex_;
    volatile bool running_;

    wxString username_;
    wxString group_;

    void OnSendMessage(wxCommandEvent&);
    void OnClose(wxCloseEvent&);
    void MessageHandlerLoop();
    void ProcessQueuedMessages();

    enum {
        ID_SEND_BUTTON = 2001,
        ID_MESSAGE_INPUT = 2002
    };
};

// 启动器主窗口
class LauncherFrame : public wxFrame {
public:
    LauncherFrame(const wxString& title);

private:
    wxTreeCtrl* tree_ctrl_;
    wxImageList* image_list_;
    std::map<wxString, int> avatar_index_map_;
    wxButton* add_group_button_;
    wxButton* add_user_button_;
    wxButton* remove_button_;
    wxButton* private_chat_button_;
    std::vector<ChatWindow*> chat_windows_;
    std::map<wxString, std::vector<wxString>> all_users_;

    void OnTreeItemActivated(wxTreeEvent& event);
    void OnAddGroup(wxCommandEvent& event);
    void OnAddUser(wxCommandEvent& event);
    void OnRemove(wxCommandEvent& event);
    void OnPrivateChat(wxCommandEvent& event);
    void OnClose(wxCloseEvent& event);
    void InitializeDefaultTree();
    void CollectAllUsers(wxTreeItemId item);
    int GetOrCreateAvatarIndex(const wxString& username);

    enum {
        ID_TREE_CTRL = 3001,
        ID_ADD_GROUP = 3002,
        ID_ADD_USER = 3003,
        ID_REMOVE = 3004,
        ID_PRIVATE_CHAT = 3005
    };

    wxDECLARE_EVENT_TABLE();
};

// 应用类
class LauncherApp : public wxApp {
public:
    bool OnInit() override;
};

wxIMPLEMENT_APP(LauncherApp);

// ============ ChatWindow 实现 ============

ChatWindow::ChatWindow(const wxString& username, const wxString& group)
    : wxFrame(NULL, wxID_ANY, 
              wxString::Format("Chat - %s @ %s", username, group),
              wxDefaultPosition, wxSize(700, 500)),
      username_(username), group_(group), running_(true)
{
    wxPanel* panel = new wxPanel(this);
    wxBoxSizer* main_sizer = new wxBoxSizer(wxVERTICAL);

    // 头像和状态栏区域
    wxBoxSizer* header_sizer = new wxBoxSizer(wxHORIZONTAL);
    
    // 生成并显示头像
    wxBitmap avatar = GenerateAvatar(username, 48);
    avatar_bitmap_ = new wxStaticBitmap(panel, wxID_ANY, avatar);
    header_sizer->Add(avatar_bitmap_, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    
    // 状态文字
    wxBoxSizer* text_sizer = new wxBoxSizer(wxVERTICAL);
    wxStaticText* username_text = new wxStaticText(panel, wxID_ANY, username);
    wxFont username_font(12, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD);
    username_text->SetFont(username_font);
    text_sizer->Add(username_text, 0, wxBOTTOM, 2);
    
    status_text_ = new wxStaticText(panel, wxID_ANY, wxString::Format("Group: %s", group));
    wxFont status_font(9, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL);
    status_text_->SetFont(status_font);
    status_text_->SetForegroundColour(wxColour(100, 100, 100));
    text_sizer->Add(status_text_, 0);
    
    header_sizer->Add(text_sizer, 1, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    main_sizer->Add(header_sizer, 0, wxEXPAND | wxALL, 10);

    // 消息显示区
    main_sizer->Add(new wxStaticText(panel, wxID_ANY, "Messages:"), 0, wxLEFT | wxTOP, 10);
    message_display_ = new wxRichTextCtrl(panel, wxID_ANY, "", wxDefaultPosition, wxDefaultSize,
                                          wxTE_MULTILINE | wxTE_READONLY | wxTE_WORDWRAP);
    message_display_->SetBackgroundColour(wxColour(245, 245, 245));
    main_sizer->Add(message_display_, 1, wxEXPAND | wxALL, 10);

    // 输入区
    main_sizer->Add(new wxStaticText(panel, wxID_ANY, "Your message:"), 0, wxLEFT, 10);
    wxBoxSizer* input_sizer = new wxBoxSizer(wxHORIZONTAL);
    message_input_ = new wxTextCtrl(panel, ID_MESSAGE_INPUT, "", wxDefaultPosition, wxDefaultSize,
                                    wxTE_PROCESS_ENTER);
    input_sizer->Add(message_input_, 1, wxEXPAND | wxRIGHT, 5);
    send_button_ = new wxButton(panel, ID_SEND_BUTTON, "Send");
    input_sizer->Add(send_button_, 0, wxEXPAND);
    main_sizer->Add(input_sizer, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 10);

    panel->SetSizer(main_sizer);

    // 事件绑定
    Bind(wxEVT_BUTTON, &ChatWindow::OnSendMessage, this, ID_SEND_BUTTON);
    Bind(wxEVT_TEXT_ENTER, &ChatWindow::OnSendMessage, this, ID_MESSAGE_INPUT);
    Bind(wxEVT_CLOSE_WINDOW, &ChatWindow::OnClose, this);

    // 初始化ROS2节点
    if(!rclcpp::ok()) {
        int argc = 0;
        char** argv = nullptr;
        rclcpp::init(argc, argv);
    }

    std::string username_str = std::string(username.mb_str());
    chat_node_ = std::make_shared<ChatNode>(username_str);

    // 设置消息回调
    chat_node_->set_message_callback([this](const std::string& sender,
                                             const std::string& timestamp,
                                             const std::string& text) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        message_queue_.push(std::make_tuple(sender, timestamp, text));
    });

    // 加入群组
    std::string group_str = std::string(group.mb_str());
    chat_node_->join_group(group_str);

    // 启动ROS2线程
    ros_spin_thread_ = std::thread([this]() {
        if(chat_node_) {
            rclcpp::spin(chat_node_);
        }
    });

    // 启动消息处理线程
    message_handler_thread_ = std::thread([this]() {
        MessageHandlerLoop();
    });

    message_display_->BeginSuppressUndo();
    message_display_->BeginAlignment(wxTEXT_ALIGNMENT_CENTRE);
    message_display_->BeginTextColour(wxColour(128, 128, 128));
    message_display_->WriteText(wxString::Format("Connected as %s in group %s", username, group));
    message_display_->EndTextColour();
    message_display_->EndAlignment();
    message_display_->EndSuppressUndo();
}

ChatWindow::~ChatWindow() {
    running_ = false;
    if(ros_spin_thread_.joinable()) {
        ros_spin_thread_.join();
    }
    if(message_handler_thread_.joinable()) {
        message_handler_thread_.join();
    }
}

void ChatWindow::MessageHandlerLoop() {
    while(running_) {
        ProcessQueuedMessages();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void ChatWindow::ProcessQueuedMessages() {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    while(!message_queue_.empty()) {
        auto [sender, timestamp, text] = message_queue_.front();
        message_queue_.pop();

        CallAfter([this, sender, text, timestamp]() {
            message_display_->BeginSuppressUndo();
            message_display_->Newline();
            
            if(sender == "System") {
                // 系统消息居中显示
                message_display_->BeginAlignment(wxTEXT_ALIGNMENT_CENTRE);
                message_display_->BeginTextColour(wxColour(128, 128, 128));
                message_display_->BeginFontSize(9);
                message_display_->WriteText(wxString::Format("%s %s", timestamp, text));
                message_display_->EndFontSize();
                message_display_->EndTextColour();
                message_display_->EndAlignment();
            } else {
                // 生成发送者头像
                wxBitmap avatar = GenerateAvatar(wxString(sender), 32);
                
                // 判断是否为自己的消息
                bool is_self = (sender == std::string(username_.mb_str()));
                
                if(is_self) {
                    // 自己的消息靠右
                    message_display_->BeginAlignment(wxTEXT_ALIGNMENT_RIGHT);
                    
                    // 头像
                    message_display_->WriteImage(avatar);
                    message_display_->WriteText(" ");
                    
                    // 消息气泡（绿色）
                    wxRichTextAttr attr;
                    attr.SetTextColour(wxColour(255, 255, 255));
                    attr.SetBackgroundColour(wxColour(137, 217, 97));
                    attr.SetAlignment(wxTEXT_ALIGNMENT_RIGHT);
                    message_display_->BeginStyle(attr);
                    message_display_->WriteText(" " + wxString(text) + " ");
                    message_display_->EndStyle();
                    
                    // 时间戳（小字灰色）
                    message_display_->WriteText(" ");
                    message_display_->BeginTextColour(wxColour(150, 150, 150));
                    message_display_->BeginFontSize(8);
                    message_display_->WriteText(timestamp);
                    message_display_->EndFontSize();
                    message_display_->EndTextColour();
                    
                    message_display_->EndAlignment();
                } else {
                    // 别人的消息靠左
                    message_display_->BeginAlignment(wxTEXT_ALIGNMENT_LEFT);
                    
                    // 发送者名字（灰色小字，单独一行）
                    message_display_->BeginTextColour(wxColour(100, 100, 100));
                    message_display_->BeginFontSize(8);
                    message_display_->WriteText(wxString(sender));
                    message_display_->EndFontSize();
                    message_display_->EndTextColour();
                    message_display_->EndAlignment();
                    
                    // 消息内容行
                    message_display_->BeginAlignment(wxTEXT_ALIGNMENT_LEFT);
                    
                    // 头像
                    message_display_->WriteImage(avatar);
                    message_display_->WriteText(" ");
                    
                    // 消息气泡（白色）
                    wxRichTextAttr attr;
                    attr.SetTextColour(wxColour(0, 0, 0));
                    attr.SetBackgroundColour(wxColour(255, 255, 255));
                    attr.SetLeftIndent(10);
                    attr.SetRightIndent(200);
                    message_display_->BeginStyle(attr);
                    message_display_->WriteText(" " + wxString(text) + " ");
                    message_display_->EndStyle();
                    
                    // 时间戳
                    message_display_->WriteText(" ");
                    message_display_->BeginTextColour(wxColour(150, 150, 150));
                    message_display_->BeginFontSize(8);
                    message_display_->WriteText(timestamp);
                    message_display_->EndFontSize();
                    message_display_->EndTextColour();
                    message_display_->EndAlignment();
            }
            
            message_display_->EndSuppressUndo();
            message_display_->ShowPosition(message_display_->GetLastPosition());
        });
    }
}

void ChatWindow::OnSendMessage(wxCommandEvent&) {
    if(!chat_node_) return;

    wxString message = message_input_->GetValue();
    if(!message.IsEmpty()) {
        std::string msg_str = std::string(message.mb_str());
        chat_node_->send_message(msg_str);
        
        // 立即显示自己发送的消息
        std::string timestamp = std::string(wxDateTime::Now().FormatTime().mb_str());
        std::string sender = std::string(username_.mb_str());
        
        // 添加到消息队列
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            message_queue_.push({sender, timestamp, msg_str});
        }
        ProcessQueuedMessages();
        
        message_input_->Clear();
        message_input_->SetFocus();
    }
}

void ChatWindow::OnClose(wxCloseEvent& event) {
    running_ = false;
    Destroy();
}

// ============ LauncherFrame 实现 ============

wxBEGIN_EVENT_TABLE(LauncherFrame, wxFrame)
    EVT_TREE_ITEM_ACTIVATED(ID_TREE_CTRL, LauncherFrame::OnTreeItemActivated)
wxEND_EVENT_TABLE()

LauncherFrame::LauncherFrame(const wxString& title)
    : wxFrame(NULL, wxID_ANY, title, wxDefaultPosition, wxSize(500, 600))
{
    wxPanel* panel = new wxPanel(this);
    wxBoxSizer* main_sizer = new wxBoxSizer(wxVERTICAL);

    // 说明文字
    wxStaticText* info_text = new wxStaticText(panel, wxID_ANY,
        "Double-click a user to start chatting\nUser will join the parent group automatically");
    wxFont info_font(9, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_ITALIC, wxFONTWEIGHT_NORMAL);
    info_text->SetFont(info_font);
    main_sizer->Add(info_text, 0, wxALL, 10);

    // 树形控件
    tree_ctrl_ = new wxTreeCtrl(panel, ID_TREE_CTRL, wxDefaultPosition, wxDefaultSize,
                                wxTR_HAS_BUTTONS | wxTR_LINES_AT_ROOT | wxTR_SINGLE);
    
    // 创建头像image list (32x32)
    image_list_ = new wxImageList(32, 32, true);
    tree_ctrl_->SetImageList(image_list_);
    
    main_sizer->Add(tree_ctrl_, 1, wxEXPAND | wxALL, 10);

    // 按钮区
    wxBoxSizer* button_sizer = new wxBoxSizer(wxHORIZONTAL);
    add_group_button_ = new wxButton(panel, ID_ADD_GROUP, "Add Group");
    add_user_button_ = new wxButton(panel, ID_ADD_USER, "Add User");
    remove_button_ = new wxButton(panel, ID_REMOVE, "Remove");
    private_chat_button_ = new wxButton(panel, ID_PRIVATE_CHAT, "Private Chat");
    button_sizer->Add(add_group_button_, 1, wxEXPAND | wxRIGHT, 5);
    button_sizer->Add(add_user_button_, 1, wxEXPAND | wxRIGHT, 5);
    button_sizer->Add(remove_button_, 1, wxEXPAND | wxRIGHT, 5);
    button_sizer->Add(private_chat_button_, 1, wxEXPAND);
    main_sizer->Add(button_sizer, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 10);

    panel->SetSizer(main_sizer);

    // 事件绑定
    Bind(wxEVT_BUTTON, &LauncherFrame::OnAddGroup, this, ID_ADD_GROUP);
    Bind(wxEVT_BUTTON, &LauncherFrame::OnAddUser, this, ID_ADD_USER);
    Bind(wxEVT_BUTTON, &LauncherFrame::OnRemove, this, ID_REMOVE);
    Bind(wxEVT_BUTTON, &LauncherFrame::OnPrivateChat, this, ID_PRIVATE_CHAT);
    Bind(wxEVT_CLOSE_WINDOW, &LauncherFrame::OnClose, this);

    // 初始化默认树结构
    InitializeDefaultTree();

    CreateStatusBar();
    SetStatusText("Ready. Double-click a user to start chatting.");
}

void LauncherFrame::InitializeDefaultTree() {
    wxTreeItemId root = tree_ctrl_->AddRoot("Chat Groups");

    // 默认群组1: Tech Team
    wxTreeItemId tech_group = tree_ctrl_->AppendItem(root, "tech_team");
    tree_ctrl_->SetItemData(tech_group, new TreeItemData("", "tech_team", true));
    
    // 为tech_team用户添加头像
    wxArrayString users;
    users.Add("Alice");
    users.Add("Bob");
    users.Add("Charlie");
    for(size_t i = 0; i < users.Count(); i++) {
        wxString user = users[i];
        int img_idx = GetOrCreateAvatarIndex(user);
        tree_ctrl_->AppendItem(tech_group, user, img_idx, img_idx,
                              new TreeItemData(std::string(user.mb_str()), "tech_team", false));
    }

    // 默认群组2: General
    wxTreeItemId general_group = tree_ctrl_->AppendItem(root, "general");
    tree_ctrl_->SetItemData(general_group, new TreeItemData("", "general", true));
    
    wxArrayString general_users;
    general_users.Add("David");
    general_users.Add("Cecilia");
    for(size_t i = 0; i < general_users.Count(); i++) {
        wxString user = general_users[i];
        int img_idx = GetOrCreateAvatarIndex(user);
        tree_ctrl_->AppendItem(general_group, user, img_idx, img_idx,
                              new TreeItemData(std::string(user.mb_str()), "general", false));
    }

    // 默认群组3: Project Alpha
    wxTreeItemId alpha_group = tree_ctrl_->AppendItem(root, "project_alpha");
    tree_ctrl_->SetItemData(alpha_group, new TreeItemData("", "project_alpha", true));
    
    wxArrayString alpha_users;
    alpha_users.Add("Frank");
    alpha_users.Add("Grace");
    for(size_t i = 0; i < alpha_users.Count(); i++) {
        wxString user = alpha_users[i];
        int img_idx = GetOrCreateAvatarIndex(user);
        tree_ctrl_->AppendItem(alpha_group, user, img_idx, img_idx,
                              new TreeItemData(std::string(user.mb_str()), "project_alpha", false));
    }

    tree_ctrl_->Expand(root);
}

int LauncherFrame::GetOrCreateAvatarIndex(const wxString& username) {
    // 检查是否已经创建过头像
    if(avatar_index_map_.find(username) != avatar_index_map_.end()) {
        return avatar_index_map_[username];
    }
    
    // 生成头像并添加到image list
    wxBitmap avatar = GenerateAvatar(username, 32);
    int index = image_list_->Add(avatar);
    avatar_index_map_[username] = index;
    return index;
}

void LauncherFrame::OnTreeItemActivated(wxTreeEvent& event) {
    wxTreeItemId item = event.GetItem();
    if(!item.IsOk()) return;

    TreeItemData* data = dynamic_cast<TreeItemData*>(tree_ctrl_->GetItemData(item));
    if(!data) return;

    // 如果是群组，切换展开/收起状态
    if(data->IsGroup()) {
        if(tree_ctrl_->IsExpanded(item)) {
            tree_ctrl_->Collapse(item);
        } else {
            tree_ctrl_->Expand(item);
        }
        return;
    }

    wxString username = data->GetUsername();
    wxString group = data->GetGroup();

    // 创建新的聊天窗口
    ChatWindow* chat_window = new ChatWindow(username, group);
    chat_windows_.push_back(chat_window);
    chat_window->Show(true);

    SetStatusText(wxString::Format("Opened chat for %s in group %s", username, group));
}

void LauncherFrame::OnAddGroup(wxCommandEvent&) {
    wxTextEntryDialog dialog(this, "Enter new group name:", "Add Group");
    if(dialog.ShowModal() == wxID_OK) {
        wxString group_name = dialog.GetValue();
        if(!group_name.IsEmpty()) {
            wxTreeItemId root = tree_ctrl_->GetRootItem();
            wxTreeItemId new_group = tree_ctrl_->AppendItem(root, group_name);
            tree_ctrl_->SetItemData(new_group, new TreeItemData("", group_name, true));
            tree_ctrl_->Expand(root);
            SetStatusText(wxString::Format("Added group: %s", group_name));
        }
    }
}

void LauncherFrame::OnAddUser(wxCommandEvent&) {
    wxTreeItemId selected = tree_ctrl_->GetSelection();
    if(!selected.IsOk()) {
        wxMessageBox("Please select a group first!", "Error", wxOK | wxICON_ERROR);
        return;
    }

    TreeItemData* data = dynamic_cast<TreeItemData*>(tree_ctrl_->GetItemData(selected));
    if(!data || !data->IsGroup()) {
        wxMessageBox("Please select a group (not a user)!", "Error", wxOK | wxICON_ERROR);
        return;
    }

    wxTextEntryDialog dialog(this, "Enter new user name:", "Add User");
    if(dialog.ShowModal() == wxID_OK) {
        wxString username = dialog.GetValue();
        if(!username.IsEmpty()) {
            wxString group = data->GetGroup();
            int img_idx = GetOrCreateAvatarIndex(username);
            tree_ctrl_->AppendItem(selected, username, img_idx, img_idx,
                                  new TreeItemData(username, group, false));
            tree_ctrl_->Expand(selected);
            SetStatusText(wxString::Format("Added user %s to group %s", username, group));
        }
    }
}

void LauncherFrame::OnRemove(wxCommandEvent&) {
    wxTreeItemId selected = tree_ctrl_->GetSelection();
    if(!selected.IsOk() || selected == tree_ctrl_->GetRootItem()) {
        wxMessageBox("Please select an item to remove!", "Error", wxOK | wxICON_ERROR);
        return;
    }

    tree_ctrl_->Delete(selected);
    SetStatusText("Item removed.");
}

void LauncherFrame::CollectAllUsers(wxTreeItemId item) {
    wxTreeItemIdValue cookie;
    wxTreeItemId child = tree_ctrl_->GetFirstChild(item, cookie);
    
    while(child.IsOk()) {
        TreeItemData* data = dynamic_cast<TreeItemData*>(tree_ctrl_->GetItemData(child));
        if(data) {
            if(data->IsGroup()) {
                wxString group_name = data->GetGroup();
                CollectAllUsers(child);
            } else {
                wxString group_name = data->GetGroup();
                wxString username = data->GetUsername();
                all_users_[group_name].push_back(username);
            }
        }
        child = tree_ctrl_->GetNextChild(item, cookie);
    }
}

void LauncherFrame::OnPrivateChat(wxCommandEvent&) {
    // 收集所有用户
    all_users_.clear();
    wxTreeItemId root = tree_ctrl_->GetRootItem();
    CollectAllUsers(root);
    
    // 展平所有用户列表（去重）
    std::set<wxString> unique_users_set;
    for(const auto& pair : all_users_) {
        for(const auto& user : pair.second) {
            unique_users_set.insert(user);
        }
    }
    
    wxArrayString user_list;
    for(const auto& user : unique_users_set) {
        user_list.Add(user);
    }
    
    if(user_list.GetCount() < 2) {
        wxMessageBox("Need at least 2 users to start a private chat!", "Error", wxOK | wxICON_ERROR);
        return;
    }
    
    // 选择第一个用户
    wxSingleChoiceDialog dlg1(this, "Select first user:", "Private Chat - User 1", user_list);
    if(dlg1.ShowModal() != wxID_OK) {
        return;
    }
    wxString user1 = dlg1.GetStringSelection();
    
    // 选择第二个用户
    wxArrayString remaining_users;
    for(const auto& user : user_list) {
        if(user != user1) {
            remaining_users.Add(user);
        }
    }
    
    wxSingleChoiceDialog dlg2(this, "Select second user:", "Private Chat - User 2", remaining_users);
    if(dlg2.ShowModal() != wxID_OK) {
        return;
    }
    wxString user2 = dlg2.GetStringSelection();
    
    // 创建私聊group名（格式：private_user1_user2，确保一致性）
    wxString group_name;
    if(user1 < user2) {
        group_name = wxString::Format("private_%s_%s", user1, user2);
    } else {
        group_name = wxString::Format("private_%s_%s", user2, user1);
    }
    
    // 检查是否已存在该私聊group
    wxTreeItemId root_item = tree_ctrl_->GetRootItem();
    wxTreeItemIdValue cookie;
    wxTreeItemId child = tree_ctrl_->GetFirstChild(root_item, cookie);
    bool group_exists = false;
    
    while(child.IsOk()) {
        if(tree_ctrl_->GetItemText(child) == group_name) {
            group_exists = true;
            break;
        }
        child = tree_ctrl_->GetNextChild(root_item, cookie);
    }
    
    // 如果不存在，创建新的private group
    if(!group_exists) {
        wxTreeItemId private_group = tree_ctrl_->AppendItem(root_item, group_name);
        tree_ctrl_->SetItemData(private_group, new TreeItemData("", group_name, true));
        
        // 添加两个用户到private group
        tree_ctrl_->AppendItem(private_group, user1, -1, -1,
                              new TreeItemData(user1, group_name, false));
        tree_ctrl_->AppendItem(private_group, user2, -1, -1,
                              new TreeItemData(user2, group_name, false));
        
        tree_ctrl_->Expand(root_item);
        SetStatusText(wxString::Format("Created private chat: %s <-> %s", user1, user2));
    }
    
    // 打开两个窗口 - 一个给user1，一个给user2
    ChatWindow* chat_window1 = new ChatWindow(user1, group_name);
    chat_windows_.push_back(chat_window1);
    chat_window1->Show(true);
    
    ChatWindow* chat_window2 = new ChatWindow(user2, group_name);
    chat_windows_.push_back(chat_window2);
    chat_window2->Show(true);
    
    SetStatusText(wxString::Format("Opened private chat: %s <-> %s in group %s", user1, user2, group_name));
}

void LauncherFrame::OnClose(wxCloseEvent& event) {
    // 关闭所有聊天窗口
    for(auto* window : chat_windows_) {
        if(window) {
            window->Close(true);
        }
    }
    chat_windows_.clear();
    Destroy();
}

// ============ LauncherApp 实现 ============

bool LauncherApp::OnInit() {
    // 初始化图像处理器
    wxInitAllImageHandlers();
    
    LauncherFrame* frame = new LauncherFrame("ROS2 Chat Launcher");
    frame->Show(true);
    return true;
}
