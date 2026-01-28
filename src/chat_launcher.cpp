#include <wx/wx.h>
#include <wx/treectrl.h>
#include "internet_chat_client/chat_node.hpp"
#include <thread>
#include <queue>
#include <mutex>
#include <memory>
#include <map>
#include <vector>

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
    wxTextCtrl* message_display_;
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
    wxButton* add_group_button_;
    wxButton* add_user_button_;
    wxButton* remove_button_;
    std::vector<ChatWindow*> chat_windows_;

    void OnTreeItemActivated(wxTreeEvent& event);
    void OnAddGroup(wxCommandEvent& event);
    void OnAddUser(wxCommandEvent& event);
    void OnRemove(wxCommandEvent& event);
    void OnClose(wxCloseEvent& event);
    void InitializeDefaultTree();

    enum {
        ID_TREE_CTRL = 3001,
        ID_ADD_GROUP = 3002,
        ID_ADD_USER = 3003,
        ID_REMOVE = 3004
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

    // 状态栏
    status_text_ = new wxStaticText(panel, wxID_ANY, 
                                    wxString::Format("User: %s | Group: %s", username, group));
    wxFont status_font(10, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD);
    status_text_->SetFont(status_font);
    main_sizer->Add(status_text_, 0, wxALL, 10);

    // 消息显示区
    main_sizer->Add(new wxStaticText(panel, wxID_ANY, "Messages:"), 0, wxLEFT | wxTOP, 10);
    message_display_ = new wxTextCtrl(panel, wxID_ANY, "", wxDefaultPosition, wxDefaultSize,
                                      wxTE_MULTILINE | wxTE_READONLY | wxTE_WORDWRAP);
    wxFont display_font(10, wxFONTFAMILY_TELETYPE, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL);
    message_display_->SetFont(display_font);
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

    message_display_->AppendText(wxString::Format(">>> Connected as %s in group %s\n", username, group));
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

        wxString formatted_message;
        if(sender == "System") {
            formatted_message = wxString::Format("[%s] %s\n", timestamp, text);
        } else {
            formatted_message = wxString::Format("[%s] %s: %s\n", timestamp, sender, text);
        }

        CallAfter([this, formatted_message]() {
            message_display_->AppendText(formatted_message);
        });
    }
}

void ChatWindow::OnSendMessage(wxCommandEvent&) {
    if(!chat_node_) return;

    wxString message = message_input_->GetValue();
    if(!message.IsEmpty()) {
        std::string msg_str = std::string(message.mb_str());
        chat_node_->send_message(msg_str);

        // 显示自己的消息
        std::string timestamp = "[" + std::string(wxDateTime::Now().FormatTime().mb_str()) + "]";
        wxString display = wxString::Format("%s %s: %s\n", timestamp, username_, message);
        message_display_->AppendText(display);

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
    main_sizer->Add(tree_ctrl_, 1, wxEXPAND | wxALL, 10);

    // 按钮区
    wxBoxSizer* button_sizer = new wxBoxSizer(wxHORIZONTAL);
    add_group_button_ = new wxButton(panel, ID_ADD_GROUP, "Add Group");
    add_user_button_ = new wxButton(panel, ID_ADD_USER, "Add User");
    remove_button_ = new wxButton(panel, ID_REMOVE, "Remove");
    button_sizer->Add(add_group_button_, 1, wxEXPAND | wxRIGHT, 5);
    button_sizer->Add(add_user_button_, 1, wxEXPAND | wxRIGHT, 5);
    button_sizer->Add(remove_button_, 1, wxEXPAND);
    main_sizer->Add(button_sizer, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 10);

    panel->SetSizer(main_sizer);

    // 事件绑定
    Bind(wxEVT_BUTTON, &LauncherFrame::OnAddGroup, this, ID_ADD_GROUP);
    Bind(wxEVT_BUTTON, &LauncherFrame::OnAddUser, this, ID_ADD_USER);
    Bind(wxEVT_BUTTON, &LauncherFrame::OnRemove, this, ID_REMOVE);
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
    tree_ctrl_->AppendItem(tech_group, "Alice", -1, -1,
                          new TreeItemData("Alice", "tech_team", false));
    tree_ctrl_->AppendItem(tech_group, "Bob", -1, -1,
                          new TreeItemData("Bob", "tech_team", false));
    tree_ctrl_->AppendItem(tech_group, "Charlie", -1, -1,
                          new TreeItemData("Charlie", "tech_team", false));

    // 默认群组2: General
    wxTreeItemId general_group = tree_ctrl_->AppendItem(root, "general");
    tree_ctrl_->SetItemData(general_group, new TreeItemData("", "general", true));
    tree_ctrl_->AppendItem(general_group, "David", -1, -1,
                          new TreeItemData("David", "general", false));
    tree_ctrl_->AppendItem(general_group, "Eve", -1, -1,
                          new TreeItemData("Eve", "general", false));

    // 默认群组3: Project Alpha
    wxTreeItemId alpha_group = tree_ctrl_->AppendItem(root, "project_alpha");
    tree_ctrl_->SetItemData(alpha_group, new TreeItemData("", "project_alpha", true));
    tree_ctrl_->AppendItem(alpha_group, "Frank", -1, -1,
                          new TreeItemData("Frank", "project_alpha", false));
    tree_ctrl_->AppendItem(alpha_group, "Grace", -1, -1,
                          new TreeItemData("Grace", "project_alpha", false));

    tree_ctrl_->Expand(root);
}

void LauncherFrame::OnTreeItemActivated(wxTreeEvent& event) {
    wxTreeItemId item = event.GetItem();
    if(!item.IsOk()) return;

    TreeItemData* data = dynamic_cast<TreeItemData*>(tree_ctrl_->GetItemData(item));
    if(!data) return;

    // 只处理用户节点，不处理群组节点
    if(data->IsGroup()) {
        SetStatusText("Please double-click a user, not a group.");
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
            tree_ctrl_->AppendItem(selected, username, -1, -1,
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
    LauncherFrame* frame = new LauncherFrame("ROS2 Chat Launcher");
    frame->Show(true);
    return true;
}
