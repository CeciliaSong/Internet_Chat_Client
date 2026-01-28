#include <wx/wx.h>
#include "internet_chat_client/chat_node.hpp"
#include <thread>
#include <queue>
#include <mutex>
#include <memory>

// Custom event for thread-safe message display
wxDECLARE_EVENT(wxEVT_CHAT_MESSAGE, wxThreadEvent);
wxDEFINE_EVENT(wxEVT_CHAT_MESSAGE, wxThreadEvent);

class ChatFrame : public wxFrame 
{
public:
    ChatFrame(const wxString& title);
    ~ChatFrame();

private:
    // UI Components
    wxTextCtrl* message_display_;
    wxTextCtrl* message_input_;
    wxChoice* group_choice_;
    wxButton* send_button_;
    wxButton* join_button_;
    wxTextCtrl* group_input_;
    wxStaticText* status_text_;

    // Chat node
    std::shared_ptr<ChatNode> chat_node_;
    std::thread ros_spin_thread_;
    std::thread message_handler_thread_;
    std::queue<std::tuple<std::string, std::string, std::string>> message_queue_;
    std::mutex queue_mutex_;
    volatile bool running_;

    // Event handlers
    void OnSendMessage(wxCommandEvent&);
    void OnJoinGroup(wxCommandEvent&);
    void OnChatMessage(wxThreadEvent&);
    void OnClose(wxCloseEvent&);
    void OnGroupSelected(wxCommandEvent&);

    // Helper methods
    void InitializeChatNode();
    void MessageHandlerLoop();
    void ProcessQueuedMessages();

    enum 
    {
        ID_SEND_BUTTON = 1001,
        ID_JOIN_BUTTON = 1002,
        ID_MESSAGE_INPUT = 1003,
        ID_GROUP_CHOICE = 1004,
        ID_GROUP_INPUT = 1005
    };
};

class ChatApp : public wxApp 
{
public:
    bool OnInit() override;
};

wxIMPLEMENT_APP(ChatApp);

bool ChatApp::OnInit() 
{
    ChatFrame* frame = new ChatFrame("ROS2 Chat Client - GUI");
    frame->Show(true);
    return true;
}

ChatFrame::ChatFrame(const wxString& title)
    : wxFrame(NULL, wxID_ANY, title, wxDefaultPosition, wxSize(800, 600)),
      chat_node_(nullptr), running_(true)
{
    // Create main panel
    wxPanel* panel = new wxPanel(this);
    wxBoxSizer* main_sizer = new wxBoxSizer(wxVERTICAL);

    // Top section: Group selection
    wxBoxSizer* group_sizer = new wxBoxSizer(wxHORIZONTAL);
    group_sizer->Add(new wxStaticText(panel, wxID_ANY, "Group:"), 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 5);
    
    group_choice_ = new wxChoice(panel, ID_GROUP_CHOICE);
    group_choice_->AppendString("general");
    group_choice_->SetSelection(0);
    group_sizer->Add(group_choice_, 1, wxEXPAND | wxRIGHT, 5);

    group_input_ = new wxTextCtrl(panel, ID_GROUP_INPUT, "", wxDefaultPosition, wxSize(150, -1));
    group_sizer->Add(new wxStaticText(panel, wxID_ANY, "New:"), 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 5);
    group_sizer->Add(group_input_, 0, wxEXPAND | wxRIGHT, 5);
    
    join_button_ = new wxButton(panel, ID_JOIN_BUTTON, "Join Group");
    group_sizer->Add(join_button_, 0, wxEXPAND);
    main_sizer->Add(group_sizer, 0, wxEXPAND | wxALL, 10);

    // Middle section: Message display
    main_sizer->Add(new wxStaticText(panel, wxID_ANY, "Chat Messages:"), 0, wxLEFT | wxTOP, 10);
    message_display_ = new wxTextCtrl(panel, wxID_ANY, "", wxDefaultPosition, wxDefaultSize, 
                                      wxTE_MULTILINE | wxTE_READONLY | wxTE_WORDWRAP);
    wxFont display_font(10, wxFONTFAMILY_TELETYPE, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL);
    message_display_->SetFont(display_font);
    main_sizer->Add(message_display_, 1, wxEXPAND | wxALL, 10);

    // Bottom section: Message input
    main_sizer->Add(new wxStaticText(panel, wxID_ANY, "Message:"), 0, wxLEFT, 10);
    wxBoxSizer* input_sizer = new wxBoxSizer(wxHORIZONTAL);
    message_input_ = new wxTextCtrl(panel, ID_MESSAGE_INPUT, "", wxDefaultPosition, wxDefaultSize, 
                                    wxTE_PROCESS_ENTER | wxTE_WORDWRAP);
    input_sizer->Add(message_input_, 1, wxEXPAND | wxRIGHT, 5);

    send_button_ = new wxButton(panel, ID_SEND_BUTTON, "Send");
    input_sizer->Add(send_button_, 0, wxEXPAND);
    main_sizer->Add(input_sizer, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 10);

    // Status bar
    status_text_ = new wxStaticText(panel, wxID_ANY, "Ready");
    wxFont status_font(9, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL);
    status_text_->SetFont(status_font);
    main_sizer->Add(status_text_, 0, wxLEFT | wxRIGHT | wxBOTTOM, 10);

    panel->SetSizer(main_sizer);

    // Bind events
    Bind(wxEVT_BUTTON, &ChatFrame::OnSendMessage, this, ID_SEND_BUTTON);
    Bind(wxEVT_BUTTON, &ChatFrame::OnJoinGroup, this, ID_JOIN_BUTTON);
    Bind(wxEVT_CHOICE, &ChatFrame::OnGroupSelected, this, ID_GROUP_CHOICE);
    Bind(wxEVT_TEXT_ENTER, &ChatFrame::OnSendMessage, this, ID_MESSAGE_INPUT);
    Bind(wxEVT_THREAD, &ChatFrame::OnChatMessage, this);
    Bind(wxEVT_CLOSE_WINDOW, &ChatFrame::OnClose, this);

    // Initialize chat node
    InitializeChatNode();

    // Start ROS spin thread
    ros_spin_thread_ = std::thread([this]() 
    {
        if(chat_node_) 
        {
            rclcpp::spin(chat_node_);
        }
    });

    // Start message handler thread
    message_handler_thread_ = std::thread([this]() 
    {
        MessageHandlerLoop();
    });

    CreateStatusBar();
    SetStatusText("Connected. Ready to chat.", 0);
}

ChatFrame::~ChatFrame() 
{
    running_ = false;
    if(ros_spin_thread_.joinable()) 
    {
        ros_spin_thread_.join();
    }
    if(message_handler_thread_.joinable()) 
    {
        message_handler_thread_.join();
    }
    rclcpp::shutdown();
}

void ChatFrame::InitializeChatNode()    
{
    // Get username from user
    wxTextEntryDialog username_dialog(this, "Enter your username:", "Chat Client");
    if(username_dialog.ShowModal() != wxID_OK) 
    {
        username_dialog.SetValue("Anonymous");
    }

    std::string username = std::string(username_dialog.GetValue().mb_str());
    if(username.empty()) 
    {
        username = "Anonymous";
    }

    if(!rclcpp::ok()) 
    {
        int argc = 0;
        char** argv = nullptr;
        rclcpp::init(argc, argv);
    }

    chat_node_ = std::make_shared<ChatNode>(username);

    // Set message callback
    chat_node_->set_message_callback([this](const std::string& sender, const std::string& timestamp, const std::string& text) 
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        message_queue_.push(std::make_tuple(sender, timestamp, text));
    });

    status_text_->SetLabel(wxString::Format("Connected as: %s in group: %s", username, "general"));
}

void ChatFrame::MessageHandlerLoop() 
{
    while(running_) 
    {
        ProcessQueuedMessages();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void ChatFrame::ProcessQueuedMessages() 
{
    std::lock_guard<std::mutex> lock(queue_mutex_);
    while(!message_queue_.empty()) 
    {
        auto [sender, timestamp, text] = message_queue_.front();
        message_queue_.pop();

        wxString formatted_message;
        if(sender == "System") 
        {
            formatted_message = wxString::Format("[%s] %s\n", timestamp, text);
        } 
        else 
        {
            formatted_message = wxString::Format("[%s] %s: %s\n", timestamp, sender, text);
        }

        message_display_->AppendText(formatted_message);
    }
}

void ChatFrame::OnSendMessage(wxCommandEvent&) 
{
    if(!chat_node_) return;

    wxString message = message_input_->GetValue();
    if(!message.IsEmpty()) 
    {
        std::string msg_str = std::string(message.mb_str());
        chat_node_->send_message(msg_str);
        
        // Display local message
        std::string timestamp = "[" + std::string(wxDateTime::Now().FormatTime().mb_str()) + "]";
        wxString display = wxString::Format("%s %s: %s\n", timestamp, chat_node_->get_username(), msg_str);
        message_display_->AppendText(display);
        
        message_input_->Clear();
        message_input_->SetFocus();
    }
}

void ChatFrame::OnJoinGroup(wxCommandEvent&) 
{
    if(!chat_node_) return;

    wxString group_name = group_input_->GetValue();
    if(!group_name.IsEmpty()) 
    {
        std::string group_str = std::string(group_name.mb_str());
        chat_node_->join_group(group_str);

        // Add to choice if not already there
        if(group_choice_->FindString(group_name) == wxNOT_FOUND) {
            group_choice_->AppendString(group_name);
        }
        group_choice_->SetStringSelection(group_name);

        message_display_->AppendText(wxString::Format(">>> Joined group: %s\n", group_name));
        group_input_->Clear();
        status_text_->SetLabel(wxString::Format("Group: %s", group_name));
    }
}

void ChatFrame::OnGroupSelected(wxCommandEvent&) 
{
    if(!chat_node_) return;

    wxString group_name = group_choice_->GetStringSelection();
    if(!group_name.IsEmpty()) 
    {
        std::string group_str = std::string(group_name.mb_str());
        if(group_str != chat_node_->get_current_group()) 
        {
            chat_node_->join_group(group_str);
            message_display_->AppendText(wxString::Format(">>> Switched to group: %s\n", group_name));
            status_text_->SetLabel(wxString::Format("Group: %s", group_name));
        }
    }
}

void ChatFrame::OnChatMessage(wxThreadEvent&) 
{
    ProcessQueuedMessages();
}

void ChatFrame::OnClose(wxCloseEvent&) 
{
    running_ = false;
    Destroy();
}                                     