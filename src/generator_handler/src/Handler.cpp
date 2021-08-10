#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "teo_messages/msg/gen_cmd.hpp"
#include "teo_messages/msg/script_outcome.hpp"
#include "teo_messages/msg/handler_cmd.hpp"
#include "teo_messages/msg/map_update.hpp"
#include "teo_messages/msg/script_progress.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <deque>

#include <nlohmann/json.hpp> // if needed to parse the message
using json = nlohmann::json;

using namespace std::chrono_literals;

// Data structures for handling the scripts
struct scriptgen
{
   std::string id;
   std::string path;
   std::string jsonmsg;
   bool completed;
};

std::deque<scriptgen> ongoing;
int current=-1;
bool all_online = false;
struct nodestatus
{
  std::string id;
  bool active; 
};
std::list<nodestatus> activeNodes;
std::string status = "paused"; // can be "running", "paused"

class Handler : public rclcpp::Node
{
  public:
    Handler()
    : Node("ScriptGeneratorHandler")
    {
      std::cout << "[ScriptGeneratorHandler] >> Initializing the script generator handler.\n";
      pub_WM = this->create_publisher<teo_messages::msg::MapUpdate>("Updater", 10);      
      sub_WM = this->create_subscription<teo_messages::msg::ScriptProgress>("ScriptProgress", 10, std::bind(&Handler::sub_WM_topic_callback, this, std::placeholders::_1));
      
      pub_Gens = this->create_publisher<teo_messages::msg::GenCmd>("GenIn", 10);
      sub_Gens = this->create_subscription<teo_messages::msg::ScriptOutcome>("GenOut", 10, std::bind(&Handler::sub_Gens_topic_callback, this, std::placeholders::_1));
      
      pub_CMD = this->create_publisher<std_msgs::msg::String>("OutGenHandler", 10);
      sub_CMD = this->create_subscription<teo_messages::msg::HandlerCmd>("InGenHandler", 10, std::bind(&Handler::sub_CMD_topic_callback, this, std::placeholders::_1));            
      std::cout << "[ScriptGeneratorHandler] >> Script generator initialized.\n";      
    }

  private:
    void sub_CMD_topic_callback(const teo_messages::msg::HandlerCmd::SharedPtr msg) const
    {
        std::cout << "[ScriptGeneratorHandler] >> HandlerCmd received.\n";
       // Switching upon the CMD field
       switch (msg->cmd) {
          // Case "INSERT"
          case 'I': 
           {
              std::cout << "[ScriptGeneratorHandler] >> It's an insert cmd.\n";           
              if(status=="running"){std::cout << "[ScriptGeneratorHandler] >> Current status is running! Skipping this comand.\n";return;}
              // Creating the data
              std::cout << "[ScriptGeneratorHandler] >> Executing the comand.\n";              
              auto temp = scriptgen();
              auto parsed = json::parse(msg->jsonmsg);
              
              temp.id = parsed["id"].get<std::string>();
              temp.path = "./src/world_model/scripts/idle_activity.json";             
              temp.jsonmsg = parsed["jsonmsg"].get<std::string>();  
              temp.completed = false;
              std::cout << "[ScriptGeneratorHandler] >> The following entry will be added:.\n";                                        
              std::cout << "                            -> id: " << temp.id << "\n";              
              std::cout << "                            -> path: " << temp.path << "\n";              
              std::cout << "                            -> jsonmsg: " << temp.jsonmsg << "\n";                                          
              ongoing.push_back(temp);     
              all_online=false;
              return;      
           }
           break;
          // Case "DELETE"
          case 'D': 
           {
              std::cout << "[ScriptGeneratorHandler] >> It's an delete cmd.\n";                      
              if(status!="paused"){std::cout << "[ScriptGeneratorHandler] >> Current status is running! Skipping this comand.\n";return;}           
              // Scanning the deque to Found the element and erase it
              for(int i = 0; i < ongoing.size(); i++)
              {
                 if(ongoing[i].id == msg->jsonmsg)
                 {
                    auto temp = teo_messages::msg::GenCmd();
                    temp.targetid = ongoing[i].id; 
                    temp.cmd = 'S';
                    temp.jsonmsg = "";
                    pub_Gens->publish(std::move(temp));
                    rclcpp::sleep_for(10ms);                 
                    std::cout << "[ScriptGeneratorHandler] >> Found the selected entity in ongoing!.\n";
                    std::cout << "[ScriptGeneratorHandler] >> The following entry will be deleted:.\n";                                        
                    std::cout << "                            -> id: " << ongoing[i].id << "\n";              
                    std::cout << "                            -> path: " << ongoing[i].path << "\n";              
                    std::cout << "                            -> jsonmsg: " << ongoing[i].jsonmsg << "\n";                                          
                    std::cout << "                            -> index: " << i << "\n";                         
                    ongoing.erase(ongoing.begin()+i);
                    return;
                 }                 
              }            
              return;
           }
           break;
          // Case "CHANGE ORDER"
          case 'O': 
           {
              std::cout << "[ScriptGeneratorHandler] >> It's an change order cmd.\n";                                 
              if(status!="paused"){std::cout << "[ScriptGeneratorHandler] >> Current status is running! Skipping this comand.\n";return;}           
              auto parsed = json::parse(msg->jsonmsg);
              // Scanning the deque to Found the element and change its position
              for(int i = 0; i < ongoing.size(); i++)
              {
                 if(ongoing[i].id == parsed["id"].get<std::string>())
                 {
                    std::cout << "[ScriptGeneratorHandler] >> Found the selected entity in ongoing!.\n";
                    std::cout << "[ScriptGeneratorHandler] >> The following entry will be moved:.\n";                                        
                    std::cout << "                            -> id: " << ongoing[i].id << "\n";              
                    std::cout << "                            -> path: " << ongoing[i].path << "\n";              
                    std::cout << "                            -> jsonmsg: " << ongoing[i].jsonmsg << "\n";                                          
                    std::cout << "                            -> from: " << i << "\n";                                   
                    std::cout << "                            -> to: " << parsed["index"].get<int>() << "\n";                                                       
                    auto temp = ongoing[i];
                    ongoing.erase(ongoing.begin()+i);
                    ongoing.insert(ongoing.begin()+parsed["index"].get<int>(),temp); 
                    return;
                 }                 
              }            
           }
           break;
          // Case "GET READY"
          case 'G': 
           {
              std::cout << "[ScriptGeneratorHandler] >> It's an get ready cmd.\n";                                            
              if(status!="paused"||all_online){std::cout << "[ScriptGeneratorHandler] >> Current status is running or a node is not online! Skipping this comand.\n";return;}
                 // Make ready
                 for(int i = ongoing.size()-1; i > -1; i--)
                 {
                   std::cout << "[ScriptGeneratorHandler] >> Trying to activate to: " << ongoing[i].id << ".\n";                                                         
                   auto temp = teo_messages::msg::GenCmd();
                   temp.targetid = ongoing[i].id; 
                   temp.cmd = 'A';
                   temp.jsonmsg = "";
                   pub_Gens->publish(std::move(temp));
                   rclcpp::sleep_for(10ms);
                 }
                 // Produce
                 for(int i = ongoing.size()-1; i > -1; i--)
                 {
                   std::cout << "[ScriptGeneratorHandler] >> Asking to produce to: " << ongoing[i].id << ".\n";                  
                   auto temp = teo_messages::msg::GenCmd();
                   temp.targetid = ongoing[i].id; 
                   temp.cmd = 'G';
                   temp.jsonmsg = ongoing[i].jsonmsg;
                   pub_Gens->publish(std::move(temp));
                   rclcpp::sleep_for(10ms);
                 }                   
           }
           break;
          // Case "START"
          case 'S': 
           {
                 std::cout << "[ScriptGeneratorHandler] >> It's a start cmd.\n";                                                       
                 if(status!="paused"||!all_online){std::cout << "[ScriptGeneratorHandler] >> Current status is running or a node is not online! Skipping this comand.\n";return;}           
                 auto to_send = teo_messages::msg::MapUpdate();
                 to_send.objectid = "Script";
                 to_send.attributename = "script_vect";
                 to_send.policy = 'R';
                 std::string queue = "";
                 bool addcomma = false;
                 for(int i = ongoing.size()-1; i > -1; i--)
                 {
                     if(!ongoing[i].completed)
                     {
                     
                         if(addcomma)
                         {
                            queue=queue+",";
                         }
                         else
                         {
                            current=i;
                            addcomma=true;                            
                         }              
                         queue=queue+"\""+ongoing[i].path+"\"";                
                         
                     }
                 }
                 to_send.jsonmsg = "{\"restart\":\"s_true\",  \"update\":["+queue+"]}";
                                  std::cout << "[ScriptGeneratorHandler] >> The queue is: " << queue <<"\n";                                                       
                 pub_WM->publish(std::move(to_send));
                 status="running";                 
           }
           break;                    
           
          // Case "STOP"
          case 'T': 
           {
              std::cout << "[ScriptGeneratorHandler] >> It's a stop cmd.\n";                                                                  
              if(status!="running"){std::cout << "[ScriptGeneratorHandler] >> Current status is paused! Skipping this comand.\n";return;}
              auto to_send = teo_messages::msg::MapUpdate();
              to_send.objectid = "Script";
              to_send.attributename = "script_vect";
              to_send.policy = 'R';
              to_send.jsonmsg = "{\"restart\":\"s_true\",  \"update\":[]}";
              pub_WM->publish(std::move(to_send));
              current = -1;
              status="paused";
           }
           break;                                            
          // Case "RESTART"
          case 'R': 
           {
                 std::cout << "[ScriptGeneratorHandler] >> It's a restart cmd.\n";                                                                             
                 if(status!="paused"||!all_online){std::cout << "[ScriptGeneratorHandler] >> Current status is running or a node is not online! Skipping this comand.\n"; return;}
                 for(int i=0;i<ongoing.size();i++)
                 {
                    ongoing[i].completed=false;           
                 }            
                 return;              
           }
           break;
          // Case "ERASE ALL"
          case 'E': 
           {
              std::cout << "[ScriptGeneratorHandler] >> It's an erase all cmd.\n";                                                                                        
              if(status=="running"){std::cout << "[ScriptGeneratorHandler] >> Current status is paused! Skipping this comand.\n";return;}
               // Make them stop
                 for(int i = ongoing.size()-1; i > -1; i--)
                 {
                   auto temp = teo_messages::msg::GenCmd();
                   temp.targetid = ongoing[i].id; 
                   temp.cmd = 'S';
                   temp.jsonmsg = "";
                   pub_Gens->publish(std::move(temp));
                   rclcpp::sleep_for(10ms);
                 }                         
              ongoing.clear();
              current = -1;
              status = "paused";
              all_online=false;
              return;    
           }
           break;
          // Case "SHUTDOWN"
          case 'W': 
           {
               std::cout << "[ScriptGeneratorHandler] >> It's an shutdown cmd.\n";                                                                                                   
              if(status!="paused"){std::cout << "[ScriptGeneratorHandler] >> Current status is running! Skipping this comand.\n";return;}     
              all_online=false;                   
              // Scanning the deque to Found the element and change its position
              for(int i = 0; i < ongoing.size(); i++)
              {
                 if(ongoing[i].id == msg->jsonmsg)
                 {
                  std::cout << "[ScriptGeneratorHandler] >> Found the selected entity in ongoing!.\n";
                    std::cout << "[ScriptGeneratorHandler] >> The following entry will be shutdowned:.\n";                                        
                    std::cout << "                            -> id: " << ongoing[i].id << "\n";              
                    std::cout << "                            -> path: " << ongoing[i].path << "\n";              
                    std::cout << "                            -> jsonmsg: " << ongoing[i].jsonmsg << "\n";                                          
                    std::cout << "                            -> index: " << i << "\n"; 
                   auto temp = teo_messages::msg::GenCmd();
                   temp.targetid = ongoing[i].id; 
                   temp.cmd = 'S';
                   temp.jsonmsg = "";
                   pub_Gens->publish(std::move(temp));
                   rclcpp::sleep_for(10ms);                   
                 }                 
              }                         
           }
           break;                                 
          // Case "SHUTDOWN ALL"
          case 'A': 
           {
                  std::cout << "[ScriptGeneratorHandler] >> It's an shutdown all cmd.\n";     
              all_online=false;                                                                                                               
                 if(status!="paused"){std::cout << "[ScriptGeneratorHandler] >> Current status is running! Skipping this comand.\n";return;}
                 // Make them stop
                 for(int i = ongoing.size()-1; i > -1; i--)
                 {
                   std::cout << "[ScriptGeneratorHandler] >> The following entry will be shutdowned: " << ongoing[i].id << ".\n";                                   
                   auto temp = teo_messages::msg::GenCmd();
                   temp.targetid = ongoing[i].id; 
                   temp.cmd = 'S';
                   temp.jsonmsg = "";
                   pub_Gens->publish(std::move(temp));
                   rclcpp::sleep_for(10ms);
                 }                    
           }
           break;                                 
          // Case "CURSTATUS"
          case 'U': 
           {
              std::cout << "[ScriptGeneratorHandler] >> It's an currstatus cmd.\n";                                                                                                              
              std::string output = "";
              if(all_online)
              {
                 output = output+"1@";
              }
              else
              {
                output = output +"0@";
              }
              if(status=="paused")
              {
                output = output + "0";
              }
              else
              { 
                output = output + "1";
              }  
              std::string to_go="@ONGOING";
              std::string to_comp="@COMPLETED";    
              std::string to_cur="@CURRENT#IDLE";          
              for(int i = 0; i < ongoing.size(); i++)
              {
                 if(i==current)
                 {
                    to_cur="@CURRENT#"+ongoing[i].id;
                 }
                 else if(ongoing[i].completed)
                 {
                    to_comp=to_comp+"#"+ongoing[i].id;
                 }
                 else
                 {
                    to_go=to_go+"#"+ongoing[i].id;
                 }  
              }
              output=output+to_go+to_cur+to_comp;
              auto temp = std_msgs::msg::String();
              temp.data=output;
              std::cout << "[ScriptGeneratorHandler] >> Sending: " << output << "\n";              
              pub_CMD->publish(std::move(temp));     
           }
           break;                                                       
       }
    
    }

  private:
    void sub_Gens_topic_callback(const teo_messages::msg::ScriptOutcome::SharedPtr msg) const
    {
              bool flag=true;
              // Scanning the deque to Found the element and add to it the path and checking if can set to true the all_online
              std::cout << "[ScriptGeneratorHandler] >> ScriptOutcome received.\n";              
              for(int i = 0; i < ongoing.size(); i++)
              {
                 if(ongoing[i].id == msg->id)
                 {
                    ongoing[i].path=msg->scriptpath;
                 }   
                 if(ongoing[i].path=="./src/world_model/scripts/idle_activity.json")
                 {
                   flag=false;
                 }             
                   std::cout << "[ScriptGeneratorHandler] >> The following entry was updated to:\n";                                        
                    std::cout << "                            -> id: " << ongoing[i].id << "\n";              
                    std::cout << "                            -> path: " << ongoing[i].path << "\n";              
                    std::cout << "                            -> jsonmsg: " << ongoing[i].jsonmsg << "\n";                                          
                    std::cout << "                            -> index: " << i << "\n";                   
              }              
              std::cout << "[ScriptGeneratorHandler] >> Setting all online to: " << flag << "\n";     
              all_online=flag;
                        
    }
        
  private:
    void sub_WM_topic_callback(const teo_messages::msg::ScriptProgress::SharedPtr msg) const
    {
      std::cout << "[ScriptGeneratorHandler] >> ScriptProgress received.\n";    
      if(current!=-1)
{
      if(msg->currscript!=ongoing[current].path)
      {
        std::cout << "[ScriptGeneratorHandler] >> Current script (" << ongoing[current].path << ") is different from the real one (" << msg->currscript << ").\n";
        // Case of "completed activity, going to the next"
        if(ongoing.size()!=0 && ongoing[current+1].path==msg->currscript)
        {
           ongoing[current].completed=true;
           current++;
        }
        else 
        // Case of a jump
        if( msg->currscript!="./src/world_model/scripts/idle_activity.json")
        {
          bool search=true;
          int look=0;
          for(int i=0;i<ongoing.size();i++)
          {
            ongoing[i].completed=false;
          }
          while(search && look<ongoing.size())
          {
             if(ongoing[look].path == msg->currscript)
             {
                current = look;
                search = false;
             }
             else
             { 
               ongoing[look].completed=true;
             }             
             look++;           
          }
          if(search)
          {
             // This is an "error handling" case
             std::cout << "[ScriptGeneratorHandler] >> ERROR! Activity not found!\n";
             return;
          }        
        }
        // now can be only the case of idle_activity
        else
        {
           // assuming that the current activity was stopped or was the last
           ongoing[current].completed=true;
           current=-1;           
           status = "paused";
        }
             }
             }
        std::string output = "";
              if(all_online)
              {
                 output = output+"1@";
              }
              else
              {
                output = output +"0@";
              }
              if(status=="paused")
              {
                output = output + "0";
              }
              else
              { 
                output = output + "1";
              }  
              std::string to_go="@ONGOING";
              std::string to_comp="@COMPLETED";    
              std::string to_cur="@CURRENT#IDLE";          
              for(int i = 0; i < ongoing.size(); i++)
              {
                 if(i==current)
                 {
                    to_cur="@CURRENT#"+ongoing[i].id;
                 }
                 else if(ongoing[i].completed)
                 {
                    to_comp=to_comp+"#"+ongoing[i].id;
                 }
                 else
                 {
                    to_go=to_go+"#"+ongoing[i].id;
                 }  
              }
              output=output+to_go+to_cur+to_comp;
              auto temp = std_msgs::msg::String();
              temp.data=output;
              std::cout << "[ScriptGeneratorHandler] >> Sending: " << output << "\n";              
              pub_CMD->publish(std::move(temp));         

    
  }
      
  rclcpp::Publisher<teo_messages::msg::MapUpdate>::SharedPtr pub_WM;
  rclcpp::Subscription<teo_messages::msg::ScriptProgress>::SharedPtr sub_WM;   
  rclcpp::Publisher<teo_messages::msg::GenCmd>::SharedPtr pub_Gens;
  rclcpp::Subscription<teo_messages::msg::ScriptOutcome>::SharedPtr sub_Gens;   
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_CMD;
  rclcpp::Subscription<teo_messages::msg::HandlerCmd>::SharedPtr sub_CMD;   
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  current=-1;
  rclcpp::spin(std::make_shared<Handler>());
  rclcpp::shutdown();
  return 0;
}
