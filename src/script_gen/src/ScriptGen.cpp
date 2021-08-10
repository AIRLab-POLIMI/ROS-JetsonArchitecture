#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "teo_messages/msg/gen_cmd.hpp"
#include "teo_messages/msg/script_outcome.hpp"
#include <iostream>
#include <fstream>

#include <nlohmann/json.hpp> // if needed to parse the message
using json = nlohmann::json;

using namespace std::chrono_literals;

bool ready = false; 
std::string genName = "ScriptGenExample";

class ScriptGen : public rclcpp::Node
{
  public:
    ScriptGen()
    : Node("ScriptGenExample")
    {
      std::cout << "[ScriptGenExample] >> Initializing the script generator.\n";
      pub = this->create_publisher<teo_messages::msg::ScriptOutcome>("GenOut", 10);
      sub = this->create_subscription<teo_messages::msg::GenCmd>("GenIn", 10, std::bind(&ScriptGen::topic_callback, this, std::placeholders::_1));
      std::cout << "[ScriptGenExample] >> Script generator initialized.\n";      
    }

  private:
    void topic_callback(const teo_messages::msg::GenCmd::SharedPtr msg) const
    {
      std::cout << "[ScriptGenExample] >> Recived a new CMD.\n";
      std::cout << "                   >> targetid: " << msg->targetid << ".\n";
      std::cout << "                   >> cmd: " << msg->cmd << ".\n";
      std::cout << "                   >> jsonmsg: " << msg->jsonmsg << ".\n";      
      
      // Checking that the message is for this precise generator, if not return
      if (genName != msg->targetid)
      {
          std::cout << "[ScriptGenExample] >> Not target of this message.\n";
          return;
      }
      
      std::cout << "[ScriptGenExample] >> Switching upon cmd value.\n";      
      // Checking the issued CMD
      if(msg->cmd == 71)
      {
         // The generation of a new script is required, check if the node is ready
         if(!ready)
         {
            return;
         }
         
         // Process here the jsonmsg
         // TODO ...
         // For sake of example, here it will generate a template script that test the robot
         
         // First of all, write the file in the folder "script" of the worldmodel
         std::string path = "src/world_model/scripts/test_act.json";
         std::ofstream myfile(path);        
         if(myfile.is_open())
         {std::cout << "[ScriptGenExample] >> file opened\n";}
         std::cout << "[ScriptGenExample] >> Writing in world_model/scripts/test_act.json.\n";        
         std::cout << "[ScriptGenExample] >> |";
 myfile << "{\n";
         std::cout << "|"; 
 myfile << "\"activityname\":\"test_act\",\n";
         std::cout << "|"; 
 myfile << "\"alfarulemessages\":[\n";
         std::cout << "|"; 
 myfile << "{\n";
         std::cout << "|"; 
 myfile << "\"rulecode\":0,\n";
         std::cout << "|"; 
 myfile << "\"id\":\"kick_starter\",\n";
         std::cout << "|"; 
 myfile << "\"intopic\":\"\",\n";
         std::cout << "|"; 
 myfile << "\"outtopic\":\"\",\n";
         std::cout << "|"; 
 myfile << "\"action\":67,\n";
         std::cout << "|"; 
 myfile << "\"mutexcmd\":85\n";
         std::cout << "|"; 
 myfile << "}\n";
         std::cout << "|"; 
 myfile << "],\n";
         std::cout << "|"; 
 myfile << "\"omegarulemessages\":[\n";
         std::cout << "|"; 
 myfile << "{\n";
         std::cout << "|"; 
 myfile << "\"rulecode\":0,\n";
         std::cout << "|"; 
 myfile << "\"id\":\"kick_starter\",\n";
         std::cout << "|"; 
 myfile << "\"intopic\":\"\",\n";
         std::cout << "|"; 
 myfile << "\"outtopic\":\"\",\n";
         std::cout << "|"; 
 myfile << "\"action\":82,\n";
         std::cout << "|"; 
 myfile << "\"mutexcmd\":76\n";
         std::cout << "|"; 
 myfile << "}\n";
         std::cout << "|"; 
 myfile << "],\n";
         std::cout << "|"; 
 myfile << "\"goal0\":{\n";
         std::cout << "|"; 
 myfile << "\"goalid\":\"test_act_goal\",\n";
         std::cout << "|"; 
 myfile << "\"act0\":{\n";
         std::cout << "|"; 
 myfile << "\"blk0\":{\n";
         std::cout << "|"; 
 myfile << "\"type\":\"instruction\",\n";
         std::cout << "|"; 
 myfile << "\"flagged\":\"flag\",\n";
         std::cout << "|"; 
 myfile << "\"move\":[\n";
         std::cout << "|"; 
 myfile << "{\n";
         std::cout << "|"; 
 myfile << "\"v_x\":\"10\",\n";
         std::cout << "|"; 
 myfile << "\"v_y\":\"0\",\n";
         std::cout << "|"; 
 myfile << "\"v_theta\":\"0\",\n";
         std::cout << "|"; 
 myfile << "\"wait_t\":1000,\n";
         std::cout << "|"; 
 myfile << "\"duration\":1000\n";
         std::cout << "|"; 
 myfile << "},\n";
         std::cout << "|"; 
 myfile << "{\n";
         std::cout << "|"; 
 myfile << "\"v_x\":\"0\",\n";
         std::cout << "|"; 
 myfile << "\"v_y\":\"10\",\n";
         std::cout << "|"; 
 myfile << "\"v_theta\":\"0\",\n";
         std::cout << "|"; 
 myfile << "\"wait_t\":1000,\n";
         std::cout << "|"; 
 myfile << "\"duration\":1000\n";
         std::cout << "|"; 
 myfile << "},\n";
         std::cout << "|"; 
 myfile << "{\n";
         std::cout << "|"; 
 myfile << "\"v_x\":\"0\",\n";
         std::cout << "|"; 
 myfile << "\"v_y\":\"-10\",\n";
         std::cout << "|"; 
 myfile << "\"v_theta\":\"0\",\n";
         std::cout << "|"; 
 myfile << "\"wait_t\":1000,\n";
         std::cout << "|"; 
 myfile << "\"duration\":1000\n";
         std::cout << "|"; 
 myfile << "},\n";
         std::cout << "|"; 
 myfile << "{\n";
         std::cout << "|"; 
 myfile << "\"v_x\":\"-10\",\n";
         std::cout << "|"; 
 myfile << "\"v_y\":\"0\",\n";
         std::cout << "|"; 
 myfile << "\"v_theta\":\"0\",\n";
         std::cout << "|"; 
 myfile << "\"wait_t\":1000,\n";
         std::cout << "|"; 
 myfile << "\"duration\":1000\n";
         std::cout << "|"; 
 myfile << "},\n";
         std::cout << "|"; 
 myfile << "{\n";
         std::cout << "|"; 
 myfile << "\"v_x\":\"0\",\n";
         std::cout << "|"; 
 myfile << "\"v_y\":\"0\",\n";
         std::cout << "|"; 
 myfile << "\"v_theta\":\"50\",\n";
         std::cout << "|"; 
 myfile << "\"wait_t\":1000,\n";
         std::cout << "|"; 
 myfile << "\"duration\":1000\n";
         std::cout << "|"; 
 myfile << "},\n";
         std::cout << "|"; 
 myfile << "{\n";
         std::cout << "|"; 
 myfile << "\"v_x\":\"0\",\n";
         std::cout << "|"; 
 myfile << "\"v_y\":\"0\",\n";
         std::cout << "|"; 
 myfile << "\"v_theta\":\"-50\",\n";
         std::cout << "|"; 
 myfile << "\"wait_t\":1000,\n";
         std::cout << "|"; 
 myfile << "\"duration\":1000\n";
         std::cout << "|"; 
 myfile << "}\n";
         std::cout << "|"; 
 myfile << "],\n";
         std::cout << "|"; 
 myfile << "\"led\":[\n";
         std::cout << "|"; 
 myfile << "{\n";
         std::cout << "|"; 
 myfile << "\"r\":\"1\",\n";
         std::cout << "|"; 
 myfile << "\"g\":\"0\",\n";
         std::cout << "|"; 
 myfile << "\"b\":\"0\",\n";
         std::cout << "|"; 
 myfile << "\"turnas\":1,\n";
         std::cout << "|"; 
 myfile << "\"wait_t\":0,\n";
         std::cout << "|"; 
 myfile << "\"duration\":2000\n";
         std::cout << "|"; 
 myfile << "},\n";
         std::cout << "|"; 
 myfile << "{\n";
         std::cout << "|"; 
 myfile << "\"r\":\"0\",\n";
         std::cout << "|"; 
 myfile << "\"g\":\"1\",\n";
         std::cout << "|"; 
 myfile << "\"b\":\"0\",\n";
         std::cout << "|"; 
 myfile << "\"turnas\":1,\n";
         std::cout << "|"; 
 myfile << "\"wait_t\":0,\n";
         std::cout << "|"; 
 myfile << "\"duration\":2000\n";
         std::cout << "|"; 
 myfile << "},\n";
         std::cout << "|"; 
 myfile << "{\n";
         std::cout << "|"; 
 myfile << "\"r\":\"0\",\n";
         std::cout << "|"; 
 myfile << "\"g\":\"0\",\n";
         std::cout << "|"; 
 myfile << "\"b\":\"1\",\n";
         std::cout << "|"; 
 myfile << "\"turnas\":1,\n";
         std::cout << "|"; 
 myfile << "\"wait_t\":0,\n";
         std::cout << "|"; 
 myfile << "\"duration\":2000\n";
         std::cout << "|"; 
 myfile << "},\n";
         std::cout << "|"; 
 myfile << "{\n";
         std::cout << "|"; 
 myfile << "\"r\":\"1\",\n";
         std::cout << "|"; 
 myfile << "\"g\":\"0\",\n";
         std::cout << "|"; 
 myfile << "\"b\":\"0\",\n";
         std::cout << "|"; 
 myfile << "\"turnas\":1,\n";
         std::cout << "|"; 
 myfile << "\"wait_t\":0,\n";
         std::cout << "|"; 
 myfile << "\"duration\":2000\n";
         std::cout << "|"; 
 myfile << "},\n";
         std::cout << "|"; 
 myfile << "{\n";
         std::cout << "|"; 
 myfile << "\"r\":\"0\",\n";
         std::cout << "|"; 
 myfile << "\"g\":\"1\",\n";
         std::cout << "|"; 
 myfile << "\"b\":\"0\",\n";
         std::cout << "|"; 
 myfile << "\"turnas\":1,\n";
         std::cout << "|"; 
 myfile << "\"wait_t\":0,\n";
         std::cout << "|"; 
 myfile << "\"duration\":2000\n";
         std::cout << "|"; 
 myfile << "},\n";
         std::cout << "|"; 
 myfile << "{\n";
         std::cout << "|"; 
 myfile << "\"r\":\"0\",\n";
         std::cout << "|"; 
 myfile << "\"g\":\"0\",\n";
         std::cout << "|"; 
 myfile << "\"b\":\"1\",\n";
         std::cout << "|"; 
 myfile << "\"turnas\":1,\n";
         std::cout << "|"; 
 myfile << "\"wait_t\":0,\n";
         std::cout << "|"; 
 myfile << "\"duration\":2000\n";
         std::cout << "|"; 
 myfile << "}\n";
         std::cout << "|"; 
 myfile << "],\n";
         std::cout << "|"; 
 myfile << "\"audioout\":[\n";
         std::cout << "|"; 
 myfile << "{\n";
         std::cout << "|"; 
 myfile << "\"track_path\":\"Silence\",\n";
         std::cout << "|"; 
 myfile << "\"wait_t\":1,\n";
         std::cout << "|"; 
 myfile << "\"duration\":1\n";
         std::cout << "|"; 
 myfile << "}\n";
         std::cout << "|"; 
 myfile << "]\n";
         std::cout << "|"; 
 myfile << "},\n";
         std::cout << "|"; 
 myfile << "\"blk1\":{\n";
         std::cout << "|"; 
 myfile << "\"type\":\"instruction\",\n";
         std::cout << "|"; 
 myfile << "\"flagged\":\"flag\",\n";
         std::cout << "|"; 
 myfile << "\"move\":[\n";
         std::cout << "|"; 
 myfile << "{\n";
         std::cout << "|"; 
 myfile << "\"v_x\":\"0\",\n";
         std::cout << "|"; 
 myfile << "\"v_y\":\"0\",\n";
         std::cout << "|"; 
 myfile << "\"v_theta\":\"0\",\n";
         std::cout << "|"; 
 myfile << "\"wait_t\":1000,\n";
         std::cout << "|"; 
 myfile << "\"duration\":1000\n";
         std::cout << "|"; 
 myfile << "}\n";
         std::cout << "|"; 
 myfile << "],\n";
         std::cout << "|"; 
 myfile << "\"led\":[\n";
         std::cout << "|"; 
 myfile << "{\n";
         std::cout << "|"; 
 myfile << "\"r\":\"1\",\n";
         std::cout << "|"; 
 myfile << "\"g\":\"1\",\n";
         std::cout << "|"; 
 myfile << "\"b\":\"1\",\n";
         std::cout << "|"; 
 myfile << "\"turnas\":1,\n";
         std::cout << "|"; 
 myfile << "\"wait_t\":1000,\n";
         std::cout << "|"; 
 myfile << "\"duration\":1000\n";
         std::cout << "|"; 
 myfile << "}\n";
         std::cout << "|"; 
 myfile << "],\n";
         std::cout << "|"; 
 myfile << "\"audioout\":[\n";
         std::cout << "|"; 
 myfile << "{\n";
         std::cout << "|"; 
 myfile << "\"track_path\":\"plugins/audio_control/songs/sounds_Excited_R2D2.wav\",\n";
         std::cout << "|"; 
 myfile << "\"wait_t\":0,\n";
         std::cout << "|"; 
 myfile << "\"duration\":4000\n";
         std::cout << "|"; 
 myfile << "}\n";
         std::cout << "|"; 
 myfile << "]\n";
         std::cout << "|"; 
 myfile << "},\n";
         std::cout << "|"; 
 myfile << "\"blk2\":{\n";
         std::cout << "|"; 
 myfile << "\"type\":\"jump\",\n";
         std::cout << "|"; 
 myfile << "\"toparse\":\"! {TRUE}\",\n";
         std::cout << "|"; 
 myfile << "\"truejump\":\"goal0/act0/blk4\",\n";
         std::cout << "|"; 
 myfile << "\"falsejump\":\"\"\n";
         std::cout << "|"; 
 myfile << "},\n";
         std::cout << "|"; 
 myfile << "\"blk3\":{\n";
         std::cout << "|"; 
 myfile << "\"type\":\"jump\",\n";
         std::cout << "|"; 
 myfile << "\"toparse\":\"! {TRUE}\",\n";
         std::cout << "|"; 
 myfile << "\"truejump\":\"goal0/act0/blk0\",\n";
         std::cout << "|"; 
 myfile << "\"falsejump\":\"\"\n";
         std::cout << "|"; 
 myfile << "},\n";
         std::cout << "|"; 
 myfile << "\"blk4\":{\n";
         std::cout << "|"; 
 myfile << "\"type\":\"terminus\"\n";
         std::cout << "|"; 
 myfile << "}\n";
         std::cout << "|"; 
 myfile << "}\n";
         std::cout << "|"; 
 myfile << "}\n";
         std::cout << "|\n"; 
 myfile << "}\n";
         std::cout << "[ScriptGenExample] >> Write operations completed\n";  
         myfile.close();
         std::cout << "[ScriptGenExample] >> File Closed\n";  
         // Publishing the message
         auto returnmsg = teo_messages::msg::ScriptOutcome();
         returnmsg.id = genName;
         returnmsg.scriptpath = path;
         pub->publish(std::move(returnmsg));      
      }
      else if(msg->cmd == 'A')
      {
         // If the node is already initialized
         if(ready)
         { return; }
         // Here instantiate everything is needed for this script generator
         ready = true;
      }
      else if(msg->cmd == 'S')
      {
         // If the node is already switched off
         if(!ready)
         { return; }
         // Here deallocate 

      }
      return;
    }

    rclcpp::Publisher<teo_messages::msg::ScriptOutcome>::SharedPtr pub;
    rclcpp::Subscription<teo_messages::msg::GenCmd>::SharedPtr sub;    

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScriptGen>());
  rclcpp::shutdown();
  return 0;
}
