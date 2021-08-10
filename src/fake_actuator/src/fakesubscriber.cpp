#include <memory>

#include <chrono>

#include <thread>

#include <stdlib.h>

using namespace std::chrono_literals;

#include <nlohmann/json.hpp>

using json = nlohmann::json;

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

#include "teo_messages/msg/map_update.hpp"

#include "teo_messages/msg/issued_block.hpp"

#include <mutex>

#include <functional>

#include <string>

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <fstream>

#include <atomic>

   // Controller related part

   // Reads a joystick event from the joystick device (0 on success, otherwise -1)
   int read_event(int fd, struct js_event *event)
   {
   
      ssize_t bytes;

      bytes = read(fd, event, sizeof(*event));

      if (bytes == sizeof(*event))
        return 0;

      /* Error, could not read full event. */
      return -1;
      
   }

   // Returns the number of axes on the controller or 0 if an error occurs.
   size_t get_axis_count(int fd)
   {
      
      __u8 axes;

      if (ioctl(fd, JSIOCGAXES, &axes) == -1)
        return 0;

      return axes;
      
   }


   // Returns the number of buttons on the controller or 0 if an error occurs.
   size_t get_button_count(int fd)
   {  
   
      __u8 buttons;
    
      if (ioctl(fd, JSIOCGBUTTONS, &buttons) == -1)
        return 0;

     return buttons;
     
   }
   
   // Current state of an axis.
   struct axis_state 
   {
   
      short x, y;
      
   };
   
   int js;
   struct js_event event;
   struct axis_state axes[3];
   size_t axis;

   // Keeps track of the current axis state.
   // NOTE: This function assumes that axes are numbered starting from 0, and that
   // the X axis is an even number, and the Y axis is an odd number. However, this
   // is usually a safe assumption.
   // Returns the axis that the event indicated.
   size_t get_axis_state(struct js_event *event, struct axis_state axes[3])
   {
      
      size_t axis = event->number / 2;

      if (axis < 3)
      {
      
         if (event->number % 2 == 0)
            axes[axis].x = event->value;
            
        else
            axes[axis].y = event->value;
             
      }

      return axis;
      
   }

   // End of controller related part

std::string block="";
json parsedblock;
bool valid=false;

bool moveph;
bool ledph;
bool aoutph;
bool block_is_over=false;


long unsigned int moveindex;
long unsigned int ledindex;
long unsigned int aoutindex;

int elapsedmove;
int elapsedled;
int elapsedaout;

// Xbox360 controller stuff
     std::thread t;
     std::atomic<bool> kill_read;
     float max_lin_speed_x=20;
     int lin_speed_step_x=5;     
     float max_lin_speed_y=20;
     int lin_speed_step_y=5;
     float max_ang_speed=120;
     int ang_speed_step=5;
     int abs_max_x=32767;
     int abs_max_y=32767;
     int abs_max_theta=32767;    
     std::string joystick_id="/dev/input/js0";
     int green_but=0;
     int red_but=1;
     int blue_but=2;
     int yellow_but=3;
     std::string green_col="L_ON_0 1 0 0_R_ON_0 1 0 1";
     std::string red_col="L_ON_1 0 0 1_R_ON_1 0 0 1";
     std::string blue_col="L_ON_0 0 1 1_R_ON_0 0 1 1";
     std::string yellow_col="L_ON_1 1 0 1_R_ON_1 1 0 1";
     long unsigned int speed_axis=0;
     int inv_speed=-1;
     long unsigned int rot_axis=2;
     int inv_rot=1;
     long unsigned int audio_axis=1;
     int audio_but_1=-32767;
     int audio_but_2=32767;
     int audio_but_3=-32767;          
     int audio_but_4=32767;     
     std::string audio_1_path="/home/FIXHERE/teo_ws/plugins/audio_control/songs/sounds_Excited_R2D2.wav";
     std::string audio_2_path="/home/FIXHERE/teo_ws/plugins/audio_control/songs/sounds_Laughing_R2D2.wav";
     std::string audio_3_path="/home/FIXHERE/teo_ws/plugins/audio_control/songs/sounds_Sad_R2D2.wav";
     std::string audio_4_path="/home/FIXHERE/teo_ws/plugins/audio_control/songs/polka_del_gatto.wav"; 
     int stop_but=6;
     int play_but=5;
     int pause_but=7;     
     int safety_lock=4;
     int lock_robot = 8;
     int safety_in;
     bool prevent_in = false;

     float speed_x_a;
     float speed_y_a;        
     float speed_y_b;
     float speed_x_b;
     float rot_a;
     
     std::mutex prevent;


rclcpp::Publisher<teo_messages::msg::MapUpdate>::SharedPtr pub;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_led;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_speed;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_audio; 
std::chrono::steady_clock::time_point clock_start;

     void ReadCMD()
      {  
      
         auto message_key = std_msgs::msg::String();
         auto message_audio = std_msgs::msg::String();                     
         auto message_joy = std_msgs::msg::String();   
                         
         std::cout << "[Actuator] >> Ready to listen to commands!\n";                  		
              
         /* This loop will exit if the controller is unplugged. */
         while (read_event(js, &event) == 0 || kill_read)
         {
            // flag to terminate the thread
            if(kill_read) 
            {
                
                return;
                 
            }            
            
            if(event.type == JS_EVENT_BUTTON && event.number == safety_lock) 
            {

               if (event.value==1)
               {
                                              
                  safety_in = 1;
                  std::cout << "[Actuator] >>Safety Lock disengaged\n";       
                  // shutting down everything
                  message_key.data = "L_OFF_LET_R_OFF_LET";
                  publisher_led->publish(message_key);
                  message_audio.data = "Play@";
                  publisher_audio->publish(message_audio);
                  message_joy.data="RST_";
                  publisher_speed->publish(message_joy); 

               } 
                     
               else if (event.value==0)
                     
               {
                      
                 safety_in = 0;
                 std::cout << "[Actuator] >>Safety Lock engaged\n";    
                 // shutting down everything
                 message_key.data = "L_OFF_LET_R_OFF_LET";
                 publisher_led->publish(message_key);
                 message_audio.data = "Pause@";
                 publisher_audio->publish(message_audio);
                 message_joy.data="RST_";
                 publisher_speed->publish(message_joy); 
                     
               }
               
               continue;  
                     
            }
            
            if (safety_in == 0)
	    {
	    
               std::cout << "[Actuator] >>Safety Lock engaged!\n"; 	                                                         	    
	       continue;
	    
	    }
            switch (event.type)
            {
         
                case JS_EVENT_BUTTON:
                {                
                   if(event.number == lock_robot)
                   {
                   
                     if (event.value==1)
                     {
                         prevent_in=!prevent_in;
                         if(prevent_in)
                         {
                            prevent.lock();
                         }
                         else
                         {
                           prevent.unlock();
                         } 
                     }
                     break;
                     
                   } else
                     
                   if(event.number == green_but) 
                   {
                      
                      if (event.value==1)
                      {
                        
                         message_key.data = green_col;
                         // >(>.<)<
                          
                      }
                        
                      else 
                      {
                        
                         message_key.data = "L_OFF_LET_R_OFF_LET";
                          
                      }
                      
                      std::cout << "[Actuator] >>Publishing message for led: " << message_key.data << ".\n";
                      publisher_led->publish(message_key);
                      break;
                     
                  } else 
                  
                  if(event.number == red_but) 
                  {
                     

                     if (event.value==1)
                     {
                       
                         message_key.data = red_col;
                         
                     }
                       
                     else 
                     {
                       
                         message_key.data = "L_OFF_LET_R_OFF_LET";
                         
                     }
                       
                     std::cout << "[Actuator] >>Publishing message for led: " << message_key.data << ".";                     
                     publisher_led->publish(message_key);
                     break;  
                         
                  } else 
                  
                  if(event.number == blue_but) 
                  {        
                               
                     if (event.value==1)
                     {
                       
                         message_key.data = blue_col;
                         
                     }
                       
                     else 
                     {
                      
                         message_key.data = "L_OFF_LET_R_OFF_LET";
                         
                     }
                     
                     std::cout << "[Actuator] >>Publishing message for led: " << message_key.data << ".";                     
                     publisher_led->publish(message_key);
                     break; 
                         
                  } else 
                  
                  if(event.number == yellow_but) 
                  {

                     if (event.value==1)
                     {
                       
                        message_key.data = yellow_col;
                         
                     }
                       
                     else 
                     {
                       
                         message_key.data = "L_OFF_LET_R_OFF_LET";
                         
                     }
                     
                     std::cout << "[Actuator] >>Publishing message for led: " << message_key.data << ".";                     
                     publisher_led->publish(message_key);
                     break;    
                                                                            
                  }  else 
                  
                  if(event.number == pause_but) 
                  {

                     if (event.value==1)
                     {
                       
                        message_audio.data = "Pause@";
                        std::cout << "[Actuator] >>Publishing message for audio: " <<  message_audio.data << ".";
                        publisher_audio->publish(message_audio);
                         
                     }                       
 
                     break;                                
                                                                            
                  }  else 
                  
                  if(event.number == stop_but) 
                  {

                     if (event.value==1)
                     {
                                              
                        message_audio.data = "Stop@";
                        std::cout << "[Actuator] >>Publishing message for audio: " <<  message_audio.data << ".";                             
                        publisher_audio->publish(message_audio);
                                                
                     }
                       
                     break;                        
                                                                            
                  }  else 
                  
                  if(event.number == play_but) 
                  {

                     if (event.value==1)
                     {
                                              
                        message_audio.data = "Play@";
                        std::cout << "[Actuator] >>Publishing message for audio: " <<  message_audio.data << ".";                             
                        publisher_audio->publish(message_audio);
                                                
                     }
                       
                     break; 
                     
                  }                                                                    
                               
               }   
               
               break;
               
                
               case JS_EVENT_AXIS:
               {
               
                  axis=get_axis_state(&event, axes);

                  if(axis == speed_axis || axis == rot_axis) 
                  {
                   
                     // Acting on whole speed
                     message_joy.data="RST_"+std::to_string(speed_x_a*((axes[speed_axis].y*lin_speed_step_x)/abs_max_x))+"_"+std::to_string(speed_y_a*((axes[speed_axis].x*lin_speed_step_y)/abs_max_y))+"_"+std::to_string((rot_a)*((-1*(axes[rot_axis].y-axes[rot_axis].x)/2*ang_speed_step)/abs_max_theta));

                     std::cout << "[Actuator] >>Publishing message for speed: " << message_joy.data << ".\n";
                     publisher_speed->publish(message_joy);
                     break;
                    
                  }  
                 
                  else if (axis == audio_axis)
                  {
                    
                     if(axes[audio_axis].y == audio_but_1) 
                     {
              
                        message_audio.data = "New@"+audio_1_path+"@0";
                        std::cout << "[Actuator] >>Publishing message for audio: " <<  message_audio.data << ".";                             
                        publisher_audio->publish(message_audio);
                        break;     
                         
                     }
                       
                     else if(axes[audio_axis].y == audio_but_2)
                     {

                        message_audio.data = "New@"+audio_2_path+"@0";
                        std::cout << "[Actuator] >>Publishing message for audio: " <<  message_audio.data << ".";                             
                        publisher_audio->publish(message_audio);
                        break;     
                         
                     }
                    
                     else if(axes[audio_axis].x == audio_but_3)
                     {

                        message_audio.data = "New@"+audio_3_path+"@0";
                        std::cout << "[Actuator] >>Publishing message for audio: " <<  message_audio.data << ".";                             
                        publisher_audio->publish(message_audio);
                        break;     
                         
                     }
                    
                     else if(axes[audio_axis].x == audio_but_4)
                     {

                        message_audio.data = "New@"+audio_4_path+"@0";
                        std::cout << "[Actuator] >>Publishing message for audio: " <<  message_audio.data << ".";                             
                        publisher_audio->publish(message_audio);
                        break;     
                         
                     }                                                              
                                                                           
                  }                                                                                                               
                 
               }
               
               break;
                 
               default:
                 /* Ignore init events. */
               break;
            
           }
        
        }
      
     }     

class FakeActuator : public rclcpp::Node
{
  public:
    FakeActuator()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<teo_messages::msg::IssuedBlock>(
      "InActuator", 10, std::bind(&FakeActuator::topic_callback, this, _1));
      pub=this->create_publisher<teo_messages::msg::MapUpdate>("Updater",10);
      
      // Creating the publisher for the led topic
      publisher_led = this->create_publisher<std_msgs::msg::String>("TeoSim/Led_Control", 10);
        
      // Creating the publisher for the speed topic
      publisher_speed = this->create_publisher<std_msgs::msg::String>("TeoSim/Speed_Control", 10);
        
      // Creating the publisher for the audio topic
      publisher_audio = this->create_publisher<std_msgs::msg::String>("TeoSim/Audio_Control", 10);  
      
       clock_start = std::chrono::steady_clock::now();
       
    }

  private:
    void topic_callback(const teo_messages::msg::IssuedBlock::SharedPtr msg) const
    { 
      std::cout << "[Actuator] >> Recieved a new message for the World Model.\n";
      std::cout << msg->block;      
      // Checking if the issued block is changed or not
      if(msg->changed)
      {
         std::cout << "[Actuator] >> Received a new block!\n";
         
         block = msg->block;
         parsedblock = json::parse(block);
         // Resetting index
         moveindex=0;
         auto tempm = teo_messages::msg::MapUpdate();
         tempm.objectid = "Script";
         tempm.attributename = "submove";
         tempm.policy = 'R';
         tempm.jsonmsg = std::to_string(moveindex);
         pub->publish(std::move(tempm));	         
         ledindex=0;
         auto templ = teo_messages::msg::MapUpdate();
         templ.objectid = "Script";
         templ.attributename = "subled";
         templ.policy = 'R';
         templ.jsonmsg = std::to_string(ledindex);
         pub->publish(std::move(templ));	         
         aoutindex=0;
         auto tempa = teo_messages::msg::MapUpdate();
         tempa.objectid = "Script";
         tempa.attributename = "subaout";
         tempa.policy = 'R';
         tempa.jsonmsg = std::to_string(aoutindex);
         pub->publish(std::move(tempa));	         
         elapsedmove=0;
         elapsedled=0;
         elapsedaout=0;
         
         moveph=false;
         ledph=false;
         aoutph=false;
         block_is_over=false;         
         valid=true;

      }      
      else
      {
         std::cout << "[Actuator] >> Received the old block!\n";                 
      }
      
    }
    rclcpp::Subscription<teo_messages::msg::IssuedBlock>::SharedPtr subscription_;
};

void processing()
{
   while(1)
   {
      if(valid)
      {   
          while(1)
          {
           if(prevent.try_lock()){break;}
           std::this_thread::sleep_for(100ms);
           std::cout<<"lock failed\n";
          }
          // checking if there is an move under the head                
          if(parsedblock["move"].size()>moveindex)
          {      
              // checking if it is waiting
              if(!moveph)
              {
                  // it is in wait phase
                  // checking if it's elapsed enought time
                  if(elapsedmove>=parsedblock["move"][ledindex]["wait_t"])
                  {
                     // if yes, set the flag for the next phase
                     moveph=true;
                     elapsedmove=0;
                     // send the move to the simulator
                     auto temp_message = std_msgs::msg::String(); 
                     temp_message.data = "RST_"+parsedblock["move"][moveindex]["v_x"].get<std::string>()+"_"+parsedblock["move"][moveindex]["v_y"].get<std::string>()+"_"+parsedblock["move"][moveindex]["v_theta"].get<std::string>();
                     std::cout << temp_message.data << "\n";
                     publisher_speed->publish(temp_message);                     
                     std::cout << "[Actuator] >> waited enougth for move!\n";
                  }
              }
              // checking if it is executing
              if(moveph)
              {
                 // it is in action phase
                 // checking if it's elapsed enought time
                  if(elapsedmove>=parsedblock["move"][moveindex]["duration"])
                  {
                     // the action is complete!
                     std::cout << "[Actuator] >> move completed!\n";       
                     // Send the update to the WM
                     auto upd = teo_messages::msg::MapUpdate();
                     upd.objectid = "RobotStatus";
                     upd.attributename = "speed";
                     upd.policy = 'A';
                     std::chrono::steady_clock::time_point clock_now = std::chrono::steady_clock::now();
                     upd.jsonmsg = "{\"xspeed\": "+ parsedblock["move"][moveindex]["v_x"].get<std::string>() + ", \"yspeed\": " + parsedblock["move"][moveindex]["v_y"].get<std::string>() + ", \"thetaspeed\": " + parsedblock["move"][moveindex]["v_theta"].get<std::string>() + ", \"duration\": " + std::to_string(parsedblock["move"][moveindex]["duration"].get<int>()) + ",  \"startingtime\": " +  std::to_string(std::chrono::duration_cast < std::chrono::milliseconds > (clock_now - clock_start).count()) + "}";
                     pub->publish(std::move(upd));	                                                             
                     moveindex++;
                     moveph=false;
                     elapsedmove=0;
                     // Send the completion message to the WM
                     auto temp = teo_messages::msg::MapUpdate();
                     temp.objectid = "Script";
                     temp.attributename = "submove";
                     temp.policy = 'R';
                     temp.jsonmsg = std::to_string(moveindex);
                     pub->publish(std::move(temp));	
                     // send a move reset to the simulator
                     auto temp_message = std_msgs::msg::String(); 
                     temp_message.data="RST_";
                     publisher_speed->publish(temp_message);                 
                  }
              }
          }         
          // checking if there is an audioout under the head                
          if(parsedblock["audioout"].size()>aoutindex)
          {      
              // checking if it is waiting
              if(!aoutph)
              {
                  // it is in wait phase
                  // checking if it's elapsed enought time
                  if(elapsedaout>=parsedblock["audioout"][aoutindex]["wait_t"])
                  {
                     // if yes, set the flag for the next phase
                     aoutph=true;
                     elapsedaout=0;
                     // send the audio to the simulator
                     auto temp_message = std_msgs::msg::String(); 
                     std::cout << parsedblock["audioout"][aoutindex] << "\n";
                     temp_message.data = "New@"+parsedblock["audioout"][aoutindex]["track_path"].get<std::string>();
                     std::cout << temp_message.data << "\n";
                     if(parsedblock["audioout"][aoutindex]["track_path"].get<std::string>()!="Silence")
                     {
                        publisher_audio->publish(temp_message);                     
                     }
                     else
                     {
                        std::cout << "[Actuator] >> Silence... \n";
                     }
                     std::cout << "[Actuator] >> waited enougth for audio!\n";
                  }
              }
              // checking if it is executing
              if(aoutph)
              {
                 // it is in action phase
                 // checking if it's elapsed enought time
                  if(elapsedaout>=parsedblock["audioout"][aoutindex]["duration"])
                  {
                     // the action is complete!
                     std::cout << "[Actuator] >> audio completed!\n";      
                     // Send the update to the WM
                     auto upd = teo_messages::msg::MapUpdate();
                     upd.objectid = "RobotStatus";
                     upd.attributename = "audioout";
                     upd.policy = 'A';
                     std::chrono::steady_clock::time_point clock_now = std::chrono::steady_clock::now();
                     upd.jsonmsg = "{\"duration\": " + std::to_string(parsedblock["audioout"][aoutindex]["duration"].get<int>()) + ",  \"startingtime\": " + std::to_string(std::chrono::duration_cast < std::chrono::milliseconds > (clock_now - clock_start).count()) + ", \"trackpath\": \"" + parsedblock["audioout"][aoutindex]["track_path"].get<std::string>() + "\"}" ;                     
                     pub->publish(std::move(upd));                                                          
                     aoutindex++;
                     aoutph=false;
                     elapsedaout=0;
                     // Send the completion message to the WM
                     auto temp = teo_messages::msg::MapUpdate();
                     temp.objectid = "Script";
                     temp.attributename = "subaout";
                     temp.policy = 'R';
                     temp.jsonmsg = std::to_string(aoutindex);
                     pub->publish(std::move(temp));
                     // send a audio reset to the simulator
                     auto temp_message = std_msgs::msg::String(); 
                     temp_message.data="Stop@";
                     publisher_audio->publish(temp_message);  
                  }
              }
          }  
          // checking if there is a led under the head
          if(parsedblock["led"].size()>ledindex)
          {
              // checking if it is waiting
              if(!ledph)
              {
                  // it is in wait phase
                  // checking if it's elapsed enought time
                  if(elapsedled>=parsedblock["led"][ledindex]["wait_t"])
                  {
                     // if yes, set the flag for the next phase
                     ledph=true;
                     elapsedled=0;
                     // send the move to the simulator
                     auto temp_message = std_msgs::msg::String(); 
                     std::cout << parsedblock["led"][ledindex]["turnas"] << "\n";
                     temp_message.data = "L_ON_"+parsedblock["led"][ledindex]["r"].get<std::string>()+" "+parsedblock["led"][ledindex]["g"].get<std::string>()+" "+parsedblock["led"][ledindex]["b"].get<std::string>()+" "+"1"+"_R_ON_"+parsedblock["led"][ledindex]["r"].get<std::string>()+" "+parsedblock["led"][ledindex]["g"].get<std::string>()+" "+parsedblock["led"][ledindex]["b"].get<std::string>()+" "+"1";
                     if(parsedblock["led"][ledindex]["turnas"]==1)
                     {
                        std::cout << temp_message.data << "\n";
                        publisher_led->publish(temp_message);                     
                     }
                     std::cout << "[Actuator] >> waited enougth for led!\n";
                  }
              }
              // checking if it is executing
              if(ledph)
              {
                 // it is in action phase
                 // checking if it's elapsed enought time
                  if(elapsedled>=parsedblock["led"][ledindex]["duration"])
                  {
                     // the action is complete!
                     std::cout << "[Actuator] >> led completed!\n";                                           
                     ledindex++;
                     ledph=false;
                     elapsedled=0;
                     // Send the completion message to the WM
                     auto temp = teo_messages::msg::MapUpdate();
                     temp.objectid = "Script";
                     temp.attributename = "subled";
                     temp.policy = 'R';
                     temp.jsonmsg = std::to_string(ledindex);
                     pub->publish(std::move(temp));
                     // send a move reset to the simulator
                     auto temp_message = std_msgs::msg::String(); 
                     temp_message.data="L_OFF_LET_R_OFF_LET";
                     publisher_led->publish(temp_message);  
                  }
              }
          }     
         prevent.unlock();                                         
      }

      // if the block is over and is re-submitted, allow to re-execute it
       if ((parsedblock["move"].size() == (moveindex)) && (parsedblock["led"].size() == (ledindex)) && (parsedblock["audioout"].size() == (aoutindex))) {
      std::cout << "[Actuator] >> consumed the entire block!\n";
      block_is_over=true;
          // Printing current progress
         std::cout << "[Actuator]: moves = " << moveindex << "/" << parsedblock["move"].size() << ", leds = " << ledindex << "/" << parsedblock["led"].size() << ", audioout = " << aoutindex << "/" << parsedblock["audioout"].size() << "\n";          
      }


      // Let 100ms pass before checking conditions, this is just an approximation
      if (!rclcpp::sleep_for(100ms)) {
         return; // Return if the sleep failed (e.g. on ctrl-c).
      }
               
      if(valid)
      {                  
         // Incrementing the elapsed time for all
         elapsedmove+=100;
         elapsedled+=100;
         elapsedaout+=100;
         
      }

   }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  js = open(joystick_id.c_str(), O_RDONLY);
  speed_x_a = max_lin_speed_x*inv_speed/lin_speed_step_x;
  speed_y_a = max_lin_speed_y*inv_speed/lin_speed_step_y;        
  rot_a = (inv_rot*max_ang_speed)/ang_speed_step;  
  safety_in = 0;  
  kill_read=false;
  t=std::thread(ReadCMD);  
  std::thread actuator(processing);
  rclcpp::spin(std::make_shared<FakeActuator>());
  rclcpp::shutdown();  
  actuator.join();   
  kill_read=true;
  t.join();   
   close(js);  
  return 0;
}
