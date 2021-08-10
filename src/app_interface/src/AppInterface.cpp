#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <assert.h>
#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "teo_messages/msg/handler_cmd.hpp"

#define PORT 8888               //The server side listens to the port number, this port number can be customized
#define MAX_BUFFER 1024         //Maximum data buffer

std::string currentstatus = "";

using namespace std::chrono_literals;

class Messager : public rclcpp::Node
{
    public:
    Messager()
    : Node("Messager")
    {
      std::cout << "[Messager] >> Initializing the Messager.\n";
      pub_CMD = this->create_publisher<teo_messages::msg::HandlerCmd>("InGenHandler", 10);
      sub_CMD = this->create_subscription<std_msgs::msg::String>("OutGenHandler", 10, std::bind(&Messager::topic_callback, this, std::placeholders::_1));            
      std::cout << "[Messager] >> Messager initialized.\n";      
   }
   
   private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      // Updating the situation
      std::cout << "[Messager] >> Got update: " << msg->data << ".\n";
      currentstatus =  msg->data;
    }
   
   
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_CMD;
  rclcpp::Publisher<teo_messages::msg::HandlerCmd>::SharedPtr pub_CMD;   
  
  public: void send(char cmd, std::string jsonmsg)
  {
      auto msg = teo_messages::msg::HandlerCmd();
      msg.cmd = cmd;     
      msg.jsonmsg = jsonmsg;
      pub_CMD->publish(std::move(msg));  
  }
  
};

#define TAG "Service"
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;  
    struct sockaddr_in server_addr, client_addr;
    int server_sockfd, client_sockfd;
    int size, write_size;
    char buffer[MAX_BUFFER];  
    auto messager = std::make_shared<Messager>();   
    executor.add_node(messager);  
    
    auto spin_executor = [&executor]() {
    executor.spin();
    };

    // Launch both executors
    std::thread execution_thread(spin_executor);


    if ((server_sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1)    //Create Socket
    {
        perror("Socket Created Failed!\n");
        exit(1);
    }
    printf("Socket Create Success!\n");

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(PORT);
    bzero(&(server_addr.sin_zero), 8);

    int opt = 1;
    int res = setsockopt(server_sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));    //Set address multiplexing
    if (res < 0)
    {
        perror("Server reuse address failed!\n");
        exit(1);
    }

    if (bind(server_sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) == -1)  //Binding local address
    {
        perror("Socket Bind Failed!\n");
        exit(1);
    }
    printf("Socket Bind Success!\n");

    if (listen(server_sockfd, 5) == -1)                 //monitor
    {
        perror("Listened Failed!\n");
        exit(1);
    }
    printf("Listening ....\n");

    socklen_t len = sizeof(client_addr);

    printf("waiting connection...\n");
    if ((client_sockfd = accept(server_sockfd, (struct sockaddr *)&client_addr, &len)) == -1)  //Wait for the client to connect
    {
        perror("Accepted Failed!\n");
        exit(1);
    }

    printf("connection established!\n");
    printf("waiting message...\n");
    while (1)
    {
        memset(buffer, 0, sizeof(buffer));                             //Empty the data buffer

        if ((size = read(client_sockfd, buffer, MAX_BUFFER)) == -1)    //Read client data
        {
            perror("Recv Failed!\n");
            exit(1);
        }

        if (size != 0)                                               
        {
            buffer[size] = '\0';
            printf("Recv msg from client: %s\n", buffer);    
            std::string str(buffer);
            // Understand which command is
            std::stringstream splitted(str);
            std::string segment;
            std::vector < std::string > seglist;
            std::string output="";
            while (std::getline(splitted, segment, '#')) {
              seglist.push_back(segment);
            }
            // Case of restart
            std::cout << "[AppInterface] >> Processing: " << seglist[0] <<"\n";
            if(seglist[0]=="Restart")
            { 
               std::cout<<"[AppInterface] >> Restart cmd received!\n";
               messager->send('R',"");                         
               // TODO send restart message
            }
            else if(seglist[0]=="ClearAll")
            {
               std::cout<<"[AppInterface] >> ClearAll cmd received!\n";  
               messager->send('E',"");          
               // TODO send clear all message
            } 
            else if(seglist[0]=="UpdateMe")
            {
               // TODO send update back message
               std::cout<<"[AppInterface] >> UpdateMe cmd received!\n";   
               messager->send('U',"");                                                           
               rclcpp::sleep_for(100ms);
               output=currentstatus+"$";
            }
            else if(seglist[0]=="Insert")
            {
               // TODO send insert message, parsing it
               std::cout<<"[AppInterface] >> Insert cmd received!\n"; 
               messager->send('I',seglist[1]);              
            }            
            else if(seglist[0]=="Button")
            {
               if(seglist[1]=="0")
               {
                  //it's a get ready
                  std::cout<<"[AppInterface] >> GetReady cmd received!\n";
                  messager->send('G',"");
               }
               else if(seglist[1]=="1")
               {
                  //it's a start
                  std::cout<<"[AppInterface] >> Start cmd received!\n";  
                  messager->send('S',"");                  
               }
               else
               {
                  //it's a stop
                  std::cout<<"[AppInterface] >> Stop cmd received!\n";    
                  messager->send('T',"");                  
                  
               }
            }       
            else if(seglist[0]=="MoveFrom")
            {
               // TODO it's a stop
               std::cout<<"[AppInterface] >> MoveFrom cmd received!\n";  
               messager->send('O',seglist[1]);             
            }  
            else if(seglist[0]=="Delete")
            {
               // TODO it's a delete
                messager->send('D',seglist[1]);  
               std::cout<<"[AppInterface] >> Delete cmd received!\n";               
            }                                                                           
            strncpy(buffer, output.c_str(),output.size()); 

            if ((write_size = write(client_sockfd, buffer, MAX_BUFFER)) > 0)   //Send the received data back to the client
            {
                printf("Sent msg to client successfully!\n");
            }

        }
    }

    close(client_sockfd);   //Close Socket
    close(server_sockfd);
    rclcpp::shutdown();
    execution_thread.join();
    
}
