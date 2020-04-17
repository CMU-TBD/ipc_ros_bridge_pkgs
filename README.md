# IPC_ROS_BRIDGE
Copyright - Transportation, Bots, and Disability Lab, Carnegie Mellon University  
Released under MIT License  

This ROS package relays ROS Topics between ROS & [IPC](https://cs.cmu.edu/~ipc) Worlds.

## How to Install:
1. Compile and build IPC library from [cs.cmu.edu/~ipc](https://cs.cmu.edu/~ipc).
2. Clone this repo into ROS workspace.
3. Set the `IPC_DIR` and `IPC_LIB_PATH` environment variable. The first should be the root of the IPC directory and the second is the path to the `libipc.a` static library.

## How to Run A Relay on ROS that transfer messages to and from IPC:
1. Check if the Message type already exist, if not create the definition of the ROS Topic type you going to use:
    1. First, create a header that defines the IPC `struct` and string format. Then, put it in the `include/structure` folder.
        ```
            #pragma once

            // Define the struct here
            typedef struct 
            {
                char* Data;
            } std_msgs_string;

            // Define the constants
            const char* std_msgs_string_format  = "{string}";
        ```
    2. Create a derived class from `IntermediateType<M>` that converts the `struct` into the correct Message type and vice versa. This is done through two virtual methods, `constructStructFromMessage` and `ContainerToMessage`. Here's an example for `std_msgs/String` message type.
        ```
            #pragma once
            #include "../intermediate_type.h"
            #include "std_msgs/String.h"
            #include "../structure/std_msgs_string.h"
            #include <iostream>


            class StdMsgsString: public IntermediateType<std_msgs::String>
            {
            public:

                StdMsgsString(const std::string name = "std_msgs_string")
                    :IntermediateType(name, std_msgs_string_format){
                }

                virtual void* constructStructFromMessage(void* _msg){
                    std_msgs::String *msg = (std_msgs::String *) _msg;
                    std_msgs_string *s = new std_msgs_string();
                    s->Data = &(msg->data[0]);
                    return s;
                }

                virtual std_msgs::String ContainerToMessage(void* _container)
                {
                    std_msgs::String msg;
                    // create new message
                    std_msgs_string *container = (std_msgs_string *) _container;
                    msg.data = container->Data;
                    return msg;
                }
            };
        ```
2. Write a ROS Node that create the linkage between ROS and IPC.
    ```
        auto ipc = IPCBridge(nh, "ROS_IPC_Demo");
        std::cout << "Connected, Starting Relay ...." << std::endl;
        // Relay Message from IPC to ROS and publish on topic "fromIPC"
        ipc.RelayTopicFromIPC<std_msgs::String, StdMsgsString>("ROSMSG1","fromIPC");
        // Relay the Message from topic "t" to IPC
        ipc.RelayTopicToIPC<std_msgs::String, StdMsgsString>("IPCMSG1","t"); 
        ipc.RelayTopicToIPC<std_msgs::Bool, StdMsgsBool>("IPCMSG2","b"); 

        std::cout << "Starting Wait ...." << std::endl;
        ros::Duration wait(10);
        ipc.spin(wait);

        std::cout << "Stopping Relay ...." << std::endl;

        // Stop Mesage
        ipc.StopRelayTopicFromIPC<std_msgs::String>("fromIPC");
        ipc.StopRelayTopicToIPC<std_msgs::String>("t"); 

        ipc.Disconnect();
    ```
3. For both `RelayTopicToIPC` and `RelayTopicFromIPC`, the first argument is the IPC Message Name and the second argument is the ROS Topic Name.
4. The messages will only be passed once `ipc.spin()` is called. You can also specify a duration with `ipc.spin(ros::Duration d)`.

## How to receive IPC message in C++ Ros Nodes:
Currently, we also support receiving IPC message in C++ ROS Node where the message is transformed into a ROS Message.
```

void receiveMsg(std_msgs::String msg, void* arg){
    std::cout << "Received:" << msg.data << std::endl;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "receive_node");
    ros::NodeHandle nh;
    auto ipc = IPCBridge(nh, "ros_ipc_receive_example");

    // Specify the ROS Message type to convert to and the converter
    ipc.ReceiveTopicFromIPC<std_msgs::String, StdMsgsString>("ROSMSG1", receiveMsg);
    ipc.spin();
    ipc.Disconnect();
```

## How to Run in IPC.
1. Make sure the compiler has reference to the directory of `include/structure` (example: `-I$(Home)/ros_ws/src/ipc_ros_bridge/include/structure`).
2. When defining the message, use the provided `struct` and `format` in the defined message.
    ```

    #include <iostream>
    #include <chrono>
    #include <thread>

    #include "ipc.h"
    #include "std_msgs_string.h" //The structure file defined in the ROS Package

    #define TASKNAME "ROS_IPC_Demo"
    #define MSGNAME "ROSMSG1"

    int main()
    {
        // connect to IPC Central
        // will fail if doesn't find it
        IPC_connect(TASKNAME);

        // define message
        IPC_defineMsg(MSGNAME, IPC_VARIABLE_LENGTH, std_msgs_string_format);

        // create data type
        std_msgs_string d;
        std::string tmp = "Hello from IPC World";
        d.Data = &tmp[0];    

        while(true){
            // send the message
            IPC_publishData(MSGNAME, &d);
            // sleep
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    ```

## [Changelog](CHANGELOG.md)