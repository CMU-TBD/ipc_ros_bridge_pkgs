

#include <iostream>

#include "ipc.h"
#include "ipc_ros_bridge/structure/std_msgs_string.h"

#define TASKNAME "ROS_IPC_Demo"
#define MSGNAME "IPCMSG1"

static void msgHandler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
                       void *clientData)
{
    // location to save the data
    std_msgs_string* data = (std_msgs_string*) callData;
    // print data
    std::cout << data->Data << std::endl;

    // free up the used data
    // IPC_freeDataElements(IPC_msgInstanceFormatter(msgRef), &data);
    IPC_freeByteArray(callData);
}

int main()
{

    // connect to IPC Central
    // will fail if doesn't fine it
    IPC_connect(TASKNAME);

    // define message
    IPC_defineMsg(MSGNAME, IPC_VARIABLE_LENGTH, STD_MSGS_STRING_FORMAT);

    // subscribe to message
    IPC_subscribeData(MSGNAME, msgHandler, (void *)TASKNAME);

    // run forever 
    IPC_dispatch();
}
