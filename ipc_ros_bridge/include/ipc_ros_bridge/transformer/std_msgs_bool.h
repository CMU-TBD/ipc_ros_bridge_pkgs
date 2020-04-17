#pragma once
#include "../intermediate_type.h"
#include "std_msgs/Bool.h"
#include "../structure/std_msgs_bool.h"


class StdMsgsBool: public IntermediateType<std_msgs::Bool>
{
public:
    
    StdMsgsBool(const std::string name = STD_MSGS_BOOL_NAME)
        :IntermediateType(name, STD_MSGS_BOOL_FORMAT){
    }

    virtual void* constructStructFromMessage(void* _msg){
        std_msgs::Bool *msg = (std_msgs::Bool *) _msg;
        std_msgs_bool *s = new std_msgs_bool();
        s->Data = msg->data ? 1 : 0;
        return s;
    }

    virtual std_msgs::Bool ContainerToMessage(void* _container)
    {
        std_msgs::Bool msg;
        // create new message
        std_msgs_bool *container = (std_msgs_bool *) _container;
        msg.data = container->Data;
        return msg;
    }
};
