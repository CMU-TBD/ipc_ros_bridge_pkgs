#pragma once
#include "../intermediate_type.h"
#include "std_msgs/String.h"
#include "../structure/std_msgs_string.h"
#include <iostream>


class StdMsgsString: public IntermediateType<std_msgs::String>
{
public:

    StdMsgsString(const std::string name = STD_MSGS_STRING_NAME)
        :IntermediateType(name, STD_MSGS_STRING_FORMAT){
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
