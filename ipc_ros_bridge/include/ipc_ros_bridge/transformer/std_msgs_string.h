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

    virtual bool constructStructFromMessage(void* _msg, void* &ptr){
        std_msgs::String *msg = (std_msgs::String *) _msg;
        std::cout << msg->data << std::endl;
        // std_msgs_string *s = new std_msgs_string();
        // s->Data = &(msg->data[0]);
        char* cstr = new char[msg->data.size() + 1];
        std::strcpy (cstr, msg->data.c_str());
        std::cout << cstr << std::endl;
        ptr = cstr;
        return true;
    }

    virtual void publishData(void* _msg){
        std_msgs::String *msg = (std_msgs::String *) _msg;
        const char* cstr = msg->data.c_str();
        IPC_publishData(getName(), &cstr);
    }

    virtual std_msgs::String ContainerToMessage(void* _container)
    {
        std_msgs::String msg;
        // create new message
        char ** container = (char **) _container;
        msg.data = std::string(*container);
        return msg;
    }
};
