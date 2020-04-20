#pragma once
#include "../intermediate_type.h"
#include "std_msgs/Int32.h"
#include "../structure/std_msgs_int32.h"


class StdMsgsInt32: public IntermediateType<std_msgs::Int32>
{
public:
    
    StdMsgsInt32(const std::string name = STD_MSGS_INT32_NAME)
        :IntermediateType(name, STD_MSGS_INT32_FORMAT){
    }

    virtual void* constructStructFromMessage(void* _msg){
        std_msgs::Int32 *msg = (std_msgs::Int32 *) _msg;
        std_msgs_int32 *s = new std_msgs_int32();
        s->Data = msg->data;
        return s;
    }

    virtual std_msgs::Int32 ContainerToMessage(void* _container)
    {
        std_msgs::Int32 msg;
        // create new message
        std_msgs_int32 *container = (std_msgs_int32 *) _container;
        msg.data = container->Data;
        return msg;
    }
};
