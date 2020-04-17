
#pragma once
#include <iostream>
#include "ipc.h"
#include "ros/ros.h"

template <class M>
class IntermediateType
{
protected:
    std::string name;
    char* format;
public:

    IntermediateType(const std::string _name, const char* _format):
        name(_name)
    {
        format = const_cast<char*>(_format);
    }

    // For ROS -> IPC
    virtual void* constructStructFromMessage(void* msg) {};
    // For IPC -> ROS
    virtual M ContainerToMessage(void* container) {};

    char* getName(){
        return &name[0];
    }

    char* getFormatString(){
        return format;
    }
};
