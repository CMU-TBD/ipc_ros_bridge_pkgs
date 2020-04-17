#include "ipc_ros_bridge/ipc_bridge.h"
#include "ros/ros.h"

IPCBridge::IPCBridge()
{
  // Empty Constructor
}

IPCBridge::IPCBridge(ros::NodeHandle _nh, std::string _taskName)
{
  Connect(_nh, _taskName);
}

// Deconstructor
IPCBridge::~IPCBridge()
{
  // Disconnect
  Disconnect();
}

void IPCBridge::Connect(ros::NodeHandle _nh, std::string _taskName)
{
  // Start IPC
  IPC_connect(&_taskName[0]);
  nh = _nh;
  taskName = _taskName;
  connected = true;
}

void IPCBridge::Disconnect()
{
  if (connected)
  {
    IPC_disconnect();
    connected = false;
  }
}

void IPCBridge::spin()
{
  ros::Duration d(0);
  spin(d);
}

void IPCBridge::spin(ros::Duration duration)
{
  auto startTime = ros::Time::now();
  // loop while ros is running
  while (ros::ok() && (duration.isZero() || (ros::Time::now() - startTime) <= duration))
  {
    // dispatch some IPC message
    IPC_listenWait(10);  // 10 milliseconds
    // spin once
    ros::spinOnce();
  }
}