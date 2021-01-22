# Change log

<!-- ## [Unreleased] -->

## Known Issues:
1. Our implementation does not support the `query/response` design.

## [0.3.1] - 2021-01-21
- **[Fixed]** The CMAKELIST now puts in the correct kernal version.
- **[Fixed]** Change virtual function to pure virtual function to prevent warnings.

## [0.3.0] - 2020-04-28
#### Added
- Added definitions for `geometer_msgs::Point`, `geometer_msgs::Quaternion`, and `geometer_msgs::Pose`.
- Added additional examples for them.

#### Changed
- [**Breaking**] Changed the way publish work, instead of returning the value, you now call `sendToIPC(void* data)` to send the data. This removed the complexity of managing memory of created items.
- [**Breaking**] Changed how single element ros messages are converted, instead of a struct, it now just sends the data with no wrappers. This increases the flexibility on the IPC side.
- Updated all existing definition/structures to match new patterns.

#### Removed
- Removed `void *` parameter for all callbacks. To pass variables to the callback functions, use `std::bind`.

## [0.2.1] - 2020-04-21
#### Added
- Added method `isConnected` to check if the program is still connected to IPC.

#### Changed
- Check if IPC is connected before calling IPC related methods
- Changed the default level of `IPC_VERBOSITY` to  `IPC_Exit_On_Errors`. The original behaviors is to exit upon error instead of handling it. Now, we handle the errors.

#### Fixed
- Fixed definition/structure files where `std_msgs::Float64` and `std_msgs::Int32` were using pointers in the struct instead of actual value.

## [0.2.0] - 2020-04-17
#### Changed:
- Reorganized the packages and made it a meta package.
    - Split examples into a seperate repo
    - reorganized the header to only contain template methods and move the non-template methods into a cpp file.
- Made `RelayTopicFromIPC` to rely on `ReceiveTopicFromIPC` using callbacks.
- Made `StopRelayTopicFromIPC` to rely on `StopReceiveTopicFromIPC` using callbacks.

#### Added:
- Added `ReceiveTopicFromIPC` where users can register callbacks to be called when we receive an IPC Message. The callback has the format `void (M, void*)` where `M` is the message and `void*` is a pointer that you can use to pass in additional arguments to the callback function. The callback signature might change in the future, worth exploring whether `std::bind` or other ways of attaching info to callback might be better. `void*` also increase chance of memory leak.
- Added `StopReceiveTopicFromIPC` for stopping the callbacks

#### Fixed:
- Fixed issue where other packages cannot use the library and header.

## [0.1.0] - 2020-03-02
Initial Release 