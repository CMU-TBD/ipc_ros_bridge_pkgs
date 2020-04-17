# Change log

## [0.2.0] - 2020-04-17
### Changed:
- Reorganized the packages and made it a meta package.
    - Split examples into a seperate repo
    - reorganized the header to only contain template methods and move the non-template methods into a cpp file.
- Made `RelayTopicFromIPC` to rely on `ReceiveTopicFromIPC` using callbacks.
- Made `StopRelayTopicFromIPC` to rely on `StopReceiveTopicFromIPC` using callbacks.

### Added:
- Added `ReceiveTopicFromIPC` where users can register callbacks to be called when we receive an IPC Message. The callback has the format `void (M, void*)` where `M` is the message and `void*` is a pointer that you can use to pass in additional arguments to the callback function. The callback signature might change in the future, worth exploring whether `std::bind` or other ways of attaching info to callback might be better. `void*` also increase chance of memory leak.
- Added `StopReceiveTopicFromIPC` for stopping the callbacks

### Fixed:
- Fixed issue where other packages cannot use the library and header.

## [0.1.0] - 2020-03-02
Initial Release 