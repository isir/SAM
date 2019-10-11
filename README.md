# SAM

## QtCreator setup

### Device

Tools > Options > Devices

Add a new `Generic Linux Device` with these parameters:

- Hostname/IP address: `10.10.0.1`
- Username: `root`

### Kit

Tools > Options > Kits

Add the debugger: `/path/to/sdk/sysroots/x86_64-soysdk-linux/usr/bin/arm-oe-linux-gnueabi/arm-oe-linux-gnueabi-gdb`

Add the compiler: `/path/to/sdk/sysroots/x86_64-soysdk-linux/usr/bin/arm-oe-linux-gnueabi/arm-oe-linux-gnueabi-g++`

Finally, create the kit: 

- Device type: `Generic Linux Device`
- Device: Use the one you created earlier
- Sysroot: `/path/to/sdk/sysroots/cortexa7t2hf-neon-vfpv4-oe-linux-gnueabi`
- Compiler: 
  - C: `<No compiler>`
  - C++: Use the one you created earlier
- Debugger: Use the one you created earlier
- Qt version: `None`
  
### CMake configuration

Projects > Build > Cmake > Add File

![Image](/doc/images/cmake_toolchain_file.png)

Key | Value
------------ | -------------
CMAKE_TOOLCHAIN_FILE | /path/to/sdk/sysroots/x86_64-soysdk-linux/usr/share/cmake/OEToolchainConfig.cmake

Projects > Build > Cmake > Add String

Key | Value
------------ | -------------
CMAKE_TRY_COMPILE_TARGET_TYPE | STATIC_LIBRARY

Projects > Run > Alternate executable on device: `/opt/sam`

### Formatting

- Help > About plugins > Enable Beautifier
- Tools > Settings > Beautifier
  - General tab > Select `ClangFormat` and tick `Enable auto format on file save`
  - Clang Format tab > Select `WebKit` as the predefined style
