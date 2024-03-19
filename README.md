# MIE438 Project - Gestura

## Setup Instructions

Assuming you are using VS Code:

1. Follow the tutorials [here](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/get-started/index.html) and [here](https://github.com/espressif/vscode-esp-idf-extension/blob/master/docs/tutorial/install.md) to install the ESP-IDF extension and everything else in the required toolchain.
    * On Linux, you may need to install udev rules to make the debugger work. The extension output is broken and will just tell you `[object Object]`; the correct udev rules can be found [here](https://github.com/espressif/openocd-esp32/blob/master/contrib/60-openocd.rules).
2. Connect *both* USB ports on the ESP32 -- the one labelled "USB" is for JTAG debugging/code upload and the one labelled "UART" is for the console.
3. At the bottom left corner, select the correct device to use, if it's not selected already. On Linux this will most likely be `/dev/ttyUSB0`. If both USB and UART are plugged in, there will also be a `/dev/ttyACM0`; this is for the USB/debugger while `/dev/ttyUSB0` is for the serial console.
3. If not selected already, at the bottom left corner, set the device target. Use esp32c6 as the target, and ESP32-C6 Chip (via builtin USB JTAG).
4. Build the project. There's a button on the bottom bar for this.
5. Once the project is built, if you want to set up IntelliSense or clangd, navigate to the new `build/` directory and copy the `compile_commands.json` file from inside the directory to the root of the repository. You may have to do another rebuild. After this, code completion should work.

There are buttons on the bottom bar to build and flash the project, and open the serial monitor to see `printf` outputs. Flashing may require you to start the openocd server.

## Useful Documentation

* [ESP32-C6-DevKitC-1 User Guide](https://docs.espressif.com/projects/espressif-esp-dev-kits/en/latest/esp32c6/esp32-c6-devkitc-1/user_guide.html)
* [ESP-IDF Get Started](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/get-started/index.html)
* [ESP-IDF VS Code Extension User Guide](https://github.com/espressif/vscode-esp-idf-extension/blob/master/docs/tutorial/basic_use.md)
* [ESP-IDF Examples](https://github.com/espressif/esp-idf/blob/master/examples/)

![ESP32-C6-DevKitC-1 Pinout](https://docs.espressif.com/projects/espressif-esp-dev-kits/en/latest/_images/esp32-c6-devkitc-1-pin-layout.png)
