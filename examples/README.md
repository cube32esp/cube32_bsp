# CUBE32 Examples

Build and flash any example from that example's directory. The commands below
use `hello_lvgl` as an example.

## macOS

1. List USB serial ports and identify the board port:

   ```bash
   ls /dev/tty.*
   ```

2. Set up the ESP-IDF environment:

   ```bash
   get_idf
   ```

3. Change to the example directory:

   ```bash
   cd hello_lvgl
   ```

4. Optionally remove previous build output:

   ```bash
   idf.py fullclean
   ```

5. Configure, build, and flash the example:

   ```bash
   idf.py set-target esp32s3
   idf.py menuconfig
   idf.py build
   idf.py -p /dev/tty.wchusbserial110 flash
   ```

Replace `/dev/tty.wchusbserial110` with the port found in step 1.

## Windows

1. In PowerShell, list serial ports and identify the board port:

   ```powershell
   Get-CimInstance Win32_SerialPort | Select-Object DeviceID, Name
   ```

2. Set up the ESP-IDF environment:

   ```powershell
   get_idf
   ```

3. Change to the example directory:

   ```powershell
   cd hello_lvgl
   ```

4. Optionally remove previous build output:

   ```powershell
   idf.py fullclean
   ```

5. Configure, build, and flash the example:

   ```powershell
   idf.py set-target esp32s3
   idf.py menuconfig
   idf.py build
   idf.py -p COM3 flash
   ```

Replace `COM3` with the port found in step 1.
