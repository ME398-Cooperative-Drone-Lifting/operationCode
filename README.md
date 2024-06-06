# To set up the Pi and other hardware in this framework:
## Option 1: Restore from image backup 
- Open Ubuntu 22.04 or 24.04 on a laptop or desktop computer
- Download the image from the Google Drive `/resources` folder
- Insert the SD card into the computer 
- Open the Disks program
- Select the SD card in the left sidebar
- Select `Restore from image...`
- Select the .img file you downloaded from the Drive
- Start flashing the SD card! This will take about an hour.
- Once done, eject the SD card and insert into the Pi. The RealSense and DroneKit bindings should be set up.

## Option 2: Set up from scratch [estimated time: 1-1.5 hours]
- Plug a 64 GB microSD card into your computer
- Install Ubuntu 22.04 for a Raspberry Pi 4B with the Raspberry Pi imager app
- Keep the SD attached to your computer, and open the `system-boot` directory
- Extend the USBfs buffer size to 2048 MB with the following step:
    - Add `usbcore.usbfs_memory_mb=2048` to the `cmdline.txt` file in the `/boot/` directory (using sd card adapter)
- Disable BT and add UART for DroneKit with the following step:
    - Add `dtoverlay=disable-bt` and `enable_uart=1` to the `config.txt` file in the `/boot/` directory
- Save the .txt files, eject the card, and insert it into the Pi (shiny 'pins' facing up into the PCB)
- Boot up the Pi and set it up **without WiFi**
- Once you're at the main desktop and logged in properly, begin setting up WiFi on the Pi
    - Using a personal computer already connected to WiFi, download the certificate from [here](https://services.northwestern.edu/TDClient/30/Portal/KB/ArticleDet?ID=1113) and copy it to a USB-A type thumb drive
    - Copy the certificate from the thumb drive onto the Pi desktop
    - Go to settings, select eduroam, select Protected EAP, upload certificate, set MSCHAPv2, add your username and password
    - Username: `netid@u.northwestern.edu`, password: `yourpassword`
- Update all system programs with `sudo apt-get update && sudo apt-get upgrade`
- Install Python's `pip` module with `sudo apt-get install -y python3-pip`
- Install Python 3.7 with the following steps:
    - `sudo add-apt-repository ppa:deadsnakes/ppa`
    - `sudo apt install -y python3.7 && sudo apt install -y python3.7-distutils`
    - `sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.7 1 && sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.10 2`
    - `sudo update-alternatives --config python` and select Python 3.7
- Install `openssh-server` with `sudo apt-get -y install openssh-server` on the Pi 4
    - Once the SSH server is set up, check the IP address in Settings/WiFi and write it down for recurring use in the execution section
- Install `video4linux` drivers with `sudo apt-get -y install v4l-utils`, required to assign udev rules
- Install DroneKit with `python3.7 -m pip install dronekit`
### If you are installing the RealSense SDK:
- Install the RealSense SDK 2.0 on the Pi 4. Follow the instructions [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md), taking care not to install the kernel patches (may take 1-2 hours to compile)
- Install opencv-contrib-python (includes additional libraries) with `python3.7 -m pip install opencv-contrib-python`
- Install the Python RealSense bindings with `python3.7 -m pip install pyrealsense2`

# Execution
- `ssh` into the Pi with the following Terminal/PowerShell argument:
    - `ssh -X pi@<ip_address>` (replace `<ip_address>` with the Pi's IP address)
- Once attached to the Pi, go to the proper directory with `cd operationCode/`
- Set up the RealSense display with `export DISPLAY=:0`
- Run `rs-enumerate-devices` to prime the RealSense USB buffer (mysterious, please do not ask why this works)
- Run the DroneKit code OR directly run the RealSense script with `python3 alignment_test.py`
- Once the script is complete, end with CTRL+C
- If you want to run the control script again while the Pi is still powered, you **must** unplug the RealSense, wait 3-5 seconds, and `rs-enumerate-devices` again before execution!

# Troubleshooting
- If the connection times out, unplug the RealSense camera, wait 3-5 seconds, and then replug in the **other** Pi USB 3.0 (blue USB) port
- The scripts will **likely** work if `rs-enumerate-devices` yields an immediate response
    - If `rs-enumerate-devices` hangs or takes more than 2 seconds to execute, unplug the camera and try again
