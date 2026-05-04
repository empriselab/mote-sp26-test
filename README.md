# mote-sp26-test
directory to test the lrr-fa24-beta homeworks on Mote framework

# Guide to running ROS and these HWs on Mote
## First time setup
1. Navigate to the most recent release in the Mote-Core repo (https://github.com/empriselab/mote-core/releases/tag/mote-firmware-v0.0.0) and download the .uf2 file.
2. Hold down the bootsel button on the Pico (located next to the usb port) and then plug in the Pico to your computer
3. Drag the .uf2 file to the newly mounted RP2350 drive.
4. Navigate to https://empriselab.github.io/mote-core/configuration/ and connect to the Pico
5. Register the MAC address here (https://it.cornell.edu/wifi/register-device-doesnt-have-browser) to connect to RedRover (you may need to wait a few minutes or power cycle the board for it to get registered)
6. Navigate back to the configuration page, connect to the Pico, click on RedRover and hit enter with an empty password.
7. You should be connected! Now note your IP address for the next part.
8. Next, clone this repo and head to the usual setup section (you can skip connected to RedRover).

If you want to work off of the current branch instead of the latest release, you have to follow the setup guide in the contribution guide here (https://empriselab.github.io/mote-core/advanced/contributing/contributing.html).

## Usual setup
1. Plug in the Pico to your computer
2. Navigate to https://empriselab.github.io/mote-core/configuration/ and connect to RedRover by inputting an empty password
3. Note down your IP address
4. Go into your terminal and cd into to your local copy of this repo
5. change the .env file in this repo's folder to your IP
6. Build the docker container via `docker build .` then `docker compose up` or navigate to this folder in vs code and hit `ctrl+shift+p` then search and select "Rebuild and Reopen in container"
7. Upon connecting to your docker container, the package `mote_base` will be launched (as per https://github.com/empriselab/mote-sp26-test/blob/main/docker-entrypoint.sh), allowing you to immediately drive the robot around using Foxglove (more info in HW2)
8. Now you should be able to follow along with the HWs in the src folder.
