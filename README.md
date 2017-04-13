
GlobeCopter LED

## TODOs
- Add a Kalman filter.
- convert arduino sketch in enc directory to ESP32-idf format. 

## Abstract
This code is a program for the GlobeCopter LED ring that runs on the ESP32 board.
One core is used to send the SPI signals to the LED when a signal is received.
The other core has a Kalman filter and controls the motor with PWM. When the ring comes to
a specific angle, it will send a signal to the other core.

## How to build

### Create a cross compile environment

### Build the project
Run the following command at the root directory of this repository.
```
make
```

## Configure the ESP32
It's almost ok with the default settings, but you have to enable dual core mode to have this
operate properly.
To configure the ESP32, execute the following command.
```
make menuconfig
```

## Flash the built binary to the ESP32
Run the following command at the same directory where you built the project.
```
make flash
```

## Run the code at the ESP32
After flashing to the ESP32, it will automatically restart, and the project will be executed automatically.

## Debbuging
There are logging features defined by macros in ESP32. In order to view the logging console, run the following command.
```
make monitor
```

## Troubleshooting
### Flashing fails.
The device boot mode is not set properly. I'll recomend you to use the ESP32 dev kit, and the flashing tool will do the reset for you.




