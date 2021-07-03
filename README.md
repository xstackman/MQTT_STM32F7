# MQTT_STM32F7

This code setup a MQTT Client in a STM32F767Zi Nucleo board with RTOS.
This demo use the RTOS task to send the same data each second to the MQTT Server with the topic "pub_topic".
This demo was made with the LwIP documentation, so i did only the device configuration and some changes.

The RED Led show error in the transmision of the data.


The follow configurations are needed for your implementation:

* Setup some static IP for your local network in the LwIP General Settings.
* Setup the server IP in the main file, in the task function.

Any question please comment. 


