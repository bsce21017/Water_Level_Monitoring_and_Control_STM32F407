# Water Level Monitoring and Control using STM32F407 MCU
* An embedded system that can measure and contol the water level in a container

## Basic Functionality
- An embedded system that can measure the water level in a container. Multiple types of sensors is used for this project. The system is able to display the current water level (on LCD) in millimeters (mm). 
- There is an alarm when the water level reaches a particular level. The water level for alarm is configurable. 
- The user is able to configure the water level by using buttons attached with LCD.

## Advanced Functionality
- In an advanced implementation, we decided to add an additional control elements for example controlling the water level. The inflow to the water container shall be controlled by a valve (see solenoid valves).  You may need an optocoupler/relay for this to work. but we didn't accomplish this.
- In the same setting, we added a temperature control (by adding a heating element) that can be switched on and off depending upon the sensed temperature. 

