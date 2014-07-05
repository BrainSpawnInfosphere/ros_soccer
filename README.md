# ROS: Soccer Robot 

**Author:** Kevin Walchko

**License:** BSD

**Language:** C++

**Website:** http://github.com/walchko/soccer

Current robot I am working on

## Working

* Serial Node code integration
* Touch OSC for teleop
* Drivers for: motor drivers, IMU, and power
* Arduino ATMega code, with 2 serial ports (USB and BlueTooth)
* Message database code (handles send/receiving messages over serial)

## To Do

* Inertial Navigation code
* [libviso2](https://github.com/walchko/libviso2) integration
* Drop sensors
* Current sensors
* Go completely wireless
* General code clean-up and formatting
* Add BSD license

# Design

Key System | Interface | Purpose
:----------|:----------|:------------
Main       | init()    | Collects command line args and reads config files
Robot      | functions | Main hub of robot SW, performs all low level math (ie, not navigation)
MessageDB  | string    | IP to serial connection, interface to microcontroller
Publisher  | msgs      | publishes all out going and in coming msgs
Navigator  | msgs      | performs high level navigation


[ msgdb ]<--->[ ip_serial ]
   |
[ robot ]
  ^  |  
  |  +--------->[ pub ]
  |             | |
  +---[ nav ]<--+ +-->[ GUI ]  
  +-----[ joystick ]




[desiredVel] [desiredPos]





## Message System

* All messaging is initiated from Computer to uC - one way, with a return ACK
* All Cmd messages return an ACK with or without data. 
* If there is an error, the ACK is <e> or <E>
* IP serial needs to know if ACK has data ... I think - no

Test: rosservice call /uc0_serial "\<v\>" 

Computer (Cmd) | Cmd Size| uC (ACK)      | ACK Size | Description
:--------------|:-------:|:-------------:|:--------:|:--------------
Any Msg        |    X    |   \<e\>       |    0     | If something went wrong on uC, otherwise normal msg
Any Msg        |    X    |   \<E\>       |    0     | If something went wrong on ip_serial, otherwise normal msg
\<h\>          |    0    |   \<h\>       |    0     | Halt motors
\<msxxxxx\>    |    5    |   \<m\>       |    0     | Motor commands
\<psx\>        |    1    |   \<p\>       |    0     | Play sound
\<r\>          |    0    |   \<r\>       |    0     | Reset
\<s\>          |    0    | \<ssxx...xx\> |   20     | Sensor values (a,g,m,b,ir,batt,...)
\<S\>          |    0    | \<Ssxx...xx\> |    X     | Status, sends error msgs ... value??
\<t\>          |    0    | \<t\>         |    0     | Motor test, just turns them
\<v\>          |    0    | \<vsxx...xx\> |   10     | SW version


