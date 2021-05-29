# Week 1

## CAN (Controller Area Network)

* CAN BUS acts as a central networking system

* Modern cars can have 70 nodes or  'ECUs' thus a CAN comes handy in such cases.

* Why CAN is popular:
  1. **Low Costs** - ECUs communicate via a single CAN interface, i.e, not direct analogue signal lines, reducing errors, weight, costs
  2. **Centralized** - The CAN bus system allows for central error diagnosis and configuration.
  3. **Robust** - Robust towards failure of subsystems and electromagnetic interference making it ideal for a vehicle.
  4. **Efficient** - CAN messages are prioritized based on IDs so that the highest priority IDs are non-interrupted.
  5. **Flexible** - Each ECU contains a chip allowing it to recieve all transmitted messages, decide relevance and act accordingly.
  6. **High data transmission speeds** - (Up to 1 MBit/sec)

### What is CAN?    

As an alternative to conventional multi-wire looms, CAN Bus allows various electronic components (such as: electronic control units, microcontrollers, devices, sensors, actuators and other electronic components throughout the vehicle) to communicate on a single or dual-wire network data bus up to 1 Mb/s.

  * CAN bus is made up of two wires, CAN-H (CAN High) and CAN-L(CAN LOW) which connect to all the devices in the network.

    ![img](https://media-exp1.licdn.com/dms/image/C5612AQHqPtDTodxnWw/article-inline_image-shrink_1000_1488/0/1520237514474?e=1626307200&v=beta&t=zt9M7zne420ccx4-WoXvGfqAREGAyjuNKZmLlkHJQLw)

* **CAN Controller** -  receives the transfer data from the microcomputer integrated in the control unit/device (also known as CAN Node). The CAN controller processes this data and relays it to the CAN transceiver. Also, the CAN controller receives data from the CAN transceiver, processes it and relays it to the microcomputer integrated in the control unit/device (CAN Node).
* **CAN Transceiver** is a transmitter and receiver in one. It converts the data which the CAN controller supplies into electrical signals and sends this data over
  the data bus lines. Also, it receives data and converts this data for the CAN controller.
* **CAN Data Bus Terminal** is a resistor (R) typically of 120 ohms. It prevents data sent from being reflected at the ends and returning as an echo.

### CAN data Transfer Process and Message Structure

![img](https://media-exp1.licdn.com/dms/image/C4E12AQFHbgqJhUB-8w/article-inline_image-shrink_1000_1488/0/1520467289077?e=1626307200&v=beta&t=20BN9gV9nP2CRXwd7q8-BBlGomT9rvkx2D8aIcG83_Q)

* CAN system can be of two types based on the size of the identifiers(ID):

  1. Standard CAN: uses 11 bit identifiers in the arbitration field
  2. Extended CAN: supports a length of 29 bits for the identifier, made up of the 11 bit identifier (base identifier) and an 18 bit extension.

  ![img](https://media-exp1.licdn.com/dms/image/C5612AQFjJnz7H0L1bg/article-inline_image-shrink_1000_1488/0/1520064590531?e=1626307200&v=beta&t=Z3sJqYzaKuQr8FtSSbvqAdaenD19IrP4qS5IJueKngE)

**SF = Start Field** 

**Message Identifiers** - defines the level of priority of the data protocol. If for instance, two CAN nodes want to send their data protocols simultaneously then the one with the higher priority would takes precedence. The lower the value(srbitration ID) the higher the priority of the message.

**Control** - displays the number of items of information contained in the data field. This field allows any receiver to check whether it has received all the information transferred to it. 

**Data Field**: in this field the information is transferred to the other CAN Nodes.

**CRC(Cyclic Redundancy Check)** 

**ACK = Acknowledge Field** 

**EF = End Field**

### Why do we need CAN

* A vehicle contains a network of electronic devices that share data and information with one another.

* The protocol set rules by which electronic devices can exchange information with one another over a common serial bus. It reduced the wiring connections and the overall complexity of the system.

* The CAN controller converts messages from the nodes per the CAN protocols, which are then transmitted via the CAN transceiver over the serial bus — and vice versa

## [CAN Programming](https://www.youtube.com/watch?v=9db-q5ffYpU&list=PLERTijJOmYrApVZqiI6gtA8hr1_6QS-cs)

* CAN is a Microcontroller.
* A microcontroller is an integrated circuit (IC) device used for controlling other portions of an electronic system, usually via a microprocessor unit (MPU), memory, and some peripherals. These devices are optimized for embedded applications that require both processing functionality and agile, responsive interaction with digital, analog, or electromechanical components.
  * CAN offers data communication up to 1 Mbit/sec
  * the error Confinement and the Error Detection features make it more reliable in noise-critical environments.

Three main features:

1. high speed 
2. high noise immunity
3. error detection features

* It's a broadcast type pf BUS(Unlike a traditional network such as USB or Ethernet, or i2c,CAN does not send data point-to-point from node A to node B under the supervision of a central bus master)

  

* It's a broadcast type of Bus

* All devices can hear the transmission

* No way to send a data specifically to a node by its address or something

* All nodes will pick up the traffic on the bus.

  The CAN standard defines a communication network that links all the nodes connected to a bus and enables them to talk with one another. There may or may not be a central control node, and nodes may be added at any time even when the network is operating (hot-plugging).

  

  We use differential signals instead of digital signals.

  ![Screenshot from 2021-05-21 20-07-24](/home/utkarsh/Documents/iitb-racing/Week 1/can/module-can assets/Screenshot from 2021-05-21 20-07-24.png)

  

  ### [Differential Signalling](https://en.wikipedia.org/wiki/Differential_signaling)

  ![Screenshot from 2021-05-21 20-12-34](/home/utkarsh/Documents/iitb-racing/Week 1/can/module-can assets/Screenshot from 2021-05-21 20-12-34.png)

* Terminal **resistors are** needed in **CAN** bus systems because **CAN** communication flows **are** two-way. The termination at each **end** absorbs the **CAN** signal energy, ensuring that this **is** not reflected from the cable **ends**. Such reflections **would** cause interference and potentially damaged signals.

## Coding Task

```C++
#include
```

