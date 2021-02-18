# value-pasta-auto
I have created a lower cost approximation of the Toyota PASTA:Portable Automotive Testbed with Adaptability using consumer hardware and Arduino based software. The original didn't release details of how the hardware in the top of the case for the inputs/outputs and displays operate or are connected.
![Value-Pasta-Auto](/value-pasta-auto.jpg)

## ECU's
There are four ECU's in #value-pasta-auto
|ECU|CAN0(500kbps)|CAN1(500kbps)|CAN2(500kbps)|CAN3(500kbps)|Software Status|
|:---:|:---:|:---:|:---:|:---:|:---:|
| - | Internal CAN0 2.0 | Internal CAN1 2.0 | Internal CAN2 2.0<br>(can be CAN-FD) | MCP2515 CAN 2.0 ||
| Gateway ECU | Powertrain ECU CAN0 | Chassis ECU CAN0 | Body ECU CAN0 | OBD2 port | Gateway function complete/access from OBD2 using static ID/data |
| Powertrain ECU | Gateway CAN0 | unused | unused | unused | UDS to be done |
| Chassis ECU | Gateway CAN0 | unused | unused | unused | UDS to be done |
| Body ECU | Gateway CAN0 | unused | unused | unused | UDS to be done |

The Gateway ECU currently blocks traffic from the other BUSES to the OBD2 port unless a specific message is received on the OBD2 port or the serial DEBUG port.

## Microcontroller and CAN hardware
The micro controller hardware used in #value-pasta-auto for the ECU's is the [Teensy 4.0](https://www.pjrc.com/store/teensy40.html), which has 3 internal CAN controllers, one of which can be used for CAN-FD. The fourth CAN controller is provided by a MCP2515 controller. The CAN2.0 transceivers used are SN65HVD230DR and the CAN-FD transceiver is a MCP2562FD.

## Teensy ECU Schematic
![Teensy ECU schematic](/Hardware/teensy-ecu%20schematic%20v1.2.png)
## Teensy ECU PCB Front
![Teensy ECU pcb front](/Hardware/teensy-ecu%20v1.2.png)
<br>[Original GERBER v1.0](/Hardware/teensy-ECU%20GERBER%20files%20v1.0.zip) has a fault with the WS2818b pinout current software just uses a single SMD LED and resistor instead.
<br>[Updated GERBER v1.1](/Hardware/teensy-ECU%20GERBER%20files%20v1.1.zip) fixes the WS2818b pinout. Code would need modifying to support WS2812b.
<br>[Updated GERBER v1.2](/Hardware/teensy-ECU%20GERBER%20files%20v1.2.zip) Adds the TXnRTS and RXnBF interrupt pins to schematic.
## Bill of materials
|Name|Designator|Quantity|
|:---:|:---:|:---:|
|BS-2-1 CR2032 Holder|B1|1|
|0805 0.1uF Capacitor|C1,C101,C102,C103,C104,C105,C106|7|
|0805 22pF Capacitor|C10,C11|2|
|1206 10uF Capacitor|C107|1|
|2pin 2.54 header & jumper|H1,H2,H3,H4|4|
|MOLEX 12A-347290120|H5|1|
|MOLEX 20A-347290200|H6|1|
|SN65HVD230DR CAN Transceiver|U1,U2,U4|3|
|MCP2562FD-E/SN CAN-FD Transceiver|U3|1|
|MCP2515-I/SO CAN Controller|U5|1|
|Teensy 4.0|U6|1|
|8MHz crystal|X1|1|
|NUP2105L protection diode|D1,D2,D3,D4|4|
|WS2812 RGB LED|LED1|1|
|0805 120 Resistor|R1,R2,R3,R4|4|
|0805 10k Resistor|R5|1|

## I/O board
The I/O board is used for the Powertrain and the Chassis ECU's. These are built using MCP23017 I/O expander, Powertrain ECU uses one MCP23017 and the Chassis ECU uses two MCP23017.

## I/O Board Schematic
![I/O Board schematic](/Hardware/io-board%20schematic%20v1.0.png)
## I/O Board PCB Front
![I/O Board pcb front](/Hardware/io-board%20v1.0.png)
<br>[Original GERBER v1.0](/Hardware/io-board%20GERBER%20files%20v1.0.zip)
## Bill of materials
|Name|Designator|Quantity|
|:---:|:---:|:---:|
|1206 10uF Capacitor|C1,C2,C3,C4|4|
|0805 0.1uF Capacitor|C101,C102,C103,C104|4|
|2x8pin 2.54 header|DIO-2,DIO-1,POWER|3|
|4pin 2.54 header|NEXTION|1|
|3pin 2.54 header|AN0,AN1,AN2,AN3|4|
|2pin 2.54 header|H2|1|
|MCP23017-E/SO IO expander|U1,U2|2|
|MOLEX 20A-347290200|U3|1|
|AMS1117-3.3 Voltage regulator|U4|1|
|0805 4.7k Resistor|R1,R2|2|

* Chassis ECU I/O board
MCP23017 I/O Inputs unless otherwise stated

|Function port A MCP23017A|PIN|Function port B MCP23017A| |Function port A MCP23017B|PIN|Function port B MCP23017B|
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| Rotary Light CLK | 1 | Rotary Fr Wiper CLK | | Left DOOR window switch UP | 1 | Right DOOR window switch UP |
| Rotary Light DT | 2 | Rotary Fr Wiper DT | | Left DOOR window switch DOWN | 1 | Right DOOR window switch DOWN |
| Rotary Light PUSH | 3 | Rotary Fr Wiper PUSH | | Left DOOR lock switch | 3 | N/A |
| Shift Position switch DOWN | 4 | TURN switch L | | Right DOOR lock switch | 4 | N/A |
| Shift Position switch UP | 5 | TURN switch R | | N/A | 5 | N/A |
| Rear Wiper switch SLOW | 6 | HAZARD switch | | N/A | 6 | N/A |
| Rear Wiper switch FAST | 7 | Start BUTTON | | N/A | 7 | Ignition OUTPUT |
| HORN switch | 8 | Parking Brake switch | | HORN OUTPUT | 8 | Parking Brake OUTPUT |

|Analogue Port|Value|
|:---:|:---:|
|Ana0|Brake Potentiometer|
|Ana1|Accelerator Potentiometer|
|Ana2|Steering Potentiometer|
|Ana3|N/A|

* Powertrain ECU I/O board
MCP23017 I/O Inputs unless otherwise stated

|Function port A MCP23017A|PIN|Function port B MCP23017A| |Function port A MCP23017B|PIN|Function port B MCP23017B|
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| Rotary Control CLK | 1 | N/A | | Not Installed | 1 | Not Installed |
| Rotary Control DT | 2 | N/A | | Not Installed | 2 | Not Installed |
| Rotary Control PUSH | 3 | N/A | | Not Installed | 3 | Not Installed |
| N/A | 4 | N/A | | Not Installed | 4 | Not Installed |
| N/A | 5 | N/A | | Not Installed | 5 | Not Installed |
| N/A | 6 | N/A | | Not Installed | 6 | Not Installed |
| N/A | 7 | N/A | | Not Installed | 7 | Not Installed |
| N/A | 8 | N/A | | Not Installed | 8 | Not Installed |

|Analogue Port|Value|
|:---:|:---:|
|Ana0|Control potentiometer|
|Ana1|N/A|
|Ana2|USED FOR BLUETOOTH RX|
|Ana3|USED FOR BLUETOOTH TX|

## LED screens
Powertrain, Body and Chassis ECU's have 3.5" NEXTION HMI [NEXTION](https://www.itead.cc/display/nextion.html) serial based screens. These are connected via the I/O board where it is present.

## Arduino Bluetooth CAR
An Arduino based remote car can be controlled using #Value-Pasta-Auto

## CAN message detail per bus
The following files show the CAN msgs that are used by #value-pasta-auto as per the original design docs from [pasta-auto bus definition](https://github.com/pasta-auto/PASTA1.0/blob/master/doc/PASTA1.0%20CAN-ID%20List%20v1.0E.pdf)
<br>
![Can bus definitions](/canbus.md)

## Details of the original pasta-auto can be found at the following locations
- GitHub: [pasta-auto](https://github.com/pasta-auto)
- Twitter: [@pasta_auto](https://twitter.com/pasta_auto)
