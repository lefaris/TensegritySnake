HUTS PCB Shield
<p align="center">
<img
  width="600"
  alt="Controls assembly" src="https://github.com/user-attachments/assets/9fdf3107-c9bb-4c22-91ae-63afc063e8db" />
</p>

This repository contains the printed circuit board design files for the onboard electrical shield used by the Hyper-redundant Underactuated Tensegrity Snake (HUTS). 

Repository contents:
1. HUTS - PCB - Easy EDA Source.json
        Editable EasyEDA PCB source. Use this file for layout changes, routing updates, silkscreen edits, or new manufacturing exports.
2. HUTS - PCB - Electrical Shield Schematic.pdf
        Electrical schematic showing the battery inputs, protection circuits, buck converter, three motor drivers, and six motor outputs.
3. HUTS - PCB - GERBER .zip
        Bare-PCB manufacturing package.
4. HUTS - PCB - Layers For Ref.zip
        PDF exports of the individual PCB layers for visual inspection and documentation.
5. HUTS - PCB - CAD.step
        STEP model of the PCB.
6. HUTS - PCB - 3D View Top .png
        Top 2D view showing the components, switches, battery connectors, and connectors.
7. HUTS - PCB - 3D View Bottom.png
        Bottom 3D view showing the motor driver ICs and the 40-pin controller header.


PCB overview:
  Board outline: 42.0 mm × 70.0 mm
  Corner radius: 3.5 mm
  Copper layers: 2
  Motor outputs: 6 brushed-DC motor channels
  Motor drivers: 3 × TB6612FNG dual H-bridge drivers
  Battery inputs: 2 independently switched inputs
  Logic supply: regulated +5 V output from the Battery 
  Controller interface: 40-pin header
  Motor Driver: TB6612FNG

<p align="center">
  <img
    width="400"
    alt="HUTS PCB 3D View Bottom"
    src="https://github.com/user-attachments/assets/215da4c3-a85e-4837-802f-f34a14bc0fe5"
  />
  &nbsp;&nbsp;
  <img
    width="400"
    alt="HUTS PCB 3D View Top"
    src="https://github.com/user-attachments/assets/0ce80772-47d5-4ba9-ac35-35ed026d12e9"
  />
</p>

## Electrical Schematic

<p align="center">
  <img
    width="950"
    alt="HUTS_Electrical_Schematic_GitHub" 
    src="https://github.com/user-attachments/assets/f4cc35fd-281d-4c59-9dbf-37b5168d0213" />
  />
</p>

The HUTS PCB shield centralizes power distribution and motor control for the robot's six motor-driven tendons. Battery inputs supply three protected motor-voltage rails, while a regulated +5 V powers the motor-driver logic and onboard controller interface. Three dual H-bridge motor drivers provide independent speed and direction control for Motors 1–6.

### Component Description
**BATTERY1 and BATTERY2:**  
Two-pin JST battery connectors used to connect the onboard battery supplies. Both battery systems share a common ground.

**SWITCH1 and SWITCH2:**  
SS-12D10-G5 slide switches that independently connect or disconnect the two battery supplies.

**Q1, Q2, and Q3:**  
MOSFETs used to provide reverse-polarity protection for the three motor-voltage rails.

**C1, C2, and C3:**  
10 µF capacitors used for local filtering and voltage stabilization on the +VM1, +VM2, and +VM3 motor-supply rails.

**REGULATOR:**  
An LM2576S-5.0 buck regulator that converts the BATTERY2 voltage into the regulated +5 V supply.

**L1:**  
A 100 µH inductor used as the energy-storage element of the buck converter.

**D1:**  
An SS34 Schottky diode used as the freewheeling diode in the buck-converter circuit.

**C7:**  
A 100 µF capacitor used to filter the input of the buck converter.

**C8:**  
A 1000 µF capacitor used to filter the +5 V output and support transient current demands.

**DRIVER1, DRIVER2, and DRIVER3:**  
Three TB6612FNG dual H-bridge motor drivers. Each driver independently controls two brushed DC motors, providing six total motor-control channels.

**R1, R2, and R3:**  
10 kΩ resistors that pull the STBY input of each TB6612FNG driver high. This keeps the drivers enabled whenever the +5 V logic supply is present.

**C4, C5, and C6:**  
10 µF capacitors used for local supply decoupling near the three motor-driver ICs.

**MOTOR1 through MOTOR6:**  
Two-pin connectors used to connect the six DC motors responsible for actuating the HUTS tendons.

**PI_ZERO_HEADER:**  
A 2 × 20, 40-pin header that connects the onboard computer to the motor-driver control signals, ground, and +5 V supply.

### Power Distribution

**BATTERY1:**  
BATTERY1 supplies the +VM1 and +VM2 motor-voltage rails through SWITCH1 and the associated reverse-polarity protection circuits.

The +VM1 rail supplies DRIVER1 and Motors 1–2.

The +VM2 rail supplies DRIVER2 and Motors 3–4.

**BATTERY2:**  
BATTERY2 supplies the +VM3 motor-voltage rail through SWITCH2 and its reverse-polarity protection circuit.

The +VM3 rail supplies DRIVER3 and Motors 5–6.

BATTERY2 also supplies the LM2576S-5.0 buck converter, which generates the regulated +5 V logic supply.

**+5 V Supply:**  
The regulated +5 V rail supplies the logic side of all three TB6612FNG motor drivers and the controller header.

**Ground:**  
Both battery domains, all three motor drivers, the buck converter, and the onboard controller share a common ground.

### GPIO Connections

#### Motor 1

Motor 1 is controlled by Channel A of DRIVER1.

- PWM and speed control: `GPIO12`
- Direction input AIN1: `GPIO20`
- Direction input AIN2: `GPIO21`
- Motor-supply rail: `+VM1`

#### Motor 2

Motor 2 is controlled by Channel B of DRIVER1.

- PWM and speed control: `GPIO13`
- Direction input BIN1: `GPIO26`
- Direction input BIN2: `GPIO19`
- Motor-supply rail: `+VM1`

#### Motor 3

Motor 3 is controlled by Channel A of DRIVER2.

- PWM and speed control: `GPIO25`
- Direction input AIN1: `GPIO7`
- Direction input AIN2: `GPIO8`
- Motor-supply rail: `+VM2`

#### Motor 4

Motor 4 is controlled by Channel B of DRIVER2.

- PWM and speed control: `GPIO10`
- Direction input BIN1: `GPIO11`
- Direction input BIN2: `GPIO9`
- Motor-supply rail: `+VM2`

#### Motor 5

Motor 5 is controlled by Channel A of DRIVER3.

- PWM and speed control: `GPIO17`
- Direction input AIN1: `GPIO27`
- Direction input AIN2: `GPIO22`
- Motor-supply rail: `+VM3`

#### Motor 6

Motor 6 is controlled by Channel B of DRIVER3.

- PWM and speed control: `GPIO14`
- Direction input BIN1: `GPIO15`
- Direction input BIN2: `GPIO18`
- Motor-supply rail: `+VM3`
