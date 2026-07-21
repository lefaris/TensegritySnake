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
  
<img width="1000" alt="HUTS - PCB - 3D View Bottom" src="https://github.com/user-attachments/assets/215da4c3-a85e-4837-802f-f34a14bc0fe5" /> <img width="1000" alt="HUTS - PCB - 3D View Top " src="https://github.com/user-attachments/assets/0ce80772-47d5-4ba9-ac35-35ed026d12e9" />


