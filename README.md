# Two-Layer Neuro-Adaptive Compensation Control Applied to a 4-Wheeled Omnidirectional Mobile Robot

**ID:** 10018

## Authors and Affiliations

- **Sergio López**  
  Division of Graduate Studies and Research,  
  Tecnológico Nacional de México / Instituto Tecnológico de La Laguna,  
  Torreón, Coah., México  
  Email: [slopezh@lalaguna.tecnm.mx](mailto:slopezh@lalaguna.tecnm.mx)

- **Miguel A. Llama**  
  Division of Graduate Studies and Research,  
  Tecnológico Nacional de México / Instituto Tecnológico de La Laguna,  
  Torreón, Coah., México  
  Email: [mllama@lalaguna.tecnm.mx](mailto:mllama@lalaguna.tecnm.mx)

## Description

This repository contains all scripts and data necessary to reproduce the results presented in the paper:

- The **Two-Layer Neuro-Adaptive Compensation (TLNAC) scheme** is implemented in real-time on the **Nexus 4-WOMR** (4-Wheeled Omnidirectional Mobile Robot).  
- Desired wheel trajectories are computed using **inverse robot kinematics**.  
- The **wheel speeds** are controlled through the TLNAC controller.  
- The controller is lightweight enough to run on the **ATmega328p microcontroller** embedded in the Nexus 4-WOMR.

## Repository Structure

- `src/` – Source code for the TLNAC implementation (Arduino sketches and scripts).  
- `data/` – Input trajectories, reference signals, or experiment data.  
- `README.md` – This file.  
- `LICENSE` – License for using the code.

## Dependencies

- **Arduino IDE** (version 1.8.19 or later recommended)  
- Required Arduino libraries (listed at the top of each sketch, e.g., `Servo`, `Wire`, etc.)
- MATLAB R2016a or later (recommended: R2018b or newer)
  - Control System Toolbox
  - Robotics System Toolbox 
- Optional Python scripts for plotting results require:  
  - Python 3.8+  
  - `numpy`  
  - `matplotlib`  

## How to Use

1. Clone the repository:  
   ```bash
   git clone https://github.com/sergiolh15/Two-Layer-Neuro-Adaptive-Compensation-Control-Applied-to-a-4-Wheeled-Omnidirectional-Mobile-Robot.git

