# DynamiControl_PID

## Project Overview
DynamiControl_PID is a project focused on designing and tuning PID controllers and Lead-Lag compensators (LC) for a ball stabilizer system controlled by Arduino. It involves real-time control using a ultrasonic sensor and servo motor, with MATLAB/Simulink analysis and hardware implementation for system performance enhancement , with an emphasis on transient and steady-state performance.


## Project Objectives
- Analyze a given system and design a PID controller to achieve a specific response.
- Tune the PID controller using different rules.
- Design a Lead-Lag compensator and tune it for improved system response.
- Implement the controller and compensator in a feedback system to analyze their effect on system performance.
- Enhance transient and steady-state responses through controller design and tuning.

## Repository Structure

```
├── docs/
│   ├── report.pdf                  
│   ├── figures/                    
│   └── references/                 
├── matlab/
│   ├── pid_controller.m            
│   ├── lead_lag_compensator.m      
│   ├── system_analysis.m           
│   └── simulink_models/           
├── hardware/
│   ├── schematics/                 # Circuit diagrams for PID and Lead-Lag controllers
│   ├── code/                       # Embedded code for hardware (Arduino)
│   └── photos/                     
└── tests/
    ├── test_pid_controller.m       
    ├── test_lead_lag_compensator.m
    └── test_results/               
 
```

## How to Run
1. Clone the repository.
2. Open the MATLAB scripts in the `/matlab` folder to analyze the system and tune the controllers.
3. Use the Simulink models for more detailed analysis and simulation.
4. Follow the hardware setup instructions in the `/hardware` folder if implementing the project with hardware.


## License

This project is licensed under the [MIT License](LICENSE).

---

