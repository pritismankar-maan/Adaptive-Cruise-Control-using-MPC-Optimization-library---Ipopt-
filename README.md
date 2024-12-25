# Adaptive Cruise Control - Simplified  

In this project used 'Newton's Eqn of Motion' as my State Space model and Model Predictive Controller (Optimization Technique - IPOPT) to solve the problem and log data in real-time. The whole controller logic has been codes in cpp. Objective is to contribute to open-source and help students.  

## Features  
- Can handle dynamic change in leadCar velocity incuding the scenario when the leadCar comes to a stop.  
- Obeys speed, acceleration  and gap(distance) limits.
- Inputs & Controller characterstics can be tweaked (as of now its through manual code updates)
- Results are displayed and logged after each plantStep.
- Have 'qt' backed python-GUI to analyze Controller performance based on logged data after successful completion of test case.  

## Installation  
TBA (Need another fresh linux to verify)
- Need to install IPOPT (Refer their original website) for controller to work  and Qt for analysis.

## Glossary
 
