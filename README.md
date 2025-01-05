# Adaptive Cruise Control - Simplified  

In this project, used 'Newton's Eqn of Motion' as my State Space model/plant (check 'Glossary') and Model Predictive Controller (Optimization Technique - IPOPT) to control pose of my simulated car based or real-time/iteration-based leadCar speed changes. 

We have a logging feature for each 'test case' as well and a post-analysis python script to assess Controller's performance. The whole Controller logic has been coded in cpp to promote compatibility with Embedded systems. 

Objective here is to contribute to open-source and help students and educators.  

## Assumptions
- Car & leadCar are point mass & move along an 1-D plane for simplicity.
- None of the cars can reverse.

## Results
![Test Case -1 - At low speed](/output/ss_after_fix/TC5.png)
![Test Case -2 - At high speed](/output/ss_after_fix/TC8.png)  

## Features  
- Can handle dynamic change in leadCar velocity incuding the scenario when the leadCar comes to a stop.  
- Random noise has been introduced to 'simulated' LIDAR reading to mimic real-time sensor readings.
- Obeys speed, acceleration  and gap(distance) limits.
- Inputs & Controller characterstics can be tweaked (as of now its through manual code updates)
- Results are displayed and logged after each plantStep.
- Have 'qt' backed python-GUI to analyze Controller performance based on logged data after successful completion of test case.  

## Installation  
TBA (Need another fresh linux to verify)
- Need to install IPOPT (Refer their original website) for controller to work  and Qt for analysis.

## Future Work
- Introduce Jerk state variable in our model to make ride comfortable
- Introduce slack variable to 'Distance Inequality' and convert it into a weak constraint. This will help in handling residual noise from 'simulated' LIDAR readings. 

## Datasheet for changing Inputs & Controller parameters
![Tweaks Datasheet](/output/ss_after_fix/tweaks.png)

## Glossary
### Model & Controller Design
![State Space model with constraint](/output/Model.jpeg)
![Cost function](/output/cost.jpeg)

 
