# Continuously Calculated Impact Point or CCIP

## Project Name: CCIP 
CCIP is a calculation from a weapon's sighting system which predicts its payload's point of impact. This continuously  updated calculation is used for displaying information to a person's HUD or in our case visualize the tracking of a payload. 

### MVP (Minimum Viable Product) 
- Visually see tracking of destination object / payloads trajectory 

### Run Application
```
python3 testCCIP.py
```
Running testCCIP.py will create a 3x3 plot where the first row represents the planes movement in the NED coordinate frame with respect to time in seconds. The second row has the intertial velocities of the payload with respect to the inertial frame over time in seconds. The third row shows the payload's coordinates in the NED frame with respect to time. 

You will be able to see how our payload successfully intercepts the target at x=100, y=100, z=0. Inside our testing script you will also be able to edit the target location. If you would like to change the altitude, you will need to set the parameters of pd for the UAV vehcicle state. The higher the altitude, the more calculations will be required and will take longer to process. 

#### Measurements
1. Average time it takes to deliver a payload in response to a new target point nearby (Need to test/simulate, What are good response times?)
2. Accuracy of payload when we add in gusty wind

Add more stuff and change all this etc.

## Authors & Contacts:
Tanner Muirhead - tmuirhea@ucsc.edu

Kevin Jesubalan - kjesubal@ucsc.edu

Alexander Schulte - aschulte@ucsc.edu