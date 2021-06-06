# Continuously Calculated Impact Point or CCIP
 
## Project Name: CCIP
CCIP is a calculation from a weapon's sighting system which predicts its payload's point of impact. This continuously  updated calculation is used for displaying information to a person's HUD or in our case visualizing the trajectory of the payload.
 
### MVP (Minimum Viable Product)
- Visually see tracking of destination object / payloads trajectory
 
### Run Application
```
python3 testCCIP.py
```
### What am I looking at?
Running testCCIP.py will create a 3x3 plot where the first row represents the UAV's movement in the NED coordinate frame with respect to time. The second row plots the inertial velocities of the payload with respect to time in seconds. The third row shows the payload's coordinates in the NEU frame with respect to time.
 
You will be able to see how our payload successfully intercepts the target at x_inertial=(targetlocationx), y_inertial=(targetlocationy), z_inertial=0. Inside our testing script, you will also be able to edit the target location. If you would like to change the altitude, you will need to set the parameters of pd for the UAV vehicle state. The higher the altitude, the more calculations will be required and will take longer to process.
 
### Progress Check
As of now, our system can calculate and drop its payload accurately within 5 meters of the target at a height up to 1000 meters. This altitude is not the limit; it just takes a long time to continuously calculate drop positions from that high. This program will also work traveling any direction. We have also implemented a small state machine which checks to see if our UAV is too close to the target. If the UAV is too close, it will navigate away and re-approach. After the payload has been released, the UAV will continue to maintain altitude without a heading until another target is acquired.

### Bugs To Report
There are no major bugs to report but we would like to make the estimations more accurate.

### To-Do
1. We are going to add multiple targets to destroy so our UAV will "fire and forget" its payload and immediately change course to the next target automatically.
2. We also want to include wind estimation for our time of flight and position release functions
3. Implement different payload options such as missiles and spheres
4. Create a GUI simulation with a release button
 
### Authors & Contacts:
Tanner Muirhead - tmuirhea@ucsc.edu
 
Kevin Jesubalan - kjesubal@ucsc.edu
 
Alexander Schulte - aschulte@ucsc.edu
 

