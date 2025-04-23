## Mechanical Subsystem
All the items that have been mentioned in this document refer directly to the file name.


## Files and their description

**Turtlebot Assembly files:**
- Turtlebot Assembly:- Base turtlebot burger
- Turtlebot & Launcher Mechanism Assembly: Final Turtlebot Burger assembly with the launcher and the ball feeder system.

**Reload Tube Folder:**
- Joint2: attatches 18cm pipe to the launcher 
- Joint: attatches 15cm pipe to the 18cm pipe 
- Reload Tubes assem: the entire assembly file with pipes and joints
- Tube 18cm: attaches to the launcher directly using a press-fit joint
- Tube 15cm
- Tubes holder: Attaches to turtlebot for holding the 18cm tube 

**Launcher Folder:**
- DC motor & servo platform: Attaches DC motors and servo to the turtlebot 
- DC motor gear: 30 teeth 
- JGB37-520 left assem: assembly file with motor
- JGB37-520 left bracket  
- JGB37-520 right assem: assembly filw with motor  
- JGB37-520 right bracket  
- SG90 servo assem (rack and pinion)  
- SG90 servo bracket (rack and pinion)  
- ball platform - Holds the ball and has the rack (15 teeth)
- flywheel - File has the flywheel plus the gear (12 teeth) 
- launcher + reload tubes assem  
- launcher  
- pinion gear (rack and pinion)  



## 3D Printing
CAD files can be saved as STL files to 3D Print.

**Parts to be printed:**
- Launcher 
- JGB37-520 left bracket
- JGB37-520 right bracket  
- SG90 Servo Bracket 
- Ball Platform
- Joint 2
- Joint
- Tubes Holder
- 2X Flywheel
- SG90 servo Bracket (heat sensor)

**Specifications for 3D Printing:**
- Infill: 15%
- Material: PLA 


## Laser Cut:
Using a 5mm acrylic sheet the following needs to be laser cut

_Parts to be Laser cut:_
- 2X DC Motor gear 
- Pinion Gear 
- DC Motor and Servo Platform



## Assembly Plan

**Assembly Step 1: SG90 Servo + Heat Sensor Mounting**
- Mount the SG90 servo to the front of the second level using M4×15 screws.
- Attach the sensor bracket to the servo horn using M2×10 screws.
- Fix the AMG8833 heat sensor onto the bracket using M2×10 screws.

**Assembly Step 2: Mount Pinion and Attach Servo Platform**
- Attach the pinion gear to the output shaft of the SG90 servo.
- Mount the servo onto the platform bracket using M2×15 screws.
- Fix the entire servo and motor platform onto the top layer using M3×15 screws through the holes aligned with the hex standoffs.

**Assembly Step 3: Mount Flywheel Launcher Assembly**
- Mount the JGB37-520 motors onto the launcher platform using M2×15 and M4×15 screws.
- Attach the 30-tooth motor gear to the motor shaft.
- Wrap the flywheel with 8 rubber bands each to increase friction.
- Slide the 3D printed shaft through the flywheel and secure it so the 12-tooth flywheel gear meshes precisely with the motor gear (gear ratio 30:12).
- Insert the ball holder platform from above
- Extend the third platform layer by attaching M3×20 hex standoffs to the existing mounting holes.
- Secure the entire launcher assembly onto the TurtleBot chassis using M4X15 and M3X15 screws through the designated mounting holes, aligning the pinion (29T) on the servo with the rack (15T) for accurate engagement.

**Assembly Step 4: Ball Feeder Pipe Installation**
- Mount the pipe support bracket to the TurtleBot platform using M2×20 screws.
- Press-fit the 15 cm PVC pipe into the 3D printed 90° elbow joint.
- Connect the 15 cm pipe to the 18 cm PVC pipe using the same elbow joint.
- Press-fit the other end of the 18 cm pipe into the 3D printed joint2,
- Press-fit joint2 onto the launcher housing and finally, rest the assembled tube system onto the tube holder.



## Launcher Calculations 

**Specifications & Assumptions:**
- Mass of Ping Pong Ball: 2.7 g (0.0027 kg)
- Target Launch Height: 1.5 m
- Flywheel Diameter: 5 cm (0.025 m radius)
- Motor Used: JGB37-520 (nominal no-load speed ~960 RPM)
- Gear Ratio: 30:12 (2.5× step-up)

**Required Launch Velocity:**
v = √(2gh) = √(2 × 9.81 × 1.5) ≈ 5.42 m/s

**Required Flywheel RPM:**
ω = v / r = 5.42 / 0.025 = 216.8 rad/s
RPM = (216.8 × 60) / (2π) ≈ 2072 RPM

**Achieved Flywheel RPM:**
With gear ratio of 2.5× (30 teeth : 12 teeth):
Achieved RPM = 960 × 2.5 = 2400 RPM

Our flywheel system achieves an RPM of 2400, which exceeds the required 2072 RPM. This confirms our system can successfully launch the ping pong ball to at least 1.5 meters vertically, satisfying the project requirements with a performance buffer (accounting for air resistance and energy loss through friction)



## Mechanical and Assembly Recommendations
- Ensure motor shafts and flywheels are securely fastened to prevent slippage at high RPMs.
- Align gears and flywheels properly to minimize vibration during operation.
- Test with gradual motor ramp-up to check for imbalance or resonance.
- Ensure the PVC ball guide path has minimal friction and doesn’t interfere with launch direction.







