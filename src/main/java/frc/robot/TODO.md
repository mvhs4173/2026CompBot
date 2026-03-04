- [ ] Switch Indexer to using two motors. Start by using the same type of code as Shooter, then we will switch it to adjust the output voltage to each motor based on current to more accurately share the load across both motors.

- [ ] Switch lead screw control from using mirrored voltage (follow) to using mirrored position, ie use the same pid constants, but different controllers.

- [ ] Switch Indexer to using PID to have more constant velocity for better control and predictability.

- [ ] Apply motor voltage and give controller velocity setpoint for shooter.

- [ ] Make bare minimum Auto Commmand (Sequential Command Group)

- [ ] Override sendable methods in Indexer and other subsystems so it more accurately shows information for debug.

- [ ] Measure and fill temporary constants, ie Shooter hood angle minimum and maximum.

---

- [ ] Create a flow chart of auto options (Consult any on your team interested in strategy [Drive team probably])
**IMPORTANT** Do not overcomplicate this! 3 static auto options should be sufficient for now. Keep in mind where you start, what you do first, what you do last, and where you need to go to do what you need to do.

- [ ] Create Sendables for each subsystem (Add data like wheel orientation, intake position, shooter velocity)

- [ ] Create Shuffleboard/Smartdashboard/etc interface for your drive team (Camera, subsystem data, etc)

- [ ] Switch Shooter to using integrated pid control.

- [ ] Create control diagram for operators (What buttons do what and who uses them)

- [ ] Add javadoc comments for every function for easier troubleshooting

- [ ] Create static and dynamic pathing options for auto period and auto shooting
