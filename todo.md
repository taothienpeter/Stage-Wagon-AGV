# To-Do List: Parameters for Swerve Drive Robot

- [x] Identify motor specifications (KV, max current, encoder type)
- [x] List required ODrive firmware version
- [ ] recheck for stepper homing sequence
- [ ] test homingseq => work with the class
- [ ] test true homing sequence => accurate enough
- [ ] test encoder with class => running
- [ ] check for scale affector (from speed limit of both motor) to meet the desired trajectory
- [ ] check for limit of both motors.   
# To-Do coding: 

- [ ] Make option to select types of data need to print out
- [ ] Make option to print data for ploting
- [ ] (optional) make script to use combination of accel & speed variation to get the limit of turning motor
- [ ] Constraints define robot physical limitations: Max speed, max acceleration, max centripital acceleration, etc. Can be estimated or obtained through robot characterization testing.
- [ ] set option for checkign errors every odrive runs

# Optimise To-Do list
- [ ] Turn all the funtions with while() to reduse the code's complexity
- [ ] Make a config autoloader stript for quick load through wifi