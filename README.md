# Lab_5

Both of the supplimentary files for part a and b contain very similar code however variations in motor controls vary slighly to make up for discrepancies in motors.
this ReadMe will contain dialoge for the videos as none was present during the filming.

Part A
the bot utilizes a localization feature where it rotates until the beacon can be seen. this however is very choppy because i used the mills time feature. if i had used a different approach the operation of the robot would have been smoother and faster in completion of the task. the forward motion of the robot tends to drift off track due to the unability to find the sweet spot to ballance out the motors. this could have been inproved through further tweeking of the motor system.

to prevent IR light from reaching the beacon when it should a heat shrink wrap was used to create a cone this proved better than Tape as the IR light would pass through regardless.

Due to the long nature of the heat shrink wrap my glasses case was used infront of the beacon at the 2m mark. this allowed for the robot to interact with the limit switch of the beacon while still beaing able to see the beacon.

after the beacon state was changed the robot entered the reverse state however due to the glasses case it would knock the robot off track causeing it to drift in a direction off the correct path.

Part B
this section utilizes similar features to part a however the begining of operation is hard coded to allow for the robot to navigate around the box 1/3 from the beacon. after making 4 90 degree turns the robot enters the automatic navigation stage and proceeds tp locate the beacon similariliy to part a. the initial operation of the bot went very smoothly until the automatic navigation kicked in, however this is due to the previously explained issue with using the mills time while navigating. after the beacons state is changes the robot reverses and spins 180 degrees however due to the glasses case the robot would vere off course. the glasses case was necissary however as the beacon wouldnt be noticed unless it was there.

Due to time constraint part c was not completed however the framework is there for future attempts.

