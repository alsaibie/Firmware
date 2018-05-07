## DP_ATT_Control Tests

### Mixing

- Converted the multirotor mixer into a function within dp_att_control module
- Adopted it to suit dolphin orientation where roll acts in the way yaw does with a multirotor 
- and thrust bidirectional scale, since the multirotor accepts 0 to 1, I need -1 to 1
- TODO: make sure the att controller reads -1 to 1 for thrust properly
- TODO: apply an output thrust model to fit the requested output to the motor close model
- TODO: Run ramp up/down tests as well as square wave (similar to sine wave's offset frequencies but squarewave this time. 