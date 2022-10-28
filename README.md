# Package: RLB Turtle emulator
Used to emulate the physical behavior of the Turtlebot3 robot. The emulator only simulates movement dynamics, and does not simulate sensors. A single emulator node can emulate multiple turtlebots

To run:
```
ros2 run rlb_viz rlb_viz
```

*Note: While the emulator does not simulate any sensors, it can be made to return an empty scan as placeholder for compatibility's sake if desired*
