
# Bug2 Navigation Algorithm Simulation in Webots

This project simulates a mobile robot navigating using the **Bug2 algorithm** in the **Webots** robotic simulation environment.

## Project Structure

```
webots Bug2/
├── controllers/
│   └── robot/
│       └── robot.py        # Python controller implementing Bug2
├── worlds/
│   ├── world.wbt           # The simulation world
│   └── .world.wbproj       # Webots project metadata
```

## Requirements

- [Webots](https://cyberbotics.com/) (recommended version: R2023b or later)
- Python 3.x (configured inside Webots if using a Python controller)

## Description

This simulation demonstrates the **Bug2 path planning algorithm** — a simple reactive navigation strategy for moving a robot toward a goal while avoiding obstacles.

The main controller `robot.py` defines robot behavior. The robot will attempt to reach a target location by moving toward it directly, and if it encounters obstacles, it will follow the obstacle boundary until it can resume direct motion.

## How to Run

1. Open **Webots**.
2. Go to `File -> Open World...` and select `webots Bug2/worlds/world.wbt`.
3. Press the **Play** button to start the simulation.

The robot should begin navigating the environment using the Bug2 algorithm.

## Notes

- The robot and world are kept minimal to emphasize algorithm behavior.
- All logic is contained in `robot.py`.

## License

This project is provided for educational and research purposes.
