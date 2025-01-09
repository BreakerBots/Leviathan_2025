# Leviathan 2025

BreakerBots' FRC robot code for the 2025 season. This repository contains the Java code that powers our robot "Leviathan".

## Project Structure

The codebase follows WPILib's command-based programming paradigm and is organized as follows:

```
src/main/java/frc/robot/
├── commands/          # Robot commands/actions
│   ├── Autos.java    # Autonomous routines
│   └── ExampleCommand.java
├── subsystems/       # Hardware control modules
│   └── ExampleSubsystem.java
├── Constants.java    # Robot-wide configuration
├── Main.java        # Program entry point
├── Robot.java       # Core robot functionality
└── RobotContainer.java # Robot composition & control bindings
```

## Key Components

### BreakerLib
   - A standerdized utility library for BreakerBots WPILib projects
   - Includes various device drivers, utils, and more

### Core Classes
- `Robot.java`: Manages robot lifecycle and mode transitions (auto, teleop, etc.)
- `RobotContainer.java`: Creates/configures subsystems and commands
- `Constants.java`: Stores robot-wide configuration values

### Commands
Commands define actions the robot can perform. Located in `commands/`:
- `Autos.java`: Autonomous routine factory
- `ExampleCommand.java`: Template for new commands

### Subsystems
Subsystems represent robot hardware abstractions. Located in `subsystems/`:
- `Drivetrain.java`: User Implementation of BreakerLib's "BreakerSwerveDrive" class, which uses the Phoenix Swerve API

## Getting Started

### Prerequisites
- WPILib 2025 VS Code
- Java 11 or newer
- Git
- ChoreoLib
- PhotonLib
- PathPlannerLib
- Phoenix 6
- DogLog
- GVersion

### Setup
1. Clone the repository:
   ```bash
   git clone git@github.com:BreakerBots/Leviathan_2025.git
   ```
   or:
      1. Open Github Desktop
      2. Navigate to: Current Repository -> Add -> Clone New Repository
      3. Select "BreakerBots/Leviathan_2025"

2. Open the project in WPILib VS Code

3. Build the project:
   ```bash
   ./gradlew build
   ```
   or:
      1. Press Ctrl+Shift+P in VS Code
      2. Select "Build Robot Code"

### Deployment
To deploy to the robot:
1. Connect to the robot's network or VH-109 Access Point 
3. Pres Shift+F5 in VS Code

## Project Status

This is a template project that needs to be developed for the 2025 season. Key tasks:
- [ ] Define robot subsystems
- [ ] Create game-specific commands
- [ ] Configure autonomous routines
- [ ] Set up driver controls
- [ ] Add constants for hardware IDs

## Additional Resources

- [WPILib Documentation](https://docs.wpilib.org/)
- [FRC Control System Documentation](https://docs.wpilib.org/en/stable/docs/zero-to-robot/introduction.html)
- [Command-Based Programming](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html)

## License

This project is licensed under the WPILib BSD License - see WPILib-License.md for details.
