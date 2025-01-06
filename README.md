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

### Core Classes
- `Robot.java`: Manages robot lifecycle and mode transitions (auto, teleop, etc.)
- `RobotContainer.java`: Creates/configures subsystems and commands
- `Constants.java`: Stores robot-wide configuration values

### Commands
Commands define actions the robot can perform. Located in `commands/`:
- `Autos.java`: Autonomous routine factory
- `ExampleCommand.java`: Template for new commands

### Subsystems
Subsystems control robot hardware. Located in `subsystems/`:
- `ExampleSubsystem.java`: Template for new subsystems

## Getting Started

### Prerequisites
- WPILib 2024/25 VS Code
- Java 11 or newer
- Git

### Setup
1. Clone the repository:
   ```bash
   git clone git@github.com:BreakerBots/Leviathan_2025.git
   ```

2. Open the project in WPILib VS Code

3. Build the project:
   ```bash
   ./gradlew build
   ```

### Deployment
To deploy to the robot:
1. Connect to the robot's network
2. Press Ctrl+Shift+P in VS Code
3. Select "Deploy Robot Code"

## Contributing

1. Create a new branch for your feature
2. Make your changes
3. Test thoroughly
4. Submit a pull request

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
