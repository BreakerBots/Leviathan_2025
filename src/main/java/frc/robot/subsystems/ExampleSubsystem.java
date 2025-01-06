// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
  // === HARDWARE ===
  // Define hardware components here (motors, sensors, etc.)
  
  // === STATE VARIABLES ===
  // Track internal state of the subsystem here
  
  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {}

  /**
   * Creates a command that performs a one-time action
   * Example usage: Setting a solenoid, zeroing sensors
   */
  public Command exampleMethodCommand() {
    return runOnce(
        () -> {
          // One-time action code goes here
        });
  }

  /**
   * Checks if some condition is met
   * Example usage: Checking if a sensor is triggered or position reached
   * @return true if condition is met, false otherwise
   */
  public boolean exampleCondition() {
    return false;
  }

  @Override
  public void periodic() {
    // This method runs every 20ms (50 times per second)
    // Common tasks to do here:
    // 1. Update sensor readings
    // 2. Run control loops (PID, etc.)
    // 3. Update dashboard values
    // 4. Update internal state variables
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
