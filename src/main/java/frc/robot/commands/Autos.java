// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /**
   * Creates an example autonomous routine
   * 
   * Command Sequence:
   * 1. Runs example method command (one-time action)
   * 2. Runs example command until it completes
   *
   * @param subsystem The subsystem needed for this auto routine
   * @return A command that will run during autonomous
   */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    // Commands.sequence runs multiple commands in order
    // Each command must finish before the next starts
    return Commands.sequence(
        subsystem.exampleMethodCommand(), 
        new ExampleCommand(subsystem)
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
