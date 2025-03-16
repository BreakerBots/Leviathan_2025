// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RumbleCommand extends Command {
  /** Creates a new TimedRumbleCommand. */
  private GenericHID hid;
  private RumbleType rumbleType;
  private double value;
  public RumbleCommand(GenericHID hid, RumbleType rumbleType, double value) {
    this.hid = hid;
    this.value = value;
    this.rumbleType = rumbleType;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hid.setRumble(rumbleType, value);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hid.setRumble(rumbleType, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
