// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.BreakerLib.swerve.BreakerSwerveDrivetrain;
import static frc.robot.Constants.DriveConstants.*;

public class Drivetrain extends BreakerSwerveDrivetrain {
  /** Creates a new Drivetrain. */
  public Drivetrain() {
    super(DRIVETRAIN_CONSTANTS, FrontLeft, FrontRight, BackLeft, BackRight);
  }
}
