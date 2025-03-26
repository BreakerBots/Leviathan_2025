// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BreakerLib.driverstation.BreakerInputStream;
import frc.robot.BreakerLib.driverstation.BreakerInputStream2d;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HeadingSnap extends Command {
  /** Creates a new HeadingSnap. */
  private Drivetrain drivetrain;
  private SwerveRequest.FieldCentric request;
  private BreakerInputStream streamX, streamY;
  private Supplier<Rotation2d> headingGoalSupplier;
  private ProfiledPIDController pid;
  public HeadingSnap(Supplier<Rotation2d> headingGoalSupplier, Drivetrain drivetrain, BreakerInputStream2d linearInputStream) {
    this.drivetrain = drivetrain;
    request = new SwerveRequest.FieldCentric();
    request.DriveRequestType = DriveRequestType.Velocity;
    pid = new ProfiledPIDController(2.5, 0, 01, new TrapezoidProfile.Constraints(0.5, 0.5));
    pid.enableContinuousInput(-Math.PI, Math.PI);
    this.headingGoalSupplier = headingGoalSupplier;
    streamX = linearInputStream.getX();
    streamY = linearInputStream.getY();
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.reset(drivetrain.getLocalizer().getPose().getRotation().getRadians(), drivetrain.getLocalizer().getFieldRelativeSpeeds().omegaRadiansPerSecond);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    request.VelocityX = streamX.get();
    request.VelocityY = streamY.get();
    request.RotationalRate = pid.calculate(drivetrain.getLocalizer().getPose().getRotation().getRadians(), headingGoalSupplier.get().getRadians());
    drivetrain.setControl(request);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    request.VelocityX = streamX.get();
    request.VelocityY = streamY.get();
    request.RotationalRate = 0;
    drivetrain.setControl(request);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
