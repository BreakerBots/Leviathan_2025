// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class TimedWaitUntilCommand extends Command {
  /** Creates a new TimedWaitUntill. */
  private BooleanSupplier condition;
  private double satisfactionTimeSeconds;
  private final Timer timer = new Timer();
  public TimedWaitUntilCommand(BooleanSupplier condition, double satisfactionTimeSeconds) {
    this.condition = condition;
    this.satisfactionTimeSeconds = satisfactionTimeSeconds;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (condition.getAsBoolean()) {
      timer.start();
    } else {
      timer.stop();
      timer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(satisfactionTimeSeconds) && condition.getAsBoolean();
  }
}
