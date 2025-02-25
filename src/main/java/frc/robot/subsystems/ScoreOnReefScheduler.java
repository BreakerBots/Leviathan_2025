// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ButtonBoard;
import frc.robot.ReefPosition;
import frc.robot.BreakerLib.util.TimestampedValue;
import frc.robot.ReefPosition.ReefBranch;
import frc.robot.ReefPosition.ReefLevel;
import frc.robot.subsystems.superstructure.Superstructure;

public class ScoreOnReefScheduler {
  /** Creates a new ScoreOnReefScheduler. */
  private ButtonBoard buttonBoard;
  private Superstructure superstructure;
  private Optional<TimestampedValue<ReefBranch>> selectedBranch = Optional.empty();
  private Optional<TimestampedValue<ReefLevel>> selectedLevel = Optional.empty();
  private static final double kSelectionTimeout = 0.75;
  
  public ScoreOnReefScheduler(ButtonBoard buttonBoard, Superstructure superstructure) {
    this.buttonBoard = buttonBoard;
    this.superstructure = superstructure;
   
  }

  public void bind() {
    buttonBoard.getLevelButtons().getL1Button().onTrue(queueLevelSelection(ReefLevel.L1));
    buttonBoard.getLevelButtons().getL2Button().onTrue(queueLevelSelection(ReefLevel.L2));
    buttonBoard.getLevelButtons().getL3Button().onTrue(queueLevelSelection(ReefLevel.L3));
    buttonBoard.getLevelButtons().getL4Button().onTrue(queueLevelSelection(ReefLevel.L4));

    buttonBoard.getReefButtons().getReefButtonA().onTrue(queueBranchSelection(ReefBranch.A));
    buttonBoard.getReefButtons().getReefButtonB().onTrue(queueBranchSelection(ReefBranch.B));
    buttonBoard.getReefButtons().getReefButtonC().onTrue(queueBranchSelection(ReefBranch.C));
    buttonBoard.getReefButtons().getReefButtonD().onTrue(queueBranchSelection(ReefBranch.D));
    buttonBoard.getReefButtons().getReefButtonE().onTrue(queueBranchSelection(ReefBranch.E));
    buttonBoard.getReefButtons().getReefButtonF().onTrue(queueBranchSelection(ReefBranch.F));
    buttonBoard.getReefButtons().getReefButtonG().onTrue(queueBranchSelection(ReefBranch.G));
    buttonBoard.getReefButtons().getReefButtonH().onTrue(queueBranchSelection(ReefBranch.H));
    buttonBoard.getReefButtons().getReefButtonI().onTrue(queueBranchSelection(ReefBranch.I));
    buttonBoard.getReefButtons().getReefButtonJ().onTrue(queueBranchSelection(ReefBranch.J));
    buttonBoard.getReefButtons().getReefButtonK().onTrue(queueBranchSelection(ReefBranch.K));
    buttonBoard.getReefButtons().getReefButtonL().onTrue(queueBranchSelection(ReefBranch.L));

    buttonBoard.getRightButtons().getHighRightButton()
      .and(new Trigger(superstructure::endEffectorHasCoral))
      .onTrue(
        Commands.deferredProxy(
          () -> queueBranchSelection(
            ReefBranch.getClosest(
              superstructure.getDrivetrain().getLocalizer().getPose(), 
              DriverStation.getAlliance().orElse(Alliance.Blue)
            )
        )
      )
    );
  }

  private Command queueLevelSelection(ReefLevel level) {
    return Commands.runOnce(() -> selectedLevel = Optional.of(new TimestampedValue<ReefLevel>(level, Timer.getFPGATimestamp()))).andThen(checkSelectons());
  }

  private Command queueBranchSelection(ReefBranch branch) {
    return Commands.runOnce(() -> selectedBranch = Optional.of(new TimestampedValue<ReefBranch>(branch, Timer.getFPGATimestamp()))).andThen(checkSelectons());
  }

  private Command checkSelectons() {
    return Commands.runOnce(this::checkTimeouts).andThen(
      Commands.either(
        Commands.deferredProxy(() -> superstructure.scoreOnReef(new ReefPosition(selectedLevel.get().getValue(), selectedBranch.get().getValue()))).andThen(
          Commands.runOnce(this::clearSelections)
        ),
        Commands.none(),
        () -> selectedLevel.isPresent() && selectedBranch.isPresent() && !buttonBoard.getRightButtons().getHighRightSwitch().getAsBoolean()
      )
    );
    
  }

  private void clearSelections() {
    selectedBranch = Optional.empty();
    selectedLevel = Optional.empty();
  }

  private void checkTimeouts() {
    if (selectedBranch.isPresent() && (Timer.getFPGATimestamp() - selectedBranch.get().getTimestamp().in(Units.Seconds) > kSelectionTimeout)) {
      selectedBranch = Optional.empty();
    }

    if (selectedLevel.isPresent() && (Timer.getFPGATimestamp() - selectedLevel.get().getTimestamp().in(Units.Seconds) > kSelectionTimeout)) {
      selectedLevel = Optional.empty();
    }
  }

  
}
