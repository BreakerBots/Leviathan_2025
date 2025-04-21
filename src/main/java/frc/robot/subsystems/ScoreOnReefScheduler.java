// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.Queue;
import java.util.function.Supplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ButtonBoard;
import frc.robot.ReefPosition;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import frc.robot.BreakerLib.util.TimestampedValue;
import frc.robot.ReefPosition.ReefBranch;
import frc.robot.ReefPosition.ReefLevel;
import frc.robot.subsystems.superstructure.Superstructure;

public class ScoreOnReefScheduler extends SubsystemBase{
  /** Creates a new ScoreOnReefScheduler. */
  private ButtonBoard buttonBoard;
  private Superstructure superstructure;
  private BreakerXboxController controller;
    private Optional<TimestampedValue<ReefLevel>> selectedLevel;
    private Optional<TimestampedValue<Supplier<ReefBranch>>> selectedBranch;
    private Optional<Pair<ReefPosition, Command>> activeScoreingCommand;
  private static final double kSelectionTimeout = 0.75;
  
  public ScoreOnReefScheduler(ButtonBoard buttonBoard, BreakerXboxController controller, Superstructure superstructure) {
    this.buttonBoard = buttonBoard;
    this.controller = controller;
    this.superstructure = superstructure;
    selectedLevel = Optional.empty();
    selectedBranch = Optional.empty();
    activeScoreingCommand = Optional.empty();
   
  }

  public void bind() {
    
    Trigger manualOverride = buttonBoard.getRightButtons().getHighRightSwitch();
    Trigger manualOverrideNegated = manualOverride.negate();

    manualOverride.onTrue(Commands.runOnce(() -> {
      selectedLevel = Optional.empty();
      selectedBranch = Optional.empty();
      if (activeScoreingCommand.isPresent()) {
        activeScoreingCommand.get().getSecond().cancel();
        activeScoreingCommand = Optional.empty();
      }
    }));

    buttonBoard.getLevelButtons().getL1Button().and(manualOverrideNegated).onTrue(queueLevelSelection(ReefLevel.L1));
    buttonBoard.getLevelButtons().getL2Button().and(manualOverrideNegated).onTrue(queueLevelSelection(ReefLevel.L2));
    buttonBoard.getLevelButtons().getL3Button().and(manualOverrideNegated).onTrue(queueLevelSelection(ReefLevel.L3));
    buttonBoard.getLevelButtons().getL4Button().and(manualOverrideNegated).onTrue(queueLevelSelection(ReefLevel.L4));

    buttonBoard.getReefButtons().getReefButtonA().and(manualOverrideNegated).whileTrue(queueBranchSelection(() -> ReefBranch.A));
    buttonBoard.getReefButtons().getReefButtonB().and(manualOverrideNegated).whileTrue(queueBranchSelection(() -> ReefBranch.B));
    buttonBoard.getReefButtons().getReefButtonC().and(manualOverrideNegated).whileTrue(queueBranchSelection(() -> ReefBranch.C));
    buttonBoard.getReefButtons().getReefButtonD().and(manualOverrideNegated).whileTrue(queueBranchSelection(() -> ReefBranch.D));
    buttonBoard.getReefButtons().getReefButtonE().and(manualOverrideNegated).whileTrue(queueBranchSelection(() -> ReefBranch.E));
    buttonBoard.getReefButtons().getReefButtonF().and(manualOverrideNegated).whileTrue(queueBranchSelection(() -> ReefBranch.F));
    buttonBoard.getReefButtons().getReefButtonG().and(manualOverrideNegated).whileTrue(queueBranchSelection(() -> ReefBranch.G));
    buttonBoard.getReefButtons().getReefButtonH().and(manualOverrideNegated).whileTrue(queueBranchSelection(() -> ReefBranch.H));
    buttonBoard.getReefButtons().getReefButtonI().and(manualOverrideNegated).whileTrue(queueBranchSelection(() -> ReefBranch.I));
    buttonBoard.getReefButtons().getReefButtonJ().and(manualOverrideNegated).whileTrue(queueBranchSelection(() -> ReefBranch.J));
    buttonBoard.getReefButtons().getReefButtonK().and(manualOverrideNegated).whileTrue(queueBranchSelection(() -> ReefBranch.K));
    buttonBoard.getReefButtons().getReefButtonL().and(manualOverrideNegated).whileTrue(queueBranchSelection(() -> ReefBranch.L));

    buttonBoard.getLevelButtons().getL1Button().and(manualOverride).onTrue(superstructure.scoreOnReefManual(ReefPosition.ReefLevel.L1));
    buttonBoard.getLevelButtons().getL2Button().and(manualOverride).onTrue(superstructure.scoreOnReefManual(ReefPosition.ReefLevel.L2));
    buttonBoard.getLevelButtons().getL3Button().and(manualOverride).onTrue(superstructure.scoreOnReefManual(ReefPosition.ReefLevel.L3));
    buttonBoard.getLevelButtons().getL4Button().and(manualOverride).onTrue(superstructure.scoreOnReefManual(ReefPosition.ReefLevel.L4));
    
    buttonBoard.getRightButtons().getHighRightButton()
      .and(manualOverrideNegated)
      .and(new Trigger(superstructure::endEffectorHasCoral))
      .whileTrue(
        queueBranchSelection(() -> ReefBranch.A)
      );

    Trigger driverAllignL4 = controller.getDPad().getDown();
    Trigger driverAllignL3 = controller.getBackButton();

    driverAllignL3
    .and(manualOverrideNegated)
    .and(new Trigger(superstructure::endEffectorHasCoral))
    .onTrue(
      queueAllSelections(ReefLevel.L3, this::getClosestBranch)
    );

    driverAllignL4
    .and(manualOverrideNegated)
    .and(new Trigger(superstructure::endEffectorHasCoral))
    .onTrue(
        queueAllSelections(ReefLevel.L4, this::getClosestBranch)
    );

    driverAllignL4.and(manualOverride).onTrue(superstructure.scoreOnReefManual(ReefLevel.L4));
    driverAllignL3.and(manualOverride).onTrue(superstructure.scoreOnReefManual(ReefLevel.L3));

    controller.getDPad().getUp().and(manualOverrideNegated).onTrue(
      forceAlternateBranch()
    );

  }

  private ReefBranch getClosestBranch() {
    return ReefBranch.getClosest(
            superstructure.getDrivetrain().getLocalizer().getPose(), 
            DriverStation.getAlliance().orElse(Alliance.Blue)
            );
  }

  private Command queueAllSelections(ReefLevel level, Supplier<ReefBranch> branchSelection) {
    return Commands.runOnce(() -> selectedLevel = Optional.of(new TimestampedValue<ReefLevel>(level, Timer.getFPGATimestamp())))
    .alongWith(Commands.runOnce(() -> selectedBranch = Optional.of(new TimestampedValue<Supplier<ReefBranch>>(branchSelection, Timer.getFPGATimestamp()))).andThen(updateCmd()))
    .andThen(updateCmd());
  }

  private Command queueLevelSelection(ReefLevel level) {
    return new RepeatCommand(Commands.runOnce(() -> selectedLevel = Optional.of(new TimestampedValue<ReefLevel>(level, Timer.getFPGATimestamp()))).andThen(updateCmd()));
  }

  private Command queueBranchSelection(Supplier<ReefBranch> branchSelection) {
    return new RepeatCommand(Commands.runOnce(() -> selectedBranch = Optional.of(new TimestampedValue<Supplier<ReefBranch>>(branchSelection, Timer.getFPGATimestamp()))).andThen(updateCmd()));
  }

  private Command forceAlternateBranch() {
    return Commands.runOnce(() -> {
      if (activeScoreingCommand.isPresent()) {
        var asc = activeScoreingCommand.get();
        var branch = asc.getFirst().branch().getAlternate();
        var selection = new ReefPosition(asc.getFirst().level(), branch);
        Command cmd = superstructure.scoreOnReef(selection);
        activeScoreingCommand = Optional.of(new Pair<>(selection, cmd));
        cmd.schedule();
        selectedLevel = Optional.empty();
        selectedLevel = Optional.empty();
      }
    }).andThen(updateCmd());
  }


  private Command updateCmd() {
    return Commands.runOnce(this::update);
  }

  private void update() {
    if (activeScoreingCommand.isPresent()) {
        var command = activeScoreingCommand.get();
        if (command.getSecond().isFinished()) {
            activeScoreingCommand = Optional.empty();
        }
    }
    if (selectedBranch.isPresent() && selectedLevel.isPresent()) {
        var sb = selectedBranch.get();
        var sl = selectedLevel.get();
        if (sb.getTimestamp().in(Units.Seconds) + kSelectionTimeout < Timer.getFPGATimestamp()) {
            selectedBranch = Optional.empty();
        }

        if (sl.getTimestamp().in(Units.Seconds) + kSelectionTimeout < Timer.getFPGATimestamp()) {
            selectedBranch = Optional.empty();
        }

        if (selectedBranch.isPresent() && selectedLevel.isPresent()) {
            var selection = new ReefPosition(sl.getValue(), sb.getValue().get());
            if (activeScoreingCommand.isPresent()) {
                if (activeScoreingCommand.get().getFirst().equals(selection)) {
                    return;
                } else {
                    var cmd = superstructure.scoreOnReef(selection);
                    activeScoreingCommand = Optional.of(new Pair<>(selection, cmd));
                    cmd.schedule();
                    selectedLevel = Optional.empty();
                    selectedLevel = Optional.empty();
                }
            } else {
                    var cmd = superstructure.scoreOnReef(selection);
                    activeScoreingCommand = Optional.of(new Pair<>(selection, cmd));
                    cmd.schedule();
                    selectedLevel = Optional.empty();
                    selectedLevel = Optional.empty();
            }
        }
    }
  }

  @Override
  public void periodic() {
      update();
  }


  
}
