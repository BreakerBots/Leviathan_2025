// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ReefPosition;
import frc.robot.ReefPosition.ReefBranch;
import frc.robot.ReefPosition.ReefLevel;
import frc.robot.subsystems.superstructure.Superstructure;

/** Add your docs here. */
public class Autos {
    private final Superstructure superstructure;
    private final AutoFactory autoFactory;

    private final SendableChooser<Supplier<Command>> autoChooser = new SendableChooser<>();
    private final SendableChooser<Boolean> flipChooser = new SendableChooser<>();

    private boolean flippedHorizontally = false;

    public static enum StartPosition {
        ONE,
        TWO,
        THREE;

        public static StartPosition fromDriverStation() {
            final var loc = DriverStation.getLocation();
            final var i = loc.orElseThrow(); // maybe change this.
            return switch (i) {
                case 1 -> ONE;
                case 2 -> TWO;
                case 3 -> THREE;
                default -> {
                    System.out.println("literally impossible");
                    yield ONE;
                }
            };
        }
    }

    public Autos(Superstructure superstructure) {
        this.superstructure = superstructure;
        autoFactory = superstructure.getDrivetrain().getAutoFactory();
        setupChooser();
    }

    public Command getSelectedAuto() {
        flippedHorizontally = flipChooser.getSelected();
        return autoChooser.getSelected().get();
    }

    private void setupChooser() {
        autoChooser.setDefaultOption("Start -> JKLA", () -> startThenJKLA(StartPosition.fromDriverStation()));
        autoChooser.addOption("Start -> GFED", () -> startThenGFED(StartPosition.fromDriverStation()));
        
        flipChooser.setDefaultOption("No flip", false);
        flipChooser.addOption("Flip", true);

        Shuffleboard.getTab("Autonomous").add(autoChooser);
        Shuffleboard.getTab("Autonomous").add(flipChooser);
    }

    /**
     * Given a starting point, it will then go to
     * Reef J -> K -> L -> A, each time it will also visit the Coral Player Station
     * to restock on Coral.
     */
    public Command startThenJKLA(StartPosition startPosition) {
        final var start = switch (startPosition) {
            case ONE -> "Start 1 to Reef J";
            case TWO -> "Start 2 to Reef J";
            case THREE -> "Start 3 to Reef J";
        };
        return new TrajectoryBuilder(superstructure, autoFactory.newRoutine("JKLA"))
            .setFlipped(flippedHorizontally)
            .runThenScore(start, new ReefPosition(ReefLevel.L4, ReefBranch.J))
            .runThenHP("Reef J to Coral PS")
            .runThenScore("Coral PS to Reef K", new ReefPosition(ReefLevel.L4, ReefBranch.K))
            .runThenHP("Reef K to Coral PS")
            .runThenScore("Coral PS to Reef L", new ReefPosition(ReefLevel.L4, ReefBranch.L))
            .runThenHP("Reef L to Coral PS")
            .runThenScore("Coral PS to Reef A", new ReefPosition(ReefLevel.L4, ReefBranch.A))
            .build();
    }
    // public Command startThenJKLA(StartPosition startPosition) {
    //     final var routine = autoFactory.newRoutine("JKLA");

    //     final var startToReefJ = routine.trajectory("Start 1 to Reef J");
    //     final var reefJToCoralPS = routine.trajectory("Reef J to Coral PS");
    //     final var coralPSToReefK = routine.trajectory("Coral PS to Reef K");
    //     final var reefKToCoralPS = routine.trajectory("Reef K to Coral PS");
    //     final var coralPSToReefL = routine.trajectory("Coral PS to Reef L");

    //     startToReefJ.done().onTrue(Commands.sequence(
    //         superstructure.scoreOnReefAuton(new ReefPosition(ReefLevel.L4, ReefBranch.J)).asProxy(),
    //         reefJToCoralPS.cmd().asProxy()
    //     ));

    //     reefJToCoralPS.done().onTrue(Commands.sequence(
    //         superstructure.intakeCoralFromHumanPlayer().asProxy(),
    //         coralPSToReefK.cmd().asProxy()
    //     ));

    //     coralPSToReefK.done().onTrue(Commands.sequence(
    //         superstructure.scoreOnReefAuton(new ReefPosition(ReefLevel.L4, ReefBranch.K)),
    //         reefKToCoralPS.cmd().asProxy()
    //     ));

    //     reefKToCoralPS.done().onTrue(Commands.sequence(
    //         superstructure.intakeCoralFromHumanPlayer().asProxy(),
    //         coralPSToReefL.cmd().asProxy()
    //     ));

    //     coralPSToReefL.done().onTrue(Commands.sequence(
    //         superstructure.scoreOnReefAuton(new ReefPosition(ReefLevel.L4, ReefBranch.L))
    //     ));

    //     routine.active().onTrue(Commands.sequence(
    //         startToReefJ.resetOdometry().asProxy(),
    //         startToReefJ.cmd().asProxy()
    //     ));

    //     return routine.cmd();
    // }
    // public Command startThenJKLA(StartPosition startPosition) {
    //     final var routine = autoFactory.newRoutine("JKLA");
    //     final var startToReefJ = switch (startPosition) {
    //         case ONE -> runTrajectoryThenScore(routine, "Start 1 to Reef J", new ReefPosition(ReefLevel.L4, ReefBranch.J), true);
    //         case TWO -> runTrajectoryThenScore(routine, "Start 2 to Reef J", new ReefPosition(ReefLevel.L4, ReefBranch.J), true);
    //         case THREE -> runTrajectoryThenScore(routine, "Start 3 to Reef J", new ReefPosition(ReefLevel.L4, ReefBranch.J), true);
    //     };

    //     final var reefJToCoralPS = runTrajectoryThenHumanPlayer(routine, "Reef J to Coral PS");
    //     final var coralPSToReefK = runTrajectoryThenScore(routine, "Coral PS to Reef K", new ReefPosition(ReefLevel.L4, ReefBranch.K));
    //     final var reefKToCoralPS = runTrajectoryThenHumanPlayer(routine, "Reef K to Coral PS");
    //     final var coralPSToReefL = runTrajectoryThenScore(routine, "Coral PS to Reef L", new ReefPosition(ReefLevel.L4, ReefBranch.L));
    //     final var reefLToCoralPS = runTrajectoryThenHumanPlayer(routine, "Reef L to Coral PS");
    //     final var coralPSToReefA = runTrajectoryThenScore(routine, "Coral PS to Reef A", new ReefPosition(ReefLevel.L4, ReefBranch.A));

    //     routine.active().onTrue(Commands.sequence(
    //         startToReefJ,
    //         reefJToCoralPS,
    //         coralPSToReefK,
    //         reefKToCoralPS,
    //         coralPSToReefL,
    //         reefLToCoralPS,
    //         coralPSToReefA
            
    //     ));

    //     return routine.cmd();
    // }

    public Command startThenGFED(StartPosition startPosition) { // I would like to put all these strings in a nice enum eventually to avoid typos, sounds like a job for the freshies.
        final var start = switch (startPosition) {
            case ONE -> "Start 1 to Reef G";
            case TWO -> "Start 2 to Reef G";
            case THREE -> "Start 3 to Reef G";
        };
        return new TrajectoryBuilder(superstructure, autoFactory.newRoutine("GFED"))
            .setFlipped(flippedHorizontally)
            .runThenScore(start, new ReefPosition(ReefLevel.L4, ReefBranch.G))
            .runThenHP("Reef G to Coral PS2")
            .runThenScore("Coral PS2 to Reef F", new ReefPosition(ReefLevel.L4, ReefBranch.F))
            .runThenHP("Reef F to Coral PS2")
            .runThenScore("Coral PS2 to Reef E", new ReefPosition(ReefLevel.L4, ReefBranch.E))
            .runThenHP("Reef E to Coral PS2")
            .runThenScore("Coral PS2 to Reef D", new ReefPosition(ReefLevel.L4, ReefBranch.D))
            .build();
        // final var routine = autoFactory.newRoutine("GFED");
        // final var startToReefG = switch (startPosition) {
        //     case ONE -> runTrajectoryThenScore(routine, "Start 1 to Reef G", new ReefPosition(ReefLevel.L4, ReefBranch.G), true);
        //     case TWO -> runTrajectoryThenScore(routine, "Start 2 to Reef G", new ReefPosition(ReefLevel.L4, ReefBranch.G), true);
        //     case THREE -> runTrajectoryThenScore(routine, "Start 3 to Reef G", new ReefPosition(ReefLevel.L4, ReefBranch.G), true);
        // };

        // final var reefGToCoralPS2 = runTrajectoryThenHumanPlayer(routine, "Reef G to Coral PS2");
        // final var coralPS2ToReefF = runTrajectoryThenScore(routine, "Coral PS2 to Reef F", new ReefPosition(ReefLevel.L4, ReefBranch.F));
        // final var reefFToCoralPS2 = runTrajectoryThenHumanPlayer(routine, "Reef F to Coral PS2");
        // final var coralPS2ToReefE = runTrajectoryThenScore(routine, "Coral PS2 to Reef E", new ReefPosition(ReefLevel.L4, ReefBranch.E));
        // final var reefEToCoralPS2 = runTrajectoryThenHumanPlayer(routine, "Reef E to Coral PS2");
        // final var coralPS2ToReefD = runTrajectoryThenScore(routine, "Coral PS2 to Reef D", new ReefPosition(ReefLevel.L4, ReefBranch.D));

        // routine.active().onTrue(Commands.sequence(
        //     startToReefG,
        //     reefGToCoralPS2,
        //     coralPS2ToReefF,
        //     reefFToCoralPS2,
        //     coralPS2ToReefE,
        //     reefEToCoralPS2,
        //     coralPS2ToReefD
        // ));

        // return routine.cmd();
    }

    // public Command start3ThenGDCB() {
    //     final var routine = autoFactory.newRoutine("3->GDCB");
    //     return Commands.sequence(
    //         runTrajectoryThenScore(routine, "Start 3 to Reef G", new ReefPosition(ReefLevel.L4, ReefBranch.G)).cmd(),
    //         runTrajectoryThenHumanPlayer(routine, "Reef G to Coral PS2").cmd(),
    //         runTrajectoryThenScore(routine, "Coral PS2 to Reef D", new ReefPosition(ReefLevel.L4, ReefBranch.D)).cmd(),
    //         runTrajectoryThenHumanPlayer(routine, "Reef D to Coral PS2").cmd(),
    //         runTrajectoryThenScore(routine, "Coral PS2 to Reef C", new ReefPosition(ReefLevel.L4, ReefBranch.C)).cmd(),
    //         runTrajectoryThenHumanPlayer(routine, "Reef C to Coral PS2").cmd(),
    //         runTrajectoryThenScore(routine, "Coral PS2 to Reef B", new ReefPosition(ReefLevel.L4, ReefBranch.B)).cmd()
    //     );
    // }s
}
