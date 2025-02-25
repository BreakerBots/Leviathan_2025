// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.ReefPosition;
import frc.robot.ReefPosition.ReefBranch;
import frc.robot.ReefPosition.ReefLevel;
import frc.robot.BreakerLib.physics.BreakerVector2;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.superstructure.Superstructure;

/** Add your docs here. */
public class Autos {
    private final Superstructure superstructure;
    private final Drivetrain drivetrain;
    private final AutoFactory autoFactory;

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
                default -> throw new IllegalStateException();
            };
        }
    }

    public Autos(Superstructure superstructure) {
        this.superstructure = superstructure;
        this.drivetrain = superstructure.getDrivetrain();
        autoFactory = superstructure.getDrivetrain().getAutoFactory();
    }

    public Command testPath() {
        final var routine = autoFactory.newRoutine("testPath");
        final var startTraj = routine.trajectory("Start 1 to Reef J");
        final var toCoralPS = routine.trajectory("Reef J to Coral PS");

        startTraj
            .done()
            .onTrue(Commands.sequence( // note: scoreOnReef is not implemented
                superstructure.scoreOnReef(new ReefPosition(ReefLevel.L4, ReefBranch.L)),
                toCoralPS.cmd()
            ));


        routine.active().onTrue(Commands.sequence(
            startTraj.resetOdometry(),
            startTraj.cmd()
        ));

        return routine.cmd();
    }

    /**
     * Given a starting point, it will then go to
     * Reef J -> K -> L -> A, each time it will also visit the Coral Player Station
     * to restock on Coral.
     */
    public Command startThenJKLA(StartPosition startPosition) {
        final var startToReefJ = switch (startPosition) {
            case ONE -> runTrajectoryThenScore("Start 1 to Reef J", new ReefPosition(ReefLevel.L4, ReefBranch.J), true);
            case TWO -> runTrajectoryThenScore("Start 2 to Reef J", new ReefPosition(ReefLevel.L4, ReefBranch.J), true);
            case THREE -> runTrajectoryThenScore("Start 3 to Reef J", new ReefPosition(ReefLevel.L4, ReefBranch.J), true);
        };

        final var reefJToCoralPS = runTrajectoryThenHumanPlayer("Reef J to Coral PS");
        final var coralPSToReefK = runTrajectoryThenScore("Coral PS to Reef K", new ReefPosition(ReefLevel.L4, ReefBranch.K));
        final var reefKToCoralPS = runTrajectoryThenHumanPlayer("Reef K to Coral PS");
        final var coralPSToReefL = runTrajectoryThenScore("Coral PS to Reef L", new ReefPosition(ReefLevel.L4, ReefBranch.L));
        final var reefLToCoralPS = runTrajectoryThenHumanPlayer("Reef L to Coral PS");
        final var coralPSToReefA = runTrajectoryThenScore("Coral PS to Reef A", new ReefPosition(ReefLevel.L4, ReefBranch.A));

        return Commands.sequence(
            startToReefJ,
            reefJToCoralPS,
            coralPSToReefK,
            reefKToCoralPS,
            coralPSToReefL,
            reefLToCoralPS,
            coralPSToReefA
        );
    }

    public Command startThenGFED(StartPosition startPosition) { // I would like to put all these strings in a nice enum eventually to avoid typos, sounds like a job for the freshies.
        final var startToReefG = switch (startPosition) {
            case ONE -> runTrajectoryThenScore("Start 1 to Reef G", new ReefPosition(ReefLevel.L4, ReefBranch.G), true);
            case TWO -> runTrajectoryThenScore("Start 2 to Reef G", new ReefPosition(ReefLevel.L4, ReefBranch.G), true);
            case THREE -> runTrajectoryThenScore("Start 3 to Reef G", new ReefPosition(ReefLevel.L4, ReefBranch.G), true);
        };

        return Commands.sequence(
            startToReefG,
            runTrajectoryThenHumanPlayer("Reef G to Coral PS2"),
            runTrajectoryThenScore("Coral PS2 to Reef F", new ReefPosition(ReefLevel.L4, ReefBranch.F)),
            runTrajectoryThenHumanPlayer("Reef F to Coral PS2"),
            runTrajectoryThenScore("Coral PS2 to Reef E", new ReefPosition(ReefLevel.L4, ReefBranch.E)),
            runTrajectoryThenHumanPlayer("Reef E to Coral PS2"),
            runTrajectoryThenScore("Coral PS2 to Reef D", new ReefPosition(ReefLevel.L4, ReefBranch.D))
        );
        // final var routine = autoFactory.newRoutine("Start2->GFED");

        // final var startToG = switch (startPosition) {
        //     case ONE -> routine.trajectory("Start 1 to Reef G");
        //     case TWO -> routine.trajectory("Start 2 to Reef G");
        //     case THREE -> routine.trajectory("Start 3 to Reef G");
        // };

        // final var reefGToPS2 = routine.trajectory("Reef G to Coral PS2");
        // final var ps2ToReefF = routine.trajectory("Coral PS2 to Reef F");
        // final var reefFToPS2 = routine.trajectory("Reef F to Coral PS2");
        // final var ps2ToReefE = routine.trajectory("Coral PS2 to Reef E");
        // final var reefEToPS2 = routine.trajectory("Reef E to Coral PS2");
        // final var ps2ToReefD = routine.trajectory("Coral PS2 to Reef D");
        // // This is pretty repetitive, might be worth making some kind of builder for this
        // // after we verify that it works at all.
        // startToG.done().onTrue(Commands.sequence(
        //     superstructure.scoreOnReef(new ReefPosition(ReefLevel.L4, ReefBranch.G)),
        //     reefGToPS2.cmd()
        // ));

        // reefGToPS2.done().onTrue(Commands.sequence(
        //     superstructure.intakeCoralFromHumanPlayer(),
        //     ps2ToReefF.cmd()
        // ));

        // ps2ToReefF.done().onTrue(Commands.sequence(
        //     superstructure.scoreOnReef(new ReefPosition(ReefLevel.L4, ReefBranch.F)),
        //     reefFToPS2.cmd()
        // ));

        // reefFToPS2.done().onTrue(Commands.sequence(
        //     superstructure.intakeCoralFromHumanPlayer(),
        //     ps2ToReefE.cmd()
        // ));

        // ps2ToReefE.done().onTrue(Commands.sequence(
        //     superstructure.scoreOnReef(new ReefPosition(ReefLevel.L4, ReefBranch.E)),
        //     reefEToPS2.cmd()
        // ));

        // reefEToPS2.done().onTrue(Commands.sequence(
        //     superstructure.intakeCoralFromHumanPlayer(),
        //     ps2ToReefD.cmd()
        // ));

        // routine.active().onTrue(Commands.sequence(
        //     startToG.resetOdometry(),
        //     startToG.cmd()
        // ));

        // return routine.cmd();
    }

    // This autopath is really bad, but I couldn't think of any more interesting paths for coral.
    public Command start3ThenABC() {
        final var routine = autoFactory.newRoutine("Start3->ABC");
        
        final var startToA = routine.trajectory("Start 3 to Reef A");

        final var reefAToPS2 = routine.trajectory("Reef A to Coral PS2");
        final var ps2ToReefB = routine.trajectory("Coral PS2 to Reef B");
        final var reefBToPS2 = routine.trajectory("Reef B to Coral PS2");
        final var ps2ToReefC = routine.trajectory("Coral PS2 to Reef C");

        startToA.done().onTrue(Commands.sequence(
            superstructure.scoreOnReef(new ReefPosition(ReefLevel.L4, ReefBranch.A)),
            reefAToPS2.cmd()
        ));

        reefAToPS2.done().onTrue(Commands.sequence(
            superstructure.intakeCoralFromHumanPlayer(),
            ps2ToReefB.cmd()
        ));

        ps2ToReefB.done().onTrue(Commands.sequence(
            superstructure.scoreOnReef(new ReefPosition(ReefLevel.L4, ReefBranch.B)),
            reefBToPS2.cmd()
        ));

        reefBToPS2.done().onTrue(Commands.sequence(
            superstructure.intakeCoralFromHumanPlayer(),
            ps2ToReefC.cmd()
        ));

        ps2ToReefC.done().onTrue(Commands.sequence(
            superstructure.scoreOnReef(new ReefPosition(ReefLevel.L4, ReefBranch.C))
        ));

        routine.active().onTrue(Commands.sequence(
            startToA.resetOdometry(),
            startToA.cmd()
        ));

        return routine.cmd();
    }

    public Command start3ThenGDCB() {
        final var routine = autoFactory.newRoutine("Start3->GDCB");
        
        final var startToG = routine.trajectory("Start 3 to Reef G");

        final var reefGToPS2 = routine.trajectory("Reef G to Coral PS2");
        final var ps2ToReefD = routine.trajectory("Coral PS2 to Reef D");
        final var reefDToPS2 = routine.trajectory("Reef D to Coral PS2");
        final var ps2ToReefC = routine.trajectory("Coral PS2 to Reef C");
        final var reefCToPS2 = routine.trajectory("Reef C to Coral PS2");
        final var ps2ToReefB = routine.trajectory("Coral PS2 to Reef B");

        startToG.done().onTrue(Commands.sequence(
            superstructure.scoreOnReef(new ReefPosition(ReefLevel.L4, ReefBranch.A)),
            reefGToPS2.cmd()
        ));

        reefGToPS2.done().onTrue(Commands.sequence(
            superstructure.intakeCoralFromHumanPlayer(),
            ps2ToReefD.cmd()
        ));

        ps2ToReefD.done().onTrue(Commands.sequence(
            superstructure.scoreOnReef(new ReefPosition(ReefLevel.L4, ReefBranch.B)),
            reefDToPS2.cmd()
        ));

        reefDToPS2.done().onTrue(Commands.sequence(
            superstructure.intakeCoralFromHumanPlayer(),
            ps2ToReefC.cmd()
        ));

        ps2ToReefC.done().onTrue(Commands.sequence(
            superstructure.scoreOnReef(new ReefPosition(ReefLevel.L4, ReefBranch.C)),
            reefCToPS2.cmd()
        ));

        reefCToPS2.done().onTrue(Commands.sequence(
            superstructure.intakeCoralFromHumanPlayer(),
            ps2ToReefB.cmd()
        ));

        ps2ToReefB.done().onTrue(Commands.sequence(
            superstructure.scoreOnReef(new ReefPosition(ReefLevel.L4, ReefBranch.C))
        ));

        routine.active().onTrue(Commands.sequence(
            startToG.resetOdometry(),
            startToG.cmd()
        ));


        return routine.cmd();
    }

    private Command runTrajectoryThenScore(String traj, ReefPosition reefPosition) {
        return runTrajectoryThenScore(traj, reefPosition, false);
    }

    private Command runTrajectoryThenScore(String traj, ReefPosition reefPosition, boolean resetOdometry) {
        final var routine = autoFactory.newRoutine("score");

        final var trajectory = routine.trajectory(traj);

        final var seq = resetOdometry
            ? Commands.sequence(trajectory.resetOdometry(), trajectory.cmd())
            : trajectory.cmd();

        routine.active().onTrue(seq);

        final var finalPos = new BreakerVector2(trajectory.getFinalPose().orElseThrow().getTranslation());
        final var finalRot = trajectory.getFinalPose().orElseThrow().getRotation().getRadians();
        final var backupTimer = new Timer();
        
        // not super happy with this solution.
        return Commands.sequence(
            routine.cmd(() -> {
                if (backupTimer.advanceIfElapsed(0.2)) return true;

                final var pos = new BreakerVector2(superstructure.getDrivetrain().getLocalizer().getPose().getTranslation());
                final var rot = drivetrain.getLocalizer().getPose().getRotation().getRadians();
                
                final var dx = Math.abs(pos.getX() - finalPos.getX());
                final var dy = Math.abs(pos.getY() - finalPos.getY());
                final var dt = Math.abs(finalRot - rot);

                if (dx < 0.5 && dy < 0.5 && dt < 0.5 & !backupTimer.isRunning()) {
                    backupTimer.start();
                }

                return dx < 0.1 && dy < 0.1 && dt < 0.1;

            }),
            superstructure.scoreOnReef(reefPosition)
        );
    }

    private Command runTrajectoryThenHumanPlayer(String traj) {
        final var routine = autoFactory.newRoutine("human-player");

        final var trajectory = routine.trajectory(traj);

        final var intakeCmd = superstructure.intakeCoralFromHumanPlayer();
        routine.active().onTrue(Commands.sequence(
            trajectory.resetOdometry(),
            trajectory.cmd()
        ));

        final var finalPos = new BreakerVector2(trajectory.getFinalPose().orElseThrow().getTranslation());
        final var finalRot = trajectory.getFinalPose().orElseThrow().getRotation().getRadians();
        final var backupTimer = new Timer();

        return Commands.sequence(
            routine.cmd(() -> {
                if (backupTimer.advanceIfElapsed(0.2)) return true;

                final var pos = new BreakerVector2(drivetrain.getLocalizer().getPose().getTranslation());
                final var rot = drivetrain.getLocalizer().getPose().getRotation().getRadians();

                final var dx = Math.abs(pos.getX() - finalPos.getX());
                final var dy = Math.abs(pos.getY() - finalPos.getY());
                final var dt = Math.abs(finalRot - rot);
                
                if (dx < 0.5 && dy < 0.5 && dt < 0.5 & !backupTimer.isRunning()) {
                    backupTimer.start();
                }

                return dx < 0.1 && dy < 0.1 && dt < 0.1;

            }),
            intakeCmd
        );
    }
}
