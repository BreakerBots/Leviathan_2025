// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.function.Supplier;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.ReefPosition;
import frc.robot.ReefPosition.ReefBranch;
import frc.robot.ReefPosition.ReefLevel;
import frc.robot.BreakerLib.physics.BreakerVector2;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.superstructure.Superstructure;

/** Add your docs here. */
public class Autos {
    private final Superstructure superstructure;
    private final Drivetrain drivetrain;
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
                default -> throw new IllegalStateException();
            };
        }
    }

    public Autos(Superstructure superstructure) {
        this.superstructure = superstructure;
        this.drivetrain = superstructure.getDrivetrain();
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
    }

    public Command start3ThenGDCB() {
        return Commands.sequence(
            runTrajectoryThenScore("Start 3 to Reef G", new ReefPosition(ReefLevel.L4, ReefBranch.G), true),
            runTrajectoryThenHumanPlayer("Reef G to Coral PS2"),
            runTrajectoryThenScore("Coral PS2 to Reef D", new ReefPosition(ReefLevel.L4, ReefBranch.D)),
            runTrajectoryThenHumanPlayer("Reef D to Coral PS2"),
            runTrajectoryThenScore("Coral PS2 to Reef C", new ReefPosition(ReefLevel.L4, ReefBranch.C)),
            runTrajectoryThenHumanPlayer("Reef C to Coral PS2"),
            runTrajectoryThenScore("Coral PS2 to Reef B", new ReefPosition(ReefLevel.L4, ReefBranch.B))
        );
    }

    private Command runTrajectoryThenScore(String traj, ReefPosition reefPosition) {
        return runTrajectoryThenScore(traj, reefPosition, false);
    }

    private Command runTrajectoryThenScore(String traj, ReefPosition reefPosition, boolean resetOdometry) {
        reefPosition = doFlip(reefPosition);
        final var routine = autoFactory.newRoutine("score");

        var trajectory = routine.trajectory(traj);
        if (flippedHorizontally) {
            trajectory = routine.trajectory(flipHorizontally(trajectory.getRawTrajectory()));
        }

        final var seq = resetOdometry
            ? Commands.sequence(trajectory.resetOdometry(), trajectory.cmd())
            : trajectory.cmd();

        routine.active().onTrue(seq);

        // final var finalPos = new BreakerVector2(trajectory.getFinalPose().orElseThrow().getTranslation());
        // final var finalRot = trajectory.getFinalPose().orElseThrow().getRotation().getRadians();
        // final var backupTimer = new Timer();
        
        // not super happy with this solution.
        return Commands.sequence(
            Commands.print(reefPosition.branch().toString()),
            // routine.cmd(() -> {
            //     if (backupTimer.advanceIfElapsed(0.2)) return true;

            //     final var pos = new BreakerVector2(superstructure.getDrivetrain().getLocalizer().getPose().getTranslation());
            //     final var rot = drivetrain.getLocalizer().getPose().getRotation().getRadians();
                
            //     final var dx = Math.abs(pos.getX() - finalPos.getX());
            //     final var dy = Math.abs(pos.getY() - finalPos.getY());

            //     if (dx < 0.5 && dy < 0.5 && BreakerMath.isAngleClose(finalRot, rot, 0.5) & !backupTimer.isRunning()) {
            //         backupTimer.start();
            //     }

            //     return dx < 0.1 && dy < 0.1 && BreakerMath.isAngleClose(finalRot, rot, 0.1);

            // }),
            routine.cmd(),
            superstructure.scoreOnReefAuton(reefPosition)
        );
    }

    private Command runTrajectoryThenHumanPlayer(String traj) {
        final var routine = autoFactory.newRoutine("human-player");

        var trajectory = routine.trajectory(traj);
        if (flippedHorizontally) {
            trajectory = routine.trajectory(flipHorizontally(trajectory.getRawTrajectory()));
        }

        final var intakeCmd = superstructure.intakeCoralFromHumanPlayer();
        routine.active().onTrue(Commands.sequence(
            trajectory.resetOdometry(),
            trajectory.cmd()
        ));

        // final var finalPos = new BreakerVector2(trajectory.getFinalPose().orElseThrow().getTranslation());
        // final var finalRot = trajectory.getFinalPose().orElseThrow().getRotation().getRadians();
        // final var backupTimer = new Timer();

        return Commands.sequence(
            // routine.cmd(() -> {
            //     if (backupTimer.advanceIfElapsed(0.2)) return true;

            //     final var pos = new BreakerVector2(drivetrain.getLocalizer().getPose().getTranslation());
            //     final var rot = drivetrain.getLocalizer().getPose().getRotation().getRadians();

            //     final var dx = Math.abs(pos.getX() - finalPos.getX());
            //     final var dy = Math.abs(pos.getY() - finalPos.getY());
                
            //     if (dx < 0.5 && dy < 0.5 && BreakerMath.isAngleClose(finalRot, rot, 0.5) & !backupTimer.isRunning()) {
            //         backupTimer.start();
            //     }

            //     return dx < 0.1 && dy < 0.1 && BreakerMath.isAngleClose(finalRot, rot, 0.1);

            // }),
            routine.cmd(),
            intakeCmd
        );
    }

    private Trajectory<SwerveSample> flipHorizontally(Trajectory<SwerveSample> traj) {
        final var flipped = new ArrayList<SwerveSample>();
        for (final var state : traj.samples()) {
            final var y = ChoreoAllianceFlipUtil.flipY(state.getPose().getY());
            final var heading = (2 * Math.PI) - state.getPose().getRotation().getRadians();

            var sample = new SwerveSample(
                state.t, 
                state.x, 
                y, 
                heading, 
                state.vx, 
                -state.vy, 
                -state.omega, 
                state.ax, 
                -state.ay, 
                -state.alpha, 
                new double[] {
                    state.moduleForcesX()[0],
                    state.moduleForcesX()[1],
                    state.moduleForcesX()[2],
                    state.moduleForcesX()[3],
                },
                new double[] { // FIXME... maybe?
                    state.moduleForcesY()[0],
                    state.moduleForcesY()[1],
                    state.moduleForcesY()[2],
                    state.moduleForcesY()[3],
                });
            flipped.add(sample);
        }

        return new Trajectory<SwerveSample>(traj.name(), flipped, traj.splits(), traj.events());
    }

    private ReefPosition doFlip(ReefPosition reefPosition) {
        return flippedHorizontally
            ? flipReefPosition(reefPosition)
            : reefPosition;
    }

    private ReefPosition flipReefPosition(ReefPosition reefPosition) {
        final int branch = reefPosition.branch().ordinal();
        final int last = ReefBranch.L.ordinal()+1;
        int flipped = last - branch;
        if (branch >= 6) flipped += 1;
        else flipped -= 1;

        return new ReefPosition(reefPosition.level(), ReefBranch.values()[flipped]);
    }
}
