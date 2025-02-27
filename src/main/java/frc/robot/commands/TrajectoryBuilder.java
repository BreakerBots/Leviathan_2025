package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.CoralHumanPlayerStation;
import frc.robot.ReefPosition;
import frc.robot.ReefPosition.ReefBranch;
import frc.robot.subsystems.superstructure.Superstructure;

public class TrajectoryBuilder {
    private final AutoRoutine routine;
    private final Superstructure superstructure;
    private boolean flippedHorizontally = false;

    private final List<Pair<AutoTrajectory, Command>> trajectories = new ArrayList<>();

    public TrajectoryBuilder(Superstructure superstructure, AutoRoutine routine) {
        this.superstructure = superstructure;
        this.routine = routine;
    }

    public TrajectoryBuilder setFlipped(boolean flip) {
        flippedHorizontally = flip;
        return this;
    }

    public TrajectoryBuilder runThenScore(String traj, ReefPosition reefPosition) {
        reefPosition = doFlip(reefPosition);

        var trajectory = routine.trajectory(traj);
        if (flippedHorizontally) {
            trajectory = routine.trajectory(flipHorizontally(trajectory.getRawTrajectory()));
        }

        final var cmd = superstructure.scoreOnReefAuton(reefPosition).asProxy();
        trajectories.add(new Pair<>(trajectory, cmd));

        return this;
    }

    public TrajectoryBuilder runThenHP(String traj) {
        var trajectory = routine.trajectory(traj);
        if (flippedHorizontally) {
            trajectory = routine.trajectory(flipHorizontally(trajectory.getRawTrajectory()));
        }
        
        final var station = flipCoralHumanPlayerStation(hpToString(traj));
        final var cmd = superstructure.intakeCoralFromHumanPlayerAligned(station).asProxy();
        trajectories.add(new Pair<>(trajectory, cmd));

        return this;
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
                    -state.moduleForcesY()[0],
                    -state.moduleForcesY()[1],
                    -state.moduleForcesY()[2],
                    -state.moduleForcesY()[3],
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

        if (branch == 0) flipped = 1;
        else flipped += 1;

        return new ReefPosition(reefPosition.level(), ReefBranch.values()[flipped]);
    }

    public Command build() {
        if (trajectories.size() == 0) return routine.cmd(); // will do nothing.

        final var initial = trajectories.get(0);
        var lastTraj = initial;
        for (int i = 1; i < trajectories.size(); i++) {
            final var traj = trajectories.get(i);
            lastTraj.getFirst().done().onTrue(Commands.sequence(
                lastTraj.getSecond(),
                traj.getFirst().cmd().asProxy()
            ));
            lastTraj = traj;
        }

        lastTraj.getFirst().done().onTrue(lastTraj.getSecond());

        routine.active().onTrue(Commands.sequence(
            initial.getFirst().resetOdometry().asProxy(),
            initial.getFirst().cmd().asProxy()
        ));

        return routine.cmd();
    }

    private CoralHumanPlayerStation hpToString(String traj) {
        if (traj.contains("Coral PS2")) return CoralHumanPlayerStation.LOWER;
        else return CoralHumanPlayerStation.UPPER;
    }

    private CoralHumanPlayerStation flipCoralHumanPlayerStation(CoralHumanPlayerStation pos) {
        if (!flippedHorizontally) return pos;
        return switch (pos) {
            case UPPER -> CoralHumanPlayerStation.LOWER;
            case LOWER -> CoralHumanPlayerStation.UPPER;
        };
    }
}
