package frc.robot.commands;

import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.fasterxml.jackson.core.PrettyPrinter;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.LocalADStar;
import frc.robot.BreakerLib.util.Localizer;
import frc.robot.commands.DriveToPose.NavToPoseConfig;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.superstructure.Superstructure;

public class AutoPilot2 {
    private LocalADStar pathfinder;
    private Superstructure superstructure;
    public AutoPilot2(Superstructure superstructure) {
        this.superstructure = superstructure;
        pathfinder = new LocalADStar("pathplanner/navgrid.json");
        Pathfinding.setPathfinder(pathfinder);
    }

    public Command navigateToPose(Supplier<Pose2d> goal) {
        return navigateToPose(goal, new NavToPoseConfig());
    }

    public Command navigateToPose(Supplier<Pose2d> goal, NavToPoseConfig navConfig) {
        return new NavCommand(goal, navConfig);
    }

    private class NavCommand extends Command {
        private boolean directPath = false ;
        private Supplier<Pose2d> goal;
        private Localizer loc;
        private Command simplePathingCommand;
        private Command pathfindingCommand;
        private PathConstraints pathfindingConstraints;
        private NavToPoseConfig navConfig; 

        public NavCommand(Supplier<Pose2d> goal, NavToPoseConfig navConfig) {
            this.navConfig = navConfig;
            this.goal = goal;
            this.loc = superstructure.getDrivetrain().getLocalizer();
            pathfindingConstraints = new PathConstraints(navConfig.driveMaxVelocity(), navConfig.driveMaxAcceleration(), navConfig.thetaMaxVelocity(), navConfig.thetaMaxAcceleration());
            simplePathingCommand = new DriveToPose(superstructure.getDrivetrain(), superstructure.getTipProtectionSystem(), goal, navConfig);
        }

        @Override
        public void initialize() { 
            directPath = pathfinder.straightLineCollisionFree(loc.getPose(), goal.get());
            pathfindingCommand = AutoBuilder.pathfindToPose(goal.get(), pathfindingConstraints);
            scheduleAppropreateCommand(goal.get());
        }

        @Override
        public void execute() {
            var pose = loc.getPose();
            var goalPose = goal.get();
            if (!((pose.getTranslation().getDistance(goalPose.getTranslation()) < 2) && directPath)) {
                directPath = pathfinder.straightLineCollisionFree(pose, goalPose);
            } else {
                directPath = true;
            }
            scheduleAppropreateCommand(goalPose);
        }

        @Override
        public boolean isFinished() {
            return simplePathingCommand.isFinished() ;
        }

        @Override
        public void end(boolean interrupted) {
            if (simplePathingCommand.isScheduled()) {
                simplePathingCommand.cancel();
            }
            if (pathfindingCommand != null && pathfindingCommand.isScheduled()) {
                pathfindingCommand.cancel();
            }
        }


        private void scheduleAppropreateCommand(Pose2d goalPose) {
            if (directPath && !simplePathingCommand.isScheduled()) {
                simplePathingCommand.schedule();
            } else if (!directPath) {
                if (!pathfindingCommand.isScheduled()) {
                    pathfindingCommand = AutoBuilder.pathfindToPose(goalPose, pathfindingConstraints);
                    pathfindingCommand.schedule();
                }

                if (pathfinder.getRequestRealGoalPos().getDistance(goalPose.getTranslation()) > 1e-5) {
                    pathfindingCommand = AutoBuilder.pathfindToPose(goalPose, pathfindingConstraints);
                    pathfindingCommand.schedule();
                }
            }
        }
    }
}
