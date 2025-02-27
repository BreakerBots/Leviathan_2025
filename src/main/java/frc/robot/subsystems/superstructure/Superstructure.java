// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.SuperstructureConstants.*;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;

import com.fasterxml.jackson.annotation.JsonInclude.Include;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoPilotConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.SimulationConstants;
import frc.robot.HolonomicSlewRateLimiter;
import frc.robot.ReefPosition;
import frc.robot.Robot;
import frc.robot.ReefPosition.ReefLevel;
import frc.robot.commands.AutoPilot;
import frc.robot.BreakerLib.driverstation.BreakerInputStream;
import frc.robot.BreakerLib.driverstation.BreakerInputStream2d;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerControllerRumbleType;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import frc.robot.BreakerLib.swerve.BreakerSwerveTeleopControl.TeleopControlConfig;
import frc.robot.BreakerLib.util.commands.TimedWaitUntilCommand;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorSetpoint;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorSetpoint;
import frc.robot.subsystems.EndEffector.EndEffectorSetpoint.EndEffectorFlipDirection;
import frc.robot.subsystems.EndEffector.EndEffectorWristLimits;
import frc.robot.subsystems.EndEffector.WristSetpoint;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Indexer.IndexerState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Climb.ClimbState;
import frc.robot.subsystems.Intake.IntakePivotState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.vision.ApriltagVision;

/** Add your docs here. */
public class Superstructure extends SubsystemBase {
    private Elevator elevator;
    private Indexer indexer;
    private EndEffector endEffector;
    private Intake intake;
    private Climb climb;
    private BreakerXboxController controller;
    private Drivetrain drivetrain;
    private ApriltagVision apriltagVision;
    private AutoPilot autoPilot;

    private final TipProtectionSystem tipProtectionSystem;


    private HolonomicSlewRateLimiter limiter;
    

    public Superstructure(Drivetrain drivetrain, EndEffector endEffector, Elevator elevator, Indexer indexer, Intake intake, Climb climb, ApriltagVision apriltagVision, AutoPilot autoPilot, BreakerXboxController controller) {
        this.elevator = elevator;
        this.intake = intake;
        this.indexer = indexer;
        this.endEffector = endEffector;
        this.drivetrain = drivetrain;
        this.climb = climb;
        this.controller = controller;
        this.apriltagVision = apriltagVision;
        this.autoPilot = autoPilot;
        tipProtectionSystem = new TipProtectionSystem(elevator, drivetrain.getPigeon2());
        // tipProtectionSystem = new TipProtectionSystem(elevator, drivetrain.getPigeon2());
    }


    private class SetMastStateCommand extends Command {
        private MastState mastState;
        private boolean waitForSuccess;
        private Command cmd;
        public SetMastStateCommand(MastState mastState, boolean waitForSuccess) {
            this.mastState = mastState;
            this.waitForSuccess = waitForSuccess;
            addRequirements(elevator, endEffector);
        }

        @Override
        public void initialize() {
            var elevatorSetpoint = mastState.elevatorSetpoint;
            var endEffectorSetpoint = mastState.endEffectorSetpoint;
            EndEffectorFlipDirection flipDirection = endEffectorSetpoint.wristSetpoint().getFlipDirectionFrom(endEffector.getWristAngle());

            boolean canEndEffectorFlip = canEndEffectorFlip();
            boolean doesSetpointAllowFlipping = doesElevatorSetpointAllowEndEffectorFliping(elevatorSetpoint);

            if ((canEndEffectorFlip && doesSetpointAllowFlipping) || flipDirection == EndEffectorFlipDirection.NONE) { // never flip restricted during travle or we dont flip
    
                cmd = endEffector.set(endEffectorSetpoint, waitForSuccess).alongWith(elevator.set(elevatorSetpoint, waitForSuccess));
    
            } else if ((canEndEffectorFlip && !doesSetpointAllowFlipping) && flipDirection == EndEffectorFlipDirection.FRONT_TO_BACK) {//We can flip now but wont be able to after moving the elevator
                cmd = endEffector.set(endEffectorSetpoint, waitForSuccess).alongWith(
                    Commands.waitUntil(() -> isEndEffectorSafe()).andThen(
                        elevator.set(elevatorSetpoint, waitForSuccess)
                    )
                );
    
            } else if ((!canEndEffectorFlip && doesSetpointAllowFlipping) && flipDirection == EndEffectorFlipDirection.BACK_TO_FRONT) {//we arnt able to flip now but will be able to after moving the elevator
                var intermedairySP = new EndEffectorSetpoint(new WristSetpoint(EndEffectorConstants.kMaxElevatorRestrictedSafeAngle.minus(Degrees.of(25))), endEffectorSetpoint.rollerState());
                cmd = endEffector.set(intermedairySP, false)
                .andThen(
                    Commands.waitUntil(() -> canEndEffectorFlip()),
                    endEffector.set(endEffectorSetpoint, waitForSuccess)
                ).alongWith(
                    elevator.set(elevatorSetpoint, waitForSuccess)
                );
            } else {
                cmd = Commands.print("INVALID SUPERSTRUCT SETPOINT COMMANDED");
            }
    
            cmd.initialize();
        }

        @Override
        public void execute() {
            cmd.execute();
        }

        @Override
        public void end(boolean interrupted) {
            cmd.end(interrupted);
        }

        @Override
        public boolean isFinished() {
            return cmd.isFinished();
        }
    }


    private Command setMastState(MastState mastState, boolean waitForSuccess) {
        return this.new SetMastStateCommand(mastState, waitForSuccess);
    }

    public Command intakeCoralFromGround() {
        return 
        setMastState(MastState.GROUND_CORAL_HANDOFF_PREP, true)
            .alongWith(intake.setState(IntakeState.EXTENDED_NEUTRAL, true))
            .andThen(
                setMastState(MastState.GROUND_CORAL_HANDOFF_INTAKE,false)
                    .alongWith(
                        intake.setState(IntakeState.INTAKE, false),
                        indexer.setState(IndexerState.INDEXING)),
                Commands.waitUntil(endEffector::hasCoral),
                setMastState(MastState.STOW, false)
                    .alongWith(
                        intake.setState(IntakeState.STOW, false),
                        indexer.setState(IndexerState.NEUTRAL))
            //TODO replace with proper subsys logic
        ).finallyDo(() -> intake.setState(IntakeState.EXTENDED_NEUTRAL, false).alongWith(indexer.setState(IndexerState.NEUTRAL
        )).schedule()); 
    }

    public Command reverseIntake() {
        return intake.setState(IntakeState.EXTAKE, true).alongWith(indexer.setState(IndexerState.REVERSE)).andThen(
            Commands.waitSeconds(1.0),
            intake.setState(IntakeState.EXTENDED_NEUTRAL, false).alongWith(indexer.setState(IndexerState.NEUTRAL), setMastState(MastState.STOW, false))
        );
    }

    public Command intakeCoralFromHumanPlayer() {
        if (Robot.isSimulation()) return Commands.sequence(
            Commands.print("Human Player"),
            Commands.waitTime(SimulationConstants.kWaitTime)
        );

        return setMastState(MastState.HUMAN_PLAYER_NEUTRAL, true).andThen(
            setMastState(MastState.HUMAN_PLAYER_INTAKE, false),
            Commands.waitUntil(endEffector::hasCoral),
            new WaitCommand(0.1),
            setMastState(MastState.PARTIAL_STOW, false)
        );
    }

    public Command scoreOnReefManual(ReefLevel level) {
        return setMastState(level.getNeutralMastState(), true).andThen(
            Commands.runOnce(() -> controller.setRumble(BreakerControllerRumbleType.MIXED, 0.8)),
            Commands.waitUntil(controller.getButtonA()),
            Commands.runOnce(() -> controller.setRumble(BreakerControllerRumbleType.MIXED, 0.0)),
            setMastState(level.getExtakeMastState(), false),
            new TimedWaitUntilCommand(() -> !endEffector.hasCoral(), 0.15),
            setMastState(MastState.STOW, false)
        );
    }

    public Command intakeAlgaeFromReef(boolean isHigh) {
        return setMastState(isHigh ? MastState.HIGH_REEF_ALGAE_NEUTRAL : MastState.LOW_REEF_ALGAE_NEUTRAL, true).andThen(
            setMastState(isHigh ? MastState.HIGH_REEF_ALGAE_INTAKE : MastState.LOW_REEF_ALGAE_INTAKE, false),
            Commands.waitUntil(controller.getButtonA()),
            setMastState(MastState.HOLD_ALGAE, false)
        );
    }

    public Command scoreInProcessor() {
        return setMastState(MastState.EXTAKE_ALGAE_PROCESSOR, false).andThen(
            Commands.waitSeconds(0.8),
            setMastState(MastState.PARTIAL_STOW, false)
        );
    }

    public Command scoreInBarge() {
        return setMastState(MastState.BARGE_NEUTRAL, true).andThen(
            Commands.waitUntil(controller.getButtonA()),
            setMastState(MastState.BARGE_EXTAKE, false),
            Commands.waitSeconds(0.5),
            setMastState(MastState.STOW, false)
        );
    }

    // public Command intakeAlgaeGround() {
    //     return intake.setState(IntakeState.ALGAE_NEUTRAL, true).andThen(
    //         intake.setState(IntakeState.INTAKE_ALGAE, false),
    //         Commands.waitUntil(controller.getButtonA()),
    //         intake.setState(IntakeState.HOLD_ALGAE_TRANS, true).withTimeout(3),
    //         intake.setState(IntakeState.HOLD_ALGAE, false)
    //     );
    // }

    // public Command scoreAlgaeProcessor() {
    //     return intake.setState(IntakeState.EXTAKE_ALGAE, false).andThen(
    //         Commands.waitSeconds(1.5),
    //         intake.setState(IntakeState.ALGAE_NEUTRAL, false)
    //     );
    // }

    public Command stowAll() {
        return setMastState(MastState.STOW, true).andThen(intake.setState(IntakeState.STOW, false), indexer.setState(IndexerState.NEUTRAL));
    }

    // note: when this function is implemented, make sure to stow too once it scores.
    public Command scoreOnReefAuton(ReefPosition position) {
        if (Robot.isSimulation()) {
            return Commands.deferredProxy(() -> autoPilot.navigateToPose(position.branch().getAllignPose(DriverStation.getAlliance().orElse(Alliance.Blue)), AutoPilotConstants.kDefaultNavToPoseConfig).andThen(Commands.waitTime(SimulationConstants.kWaitTime)));
        }
       return Commands.deferredProxy(
            () -> autoPilot.navigateToPose(position.branch().getAllignPose(DriverStation.getAlliance().orElse(Alliance.Blue)), AutoPilotConstants.kDefaultNavToPoseConfig))
            .alongWith(
                setMastState(MastState.PARTIAL_STOW, false)
            ).andThen(
                setMastState(position.level().getNeutralMastState(), true),
                setMastState(position.level().getExtakeMastState(), false),
                new TimedWaitUntilCommand(() -> !endEffector.hasCoral(), 0.15),
                setMastState(MastState.STOW, false)
            );
    }


    public Command scoreOnReef(ReefPosition position) {
        return Commands.runOnce(() -> controller.setRumble(BreakerControllerRumbleType.MIXED, 0.8))
            .andThen(
                Commands.waitUntil(controller.getButtonA()),
                Commands.runOnce(() -> controller.setRumble(BreakerControllerRumbleType.MIXED, 0.0)),
                Commands.deferredProxy(
            () -> autoPilot.navigateToPose(position.branch().getAllignPose(DriverStation.getAlliance().orElse(Alliance.Blue)), AutoPilotConstants.kDefaultNavToPoseConfig))
                .alongWith(
                    setMastState(MastState.PARTIAL_STOW, false)
                ))
            .andThen(
                scoreOnReefManual(position.level())
            );
    }

    // public Command climb()


    public boolean doesElevatorSetpointAllowEndEffectorFliping(ElevatorSetpoint setpoint) {
        return setpoint.getHeight().in(Meters) < kMaxHeightForEndEffectorFullMotion.in(Meter);
    }

    // private boolean isElevatorSetpointFloorLimited(ElevatorSetpoint setpoint) {
    //     return setpoint.getHeight().in(Meter) < kMaxHeightForEndEffectorFloorLimit.in(Meter);
    // }

    public boolean canEndEffectorFlip() {
        return elevator.getHeight().in(Meter) < kMaxHeightForEndEffectorFullMotion.in(Meter);
    }

    private boolean isEndEffectorFloorLimited() {
        return elevator.getHeight().in(Meter) < kMaxHeightForEndEffectorFloorLimit.in(Meter);
    }

    public boolean isEndEffectorSafe() {
        return endEffector.getWristAngle().in(Degree) < EndEffectorConstants.kMaxElevatorRestrictedSafeAngle.in(Degree);
    }

    private void endEffectorSaftyCheck() {
        boolean flip = canEndEffectorFlip();
        boolean floor = isEndEffectorFloorLimited();
        boolean safe = isEndEffectorSafe();
        if (flip) {
            if (floor) {
                endEffector.setWristLimits(EndEffectorWristLimits.FLOOR_RESTRICTED);
            } else {
                endEffector.setWristLimits(EndEffectorWristLimits.NORMAL);
            }
        } else {
            endEffector.setWristLimits(EndEffectorWristLimits.ELEVATOR_EXTENDED);
        }

        if ((elevator.getHeight().in(Meter) >= kMaxHeightForEndEffectorFullMotion.in(Meter)) && !safe) {
            elevator.forceStop(true);
        } else {
            elevator.forceStop(false);
        }
    }

    public Command climbOnDeepCage() {
        return climb.setState(ClimbState.EXTENDED, true).alongWith(
            setMastState(MastState.PARTIAL_STOW, true),
            intake.setState(IntakeState.EXTENDED_NEUTRAL, false)
        ).andThen(
            Commands.runOnce(() -> controller.setRumble(BreakerControllerRumbleType.RIGHT, 0.3)),
            Commands.waitUntil(controller.getButtonA()),
            Commands.runOnce(() -> controller.setRumble(BreakerControllerRumbleType.MIXED, 0.0)),
            climb.setState(ClimbState.CLIMBING, false)
        );
    }
 
    public Command stowClimb() {
        return climb.setState(ClimbState.STOW, true);
    }

    public boolean endEffectorHasCoral() {
        return endEffector.hasCoral();
    }

    // public Command setSuperstructureState(SuperstructureState state, SuperstructureStateSuccessType successType) {
    //     return setEndEffectorWithElevator(state.endEffectorSetpoint, state.elevatorSetpoint, successType.waitForElevatorAndEndEffector())
    //         .alongWith(
    //             intake.setState(state.intakeState, successType.waitForIntakeAndIndexer()),
    //             indexer.setState(state.indexerState)
    //             );
    // }

    // public static enum SuperstructureStateSuccessType {
    //     INSTANT,
    //     END_EFFECTOR_AND_ELEVATOR_ONLY,
    //     WAIT_FOR_ALL;

    //     public boolean waitForElevatorAndEndEffector() {
    //         return this == END_EFFECTOR_AND_ELEVATOR_ONLY || this == WAIT_FOR_ALL;
    //     }

    //     public boolean waitForIntakeAndIndexer() {
    //         return this == END_EFFECTOR_AND_ELEVATOR_ONLY || this == WAIT_FOR_ALL;
    //     }
    // }

    public static record MastState(
        ElevatorSetpoint elevatorSetpoint, 
        EndEffectorSetpoint endEffectorSetpoint
    ) {
        public static final MastState STOW = new MastState(ElevatorSetpoint.STOW, EndEffectorSetpoint.STOW);
        public static final MastState PARTIAL_STOW = new MastState(ElevatorSetpoint.STOW, EndEffectorSetpoint.EXTENDED_STOW);
        public static final MastState L1_NEUTRAL = new MastState(ElevatorSetpoint.L1, EndEffectorSetpoint.L1_NEUTRAL);
        public static final MastState L1_EXTAKE = new MastState(ElevatorSetpoint.L1, EndEffectorSetpoint.L1_EXTAKE_CORAL);
        public static final MastState L2_NEUTRAL = new MastState(ElevatorSetpoint.L2, EndEffectorSetpoint.L2_NEUTRAL);
        public static final MastState L2_EXTAKE = new MastState(ElevatorSetpoint.L2, EndEffectorSetpoint.L2_EXTAKE_CORAL);
        public static final MastState L3_NEUTRAL = new MastState(ElevatorSetpoint.L3, EndEffectorSetpoint.L3_NEUTRAL);
        public static final MastState L3_EXTAKE = new MastState(ElevatorSetpoint.L3, EndEffectorSetpoint.L3_EXTAKE_CORAL);
        public static final MastState L4_NEUTRAL = new MastState(ElevatorSetpoint.L4, EndEffectorSetpoint.L4_NEUTRAL);
        public static final MastState L4_EXTAKE = new MastState(ElevatorSetpoint.L4, EndEffectorSetpoint.L4_EXTAKE_CORAL); 
        public static final MastState HUMAN_PLAYER_NEUTRAL = new MastState(ElevatorSetpoint.HUMAN_PLAYER, EndEffectorSetpoint.INTAKE_HUMAN_PLAYER_NEUTRAL);
        public static final MastState HUMAN_PLAYER_INTAKE = new MastState(ElevatorSetpoint.HUMAN_PLAYER, EndEffectorSetpoint.INTAKE_HUMAN_PLAYER);
        public static final MastState GROUND_CORAL_HANDOFF_PREP = new MastState(ElevatorSetpoint.HANDOFF, EndEffectorSetpoint.STOW);
        public static final MastState GROUND_CORAL_HANDOFF_NEUTRAL = new MastState(ElevatorSetpoint.HANDOFF, EndEffectorSetpoint.CORAL_GROUND_HANDOFF_NEUTRAL);
        public static final MastState GROUND_CORAL_HANDOFF_INTAKE = new MastState(ElevatorSetpoint.HANDOFF, EndEffectorSetpoint.CORAL_GROUND_HANDOFF_INTAKE);

        public static final MastState HIGH_REEF_ALGAE_NEUTRAL = new MastState(ElevatorSetpoint.HIGH_REEF_ALGAE, EndEffectorSetpoint.REEF_ALGAE_HIGH_NEUTRAL);
        public static final MastState HIGH_REEF_ALGAE_INTAKE = new MastState(ElevatorSetpoint.HIGH_REEF_ALGAE, EndEffectorSetpoint.REEF_ALGAE_HIGH_INTAKE);
        public static final MastState LOW_REEF_ALGAE_NEUTRAL = new MastState(ElevatorSetpoint.HIGH_REEF_ALGAE, EndEffectorSetpoint.REEF_ALGAE_HIGH_NEUTRAL);
        public static final MastState LOW_REEF_ALGAE_INTAKE = new MastState(ElevatorSetpoint.HIGH_REEF_ALGAE, EndEffectorSetpoint.REEF_ALGAE_HIGH_INTAKE);
        public static final MastState HOLD_ALGAE = new MastState(ElevatorSetpoint.STOW, EndEffectorSetpoint.HOLD_ALGAE);

        public static final MastState BARGE_NEUTRAL = new MastState(ElevatorSetpoint.BARGE, EndEffectorSetpoint.BARGE_NEUTRAL);
        public static final MastState BARGE_EXTAKE = new MastState(ElevatorSetpoint.BARGE, EndEffectorSetpoint.BARGE_EXTAKE);

        public static final MastState EXTAKE_ALGAE_PROCESSOR = new MastState(ElevatorSetpoint.STOW, EndEffectorSetpoint.EXTAKE_ALGAE_PROCESSOR);


    }

    // public static enum IntexerState {
    //     STOW
    // }
    
    // public static record SuperstructureState(MastState mastState, ) {

    // }

    // public static class SuperstructureState {

    //     public static final SuperstructureState STOW = new SuperstructureState(ElevatorSetpoint.STOW, EndEffectorSetpoint.STOW, IntakeState.STOW, IndexerState.NEUTRAL);


    //     private ElevatorSetpoint elevatorSetpoint;
    //     private EndEffectorSetpoint endEffectorSetpoint;

    //     private IntakeState intakeState;
    //     private IndexerState indexerState;

    //     private SuperstructureState(
    //         ElevatorSetpoint elevatorSetpoint, 
    //         EndEffectorSetpoint endEffectorSetpoint, 
    //         IntakeState intakeState, 
    //         IndexerState indexerState
    //         ) {
    //         this.elevatorSetpoint = elevatorSetpoint;
    //         this.endEffectorSetpoint = endEffectorSetpoint;
    //         this.intakeState = intakeState;
    //         this.indexerState = indexerState;
    //     }

    //     public ElevatorSetpoint getElevatorSetpoint() {
    //         return elevatorSetpoint;
    //     }

    //     public EndEffectorSetpoint getEndEffectorSetpoint() {
    //         return endEffectorSetpoint;
    //     }

    //     public IndexerState getIndexerState() {
    //         return indexerState;
    //     }

    //     public IntakeState getIntakeState() {
    //         return intakeState;
    //     }

        
    // }

    public Command getDriveTeleopControlCommand(BreakerInputStream2d linear, BreakerInputStream rotational, TeleopControlConfig config) {
        var streams = tipProtectionSystem.setStreams(linear, rotational);
        return drivetrain.getTeleopControlCommand(streams.getFirst().getY(), streams.getFirst().getX(), streams.getSecond(), config);
    }

    public Drivetrain getDrivetrain() {
        return drivetrain;
    }

    @Override
    public void periodic() {
        endEffectorSaftyCheck();
        tipProtectionSystem.update();
        // tipProtectionSystem.update();
    }
}
