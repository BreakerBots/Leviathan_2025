// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.SuperstructureConstants.kMaxHeightForEndEffectorFloorLimit;
import static frc.robot.Constants.SuperstructureConstants.kMaxHeightForEndEffectorFullMotion;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.ReefPosition.ReefLevel;
import frc.robot.HolonomicSlewRateLimiter;
import frc.robot.BreakerLib.driverstation.BreakerInputStream;
import frc.robot.BreakerLib.driverstation.BreakerInputStream2d;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerControllerRumbleType;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import frc.robot.BreakerLib.swerve.BreakerSwerveTeleopControl.TeleopControlConfig;
import frc.robot.BreakerLib.util.commands.TimedWaitUntilCommand;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Climb.ClimbState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorSetpoint;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorSetpoint;
import frc.robot.subsystems.EndEffector.EndEffectorWristLimits;
import frc.robot.subsystems.EndEffector.WristSetpoint;
import frc.robot.subsystems.EndEffector.EndEffectorSetpoint.EndEffectorFlipDirection;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Indexer.IndexerState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;

/** Add your docs here. */
public class Superstructure extends SubsystemBase {
    private Elevator elevator;
    private Indexer indexer;
    private EndEffector endEffector;
    private Intake intake;
    private Climb climb;
    private BreakerXboxController controller;
    private Drivetrain drivetrain;

    // private final TipProtectionSystem tipProtectionSystem;


    private HolonomicSlewRateLimiter limiter;

    public Superstructure(Drivetrain drivetrain, EndEffector endEffector, Elevator elevator, Indexer indexer, Intake intake, BreakerXboxController controller) {
        this.elevator = elevator;
        this.intake = intake;
        this.indexer = indexer;
        this.endEffector = endEffector;
        this.drivetrain = drivetrain;
        // this.climb = climb;
        this.controller = controller;
        // tipProtectionSystem = new TipProtectionSystem(elevator, drivetrain.getPigeon2());
    }



    private Command setMastState(MastState mastState, boolean waitForSuccess) {
        var elevatorSetpoint = mastState.elevatorSetpoint;
        var endEffectorSetpoint = mastState.endEffectorSetpoint;
        EndEffectorFlipDirection flipDirection = endEffectorSetpoint.wristSetpoint().getFlipDirectionFrom(endEffector.getWristAngle());

        boolean canEndEffectorFlip = canEndEffectorFlip();
        boolean doesSetpointAllowFlipping = doesElevatorSetpointAllowEndEffectorFliping(elevatorSetpoint);

        // boolean isElevatorSetpointFloorLimited = isElevatorSetpointFloorLimited(elevatorSetpoint);
        // boolean isFloorLimited = isEndEffectorFloorLimited();

        Command cmd = null;

        if ((canEndEffectorFlip && doesSetpointAllowFlipping) || flipDirection == EndEffectorFlipDirection.NONE) { // never flip restricted during travle or we dont flip

            cmd = endEffector.set(endEffectorSetpoint, waitForSuccess).alongWith(elevator.set(elevatorSetpoint, waitForSuccess));

        } else if ((canEndEffectorFlip && !doesSetpointAllowFlipping) && flipDirection == EndEffectorFlipDirection.FRONT_TO_BACK) {//We can flip now but wont be able to after moving the elevator

            cmd = endEffector.set(endEffectorSetpoint, waitForSuccess).alongWith(
                Commands.waitUntil(this::isEndEffectorSafe).andThen(
                    elevator.set(elevatorSetpoint, waitForSuccess)
                )
            );

        } else if ((!canEndEffectorFlip && doesSetpointAllowFlipping) && flipDirection == EndEffectorFlipDirection.BACK_TO_FRONT) {
            var intermedairySP = new EndEffectorSetpoint(new WristSetpoint(EndEffectorConstants.kMaxElevatorRestrictedSafeAngle.minus(Degrees.of(15))), endEffectorSetpoint.rollerState(), endEffectorSetpoint.kickerState());
            
            cmd = endEffector.set(intermedairySP, false)
            .andThen(
                Commands.waitUntil(this::isEndEffectorSafe),
                endEffector.set(endEffectorSetpoint, waitForSuccess)
            ).alongWith(
                elevator.set(elevatorSetpoint, waitForSuccess)
            );
        } else {
            cmd = Commands.print("INVALID SUPERSTRUCT SETPOINT COMMANDED");
        }

        return cmd;
    }

    public Command intakeCoralFromGround() {
        return 
        setMastState(MastState.GROUND_CORAL_HANDOFF_NEUTRAL, true)
            .alongWith(intake.setState(IntakeState.EXTENDED_NEUTRAL, true))
            .andThen(
                setMastState(MastState.GROUND_CORAL_HANDOFF_INTAKE,false)
                    .alongWith(
                        intake.setState(IntakeState.INTAKE, false),
                        indexer.setState(IndexerState.INDEXING)),
                Commands.waitUntil(endEffector::hasCoral),
                setMastState(MastState.PARTIAL_STOW, false)
                    .alongWith(
                        intake.setState(IntakeState.STOW, false),
                        indexer.setState(IndexerState.NEUTRAL))
            
        ); 
    }

    public Command intakeCoralFromHumanPlayer() {
        return setMastState(MastState.HUMAN_PLAYER_NEUTRAL, true).andThen(
            setMastState(MastState.HUMAN_PLAYER_INTAKE, false),
            Commands.waitUntil(endEffector::hasCoral),
            setMastState(MastState.PARTIAL_STOW, false)
        );
    }

    public Command scoreOnReefManual(ReefLevel level) {
        return setMastState(level.getNeutralMastState(), true).andThen(
            Commands.runOnce(() -> controller.setRumble(BreakerControllerRumbleType.MIXED, 0.1)),
            Commands.waitUntil(controller.getButtonB()),
            setMastState(level.getExtakeMastState(), false),
            new TimedWaitUntilCommand(() -> !endEffector.hasCoral(), 0.15),
            setMastState(MastState.STOW, false)
        );
    }


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
        public static final MastState L1_EXTAKE = new MastState(ElevatorSetpoint.L1, EndEffectorSetpoint.L2_EXTAKE_CORAL);
        public static final MastState L2_NEUTRAL = new MastState(ElevatorSetpoint.L2, EndEffectorSetpoint.L2_NEUTRAL);
        public static final MastState L2_EXTAKE = new MastState(ElevatorSetpoint.L2, EndEffectorSetpoint.L2_EXTAKE_CORAL);
        public static final MastState L3_NEUTRAL = new MastState(ElevatorSetpoint.L3, EndEffectorSetpoint.L3_NEUTRAL);
        public static final MastState L3_EXTAKE = new MastState(ElevatorSetpoint.L3, EndEffectorSetpoint.L3_EXTAKE_CORAL);
        public static final MastState L4_NEUTRAL = new MastState(ElevatorSetpoint.L4, EndEffectorSetpoint.L4_NEUTRAL);
        public static final MastState L4_EXTAKE = new MastState(ElevatorSetpoint.L4, EndEffectorSetpoint.L4_EXTAKE_CORAL); 
        public static final MastState HUMAN_PLAYER_NEUTRAL = new MastState(ElevatorSetpoint.HUMAN_PLAYER, EndEffectorSetpoint.INTAKE_HUMAN_PLAYER_NEUTRAL);
        public static final MastState HUMAN_PLAYER_INTAKE = new MastState(ElevatorSetpoint.HUMAN_PLAYER, EndEffectorSetpoint.INTAKE_HUMAN_PLAYER);
        public static final MastState GROUND_CORAL_HANDOFF_NEUTRAL = new MastState(ElevatorSetpoint.HANDOFF, EndEffectorSetpoint.CORAL_GROUND_HANDOFF_NEUTRAL);
        public static final MastState GROUND_CORAL_HANDOFF_INTAKE = new MastState(ElevatorSetpoint.HANDOFF, EndEffectorSetpoint.CORAL_GROUND_HANDOFF_INTAKE);
    }
    
 

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
        // tipProtectionSystem.setStreams(linear, rotational);
        return drivetrain.getTeleopControlCommand(linear.getX(), linear.getY(), rotational, config);
    }

    @Override
    public void periodic() {
        endEffectorSaftyCheck();
        // tipProtectionSystem.update();
    }
}
