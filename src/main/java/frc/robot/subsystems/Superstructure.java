// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.SuperstructureConstants.kMaxHeightForEndEffectorFullMotion;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.subsystems.Climb.ClimbState;
import frc.robot.subsystems.Elevator.ElevatorSetpoint;
import frc.robot.subsystems.EndEffector.EndEffectorSetpoint;
import frc.robot.subsystems.Indexer.IndexerState;
import frc.robot.subsystems.Intake.IntakeState;

/** Add your docs here. */
public class Superstructure extends SubsystemBase {
    private Elevator elevator;
    private Indexer indexer;
    private EndEffector endEffector;
    private Intake intake;
    private Climb climb;
    private Drivetrain drivetrain;

    public Superstructure() {
        
    }

    public Command climb() {
        return Commands.sequence(
            stowAll()
                .alongWith(climb.setState(ClimbState.NEUTRAL_WINCH_EXTENDED_FORK, false)),
            Commands.waitUntil(climb::isForkContacting),
            climb.setState(ClimbState.ROLLED_BACK, true)
        );
    }

    public Command stowAll() {
        return Commands.parallel(
            setEndEffectorExtensionFlipProtectedWithElevator(EndEffectorSetpoint.STOW, ElevatorSetpoint.STOW, true),
            intake.setState(IntakeState.STOW, true)
        );
    }

    private Command setEndEffectorSafe(EndEffectorSetpoint setpoint, boolean waitForSuccess) {
        return Commands.sequence(
            Commands.waitUntil(() -> !setpoint.wristSetpoint().requiresFlip())
                .raceWith(
                    Commands.waitUntil(this::canEndEffectorFlip)
                ),
            endEffector.set(setpoint, waitForSuccess)
        );
    }

    private Command setEndEffectorExtensionFlipProtectedWithElevator(EndEffectorSetpoint endEffectorSetpoint, ElevatorSetpoint elevatorSetpoint, boolean waitForSuccess) {
        return Commands.either(
                setEndEffectorSafe(endEffectorSetpoint, waitForSuccess)
                    .alongWith(elevator.set(elevatorSetpoint, waitForSuccess)), 

                elevator.set(ElevatorSetpoint.STOW, false).alongWith(
                    setEndEffectorSafe(endEffectorSetpoint, false)
                ).andThen(
                    Commands.waitUntil(this::isEndEffectorSafe),
                    elevator.set(elevatorSetpoint, waitForSuccess)).alongWith(
                        Commands.waitUntil(() -> endEffector.isAtSetpoint() || !waitForSuccess)
                    ), 

                () -> doesElevatorSetpointAllowEndEffectorFliping(elevatorSetpoint) || !isEndEffectorSafe());
    }

    public Command intakeCoralFromGround() {
        return Commands.sequence(
            Commands.parallel(
                elevator.set(ElevatorSetpoint.HANDOFF, true),
                intake.setState(IntakeState.EXTENDED_NEUTRAL, true),
                setEndEffectorSafe(EndEffectorSetpoint.STOW, true)
            ),
            intake.setState(IntakeState.INTAKE, false),
            Commands.waitUntil(intake::hasCoral),
            Commands.parallel(
                indexer.setState(IndexerState.INDEXING),
                endEffector.set(EndEffectorSetpoint.CORAL_GROUND_INTAKE_HANDOFF, false)
            ),
            Commands.waitUntil(endEffector::hasCoral),
            Commands.parallel(
                indexer.setState(IndexerState.NEUTRAL),
                endEffector.set(EndEffectorSetpoint.STOW, false),
                intake.setState(IntakeState.STOW, false)
            )
        ); 
    }

    public Command intakeAlgaeFromGround() {
        return Commands.sequence(
            Commands.parallel(
                elevator.set(ElevatorSetpoint.GROUND_ALGAE, true),
                setEndEffectorSafe(EndEffectorSetpoint.ALGAE_GROUND_INTAKE_NEUTRAL, true)
            ),
            endEffector.set(EndEffectorSetpoint.ALGAE_GROUND_INTAKE, false),
            Commands.waitUntil(endEffector::hasAlgae),
            endEffector.set(EndEffectorSetpoint.ALGAE_HOLD_GROUND, false)
        );
    }

    public Command intakeCoralFromHumanPlayer() {
        return Commands.sequence(
            setEndEffectorExtensionFlipProtectedWithElevator(EndEffectorSetpoint.INTAKE_HUMAN_PLAYER_NEUTRAL, ElevatorSetpoint.HUMAN_PLAYER, true),
            endEffector.set(EndEffectorSetpoint.INTAKE_HUMAN_PLAYER, false),
            Commands.waitUntil(endEffector::hasCoral),
            Commands.parallel(
                setEndEffectorSafe(EndEffectorSetpoint.STOW, false),
                elevator.set(ElevatorSetpoint.STOW, false)
            )
        );
    }

    

    public boolean doesElevatorSetpointAllowEndEffectorFliping(ElevatorSetpoint setpoint) {
        return setpoint.getHeight().in(Meters) < kMaxHeightForEndEffectorFullMotion.in(Meter);
    }

    public boolean canEndEffectorFlip() {
        return elevator.getHeight().in(Meter) < kMaxHeightForEndEffectorFullMotion.in(Meter);
    }

    public boolean isEndEffectorSafe() {
        return endEffector.getWristAngle().in(Degree) < EndEffectorConstants.kMaxElevatorRestrictedSafeAngle.in(Degree);
    }


    // public class SuperstructureState {

    //     public static final SuperstructureState STOW = new SuperstructureState(ElevatorSetpoint.STOW, EndEffectorSetpoint.STOW, IntakeState.STOW, IndexerState.NEUTRAL, null)

    //     private ElevatorSetpoint elevatorSetpoint;
    //     private EndEffectorSetpoint endEffectorSetpoint;
    //     private IntakeState intakeState;
    //     private IndexerState indexerState;
    //     private SuperstructureState(
    //         ElevatorSetpoint elevatorSetpoint, 
    //         EndEffectorSetpoint endEffectorSetpoint, 
    //         IntakeState intakeState, 
    //         IndexerState indexerState,
    //         BooleanSupplier isSuccessfull
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

    @Override
    public void periodic() {

    }
}
