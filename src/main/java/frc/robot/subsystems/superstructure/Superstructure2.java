package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.SuperstructureConstants.*;

import java.nio.channels.Pipe;

import com.reduxrobotics.sensors.canandcolor.DigoutChannel.Index;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.ReefPosition.ReefLevel;
import frc.robot.commands.DriveToPose;
import frc.robot.ReefPosition;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Elevator.ElevatorSetpoint;
import frc.robot.subsystems.EndEffector.EndEffectorSetpoint;
import frc.robot.subsystems.EndEffector.EndEffectorSetpoint.EndEffectorFlipDirection;
import frc.robot.subsystems.EndEffector.RollerState;
import frc.robot.subsystems.EndEffector.WristSetpoint;
import frc.robot.subsystems.Indexer.IndexerState;
import frc.robot.subsystems.Intake.IntakeRollerState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.superstructure.Superstructure.MastState;

public class Superstructure2 {
    private EndEffector endEffector;
    private Elevator elevator;
    private Intake intake;
    private Indexer indexer;
    private Drivetrain drivetrain;

    public Command intakeFromGround() {
        return 
        setSuperstructureState(SuperstructureState.GROUND_INTAKE.withNeutralRollers(), true)
        .andThen(
            setSuperstructureState(SuperstructureState.GROUND_INTAKE, false),
            Commands.waitUntil(endEffector::hasCoral),
            setSuperstructureState(SuperstructureState.STOW, false)
        );
    }

    public Command intakeFromGroundForL1() {
        return
        setSuperstructureState(SuperstructureState.INTAKE_L1.withNeutralRollers(), true)
        .andThen(
            setSuperstructureState(SuperstructureState.INTAKE_L1, false),
            intake.waitForCoralGroundIntakeL1AndStopRollers(),
            setSuperstructureState(SuperstructureState.HOLD_L1, false)
        );
    }

    public Command extakeForL1FromIntake() {
        return
        setSuperstructureState(SuperstructureState.EXTAKE_L1.withNeutralRollers(), true)
        .andThen(
            setSuperstructureState(SuperstructureState.EXTAKE_L1, false),
            Commands.waitUntil(() -> !intake.hasCoral())
                .andThen(Commands.waitSeconds(0.1)),
            setSuperstructureState(SuperstructureState.HOLD_L1, false)
        );
    }

    public Commands scoreOnReefManual(ReefLevel reefLevel) {
        return setSuperstructureState(reefLevel)
    }

    public Command scoreOnReef(ReefPosition reefPosition) {
        return new DriveToPose(
            drivetrain, 
            () -> getReefAlignDriveTarget(
                drivetrain.getLocalizer().getPose(), 
                reefPosition.branch().getAlignPose(
                    DriverStation.getAlliance().orElse(Alliance.Blue)
                )
            )
        ).andThen();
    }





    private class SetSuperstructureStateCommand extends Command {
        private SuperstructureState superstructureState;
        private boolean waitForSuccess;
        private Command cmd;
        public SetSuperstructureStateCommand(SuperstructureState superstructureState, boolean waitForSuccess) {
            this.superstructureState = superstructureState;
            this.waitForSuccess = waitForSuccess;
            addRequirements(elevator, endEffector);
        }

        @Override
        public void initialize() {
           var mast = superstructureState.mastState;
           var intexer = superstructureState.intexerState;

           boolean waitForMast = waitForSuccess && superstructureState.considerMastSuccess;
           boolean waitForIntexer = waitForSuccess && superstructureState.considerIntexerSuccess;

           boolean willIntakeInterferWithEndEffectorMotionFuture = willIntakeInterferWithEndEffectorMotion(mast.endEffectorSetpoint, intexer.intakeState);
           boolean willIntakeInterferWithEndEffectorMotionNow = willIntakeInterferWithEndEffectorMotion(endEffector.getWristAngle(), mast.endEffectorSetpoint.wristSetpoint().getSetpoint(), intake.getPivotAngle());
            if (willIntakeInterferWithEndEffectorMotionFuture && willIntakeInterferWithEndEffectorMotionNow) {
                var intermedairySP = new MastState(
                    mast.elevatorSetpoint, 
                    new EndEffectorSetpoint(
                        new WristSetpoint(kMinAngleForEndEffectorInterferenceWithIntake.minus(Degrees.of(15))), 
                        mast.endEffectorSetpoint.rollerState()
                    )
                );

                cmd = setMastState(intermedairySP, false)
                .alongWith(
                    setIntexerState(IntexerState.CLEAR_END_EFFECTOR_MOTION_PATH, false)
                )
                .andThen(
                    Commands.waitUntil(() -> !willIntakeInterferWithEndEffectorMotion(mast.endEffectorSetpoint, IntexerState.CLEAR_END_EFFECTOR_MOTION_PATH.intakeState)),
                    setMastState(mast, waitForMast).alongWith(
                        Commands.waitUntil(() -> !willIntakeInterferWithEndEffectorMotion(mast.endEffectorSetpoint, intexer.intakeState))
                            .andThen(
                                setIntexerState(intexer, waitForIntexer)
                            )
                    )
                );
            } else if (willIntakeInterferWithEndEffectorMotionFuture && !willIntakeInterferWithEndEffectorMotionNow) {
                cmd = setMastState(mast, waitForMast)
                .alongWith(
                    Commands.waitUntil(() -> !willIntakeInterferWithEndEffectorMotion(mast.endEffectorSetpoint, intexer.intakeState))
                        .andThen(
                            setIntexerState(intexer, waitForIntexer)
                        )
                );
            } else if (!willIntakeInterferWithEndEffectorMotionFuture && willIntakeInterferWithEndEffectorMotionNow) {
                cmd = setIntexerState(intexer, waitForIntexer).alongWith(
                    Commands.waitUntil(() -> !willIntakeInterferWithEndEffectorMotion(mast.endEffectorSetpoint, intexer.intakeState))
                    .andThen(
                            setMastState(mast, waitForMast)
                        )
                );
            } else {
                cmd = setMastState(mast, waitForMast).alongWith(setIntexerState(intexer, waitForIntexer));
            }
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

            boolean isSimulation = Robot.isSimulation();

            if (!isSimulation)  {
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
            } else {
                cmd = Commands.waitSeconds(0.5);
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

    private Command setIntexerState(IntexerState intexerState, boolean waitForSuccess) {
        return indexer.setState(intexerState.indexerState).alongWith(intake.setState(intexerState.intakeState, waitForSuccess));
    }

    private Command setSuperstructureState(SuperstructureState superstructureState, boolean waitForSuccess) {
        return this.new SetSuperstructureStateCommand(superstructureState, waitForSuccess);

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

    public boolean doesIntakeInterferWithEndEffector() {
        return doesIntakeInterferWithEndEffector(endEffector.getWristAngle(), intake.getPivotAngle());
    }

    public boolean doesIntakeInterferWithEndEffector(Angle endEffectorAngle, Angle intakeAngle) {
        boolean a = intakeAngle.in(Degrees) >= kMinAngleForIntakeToInterfereWithEndEffector.in(Degrees);
        boolean b = endEffectorAngle.in(Degrees) < kMaxAngleForEndEffectorInterferenceWithIntake.in(Degrees);
        boolean c = endEffectorAngle.in(Degrees) > kMinAngleForEndEffectorInterferenceWithIntake.in(Degrees);
        return a && b && c;
    }

    public boolean willIntakeInterferWithEndEffectorMotion(EndEffectorSetpoint endEffectorSetpoint, IntakeState intakeState) {
        return willIntakeInterferWithEndEffectorMotion(endEffector.getWristAngle(), endEffectorSetpoint.wristSetpoint().getSetpoint(), intakeState.getPivotState().getAngle());
    }

    public boolean willIntakeInterferWithEndEffectorMotion(Angle endEffectorStartAngle, Angle endEffectorEndAngle, Angle intakeAngle) {
        if (intakeAngle.in(Degrees) >= kMinAngleForIntakeToInterfereWithEndEffector.in(Degrees)) {
            if (endEffectorStartAngle.in(Degrees) > kMaxAngleForEndEffectorInterferenceWithIntake.in(Degrees)) {
                if (endEffectorEndAngle.in(Degrees) < kMaxAngleForEndEffectorInterferenceWithIntake.in(Degrees)) {
                    return true;
                }
                return false;
            } else if (endEffectorStartAngle.in(Degrees) < kMinAngleForEndEffectorInterferenceWithIntake.in(Degrees)) {
                if (endEffectorEndAngle.in(Degrees) > kMinAngleForEndEffectorInterferenceWithIntake.in(Degrees)) {
                    return true;
                }
                return false;
            } else if (doesIntakeInterferWithEndEffector(endEffectorStartAngle, intakeAngle) || doesIntakeInterferWithEndEffector(endEffectorEndAngle, intakeAngle)) {
                return true;
            }
            return false;
        }
        return false;
    }

    private Pose2d getReefAlignDriveTarget(Pose2d robot, Pose2d goal) {
        // double xOffset =  robot.getX() - goal.getX();
        // double yOffset =  robot.getY() - goal.getY();

        return goal;
    }

    public static record IntexerState(
        IntakeState intakeState,
        IndexerState indexerState
    ) {
        public static final IntexerState CLEAR_END_EFFECTOR_MOTION_PATH = new IntexerState(IntakeState.CLEAR, IndexerState.NEUTRAL);
    }

    public static record MastState(
        ElevatorSetpoint elevatorSetpoint, 
        EndEffectorSetpoint endEffectorSetpoint
    ) {
    }
    

    public static record SuperstructureState(
        MastState mastState,
        boolean considerMastSuccess,
        IntexerState intexerState,
        boolean considerIntexerSuccess
    ) {

        public SuperstructureState(
            ElevatorSetpoint elevatorSetpoint, 
            EndEffectorSetpoint endEffectorSetpoint,
            boolean considerMastSuccess,
            IntakeState intakeState,
            IndexerState indexerState,
            boolean considerIntexerSuccess
        ) {
            this(
                new MastState(
                    elevatorSetpoint, 
                    endEffectorSetpoint
                ),
                considerMastSuccess,
                new IntexerState(
                    intakeState,
                    indexerState
                ),
                considerIntexerSuccess
            );
        }

        public SuperstructureState withNeutralRollers() {
            return new SuperstructureState(
                mastState.elevatorSetpoint,
                new EndEffectorSetpoint(
                    mastState.endEffectorSetpoint.wristSetpoint(),
                    EndEffector.RollerState.NEUTRAL
                ),
                considerMastSuccess, 
                new IntakeState(
                    IntakeRollerState.NEUTRAL, 
                    intexerState.intakeState.getPivotState()
                ),
                intexerState.indexerState, 
                considerIntexerSuccess);
        }

        public static final SuperstructureState STOW = new SuperstructureState(
            ElevatorSetpoint.STOW, 
            EndEffectorSetpoint.STOW, 
            true, 
            IntakeState.STOW, 
            IndexerState.NEUTRAL, 
            true
        );

        public static final SuperstructureState GROUND_INTAKE = new SuperstructureState(
            ElevatorSetpoint.HANDOFF, 
            EndEffectorSetpoint.CORAL_GROUND_HANDOFF_INTAKE, 
            true, 
            IntakeState.INTAKE, 
            IndexerState.INDEXING, 
            true
        );


        public static final SuperstructureState L4 = new SuperstructureState(
            ElevatorSetpoint.L4, 
            EndEffectorSetpoint.L4_EXTAKE_CORAL, 
            true, 
            IntakeState.STOW, 
            IndexerState.NEUTRAL, 
            false
        );

        public static final SuperstructureState L3 = new SuperstructureState(
            ElevatorSetpoint.L3, 
            EndEffectorSetpoint.L3_EXTAKE_CORAL, 
            true, 
            IntakeState.STOW, 
            IndexerState.NEUTRAL, 
            false
        );

        public static final SuperstructureState L2 = new SuperstructureState(
            ElevatorSetpoint.L2, 
            EndEffectorSetpoint.L2_EXTAKE_CORAL, 
            true, 
            IntakeState.STOW, 
            IndexerState.NEUTRAL, 
            false
        );

        public static final SuperstructureState INTAKE_L1 = new SuperstructureState(
            ElevatorSetpoint.STOW, 
            EndEffectorSetpoint.STOW, 
            false, 
            IntakeState.L1_INTAKE, 
            IndexerState.NEUTRAL, 
            true
        );

        public static final SuperstructureState HOLD_L1 = new SuperstructureState(
            ElevatorSetpoint.STOW, 
            EndEffectorSetpoint.STOW, 
            false, 
            IntakeState.L1_NEUTRAL, 
            IndexerState.NEUTRAL, 
            true
        );

        public static final SuperstructureState EXTAKE_L1 = new SuperstructureState(
            ElevatorSetpoint.STOW, 
            EndEffectorSetpoint.STOW, 
            false, 
            IntakeState.L1_EXTAKE, 
            IndexerState.NEUTRAL, 
            true
        );


    }



    
}
