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

    public Commands scoreOnReefManual(ReefLevel reefLevel) {
        return 
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
        var offset = robot.relativeTo(goal);
        double yDistance = Math.abs(offset.getY());
        double xDistance = Math.abs(offset.getX());
        double shiftXT =
            MathUtil.clamp(
                (yDistance / (FieldConstants.kReefFaceLength.in(Meters) * 2)) + ((xDistance - 0.3) / (FieldConstants.kReefFaceLength.in(Meters) * 3)),
                0.0,
                1.0);
        double shiftYT =
            MathUtil.clamp(yDistance <= 0.2 ? 0.0 : offset.getX() /FieldConstants.kReefFaceLength.in(Meters), 0.0, 1.0);
        return goal.transformBy(
            new Transform2d(-shiftXT * 1.5,  Math.copySign(shiftYT * 1.5 * 0.8, offset.getY()), new Rotation2d()));
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


    }



    
}
