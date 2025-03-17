package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.SuperstructureConstants.*;

import java.lang.ModuleLayer.Controller;
import java.nio.channels.Pipe;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.config.PIDConstants;
import com.reduxrobotics.sensors.canandcolor.DigoutChannel.Index;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.ReefPosition.ReefLevel;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.DriveToPose.NavToPoseConfig;
import frc.robot.ReefPosition;
import frc.robot.Robot;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import frc.robot.BreakerLib.util.commands.RumbleCommand;
import frc.robot.BreakerLib.util.logging.BreakerLog;
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
    private BreakerXboxController controller;

    public Superstructure2(EndEffector endEffector, Elevator elevator, Intake intake, Indexer indexer, Drivetrain drivetrain, BreakerXboxController controller) {
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.intake = intake;
        this.endEffector = endEffector;
        this.indexer = indexer;
        this.controller = controller;
    }

    public Command stowAll() {
        return setSuperstructureState(SuperstructureState.STOW, false);
    }

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

    public Command scoreOnReefManual(ReefLevel reefLevel) {
        return setSuperstructureState(reefLevel.getExtakeSuperstructureState().withNeutralRollers(), true)
        .andThen(
            new RumbleCommand(controller.getBaseHID(), RumbleType.kBothRumble, 0.7)
                .until(() -> controller.getButtonA().getAsBoolean()),
            setSuperstructureState(reefLevel.getExtakeSuperstructureState(), false),
            Commands.waitUntil(() -> !endEffector.hasCoral()),
            Commands.waitSeconds(0.1),
            stowAll()
        );
    }

    public Command waitForDriverConfirmation() {
        return new RumbleCommand(controller.getBaseHID(), RumbleType.kBothRumble, 0.7)
        .until(() -> controller.getButtonA().getAsBoolean());
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
        ).withTimeout(8).deadlineFor(
            waitAndExtendMastToScore(reefPosition)
        ).andThen(
            setSuperstructureState(reefPosition.level().getExtakeSuperstructureState().withNeutralRollers(), true),
            new RumbleCommand(controller.getBaseHID(), RumbleType.kBothRumble, 0.7)
                .until(() -> controller.getButtonA().getAsBoolean()),
            setSuperstructureState(reefPosition.level().getExtakeSuperstructureState(), false),
            waitForDriverConfirmation().onlyIf(() -> !DriverStation.isAutonomous()),
            Commands.waitUntil(() -> !endEffector.hasCoral()),
            Commands.waitSeconds(0.1),
            new DriveToPose(
                drivetrain, 
                () -> {
                    Pose2d scoreGoal = reefPosition.branch().getAlignPose(
                        DriverStation.getAlliance().orElse(Alliance.Blue));
                    Translation2d offset = new Translation2d(0.4, 0);
                    offset = offset.rotateBy(scoreGoal.getRotation());
                    scoreGoal.plus(new Transform2d(offset, Rotation2d.kZero));
                    return scoreGoal;
                },
                new NavToPoseConfig(
                    Meters.of(0.05), 
                    Degrees.of(5.0), 
                    MetersPerSecond.of(3.0), 
                    DegreesPerSecond.of(360), 
                    MetersPerSecondPerSecond.of(3.0), 
                    RadiansPerSecondPerSecond.of(8), 
                    Meters.of(0.05), 
                    Meters.of(0.1), 
                    new PIDConstants(0.8, 0.0, 0.0), 
                    new PIDConstants(4, 0.0, 0.0)
                )
            ),
            setSuperstructureState(SuperstructureState.STOW, false)
        );
    }

    private Command waitAndExtendMastToScore(ReefPosition reefPosition) {
        BooleanSupplier canExtend = () -> {
            Pose2d robotPose = drivetrain.getLocalizer().getPose();
            Pose2d goalPose = reefPosition.branch().getAlignPose(
                DriverStation.getAlliance().orElse(Alliance.Blue));
            
            double distance = robotPose.getTranslation().getDistance(goalPose.getTranslation());

            return distance <= 1.5;
        };
        return Commands.waitUntil(canExtend).andThen(setSuperstructureState(reefPosition.level().getExtakeSuperstructureState().withNeutralRollers(), false));
    }







    private class SetSuperstructureStateCommand extends Command {
        private SuperstructureState superstructureState;
        private boolean waitForSuccess;
        private Command cmd;
        public SetSuperstructureStateCommand(SuperstructureState superstructureState, boolean waitForSuccess) {
            this.superstructureState = superstructureState;
            this.waitForSuccess = waitForSuccess;
            addRequirements(elevator, endEffector, intake, indexer);
        }

        @Override
        public void initialize() {
           var mast = superstructureState.mastState;
           var intexer = superstructureState.intexerState;

           boolean waitForMast = waitForSuccess && superstructureState.considerMastSuccess;
           boolean waitForIntexer = waitForSuccess && superstructureState.considerIntexerSuccess;

           boolean willIntakeInterferWithEndEffectorMotionFuture = willIntakeInterferWithEndEffectorMotion(mast.endEffectorSetpoint, intexer.intakeState);
           boolean willIntakeInterferWithEndEffectorMotionNow = willIntakeInterferWithEndEffectorMotion(endEffector.getWristAngle(), mast.endEffectorSetpoint.wristSetpoint().getSetpoint(), intake.getPivotAngle());
           if (RobotBase.isReal()) {
            if (willIntakeInterferWithEndEffectorMotionFuture && willIntakeInterferWithEndEffectorMotionNow) {
                BreakerLog.log("dsdfsd", 1);
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
                BreakerLog.log("dsdfsd", 2);
                cmd = setMastState(mast, waitForMast)
                .alongWith(
                    Commands.waitUntil(() -> !willIntakeInterferWithEndEffectorMotion(mast.endEffectorSetpoint, intexer.intakeState))
                        .andThen(
                            setIntexerState(intexer, waitForIntexer)
                        )
                );
            } else if (!willIntakeInterferWithEndEffectorMotionFuture && willIntakeInterferWithEndEffectorMotionNow) {
                BreakerLog.log("dsdfsd", 3);
                cmd = setIntexerState(intexer, waitForIntexer).alongWith(
                    Commands.waitUntil(() -> !willIntakeInterferWithEndEffectorMotion(mast.endEffectorSetpoint, intexer.intakeState))
                    .andThen(
                            setMastState(mast, waitForMast)
                        )
                );
            } else {
                BreakerLog.log("dsdfsd", 4);
                cmd = setMastState(mast, waitForMast).alongWith(setIntexerState(intexer, waitForIntexer));
            }
            } else {
                cmd = Commands.waitSeconds(0.7);
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
