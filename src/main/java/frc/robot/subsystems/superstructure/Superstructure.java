package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.SuperstructureConstants.*;

import java.lang.ModuleLayer.Controller;
import java.nio.channels.Pipe;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentricFacingAngle;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import com.reduxrobotics.sensors.canandcolor.DigoutChannel.Index;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.AllianceFlipUtil;
import frc.robot.CagePosition;
import frc.robot.CoralHumanPlayerStation;
import frc.robot.ReefPosition.ReefBranch;
import frc.robot.ReefPosition.ReefLevel;
import frc.robot.commands.HeadingSnap;
import frc.robot.commands.IntakeAssist;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.DriveToPose.NavToPoseConfig;
import frc.robot.ReefPosition;
import frc.robot.Robot;
import frc.robot.BreakerLib.driverstation.BreakerInputStream;
import frc.robot.BreakerLib.driverstation.BreakerInputStream2d;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerControllerRumbleType;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import frc.robot.BreakerLib.swerve.BreakerSwerveTeleopControl.TeleopControlConfig;
import frc.robot.BreakerLib.util.commands.RumbleCommand;
import frc.robot.BreakerLib.util.commands.TimedWaitUntilCommand;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.subsystems.Climb.ClimbState;
import frc.robot.subsystems.Climb;
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
import frc.robot.subsystems.LED.Animations;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.vision.Localization;

public class Superstructure {
    private EndEffector endEffector;
    private Elevator elevator;
    private Intake intake;
    private Indexer indexer;
    private Climb climb;
    private Drivetrain drivetrain;
    private Localization localization;
    private BreakerXboxController controller;
    private TipProtectionSystem tipProtectionSystem;
    private LED led;

    public Superstructure(EndEffector endEffector, Elevator elevator, Intake intake, Indexer indexer, Climb climb, Drivetrain drivetrain, Localization localization, BreakerXboxController controller) {
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.intake = intake;
        this.endEffector = endEffector;
        this.indexer = indexer;
        this.climb = climb;
        this.localization = localization;
        this.controller = controller;
        tipProtectionSystem = new TipProtectionSystem(elevator, drivetrain.getPigeon2());
        led = new LED();
        led.setFunctional(this::defaultLedSupplier);
        // autoPilot2 = new AutoPilot2(this);
    }

    private ControlRequest defaultLedSupplier() {
        if (RobotState.isDisabled()) {
            return Animations.kDisabled;
        } else if (RobotState.isTeleop()) {
            if (endEffectorHasCoral()) {
                return Animations.kHasCoralTeleop;
            }
            return Animations.kEnabledTeleop;
        } else if (RobotState.isAutonomous()) {
            if (endEffectorHasCoral()) {
                return Animations.kHasCoralAuto;
            }
            return Animations.kEnabledAuto;
        }
        return Animations.kDisabled;
    }

    private void yealdLed() {
        led.setFunctional(this::defaultLedSupplier);
    }

    public static Alliance getAllianceSafe() {
        return DriverStation.getAlliance().orElse(Alliance.Blue);
    }

    public Drivetrain getDrivetrain() {
        return drivetrain;
    }

    public Localization getLocalization() {
        return localization;
    }

    public Command intakeAssist() {
        RobotCentricFacingAngle driveMoveReq = new RobotCentricFacingAngle().withDriveRequestType(DriveRequestType.Velocity).withHeadingPID(3.0, 0, 0);
        return new IntakeAssist(this).andThen(
            Commands.run(() -> drivetrain.setControl(driveMoveReq.withTargetDirection(localization.getPose().getRotation()).withVelocityX(MetersPerSecond.of(1.0))), drivetrain).onlyWhile(() -> !endEffectorHasCoral())
        ).onlyIf(() -> !endEffectorHasCoral());
    }

    public Command climbOnDeepCageManual() {
        return climb.setState(ClimbState.EXTENDED, true).alongWith(
            setSuperstructureState(SuperstructureState.CLIMB, false),
            led.set(Animations.kClimbExtended)
        ).andThen(
            led.set(Animations.kAwaitHuman),
            waitForDriverConfirmation(),
            led.set(Animations.kClimbing),
            climb.setState(ClimbState.CLIMBING, false)
        );
    }

    public Command snapHeadingToClosestReefFace(BreakerInputStream2d linearInputStream, BreakerInputStream rotaionalInputStream) {
        Supplier<Rotation2d> goalSup = () -> {
            var normalRot = ReefPosition.ReefBranch.getClosest(
                drivetrain.getLocalizer().getPose(), 
                getAllianceSafe())
            .getAlignPose(
                getAllianceSafe()
            ).getRotation();

            if (!endEffector.hasCoral()) {
                return normalRot.plus(Rotation2d.k180deg);
            }
            return normalRot;
        };
        return new HeadingSnap(goalSup, drivetrain, linearInputStream).asProxy().raceWith(Commands.waitSeconds(0.75).andThen(Commands.waitUntil(() -> Math.abs(rotaionalInputStream.get()) > 0.75)));
    }

    public Command climbOnDeepCage() {
        final var ally = getAllianceSafe();
        final Supplier<Pose2d> closest = () -> CagePosition.getClosest(drivetrain.getLocalizer().getPose(), ally).getClimbPose(ally);
        return climb.setState(ClimbState.EXTENDED, true).alongWith(
            setSuperstructureState(SuperstructureState.CLIMB, false),
            alignToClosestCage().asProxy(),
            led.set(Animations.kClimbExtended)
        ).andThen(
            led.set(Animations.kAwaitHuman),
            waitForDriverConfirmation(),
            led.set(Animations.kClimbExtended),
            new DriveToPose(drivetrain, tipProtectionSystem, closest).asProxy(),
            led.set(Animations.kAwaitHuman),
            waitForDriverConfirmation(),
            led.set(Animations.kClimbing),
            climb.setState(ClimbState.CLIMBING, false)
        );
    }
 
    public Command stowClimb() {
        return climb.setState(ClimbState.STOW, true).andThen(this::yealdLed);
    }

    public Command stowAll() {
        return setSuperstructureState(SuperstructureState.STOW, false);
    }

    public Command reverseIntake() {
        return 
        setSuperstructureState(SuperstructureState.INTEXER_EJECT.withNeutralRollers(), true)
        .andThen(
            led.set(Animations.kReverseIntake),
            setSuperstructureState(SuperstructureState.INTEXER_EJECT, false),
            Commands.waitSeconds(3),
            setSuperstructureState(SuperstructureState.STOW, false)
        ).finallyDo(this::yealdLed);
        
    }

    public Command intakeFromGroundAuton() {
        return led.set(Animations.kIntakeing).andThen(
            setSuperstructureState(SuperstructureState.GROUND_INTAKE, false),
            Commands.waitUntil(endEffector::hasCoral),
            Commands.waitSeconds(0.1),
            setSuperstructureState(SuperstructureState.STOW, false)
        ).finallyDo(this::yealdLed);
    }

    public Command intakeFromGround() {
        return 
        setSuperstructureState(SuperstructureState.GROUND_INTAKE.withNeutralRollers(), true)
        .andThen(
            led.set(Animations.kIntakeing),
            setSuperstructureState(SuperstructureState.GROUND_INTAKE, false),
            Commands.waitUntil(endEffector::hasCoral),
            Commands.waitSeconds(0.1),
            setSuperstructureState(SuperstructureState.STOW, false)
        ).finallyDo(this::yealdLed);
    }

    public Command intakeFromHumanPlayerManual() {
        return
        setSuperstructureState(SuperstructureState.HP_INTAKE.withNeutralRollers(), true)
        .andThen(
            led.set(Animations.kIntakeingHP),
            setSuperstructureState(SuperstructureState.HP_INTAKE, false),
            Commands.waitUntil(endEffector::hasCoral),
            Commands.waitSeconds(0.1),
            setSuperstructureState(SuperstructureState.STOW, false)
        ).finallyDo(this::yealdLed);

    }

    private Command navigateToHumanPlayer(CoralHumanPlayerStation pos) {
        return Commands.defer(() -> navigateToHumanPlayer(pos, getAllianceSafe()), Set.of(drivetrain));
    }

    private Command navigateToHumanPlayer(CoralHumanPlayerStation pos, Alliance alliance){
        var alignPose = pos.getAlignPose(alliance);
        BooleanSupplier shouldUsePathfinder = () -> {
           return alignPose.getTranslation().getDistance(drivetrain.getLocalizer().getPose().getTranslation()) > 2;
        };
        return navigateToPose(alignPose, pos.getOffsetPathfindingPose(alliance), shouldUsePathfinder, shouldUsePathfinder, new NavToPoseConfig());
    }

    public Command intakeCoralFromHumanPlayer(CoralHumanPlayerStation pos) {
        return navigateToHumanPlayer(pos).asProxy().alongWith(led.set(Animations.kAutoAllignHP))
        .andThen(
            setSuperstructureState(SuperstructureState.HP_INTAKE.withNeutralRollers(), true),
            led.set(Animations.kIntakeingHP),
            setSuperstructureState(SuperstructureState.HP_INTAKE, false),
            Commands.waitUntil(endEffector::hasCoral),
            Commands.waitSeconds(0.1),
            setSuperstructureState(SuperstructureState.STOW, false)
        ).finallyDo(this::yealdLed);
    }

    public Command intakeCoralFromHumanPlayerManual() {
        return 
        Commands.sequence(
            setSuperstructureState(SuperstructureState.HP_INTAKE.withNeutralRollers(), true),
            led.set(Animations.kIntakeingHP),
            setSuperstructureState(SuperstructureState.HP_INTAKE, false),
            Commands.waitUntil(endEffector::hasCoral),
            Commands.waitSeconds(0.1),
            setSuperstructureState(SuperstructureState.STOW, false)
        ).finallyDo(this::yealdLed);
    }

    private Command alignToCage(CagePosition cagePosition) {
        final Supplier<Pose2d> pos = () -> cagePosition.getAlignPose(getAllianceSafe());
        return new DriveToPose(drivetrain, tipProtectionSystem, pos);
    }

    private Command alignToCage(Supplier<CagePosition> cagePosition) {
        return new DriveToPose(drivetrain, tipProtectionSystem, () -> cagePosition.get().getAlignPose(getAllianceSafe()));
    }

    public Command alignToClosestCage() {
        return alignToCage(
            () -> CagePosition.getClosest(drivetrain.getLocalizer().getPose(), 
                getAllianceSafe()));
    }

    public Command removeAlgae(boolean isHigh) {
        var state = (isHigh ? SuperstructureState.REMOVE_ALGAE_HIGH : SuperstructureState.REMOVE_ALGAE_LOW);
        return setSuperstructureState(state.withNeutralRollers(), true).andThen(
            setSuperstructureState(state, false),
            waitForDriverConfirmation(),
            stowAll()
        );
    }

    public Command manualIntakeFromGroundForL1() {
        return
        setSuperstructureState(SuperstructureState.INTAKE_L1.withNeutralRollers(), true)
        .andThen(
            led.set(Animations.kIntakeingL1),
            setSuperstructureState(SuperstructureState.INTAKE_L1, false),
            intake.waitForCoralGroundIntakeL1AndStopRollers(),
            led.set(Animations.kHasCoralTeleop),
            setSuperstructureState(SuperstructureState.HOLD_L1, false)
        );
    }

    public Command intakeFromGroundForL1(BreakerInputStream2d linearInputStream, BreakerInputStream rotaionalInputStream) {
        return manualIntakeFromGroundForL1().asProxy().andThen(snapHeadingToClosestReefFace(linearInputStream, rotaionalInputStream).asProxy());
    }

    public Command extakeForL1FromIntake() {
        return
        setSuperstructureState(SuperstructureState.EXTAKE_L1.withNeutralRollers(), true)
        .andThen(
            led.set(Animations.kScoreing),
            setSuperstructureState(SuperstructureState.EXTAKE_L1, false),
            Commands.waitUntil(() -> !intake.hasCoral())
                .andThen(Commands.waitSeconds(0.35)),
            setSuperstructureState(SuperstructureState.HOLD_L1, false)
        ).finallyDo(this::yealdLed);
    }

    public Command scoreOnReefManual(ReefLevel reefLevel) {
        return setSuperstructureState(reefLevel.getExtakeSuperstructureState().withNeutralRollers(), true)
        .andThen(
            led.set(Animations.kAwaitHuman),
            new RumbleCommand(controller.getBaseHID(), RumbleType.kBothRumble, 0.7)
                .until(() -> controller.getButtonA().getAsBoolean()),
            led.set(Animations.kScoreing),
            setSuperstructureState(reefLevel.getExtakeSuperstructureState(), false),
            Commands.waitUntil(() -> !endEffector.hasCoral()),
            Commands.waitSeconds(0.1),
            stowAll()
        ).finallyDo(this::yealdLed);
    }

    public Command waitForDriverConfirmation() {
        return new RumbleCommand(controller.getBaseHID(), RumbleType.kBothRumble, 0.7)
        .until(() -> controller.getButtonA().getAsBoolean());
    }

    public Command scoreOnReefAuton(ReefPosition reefPosition) {
        return scoreOnReefMaster(reefPosition, false);
    }

    public Command scoreOnReef(ReefPosition reefPosition) {
        return scoreOnReefMaster(reefPosition, true);
    }

    private boolean linearVelNearZero() {
        ChassisSpeeds speeds = drivetrain.getChassisSpeeds();
        return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) < 0.005;
    }

    private Command scoreOnReefMaster(ReefPosition reefPosition, boolean proxyDrive) {

        Command allignCmd = navigateToReef(reefPosition.branch()).raceWith(new TimedWaitUntilCommand(this::linearVelNearZero, 1.0));

        if (proxyDrive) {
            allignCmd = allignCmd.asProxy();
        }

        return 
            allignCmd.withTimeout(10).until(() -> controller.getButtonA().getAsBoolean()).deadlineFor(
            Commands.run(
                () -> {
                    var tgt = reefPosition.branch().getAlignPose(
                        getAllianceSafe()
                    );

                    double dist = tgt.getTranslation().getDistance(drivetrain.getLocalizer().getPose().getTranslation());

                    if (dist < 3) {
                        localization.useTrigApriltagStragey(true);
                    }
                }
            ),
            led.set(Animations.kAutoAllignScoreing),
            waitAndExtendMastToScore(reefPosition)
        ).andThen(
            setSuperstructureState(reefPosition.level().getExtakeSuperstructureState().withNeutralRollers(), true),
            led.set(Animations.kAwaitHuman),
            waitForDriverConfirmation().onlyIf(() -> !DriverStation.isAutonomous()),
            led.set(Animations.kScoreing),
            setSuperstructureState(reefPosition.level().getExtakeSuperstructureState(), false),
            Commands.waitUntil(() -> !endEffector.hasCoral()).withTimeout(1.0),
            Commands.waitSeconds(0.1),
            setSuperstructureState(SuperstructureState.STOW, false)
        ).finallyDo(()->localization.useTrigApriltagStragey(false)).finallyDo(this::yealdLed);
    }

    private Command navigateToReef(ReefBranch branch) {
        return Commands.defer(() -> navigateToReef(branch, getAllianceSafe(), new NavToPoseConfig()), Set.of(drivetrain));
    }
    
    private Command navigateToReef(ReefBranch branch, Alliance alliance, NavToPoseConfig navConfig) {
        Pose2d allignGoal = branch.getAlignPose(alliance);
        Pose2d pathfindGoal = branch.getOffsetPathfindingPose(alliance);
        BooleanSupplier keepPathfiderActive = () -> {
            var botPose = drivetrain.getLocalizer().getPose();
            var closestBranch = ReefBranch.getClosest(botPose, alliance);
            boolean isClosestBranchTheTarget = closestBranch.equals(branch);
            boolean isTargetInAngleRange = Math.abs(allignGoal.getTranslation().minus(botPose.getTranslation()).getAngle().getDegrees()) < 15;
            return !(isClosestBranchTheTarget && isTargetInAngleRange);
        };
        BooleanSupplier shouldUsePathfinder = () -> {
            var closestBranch = ReefBranch.getClosest(drivetrain.getLocalizer().getPose(), alliance);
            return closestBranch.getBlueReefFaceApriltagID() != branch.getBlueReefFaceApriltagID();
        };
        return navigateToPose(allignGoal, pathfindGoal, shouldUsePathfinder, keepPathfiderActive, navConfig);
    }

    private Command navigateToPose(Pose2d allignGoal, Pose2d pathfindGoal, BooleanSupplier shouldUsePathfinder, BooleanSupplier shouldKeepPathfiderActive, NavToPoseConfig navConfig) {
        var pathfindingConstraints = new PathConstraints(navConfig.driveMaxVelocity(), navConfig.driveMaxAcceleration(), navConfig.thetaMaxVelocity(), navConfig.thetaMaxAcceleration());
        return AutoBuilder.pathfindToPose(pathfindGoal, pathfindingConstraints).onlyIf(shouldUsePathfinder).onlyWhile(shouldKeepPathfiderActive)
        .andThen(new DriveToPose(drivetrain, tipProtectionSystem, ()-> allignGoal, navConfig));
    }

    private Command waitAndExtendMastToScore(ReefPosition reefPosition) {
        BooleanSupplier canExtend = () -> {
            Pose2d robotPose = drivetrain.getLocalizer().getPose();
            Pose2d goalPose = reefPosition.branch().getAlignPose(
                getAllianceSafe());
            
            double distance = robotPose.getTranslation().getDistance(goalPose.getTranslation());

            return distance <= 2.5;
        };
        return Commands.waitUntil(canExtend).andThen(setSuperstructureState(reefPosition.level().getExtakeSuperstructureState().withNeutralRollers(), false));
    }

    public boolean endEffectorHasCoral() {
        return endEffector.hasCoral();
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

           boolean endEffectorHasCoral = endEffector.hasCoral();

           boolean willIntakeInterferWithEndEffectorMotionFuture = endEffectorHasCoral && willIntakeInterferWithEndEffectorMotion(mast.endEffectorSetpoint, intexer.intakeState);
           boolean willIntakeInterferWithEndEffectorMotionNow = endEffectorHasCoral && willIntakeInterferWithEndEffectorMotion(endEffector.getWristAngle(), mast.endEffectorSetpoint.wristSetpoint().getSetpoint(), intake.getPivotAngle());
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
                    BreakerLog.log("gsdjgs", 0);
        
                    cmd = endEffector.set(endEffectorSetpoint, waitForSuccess).alongWith(elevator.set(elevatorSetpoint, waitForSuccess));
        
                } else if ((canEndEffectorFlip && !doesSetpointAllowFlipping) && flipDirection == EndEffectorFlipDirection.FRONT_TO_BACK) {//We can flip now but wont be able to after moving the elevator
                    BreakerLog.log("gsdjgs", 1);
                    cmd = endEffector.set(endEffectorSetpoint, waitForSuccess).alongWith(
                        Commands.waitUntil(() -> isEndEffectorSafe())
                        .andThen(
                            elevator.set(elevatorSetpoint, waitForSuccess)
                        )
                    );
        
                } else if ((!canEndEffectorFlip && doesSetpointAllowFlipping) && flipDirection == EndEffectorFlipDirection.BACK_TO_FRONT) {//we arnt able to flip now but will be able to after moving the elevator
                    BreakerLog.log("gsdjgs", 2);
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

    public Command setSuperstructureState(SuperstructureState superstructureState, boolean waitForSuccess) {
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
        boolean b = endEffectorAngle.in(Degrees) <= kMaxAngleForEndEffectorInterferenceWithIntake.in(Degrees);
        boolean c = endEffectorAngle.in(Degrees) >= kMinAngleForEndEffectorInterferenceWithIntake.in(Degrees);
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

    public Command getDriveTeleopControlCommand(BreakerInputStream2d linear, BreakerInputStream rotational, TeleopControlConfig config) {
        var streams = tipProtectionSystem.setStreams(linear, rotational);
        return drivetrain.getTeleopControlCommand(streams.getFirst().getY(), streams.getFirst().getX(), streams.getSecond(), config);
    }

    public TipProtectionSystem getTipProtectionSystem() {
        return tipProtectionSystem;
    }

    private Pose2d getReefAlignDriveTarget(Pose2d robot, Pose2d goal) {
    //     Rotation2d angleToGoal =
    //     robot
    //         .getTranslation()
    //         .minus(AllianceFlipUtil.apply(FieldConstants.Reef.center))
    //         .getAngle()
    //         .minus(goal.getTranslation().minus(AllianceFlipUtil.apply(FieldConstants.Reef.center)).getAngle());
    // var offset = robot.relativeTo(goal);
    // double yDistance = Math.abs(offset.getY());
    // double xDistance = Math.abs(offset.getX());
    // double shiftXT =
    //     MathUtil.clamp(
    //         (yDistance / (FieldConstants.Reef.faceLength * 2)) + ((xDistance - 0.3) / (FieldConstants.Reef.faceLength * 4)),
    //         0.0,
    //         1.0);
    // double shiftYT =
    //     MathUtil.clamp(yDistance <= 0.2 ? 0.0 : offset.getX() / FieldConstants.Reef.faceLength, 0.0, 1.0);
    // return goal.transformBy(
    //     new Transform2d(
    //         -shiftXT * 1.0,
    //         Math.copySign(shiftYT * 1.2, offset.getY()),
    //         Rotation2d.kZero
    //         ));
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
                IndexerState.NEUTRAL, 
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

        public static final SuperstructureState CLIMB = new SuperstructureState(
            ElevatorSetpoint.STOW, 
            EndEffectorSetpoint.CLIMB, 
            true, 
            IntakeState.CLIMB, 
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

        public static final SuperstructureState HP_INTAKE = new SuperstructureState(
            ElevatorSetpoint.HUMAN_PLAYER, 
            EndEffectorSetpoint.INTAKE_HUMAN_PLAYER, 
            true, 
            IntakeState.STOW, 
            IndexerState.NEUTRAL, 
            false
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

        public static final SuperstructureState REMOVE_ALGAE_HIGH = new SuperstructureState(
            ElevatorSetpoint.HIGH_REEF_ALGAE, 
            EndEffectorSetpoint.REEF_ALGAE_HIGH_INTAKE, 
            true, 
            IntakeState.STOW, 
            IndexerState.NEUTRAL, 
            false
        );

        public static final SuperstructureState REMOVE_ALGAE_LOW = new SuperstructureState(
            ElevatorSetpoint.LOW_REEF_ALGAE, 
            EndEffectorSetpoint.REEF_ALGAE_LOW_INTAKE, 
            true, 
            IntakeState.STOW, 
            IndexerState.NEUTRAL, 
            false
        );

        public static final SuperstructureState INTEXER_EJECT = new SuperstructureState(
            ElevatorSetpoint.HANDOFF, 
            EndEffectorSetpoint.FORCE_EJECT, 
            true, 
            IntakeState.RETRACTED_EXTAKE, 
            IndexerState.REVERSE, 
            true
        );


    }



    
}
