
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Set;

import com.ctre.phoenix6.SignalLogger;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import dev.doglog.DogLogOptions;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.ReefPosition.ReefBranch;
import frc.robot.ReefPosition.ReefLevel;
import frc.robot.commands.AutoPilot;
import frc.robot.commands.AutoPilot.ProfiledPIDControllerConfig;
import frc.robot.commands.AutoPilot.NavToPoseConfig;
import frc.robot.commands.Autos;
import frc.robot.commands.Autos.StartPosition;
import frc.robot.BreakerLib.driverstation.BreakerInputStream;
import frc.robot.BreakerLib.driverstation.BreakerInputStream2d;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import frc.robot.BreakerLib.util.commands.RumbleCommand;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.BreakerLib.util.logging.BreakerLog.GitInfo;
import frc.robot.BreakerLib.util.logging.BreakerLog.Metadata;
import frc.robot.BreakerLib.util.math.functions.BreakerLinearizedConstrainedExponential;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ScoreOnReefScheduler;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.ApriltagVision;
import frc.robot.subsystems.vision.Localization;
import frc.robot.subsystems.vision.ApriltagVision.EstimationType;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // === SUBSYSTEMS ===
  // Subsystems are robot components like drivebase, arm, shooter, etc.
  // They contain the methods to control physical hardware
  private final Drivetrain drivetrain = new Drivetrain();
  private final Intake intake = new Intake();
  private final Indexer indexer = new Indexer();
  private final Elevator elevator = new Elevator();
  private final EndEffector endEffector = new EndEffector();
  private final Climb climb = new Climb(drivetrain.getPigeon2());
  // private final ApriltagVision apriltagVision = new ApriltagVision(drivetrain);
  private final Localization localization = new Localization(drivetrain);
  private final AutoPilot ap = new AutoPilot(drivetrain, drivetrain.getLocalizer());

  private final BreakerXboxController controller =
      new BreakerXboxController(OperatorConstants.kDriverControllerPort);
  private final ButtonBoard buttonBoard = new ButtonBoard(OperatorConstants.kButtonBoardPort);
  
  //private final SimpleClimb climb = new SimpleClimb();
  

  private final Superstructure superstructure2 = new Superstructure(endEffector, elevator, intake, indexer, climb, drivetrain, localization, controller);

  // private final ScoreOnReefScheduler scoreOnReefScheduler = new ScoreOnReefScheduler(buttonBoard, superstructure);

  private final ScoreOnReefScheduler scoreOnReefScheduler = new ScoreOnReefScheduler(buttonBoard, controller, superstructure2);
  
  private final Autos autos = new Autos(superstructure2);

  private BreakerInputStream driverX, driverY, driverOmega;
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    startLog();
    // Configure the trigger bindings
    configureControls();
  }

  private void startLog() {
    SignalLogger.enableAutoLogging(false);
    BreakerLog.setOptions(new DogLogOptions(() -> !DriverStation.isFMSAttached(), false, true, true, true, 20000));
    // BreakerLog.setPdh(new PowerDistribution(MiscConstants.PDH_ID, ModuleType.kRev));
    BreakerLog.addCANBus(DriveConstants.kCANBus);
    BreakerLog.setEnabled(true);

    GitInfo gitInfo = new GitInfo(BuildConstants.MAVEN_NAME, BuildConstants.GIT_REVISION, BuildConstants.GIT_SHA, BuildConstants.GIT_DATE, BuildConstants.GIT_BRANCH, BuildConstants.BUILD_DATE, BuildConstants.DIRTY);
    BreakerLog.logMetadata(new Metadata("Leviathan", 2025, "Roman Abrahamson, Sebastian Rueda, Lee Wang, Noah Lumbra, Max Xu", gitInfo));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureControls() {
    BreakerInputStream2d driverTranslation = controller.getLeftThumbstick();
    driverTranslation = driverTranslation
            .clamp(1.0)
            .deadband(Constants.OperatorConstants.TRANSLATIONAL_DEADBAND, 1.0)
            .mapToMagnitude(new BreakerLinearizedConstrainedExponential(0.15, 4.0, true))
            .scale(Constants.DriveConstants.MAXIMUM_TRANSLATIONAL_VELOCITY.in(Units.MetersPerSecond));
    driverX = driverTranslation.getY();
    driverY = driverTranslation.getX();
    driverOmega = controller.getRightThumbstick().getX()
            .clamp(1.0)
            .deadband(Constants.OperatorConstants.ROTATIONAL_DEADBAND, 1.0)
            .map(new BreakerLinearizedConstrainedExponential(0.1, 9.5, true))
            .scale(Constants.DriveConstants.MAXIMUM_ROTATIONAL_VELOCITY.minus(Units.RadiansPerSecond.of(4)).in(Units.RadiansPerSecond));

    drivetrain.setDefaultCommand(superstructure2.getDriveTeleopControlCommand(driverTranslation, driverOmega, DriveConstants.TELEOP_CONTROL_CONFIG));
    var tipProtectedInputStreams = superstructure2.getTipProtectionSystem().getStreams();
    driverTranslation = tipProtectedInputStreams.getFirst();
    driverOmega = tipProtectedInputStreams.getSecond();

    Trigger manualOverride = buttonBoard.getRightButtons().getHighRightSwitch();

    scoreOnReefScheduler.bind();

    controller.getButtonX().onTrue(elevator.home());
    controller.getButtonB().onTrue(superstructure2.reverseIntake());
    controller.getDPad().getUp().and(manualOverride).onTrue(Commands.runOnce(drivetrain::seedFieldCentric));
    controller.getDPad().getLeft().onTrue(superstructure2.removeAlgae(false));
    controller.getDPad().getRight().onTrue(superstructure2.removeAlgae(true));
    controller.getLeftBumper().onTrue(superstructure2.intakeFromGround());
    controller.getStartButton().and(manualOverride.negate())
    .onTrue(
      Commands.defer(
        () -> superstructure2.intakeCoralFromHumanPlayer(
          CoralHumanPlayerStation.getClosest(
            drivetrain.getLocalizer().getPose(), 
            DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
          )
        ), 
        Set.of(elevator, indexer, intake, endEffector)
      )
    );
    controller.getStartButton().and(manualOverride)
    .onTrue(
      superstructure2.intakeCoralFromHumanPlayerManual()
      );
    new Trigger(() -> (controller.getRightTrigger().get() >= 0.5)).onTrue(superstructure2.stowAll().alongWith(new RumbleCommand(controller.getBaseHID(), RumbleType.kBothRumble, 0.15).withTimeout(0.05)));
    new Trigger(() -> (controller.getLeftTrigger().get() >= 0.5)).onTrue(superstructure2.stowAll().alongWith(new RumbleCommand(controller.getBaseHID(), RumbleType.kBothRumble, 0.15).withTimeout(0.05)));
    buttonBoard.getRightButtons().getLowRightButton().toggleOnTrue(superstructure2.snapHeadingToClosestReefFace(driverTranslation, driverOmega).onlyWhile(manualOverride.negate()));

    buttonBoard.getRightButtons().getLowRightSwitch().and(manualOverride.negate()).onTrue(superstructure2.climbOnDeepCage());
    buttonBoard.getRightButtons().getLowRightSwitch().and(manualOverride).onTrue(superstructure2.climbOnDeepCageManual());
    buttonBoard.getRightButtons().getLowRightSwitch().onFalse(superstructure2.stowClimb());

    controller.getRightBumper().and(manualOverride).onTrue(superstructure2.manualIntakeFromGroundForL1());
    controller.getRightBumper().and(manualOverride.negate()).onTrue(superstructure2.intakeFromGroundForL1(driverTranslation, driverOmega));
    controller.getButtonY().onTrue(superstructure2.extakeForL1FromIntake());




    


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // This command will be run during the autonomous period
    // You can create different auto routines and select them via Shuffleboard/SmartDashboard
    return autos.getSelectedAuto();
    //return null;
  }
}
