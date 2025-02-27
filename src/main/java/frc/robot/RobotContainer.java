
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Set;

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
import frc.robot.subsystems.SimpleClimb;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.ApriltagVision;
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
  private final ApriltagVision apriltagVision = new ApriltagVision(drivetrain);
  private final AutoPilot ap = new AutoPilot(drivetrain, apriltagVision, drivetrain.getLocalizer());

  private final BreakerXboxController controller =
      new BreakerXboxController(OperatorConstants.kDriverControllerPort);
  private final ButtonBoard buttonBoard = new ButtonBoard(OperatorConstants.kButtonBoardPort);

  
  //private final SimpleClimb climb = new SimpleClimb();
  
  
  private final Superstructure superstructure = new Superstructure(drivetrain, endEffector, elevator, indexer, 
  intake, climb, apriltagVision, ap, controller);

  private final ScoreOnReefScheduler scoreOnReefScheduler = new ScoreOnReefScheduler(buttonBoard, superstructure);
  
  private final Autos autos = new Autos(superstructure);

  private BreakerInputStream driverX, driverY, driverOmega;
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    startLog();
    // Configure the trigger bindings
    configureControls();
  }

  private void startLog() {
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
            .mapToMagnitude(new BreakerLinearizedConstrainedExponential(0.075, 3.0, true))
            .scale(Constants.DriveConstants.MAXIMUM_TRANSLATIONAL_VELOCITY.in(Units.MetersPerSecond));
    driverX = driverTranslation.getY();
    driverY = driverTranslation.getX();
    driverOmega = controller.getRightThumbstick().getX()
            .clamp(1.0)
            .deadband(Constants.OperatorConstants.ROTATIONAL_DEADBAND, 1.0)
            .map(new BreakerLinearizedConstrainedExponential(0.12, 5.5, true))
            .scale(Constants.DriveConstants.MAXIMUM_ROTATIONAL_VELOCITY.in(Units.RadiansPerSecond));

   // drivetrain.setDefaultCommand(drivetrain.getTeleopControlCommand(driverX, driverY, driverOmega, Constants.DriveConstants.TELEOP_CONTROL_CONFIG));
    drivetrain.setDefaultCommand(superstructure.getDriveTeleopControlCommand(driverTranslation, driverOmega, DriveConstants.TELEOP_CONTROL_CONFIG));

    Trigger manualOverride = buttonBoard.getRightButtons().getHighRightSwitch();

    controller.getButtonX().onTrue(elevator.home());
    controller.getButtonB().onTrue(superstructure.reverseIntake());
    controller.getDPad().getUp().onTrue(Commands.runOnce(drivetrain::seedFieldCentric));
    controller.getDPad().getLeft().onTrue(superstructure.intakeAlgaeFromReef(false));
    controller.getDPad().getRight().onTrue(superstructure.intakeAlgaeFromReef(true));
    controller.getLeftBumper().onTrue(superstructure.intakeCoralFromGround());
    controller.getStartButton().onTrue(superstructure.intakeCoralFromHumanPlayer());
    new Trigger(() -> (controller.getRightTrigger().get() >= 0.5)).onTrue(superstructure.stowAll());

    buttonBoard.getLevelButtons().getL1Button().and(manualOverride).onTrue(superstructure.scoreOnReefManual(ReefPosition.ReefLevel.L1));
    buttonBoard.getLevelButtons().getL2Button().and(manualOverride).onTrue(superstructure.scoreOnReefManual(ReefPosition.ReefLevel.L2));
    buttonBoard.getLevelButtons().getL3Button().and(manualOverride).onTrue(superstructure.scoreOnReefManual(ReefPosition.ReefLevel.L3));
    buttonBoard.getLevelButtons().getL4Button().and(manualOverride).onTrue(superstructure.scoreOnReefManual(ReefPosition.ReefLevel.L4));

    buttonBoard.getRightButtons().getLowRightButton().and(() -> !superstructure.endEffectorHasCoral()).onTrue(superstructure.scoreInProcessor());
    buttonBoard.getRightButtons().getHighRightButton().and(() -> !superstructure.endEffectorHasCoral()).onTrue(superstructure.scoreInBarge());


    buttonBoard.getRightButtons().getLowRightSwitch().onTrue(superstructure.climbOnDeepCage());
    buttonBoard.getRightButtons().getLowRightSwitch().onFalse(superstructure.stowClimb());

    controller.getButtonY().onTrue(Commands.runOnce(() -> apriltagVision.setEstimationType(EstimationType.TRIG)));

    //scoreOnReefScheduler.bind();


    Trigger reefA = buttonBoard.getReefButtons().getReefButtonA();
    Trigger reefB = buttonBoard.getReefButtons().getReefButtonB();
    Trigger reefC = buttonBoard.getReefButtons().getReefButtonC();  
    Trigger reefD = buttonBoard.getReefButtons().getReefButtonD();
    Trigger reefE = buttonBoard.getReefButtons().getReefButtonE();
    Trigger reefF = buttonBoard.getReefButtons().getReefButtonF();
    Trigger reefG = buttonBoard.getReefButtons().getReefButtonG();
    Trigger reefH = buttonBoard.getReefButtons().getReefButtonH();
    Trigger reefI = buttonBoard.getReefButtons().getReefButtonI();
    Trigger reefJ = buttonBoard.getReefButtons().getReefButtonJ();
    Trigger reefK = buttonBoard.getReefButtons().getReefButtonK();
    Trigger reefL = buttonBoard.getReefButtons().getReefButtonL();

    Trigger reefL1 = buttonBoard.getLevelButtons().getL1Button();
    Trigger reefL2 = buttonBoard.getLevelButtons().getL2Button();
    Trigger reefL3 = buttonBoard.getLevelButtons().getL3Button();
    Trigger reefL4 = buttonBoard.getLevelButtons().getL4Button();

    Trigger allignClosest = buttonBoard.getRightButtons().getHighRightButton()
      .and(superstructure::endEffectorHasCoral);

    reefA.and(reefL1).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L1, ReefBranch.A)));
    reefB.and(reefL1).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L1, ReefBranch.B)));
    reefC.and(reefL1).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L1, ReefBranch.C)));
    reefD.and(reefL1).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L1, ReefBranch.D)));
    reefE.and(reefL1).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L1, ReefBranch.E)));
    reefF.and(reefL1).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L1, ReefBranch.F)));
    reefG.and(reefL1).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L1, ReefBranch.G)));
    reefH.and(reefL1).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L1, ReefBranch.H)));
    reefI.and(reefL1).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L1, ReefBranch.I)));
    reefJ.and(reefL1).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L1, ReefBranch.J)));
    reefK.and(reefL1).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L1, ReefBranch.K)));
    reefL.and(reefL1).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L1, ReefBranch.L)));

    reefA.and(reefL2).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L2, ReefBranch.A)));
    reefB.and(reefL2).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L2, ReefBranch.B)));
    reefC.and(reefL2).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L2, ReefBranch.C)));
    reefD.and(reefL2).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L2, ReefBranch.D)));
    reefE.and(reefL2).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L2, ReefBranch.E)));
    reefF.and(reefL2).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L2, ReefBranch.F)));
    reefG.and(reefL2).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L2, ReefBranch.G)));
    reefH.and(reefL2).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L2, ReefBranch.H)));
    reefI.and(reefL2).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L2, ReefBranch.I)));
    reefJ.and(reefL2).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L2, ReefBranch.J)));
    reefK.and(reefL2).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L2, ReefBranch.K)));
    reefL.and(reefL2).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L2, ReefBranch.L)));
    
    reefA.and(reefL3).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L3, ReefBranch.A)));
    reefB.and(reefL3).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L3, ReefBranch.B)));
    reefC.and(reefL3).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L3, ReefBranch.C)));
    reefD.and(reefL3).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L3, ReefBranch.D)));
    reefE.and(reefL3).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L3, ReefBranch.E)));
    reefF.and(reefL3).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L3, ReefBranch.F)));
    reefG.and(reefL3).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L3, ReefBranch.G)));
    reefH.and(reefL3).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L3, ReefBranch.H)));
    reefI.and(reefL3).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L3, ReefBranch.I)));
    reefJ.and(reefL3).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L3, ReefBranch.J)));
    reefK.and(reefL3).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L3, ReefBranch.K)));
    reefL.and(reefL3).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L3, ReefBranch.L)));

    reefA.and(reefL4).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L4, ReefBranch.A)));
    reefB.and(reefL4).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L4, ReefBranch.B)));
    reefC.and(reefL4).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L4, ReefBranch.C)));
    reefD.and(reefL4).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L4, ReefBranch.D)));
    reefE.and(reefL4).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L4, ReefBranch.E)));
    reefF.and(reefL4).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L4, ReefBranch.F)));
    reefG.and(reefL4).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L4, ReefBranch.G)));
    reefH.and(reefL4).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L4, ReefBranch.H)));
    reefI.and(reefL4).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L4, ReefBranch.I)));
    reefJ.and(reefL4).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L4, ReefBranch.J)));
    reefK.and(reefL4).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L4, ReefBranch.K)));
    reefL.and(reefL4).onTrue(superstructure.scoreOnReef(new ReefPosition(ReefLevel.L4, ReefBranch.L)));

    reefL1.and(allignClosest).onTrue(
      Commands.deferredProxy(
        () -> superstructure.scoreOnReef(
          new ReefPosition(
            ReefLevel.L1, 
            ReefBranch.getClosest(
              drivetrain.getLocalizer().getPose(),
               DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            )
          )
        )
      //,Set.of(drivetrain, endEffector, elevator)
    ));
    reefL2.and(allignClosest).onTrue(
      Commands.deferredProxy(
        () -> superstructure.scoreOnReef(
          new ReefPosition(
            ReefLevel.L2, 
            ReefBranch.getClosest(
              drivetrain.getLocalizer().getPose(),
               DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            )
          )
        )
      //,Set.of(drivetrain, endEffector, elevator)
    ));
    reefL3.and(allignClosest).onTrue(
      Commands.deferredProxy(
        () -> superstructure.scoreOnReef(
          new ReefPosition(
            ReefLevel.L3, 
            ReefBranch.getClosest(
              drivetrain.getLocalizer().getPose(),
               DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            )
          )
        )
      //,Set.of(drivetrain, endEffector, elevator)
    ));
    reefL4.and(allignClosest).onTrue(
      Commands.deferredProxy(
        () -> superstructure.scoreOnReef(
          new ReefPosition(
            ReefLevel.L4, 
            ReefBranch.getClosest(
              drivetrain.getLocalizer().getPose(),
               DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            )
          )
        )
      //,Set.of(drivetrain, endEffector, elevator)
    ));
    


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
  }
}
