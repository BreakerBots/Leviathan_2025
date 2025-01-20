// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.BreakerLib.driverstation.BreakerInputStream;
import frc.robot.BreakerLib.driverstation.BreakerInputStream2d;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.BreakerLib.util.logging.BreakerLog.GitInfo;
import frc.robot.BreakerLib.util.logging.BreakerLog.Metadata;
import frc.robot.BreakerLib.util.math.functions.BreakerLinearizedConstrainedExponential;
import frc.robot.subsystems.Drivetrain;
import dev.doglog.DogLogOptions;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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

  // === CONTROLLERS ===
  // Xbox controller used for main driver
  // Ports can be checked/changed in Driver Station
  private final BreakerXboxController controller =
      new BreakerXboxController(OperatorConstants.kDriverControllerPort);

  private BreakerInputStream driverX, driverY, driverOmega;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    startLog();
    // Configure the trigger bindings
    configureControls();
  }

  private void startLog() {
    BreakerLog.setOptions(new DogLogOptions(true, false, true, true, true, 20000));
    // BreakerLog.setPdh(new PowerDistribution(MiscConstants.PDH_ID, ModuleType.kRev));
    BreakerLog.addCANBus(DriveConstants.kCANBus);
    BreakerLog.setEnabled(true);

    GitInfo gitInfo = new GitInfo(BuildConstants.MAVEN_NAME, BuildConstants.GIT_REVISION, BuildConstants.GIT_SHA, BuildConstants.GIT_DATE, BuildConstants.GIT_BRANCH, BuildConstants.BUILD_DATE, BuildConstants.DIRTY);
    BreakerLog.logMetadata(new Metadata("Leviathan", 2025, "Roman Abrahamson, Lee Wang, Noah Lumbra, Max Xu", gitInfo));
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
            .map(new BreakerLinearizedConstrainedExponential(0.0, 3.0, true))
            .scale(Constants.DriveConstants.MAXIMUM_ROTATIONAL_VELOCITY.in(Units.RadiansPerSecond));


    drivetrain.setDefaultCommand(drivetrain.getTeleopControlCommand(driverX, driverY, driverOmega, Constants.DriveConstants.TELEOP_CONTROL_CONFIG));


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // This command will be run during the autonomous period
    // You can create different auto routines and select them via Shuffleboard/SmartDashboard
    return null;
  }
}
