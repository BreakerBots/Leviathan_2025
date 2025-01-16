// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.BreakerLib.swerve.BreakerSwerveDrivetrain.BreakerSwerveDrivetrainConstants;
import frc.robot.BreakerLib.swerve.BreakerSwerveDrivetrain.BreakerSwerveDrivetrainConstants.ChoreoConfig;
import frc.robot.BreakerLib.swerve.BreakerSwerveTeleopControl.HeadingCompensationConfig;
import frc.robot.BreakerLib.swerve.BreakerSwerveTeleopControl.SetpointGenerationConfig;
import frc.robot.BreakerLib.swerve.BreakerSwerveTeleopControl.TeleopControlConfig;
import frc.robot.BreakerLib.util.MechanismRatio;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.config.PIDConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class MiscConstants {
        public static final int PDH_ID = 0;
    }
  /** 
   * Constants related to operator controls and human interface devices.
   * USB ports are numbered 0-5 on the Driver Station
   */
  public static class OperatorConstants {
    /** USB port number for main driver controller (listed in Driver Station) */
    public static final int kDriverControllerPort = 0;

    public static final double TRANSLATIONAL_DEADBAND = 0.1;
    public static final double ROTATIONAL_DEADBAND = 0.1;
  }

  public static class ElevatorConstants {
    public static final int kLeftMotorID = 30;
    public static final int kRightMotorID = 31;
    public static final MechanismRatio kRotationsToMeters = new MechanismRatio(0, 0);

    public static final InvertedValue kLeftMotorInverted = InvertedValue.Clockwise_Positive;
    public static final InvertedValue kRightMotorInverted = InvertedValue.Clockwise_Positive;

    // All gains are in Meters, m/s, m/s/s, or m/s/s/s
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kG = 0;
    public static final LinearVelocity kMotionMagicCruiseVelocity = MetersPerSecond.of(2.0);
    public static final LinearAcceleration kMotionMagicAcceleration = MetersPerSecondPerSecond.of(5.0);
    public static final Measure<VelocityUnit<LinearAccelerationUnit>> kMotionMagicJerk = MetersPerSecondPerSecond.per(Second).of(15.0);


    // Thease are per-motor limits
    public static final Current kSupplyCurrentLimit = Amps.of(60);
    public static final Current kSupplyLowerCurrentLimit = Amps.of(60);
    public static final Time kSupplyLowerCurrentLimitTime = Seconds.of(0);
    public static final Current kStatorCurrentLimit = Amps.of(80);

    public static final Distance kMaxHeight = Meters.of(0.0);
    public static final Distance kMinHeight = Meters.of(0.0);

    public static final Distance kDefaultHeightTolerence = Millimeters.of(5);
    public static final LinearVelocity kDefaultVelocityTolerence = Millimeters.per(Second).of(2);
  }
  
  public static class IntakeConstants {
    public static final Angle kPivotTolerence = Degrees.of(4);
    
  }

  public static class EndEffectorConstants {
    public static final Color kAlgaeColor = new Color(0.11, 0.831, 0.69);
    public static final double kMaxColorDelta = 0.15;
    public static final double kHasAlgaeProximityThresh = 0.1;

    public static final Rotation2d kMinWristAngle = Rotation2d.fromDegrees(0.0);
    public static final Rotation2d kMaxWristAngle = Rotation2d.fromDegrees(270);

    public static final Angle kDefaultWristAngleTolerence = Degrees.of(3.5);
    public static final AngularVelocity kDefaultWristVelocityTolerence = DegreesPerSecond.of(4);

    public static final SupplyCurrentLimitConfiguration kNormalRollerCurrentLimitConfig = new SupplyCurrentLimitConfiguration(true, 30, 30, 0.2);
    public static final SupplyCurrentLimitConfiguration kAlgaeHoldRollerCurrentLimitConfig = new SupplyCurrentLimitConfiguration(true, 15, 8, 0.5);
    public static final SupplyCurrentLimitConfiguration kNormalKickerCurrentLimitConfig = new SupplyCurrentLimitConfiguration(true, 60, 30, 0.2);
    public static final SupplyCurrentLimitConfiguration kAlgaeHoldKickerCurrentLimitConfig = new SupplyCurrentLimitConfiguration(true, 15, 8, 0.5);
  }

  public static class KickerConstants {
    public static final Color kAlgaeColor = new Color(0.11, 0.831, 0.69);
    public static final double kMaxColorDelta = 0.15;
    
  }

  public static class ClimbConstants {
    public static final int kForkMotorID = 0;
    public static final int kWinchMotorID = 0;
    public static final int kForkCoder = 0;
    public static final int kForkSensor = 0;
    public static final Rotation2d kForkReverseLimit = Rotation2d.fromDegrees(0);
    public static final MechanismRatio kForkSensorGearRatio = new MechanismRatio(1,1);
    public static final Rotation2d kWinchReverseLimit = Rotation2d.fromRotations(0);
    public static final MechanismRatio kWinchRatio = new MechanismRatio(1,1);
  }

  public static class AutoConstants {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(7.5, 0, 0.8);
    public static final PIDConstants ROTATION_PID = new PIDConstants(1.5, 0,1);
    public static final ChoreoConfig CHOREO_CONFIG = new ChoreoConfig().withTranslationPID(TRANSLATION_PID).withRotationPID(ROTATION_PID);
  }

  public static class DriveConstants {
    public static final AngularVelocity MAXIMUM_MODULE_AZIMUTH_SPEED = DegreesPerSecond.of(720);
    public static final HeadingCompensationConfig HEADING_COMPENSATION_CONFIG = new HeadingCompensationConfig(
                    MetersPerSecond.of(0.05), 
                    RadiansPerSecond.of(0.001), 
                    Seconds.of(0.2),
                    new PIDConstants(1.5, 0, 0));// 1.5
    public static final SetpointGenerationConfig SETPOINT_GENERATION_CONFIG = new SetpointGenerationConfig(MAXIMUM_MODULE_AZIMUTH_SPEED);
    public static final TeleopControlConfig TELEOP_CONTROL_CONFIG = new TeleopControlConfig()
            .withHeadingCompensation(HEADING_COMPENSATION_CONFIG)
            .withSetpointGeneration(SETPOINT_GENERATION_CONFIG);
    public static final LinearVelocity MAXIMUM_TRANSLATIONAL_VELOCITY = MetersPerSecond.of(4.5);
    public static final AngularVelocity MAXIMUM_ROTATIONAL_VELOCITY = RadiansPerSecond.of(9.5);

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains_MK4i = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.5)
        .withKS(0.1).withKV(1.91).withKA(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

     // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains_MK4n = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.5)
        .withKS(0.1).withKV(1.91).withKA(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    

    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(0.1).withKI(0).withKD(0)
        .withKS(0).withKV(0.124);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The type of motor used for the drive motor
    private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
    // The type of motor used for the drive motor
    private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

    // The remote sensor feedback type to use for the steer motors;
    // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to RemoteCANcoder
    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final Current kSlipCurrent = Amps.of(120.0);

    // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                // Swerve azimuth does not require much torque output, so we can set a relatively low
                // stator current limit to help avoid brownouts without impacting performance.
                .withStatorCurrentLimit(Amps.of(80))
                .withStatorCurrentLimitEnable(true)
        );
    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    private static final Pigeon2Configuration pigeonConfigs = null;

    // CAN bus that the devices are located on;
    // All swerve devices must share the same CAN bus
    public static final CANBus kCANBus = new CANBus("canivore");

    // Theoretical free speed (m/s) at 12 V applied output;
    // This needs to be tuned to your individual robot
    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.69);

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio_MK4i = 0.0;
    private static final double kCoupleRatio_MK4n = 0.0;
    private static final double kDriveGearRatio = 8.14;
    private static final double kSteerGearRatio_MK4i = 150.0/7.0;
    private static final double kSteerGearRatio_MK4n = 18.75;
    private static final Distance kWheelRadius = Inches.of(2.0);

    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final int kPigeonId = 5;

    // These are only used for simulation
    private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
    private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
    // Simulated voltage necessary to overcome friction
    private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
    private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

    public static final BreakerSwerveDrivetrainConstants DRIVETRAIN_CONSTANTS = new BreakerSwerveDrivetrainConstants()
      .withCANBusName(kCANBus.getName())
      .withPigeon2Id(kPigeonId)
      .withPigeon2Configs(pigeonConfigs)
      .withChoreoConfig(AutoConstants.CHOREO_CONFIG);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator_MK4i =
        new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio_MK4i)
            .withCouplingGearRatio(kCoupleRatio_MK4i)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGains_MK4i)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
            .withSlipCurrent(kSlipCurrent)
            .withSpeedAt12Volts(kSpeedAt12Volts)
            .withDriveMotorType(kDriveMotorType)
            .withSteerMotorType(kSteerMotorType)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator_MK4n =
        new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio_MK4n)
            .withCouplingGearRatio(kCoupleRatio_MK4n)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGains_MK4n)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
            .withSlipCurrent(kSlipCurrent)
            .withSpeedAt12Volts(kSpeedAt12Volts)
            .withDriveMotorType(kDriveMotorType)
            .withSteerMotorType(kSteerMotorType)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage);


    // Front Left
    private static final int kFrontLeftDriveMotorId = 10;
    private static final int kFrontLeftSteerMotorId = 11;
    private static final int kFrontLeftEncoderId = 20;
    private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.456787109375+0.25);
    private static final boolean kFrontLeftSteerMotorInverted = true;
    private static final boolean kFrontLeftEncoderInverted = false;

    private static final Distance kFrontLeftXPos = Inches.of(10.375);
    private static final Distance kFrontLeftYPos = Inches.of(10.428);

    // Front Right
    private static final int kFrontRightDriveMotorId = 12;
    private static final int kFrontRightSteerMotorId = 13;
    private static final int kFrontRightEncoderId = 21;
    private static final Angle kFrontRightEncoderOffset = Rotations.of(0.428955078125-0.25);
    private static final boolean kFrontRightSteerMotorInverted = true;
    private static final boolean kFrontRightEncoderInverted = false;

    private static final Distance kFrontRightXPos = Inches.of(10.375);
    private static final Distance kFrontRightYPos = Inches.of(-10.428);

    // Back Left
    private static final int kBackLeftDriveMotorId = 14;
    private static final int kBackLeftSteerMotorId = 15;
    private static final int kBackLeftEncoderId = 22;
    private static final Angle kBackLeftEncoderOffset = Rotations.of(-0.49169921875-0.25);
    private static final boolean kBackLeftSteerMotorInverted = true;
    private static final boolean kBackLeftEncoderInverted = false;

    private static final Distance kBackLeftXPos = Inches.of(-10.375);
    private static final Distance kBackLeftYPos = Inches.of(10.375);

    // Back Right
    private static final int kBackRightDriveMotorId = 16;
    private static final int kBackRightSteerMotorId = 17;
    private static final int kBackRightEncoderId = 23;
    private static final Angle kBackRightEncoderOffset = Rotations.of(0.118896484375+0.25);
    private static final boolean kBackRightSteerMotorInverted = true;
    private static final boolean kBackRightEncoderInverted = false;

    private static final Distance kBackRightXPos = Inches.of(-10.375);
    private static final Distance kBackRightYPos = Inches.of(-10.375);


    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
        ConstantCreator_MK4n.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
            kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide, kFrontLeftSteerMotorInverted, kFrontLeftEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
        ConstantCreator_MK4n.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
            kFrontRightXPos, kFrontRightYPos, kInvertRightSide, kFrontRightSteerMotorInverted, kFrontRightEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
        ConstantCreator_MK4i.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
            kBackLeftXPos, kBackLeftYPos, kInvertLeftSide, kBackLeftSteerMotorInverted, kBackLeftEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
        ConstantCreator_MK4i.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
            kBackRightXPos, kBackRightYPos, kInvertRightSide, kBackRightSteerMotorInverted, kBackRightEncoderInverted
        );

  }
}
