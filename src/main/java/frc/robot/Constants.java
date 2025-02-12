// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.BreakerLib.swerve.BreakerSwerveDrivetrain.BreakerSwerveDrivetrainConstants;
import frc.robot.BreakerLib.swerve.BreakerSwerveDrivetrain.BreakerSwerveDrivetrainConstants.ChoreoConfig;
import frc.robot.BreakerLib.swerve.BreakerSwerveTeleopControl.HeadingCompensationConfig;
import frc.robot.BreakerLib.swerve.BreakerSwerveTeleopControl.SetpointGenerationConfig;
import frc.robot.BreakerLib.swerve.BreakerSwerveTeleopControl.TeleopControlConfig;
import frc.robot.BreakerLib.util.MechanismRatio;
import frc.robot.subsystems.Drivetrain.DrivetrainKinematicLimits;

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

    public static class AutoPilotConstants {
      
      
    }

    public static class TipProtectionSystemConstants {
      public static final DrivetrainKinematicLimits kFullyExtendedLimit = new DrivetrainKinematicLimits(MetersPerSecond.of(1), MetersPerSecondPerSecond.of(1), DegreesPerSecond.of(45), DegreesPerSecondPerSecond.of(45));
      public static final DrivetrainKinematicLimits kTenCentimeterLimit = new DrivetrainKinematicLimits(MetersPerSecond.of(5), MetersPerSecondPerSecond.of(3), DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(90));
      public static final DrivetrainKinematicLimits kTwentyCentimeterLimit = new DrivetrainKinematicLimits(MetersPerSecond.of(5), MetersPerSecondPerSecond.of(3), DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(90));
      public static final DrivetrainKinematicLimits kThirtyCentimeterLimit = new DrivetrainKinematicLimits(MetersPerSecond.of(5), MetersPerSecondPerSecond.of(3), DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(90));
    
      public static final InterpolatingTreeMap<Distance, DrivetrainKinematicLimits> kinematicLimitMap = new InterpolatingTreeMap<>(null, null);

      public static final Angle tippingThreshold = Degrees.of(0);

      static {
        kinematicLimitMap.put(Centimeter.of(10), kTenCentimeterLimit);
        kinematicLimitMap.put(Centimeter.of(20), kTwentyCentimeterLimit);
        kinematicLimitMap.put(Centimeter.of(30), kThirtyCentimeterLimit);
        kinematicLimitMap.put(Centimeter.of(40), kFullyExtendedLimit);
      }
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

  public static class SuperstructureConstants {
    public static final Distance kMaxHeightForEndEffectorFloorLimit = Centimeters.of(4);
    public static final Distance kMaxHeightForEndEffectorFullMotion = Centimeters.of(30);
    public static final CANBus kSuperstructureCANBus = new CANBus("superstructure");

    // public static final InterpolatingTreeMap<Distance, DrivetrainKinimaticLimits> kElevatorExtendedDriveKinimaticLimitsTable = 

    // private InterpolatingTreeMap<Distance, DrivetrainKinimaticLimits> getElevatorExtendedDriveKinimaticLimitsTable() {
    //   InterpolatingTreeMap<Distance, DrivetrainKinimaticLimits> table = new InterpolatingTreeMap<Double, DrivetrainKinimaticLimits>(new DrivetrainKinimaticLimits.KinimaticLimitInverseInterpolator(), new DrivetrainKinimaticLimits.KinimaticLimitInterpolator());
    // }
  }

  public static class ElevatorConstants {
    public static final int kLeftMotorID = 30;
    public static final int kRightMotorID = 31;
    public static final MechanismRatio kRotationsToMeters = new MechanismRatio(0, 0);

    public static final InvertedValue kLeftMotorInverted = InvertedValue.Clockwise_Positive;
    public static final InvertedValue kRightMotorInverted = InvertedValue.Clockwise_Positive;

    // All gains are in terms of rotor rotations and volts
    public static final double kP = 0.26;
    public static final double kI = 0;
    public static final double kD = 0.12;
    public static final double kS = 0;
    public static final double kV = 0.155;
    public static final double kA = 0.03;
    public static final double kG = 0.735;
    public static final AngularVelocity kMotionMagicCruiseVelocity = RotationsPerSecond.of(15);
    public static final AngularAcceleration kMotionMagicAcceleration = RotationsPerSecondPerSecond.of(25);
    // public static final Measure<VelocityUnit<LinearAccelerationUnit>> kMotionMagicJerk = MetersPerSecondPerSecond.per(Second).of(15.0);


    // Thease are per-motor limits
    public static final Current kSupplyCurrentLimit = Amps.of(60);
    public static final Current kSupplyLowerCurrentLimit = Amps.of(60);
    public static final Time kSupplyLowerCurrentLimitTime = Seconds.of(0);
    public static final Current kStatorCurrentLimit = Amps.of(80);
    public static final CurrentLimitsConfigs kNormalCurrentLimits = new CurrentLimitsConfigs()
      .withSupplyCurrentLimit(kSupplyCurrentLimit)
      .withSupplyCurrentLowerLimit(kSupplyLowerCurrentLimit)
      .withSupplyCurrentLowerTime(kSupplyLowerCurrentLimitTime)
      .withStatorCurrentLimitEnable(true)
      .withStatorCurrentLimit(80)
      .withStatorCurrentLimitEnable(true);

    public static final Current kHomeSupplyCurrentLimit = Amps.of(5);
    public static final Current kHomeSupplyLowerCurrentLimit = Amps.of(5);
    public static final Time kHomeSupplyLowerCurrentLimitTime = Seconds.of(0);
    public static final Current kHomeStatorCurrentLimit = Amps.of(6);
    public static final CurrentLimitsConfigs kHomeingCurrentLimits = new CurrentLimitsConfigs()
      .withSupplyCurrentLimit(kHomeSupplyCurrentLimit)
      .withSupplyCurrentLowerLimit(kHomeSupplyLowerCurrentLimit)
      .withSupplyCurrentLowerTime(kHomeSupplyLowerCurrentLimitTime)
      .withStatorCurrentLimitEnable(true)
      .withStatorCurrentLimit(kHomeStatorCurrentLimit)
      .withStatorCurrentLimitEnable(true);

    public static final Distance kMaxHeight = Meters.of(0.0);
    public static final Distance kMinHeight = Meters.of(0.0);

    public static final Distance kDefaultHeightTolerence = Millimeters.of(5);
    public static final LinearVelocity kDefaultVelocityTolerence = Millimeters.per(Second).of(2);

    public static final double kHomeingVoltage = kG - 0.1; 
    public static final Current kHomeDetectCurrentThreshold = Amps.of(3.5);
  }
  
  public static class IntakeConstants {
    public static final Angle kPivotTolerence = Degrees.of(4);
    private static final Angle kPivotEncoderMagnetOffset = Rotations.of(0.1455078125);
    private static final Angle kPivotEncoderZeroPointOffsetFromAbsolute = Rotations.of(0.029053);
    public static final Angle kPivotEncoderOffset = kPivotEncoderMagnetOffset.plus(kPivotEncoderZeroPointOffsetFromAbsolute);
    public static final int kIntakePivotMotorID = 40;
    public static final int kIntakeRollersMotorID = 41;
    public static final int kIntakeCANCoderID = 42;

    public static final MechanismRatio kPivotGearRatio = new MechanismRatio(92.25);

    public static final double kP = 0.5;
    public static final double kI = 0.0;
    public static final double kD = 2.5;
    public static final double kS = 0.0;
    public static final double kV = 12.0;
    public static final double kA = 2.5;
    public static final double kG = 0.25;
    public static final AngularVelocity kMotionMagicCruiseVelocity = RotationsPerSecond.of(3);
    public static final AngularAcceleration kMotionMagicAcceleration = RotationsPerSecondPerSecond.of(2);

    
  }

  public static class ButtonBoardConstants {
    public static final int L1_BUTTON = 1;
    public static final int L2_BUTTON = 2;
    public static final int L3_BUTTON = 3;
    public static final int L4_BUTTON = 4;

    public static final int REEF_A_BUTTON = 5;
    public static final int REEF_B_BUTTON = 6;
    public static final int REEF_C_BUTTON = 7;
    public static final int REEF_D_BUTTON = 8;
    public static final int REEF_E_BUTTON = 9;
    public static final int REEF_F_BUTTON = 10;
    public static final int REEF_G_BUTTON = 11;
    public static final int REEF_H_BUTTON = 12;
    public static final int REEF_I_BUTTON = 13;
    public static final int REEF_J_BUTTON = 14;
    public static final int REEF_K_BUTTON = 15;
    public static final int REEF_L_BUTTON = 16;

    public static final int LOW_RIGHT_BUTTON = 17;
    public static final int LOW_RIGHT_SWITCH = 18;
    public static final int HIGH_RIGHT_SWITCH = 19;
    public static final int HIGH_RIGHT_BUTTON = 20;


  }

  public static class EndEffectorConstants {
    private static final double kWristEncoderMagnetOffset = -0.3037109375;
    public static final Angle kWristEncoderOffset = Rotations.of(kWristEncoderMagnetOffset + 0.25);

    public static final MechanismRatio kWristRatio = new MechanismRatio(25).to(new MechanismRatio(15, 18)).to(new MechanismRatio(20, 56));//78.75

    public static final double kWristDiscontinuityPoint = 0.75;
    public static final Angle kMaxElevatorRestrictedSafeAngle = Degrees.of(45);

    public static final Color kAlgaeColor = new Color(0.11, 0.831, 0.69);
    public static final double kMaxColorDelta = 0.15;
    public static final double kHasAlgaeProximityThresh = 0.1;

    public static final Angle kMinWristAngle = Degrees.of(0);
    public static final Angle kMaxWristAngle = Degrees.of(270);

    public static final Angle kDefaultWristAngleTolerence = Degrees.of(3.5);
    public static final AngularVelocity kDefaultWristVelocityTolerence = DegreesPerSecond.of(4);

    public static final SoftwareLimitSwitchConfigs kElevatorExtendedLimits = new SoftwareLimitSwitchConfigs();
    public static final SoftwareLimitSwitchConfigs kFloorRestrictedLimits = new SoftwareLimitSwitchConfigs();
    public static final SoftwareLimitSwitchConfigs kNormalLimits = new SoftwareLimitSwitchConfigs();


    public static final SupplyCurrentLimitConfiguration kNormalRollerCurrentLimitConfig = new SupplyCurrentLimitConfiguration(true, 30, 30, 0.2);
    public static final SupplyCurrentLimitConfiguration kAlgaeHoldRollerCurrentLimitConfig = new SupplyCurrentLimitConfiguration(true, 15, 8, 0.5);
    public static final SupplyCurrentLimitConfiguration kNormalKickerCurrentLimitConfig = new SupplyCurrentLimitConfiguration(true, 60, 30, 0.2);
    public static final SupplyCurrentLimitConfiguration kAlgaeHoldKickerCurrentLimitConfig = new SupplyCurrentLimitConfiguration(true, 15, 8, 0.5);
    
    public static final int kEndEffectorPivotMotorID = 50;
    public static final int kEndEffectorCANCoderID = 51;
    public static final int kEndEffectorCANdiID = 52;

    //kp 8, kd 0.7, kv 8, ka 0.8, kg 0.26, vel 1.5, acc 2
    
  }

  public static class KickerConstants {
    public static final Color kAlgaeColor = new Color(0.11, 0.831, 0.69);
    public static final double kMaxColorDelta = 0.15;
    
  }

  public static class ClimbConstants {
    public static final int kClimbMotorID = 0;
    public static final int kClimbCoder = 0;

    public static final double kClimbCoderAbsoluteSensorDiscontinuityPoint = 0;
    public static final Angle kClimbCoderOffset = Radians.of(0);
    public static final Angle kExtendedThreshold = Radian.of(0);

    public static final double kClimbMotionMagicAcceleration = 0;
    public static final double kClimbMotionMagicCruiseVelocity = 0;

    public static final Rotation2d kClimbReverseLimit = Rotation2d.fromDegrees(0);
    public static final Rotation2d kClimbForwardLimit = Rotation2d.fromDegrees(0);
    public static final MechanismRatio kClimbGearRatio = new MechanismRatio(150,1);
    public static final CurrentLimitsConfigs kClimbCurrentLimits = new CurrentLimitsConfigs()
                                              .withStatorCurrentLimit(30)
                                              .withStatorCurrentLimitEnable(true)
                                              .withSupplyCurrentLimit(25)
                                              .withSupplyCurrentLimitEnable(true);
    public static final Angle kClimbingPosition = Rotations.of(0);
    public static final Angle kExtendedPosition = Rotations.of(0);
    public static final Angle kStowPosition = Rotations.of(0);
    public static final Angle kNeutralPosition = Rotations.of(0);

  }


  public static class IndexerConstants {
    public static final int kIndexerMotorID = 0;
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
            .withHeadingCompensation(HEADING_COMPENSATION_CONFIG);
            // .withSetpointGeneration(SETPOINT_GENERATION_CONFIG);
    public static final LinearVelocity MAXIMUM_TRANSLATIONAL_VELOCITY = MetersPerSecond.of(4.5);
    public static final AngularVelocity MAXIMUM_ROTATIONAL_VELOCITY = RadiansPerSecond.of(12);

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
    public static final CANBus kCANBus = new CANBus("drive");

    // Theoretical free speed (m/s) at 12 V applied output;
    // This needs to be tuned to your individual robot
    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.69);

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio_MK4i = 0.0;
    private static final double kCoupleRatio_MK4n = 0.0;
    private static final double kDriveGearRatio = 7.13;
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
    private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.118896484375 + 0.25);
    // private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.456787109375+0.25+0.5);
    private static final boolean kFrontLeftSteerMotorInverted = true;
    private static final boolean kFrontLeftEncoderInverted = false;

    private static final Distance kFrontLeftXPos = Inches.of(10.375); 
    private static final Distance kFrontLeftYPos = Inches.of(10.375);

    // Front Right
    private static final int kFrontRightDriveMotorId = 12;
    private static final int kFrontRightSteerMotorId = 13;
    private static final int kFrontRightEncoderId = 21;
    private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.49169921875 - 0.25);
    // private static final Angle kFrontRightEncoderOffset = Rotations.of(0.428955078125-0.25+0.5);
    private static final boolean kFrontRightSteerMotorInverted = true;
    private static final boolean kFrontRightEncoderInverted = false;

    private static final Distance kFrontRightXPos = Inches.of(10.375);
    private static final Distance kFrontRightYPos = Inches.of(-10.375); 

    // Back Left
    private static final int kBackLeftDriveMotorId = 14;
    private static final int kBackLeftSteerMotorId = 15;
    private static final int kBackLeftEncoderId = 22;
    private static final Angle kBackLeftEncoderOffset = Rotations.of(0.428955078125 - 0.25);
    // private static final Angle kBackLeftEncoderOffset = Rotations.of(-0.49169921875-0.25+0.5);
    private static final boolean kBackLeftSteerMotorInverted = true;
    private static final boolean kBackLeftEncoderInverted = false;

    private static final Distance kBackLeftXPos = Inches.of(-10.375);
    private static final Distance kBackLeftYPos = Inches.of(10.428);

    // Back Right
    private static final int kBackRightDriveMotorId = 16;
    private static final int kBackRightSteerMotorId = 17;
    private static final int kBackRightEncoderId = 23;
    private static final Angle kBackRightEncoderOffset = Rotations.of(0.456787109375 + 0.25);
    // private static final Angle kBackRightEncoderOffset = Rotations.of(0.118896484375+0.25+0.5);
    private static final boolean kBackRightSteerMotorInverted = true;
    private static final boolean kBackRightEncoderInverted = false;

    private static final Distance kBackRightXPos = Inches.of(-10.375);
    private static final Distance kBackRightYPos = Inches.of(-10.428);


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
