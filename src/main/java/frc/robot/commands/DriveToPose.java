package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.DrivetrainKinematicLimits;
import frc.robot.subsystems.superstructure.TipProtectionSystem;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.pathplanner.lib.config.PIDConstants;
public class DriveToPose extends Command {

  private final Drivetrain drivetrain;
  private final TipProtectionSystem tps;
  private final Supplier<Pose2d> target;

  private final ProfiledPIDController driveController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));

  private Translation2d lastSetpointTranslation = new Translation2d();
  private Translation2d lastSetpointVelocity = new Translation2d();
  private Rotation2d lastGoalRotation = Rotation2d.kZero;
  private double driveErrorAbs = 0.0;
  private double thetaErrorAbs = 0.0;
  private boolean running = false;
  private double lastTime = 0.0;
  private Supplier<Pose2d> robot;

  private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
  private DoubleSupplier omegaFF = () -> 0.0;

  private final RobotCentric driveRequest;

  private final NavToPoseConfig config;

  private TrapezoidProfile driveProfile;

  public DriveToPose(Drivetrain drivetrain, TipProtectionSystem tipProtectionSystem, Supplier<Pose2d> target) {
    this(drivetrain, tipProtectionSystem, target, new NavToPoseConfig()); //
  }

  public DriveToPose(Drivetrain drivetrain, TipProtectionSystem tipProtectionSystem, Supplier<Pose2d> target, NavToPoseConfig config) {
    this.drivetrain = drivetrain;
    this.target = target;
    tps = tipProtectionSystem;

    // Enable continuous input for theta controller
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    driveRequest = new RobotCentric();
    driveRequest.DriveRequestType = DriveRequestType.Velocity;
    this.config = config;
    robot = drivetrain.getLocalizer()::getPose;

    driveController.setPID(config.drivePID.kP, config.drivePID.kI, config.drivePID.kD);
    thetaController.setPID(config.thetaPID.kP, config.thetaPID.kI, config.thetaPID.kD);
    driveController.setConstraints(new TrapezoidProfile.Constraints(config.driveMaxVelocity.in(MetersPerSecond), config.driveMaxAcceleration.in(MetersPerSecondPerSecond)));
    thetaController.setConstraints(new TrapezoidProfile.Constraints(config.thetaMaxVelocity.in(RadiansPerSecond), config.thetaMaxAcceleration.in(RadiansPerSecondPerSecond)));

    addRequirements(drivetrain);
  }

  public DriveToPose(Drivetrain drivetrain, TipProtectionSystem tipProtectionSystem, Supplier<Pose2d> target, Supplier<Pose2d> robot, NavToPoseConfig config) {
    this(drivetrain, tipProtectionSystem, target, config);
    this.robot = robot;
  }

  public DriveToPose(
      Drivetrain drivetrain,
      TipProtectionSystem tipProtectionSystem,
      Supplier<Pose2d> target,
      Supplier<Pose2d> robot,
      Supplier<Translation2d> linearFF,
      DoubleSupplier omegaFF,
      NavToPoseConfig config
      ) {
    this(drivetrain, tipProtectionSystem, target, robot, config);
    this.linearFF = linearFF;
    this.omegaFF = omegaFF;
  }

  public record NavToPoseConfig(
      Distance driveTolerence,
      Angle thetaTolerence,


      LinearVelocity driveMaxVelocity,
      AngularVelocity thetaMaxVelocity,

      LinearAcceleration driveMaxAcceleration,
      AngularAcceleration thetaMaxAcceleration,

      LinearVelocity setpointMinVelocity,

      Distance minDistanceVelocityCorrection,

      Distance ffMinRadiusLinear,
      Distance ffMaxRadiusLinear,

      Angle ffMinErrTheta,
      Angle ffMaxErrTheta,

      double minLinearFFSReset,
      double minThetaFFSReset,

      Distance minLinearErrorReset,
      Angle minThetaErrorReset,
    
      PIDConstants drivePID,
      PIDConstants thetaPID) {

        public NavToPoseConfig() {
            this(
                Meters.of(0.01), 
                Degrees.of(1.0), 
                MetersPerSecond.of(3.0), 
                DegreesPerSecond.of(360), 
                MetersPerSecondPerSecond.of(4.0), 
                RadiansPerSecondPerSecond.of(8), 
                MetersPerSecond.of(-0.5),
                Meters.of(0.01),
                Meters.of(0.01),
                Meters.of(0.05),
                Radians.of(0),
                Radians.of(0),
                0.2,
                0.1,
                Meters.of(0.3),
                Degrees.of(15), 
                new PIDConstants(12.5, 0.1, 0), //9.5
                new PIDConstants(10.5, 0.2, 0) //
            );
        }

        public static NavToPoseConfig getIntakeAssistConfig() {
            return new NavToPoseConfig(
                Meters.of(0.015), 
                Degrees.of(1.0), 
                MetersPerSecond.of(3.0), 
                DegreesPerSecond.of(360), 
                MetersPerSecondPerSecond.of(4.0), 
                RadiansPerSecondPerSecond.of(8), 
                MetersPerSecond.of(-0.5),
                Meters.of(0.01),
                Meters.of(0.01),
                Meters.of(0.05),
                Radians.of(0),
                Radians.of(0),
                0.2,
                0.1,
                Meters.of(0.3),
                Degrees.of(15), 
                new PIDConstants(12.5, 0.1, 0), //9.5
                new PIDConstants(10.5, 0.2, 0) //
            );
            

        }
  }

  @Override
  public void initialize() {
    resetProfile();
  }

  @Override
  public void execute() {
    running = true;

    DrivetrainKinematicLimits tpsLimits = tps.getLimits();
    tpsLimits.scale(1.2, 2);
    

    driveProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                Math.min(tpsLimits.linearVelocity().in(MetersPerSecond), config.driveMaxVelocity.in(MetersPerSecond)), 
                Math.min(tpsLimits.linearAcceleration().in(MetersPerSecondPerSecond), config.driveMaxAcceleration.in(MetersPerSecondPerSecond))
            )
        );
    thetaController.setConstraints(
        new TrapezoidProfile.Constraints(
            Math.min(tpsLimits.angularVelocity().in(RadiansPerSecond), config.thetaMaxVelocity.in(RadiansPerSecond)), 
            Math.min(tpsLimits.angularAcceleration().in(RadiansPerSecondPerSecond), config.thetaMaxAcceleration.in(RadiansPerSecondPerSecond))
        )
    );

    // Get current pose and target pose
    Pose2d currentPose = robot.get();
    Pose2d targetPose = target.get();

    Pose2d poseError = currentPose.relativeTo(targetPose);
    driveErrorAbs = poseError.getTranslation().getNorm();
    thetaErrorAbs = Math.abs(poseError.getRotation().getRadians());
    double linearFFScaler =
        MathUtil.clamp(
            (driveErrorAbs - config.ffMinRadiusLinear.in(Meters))
                / (config.ffMaxRadiusLinear.in(Meters) - config.ffMinRadiusLinear.in(Meters)),
            0.0,
            1.0);
    double thetaFFScaler =
        MathUtil.clamp(
            (thetaErrorAbs - config.ffMinErrTheta.in(Radians))
                / (config.ffMaxErrTheta.in(Radians) - config.ffMinErrTheta.in(Radians)),
            0.0,
            1.0);

    // Calculate drive velocity
    // Calculate setpoint velocity towards target pose
    var direction = targetPose.getTranslation().minus(lastSetpointTranslation).toVector();
    double setpointVelocity =
        direction.norm()
                <= config.minDistanceVelocityCorrection.in(Meters)
                    // Don't calculate velocity in direction when really close
            ? lastSetpointVelocity.getNorm()
            : lastSetpointVelocity.toVector().dot(direction) / direction.norm();
    setpointVelocity = Math.max(setpointVelocity, config.setpointMinVelocity.in(MetersPerSecond));
    State driveSetpoint =
        driveProfile.calculate(
            0.02,
            new State(
                direction.norm(), -setpointVelocity), // Use negative as profile has zero at target
            new State(0.0, 0.0));
    double driveVelocityScalar =
        driveController.calculate(driveErrorAbs, driveSetpoint.position)
            + driveSetpoint.velocity * linearFFScaler;
    if (driveErrorAbs < config.driveTolerence.in(Meters)) driveVelocityScalar = 0.0;
    Rotation2d targetToCurrentAngle =
        currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle();
    Translation2d driveVelocity = new Translation2d(driveVelocityScalar, targetToCurrentAngle);
    lastSetpointTranslation =
        new Pose2d(targetPose.getTranslation(), targetToCurrentAngle)
            .transformBy(new Transform2d(driveSetpoint.position, 0.0, new Rotation2d()))
            .getTranslation();
    lastSetpointVelocity = new Translation2d(driveSetpoint.velocity, targetToCurrentAngle);

    // Calculate theta speed
    double thetaSetpointVelocity =
        Math.abs((targetPose.getRotation().minus(lastGoalRotation)).getDegrees()) < 10.0
            ? (targetPose.getRotation().minus(lastGoalRotation)).getRadians()
                / (Timer.getTimestamp() - lastTime)
            : thetaController.getSetpoint().velocity;
    double thetaVelocity =
        thetaController.calculate(
                currentPose.getRotation().getRadians(),
                new State(targetPose.getRotation().getRadians(), thetaSetpointVelocity))
            + thetaController.getSetpoint().velocity * thetaFFScaler;
    if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;
    lastGoalRotation = targetPose.getRotation();
    lastTime = Timer.getTimestamp();

    // Scale feedback velocities by input ff
    final double linearS = MathUtil.clamp(linearFF.get().getNorm() * 3.0, 0.0, 1.0);
    final double thetaS = MathUtil.clamp(Math.abs(omegaFF.getAsDouble()) * 3.0, 0.0, 1.0);
    driveVelocity =
        driveVelocity.interpolate(linearFF.get().times(Constants.DriveConstants.MAXIMUM_TRANSLATIONAL_VELOCITY.in(MetersPerSecond)), linearS);
    thetaVelocity =
        MathUtil.interpolate(
            thetaVelocity, omegaFF.getAsDouble() * Constants.DriveConstants.MAXIMUM_ROTATIONAL_VELOCITY.in(RadiansPerSecond), thetaS);
    ChassisSpeeds fieldVelocity = drivetrain.getLocalizer().getFieldRelativeSpeeds();
    Translation2d linearFieldVelocity =
        new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
    // Reset profiles if enough input or far enough away from setpoint
    if (linearS >= config.minLinearFFSReset
        || thetaS >= config.minThetaFFSReset
        || Math.abs(driveSetpoint.position - driveErrorAbs) >= config.minLinearErrorReset.in(Meters)
        || Math.abs(
                MathUtil.angleModulus(
                    currentPose.getRotation().getRadians()
                        - thetaController.getSetpoint().position))
            >= config.minThetaErrorReset.in(Radians)) {
      resetProfile();
    }

    var requestedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        driveVelocity.getX(),
        driveVelocity.getY(),
        thetaVelocity,
        currentPose.getRotation());

    driveRequest.VelocityX = requestedSpeeds.vxMetersPerSecond;
    driveRequest.VelocityY = requestedSpeeds.vyMetersPerSecond;
    driveRequest.RotationalRate = requestedSpeeds.omegaRadiansPerSecond;

    drivetrain.setControl(driveRequest);

    // Log data
    BreakerLog.log("DriveToPose/DistanceMeasured", driveErrorAbs);
    BreakerLog.log("DriveToPose/DistanceSetpoint", driveSetpoint.position);
    BreakerLog.log(
        "DriveToPose/VelocityMeasured",
        -linearFieldVelocity
                .toVector()
                .dot(targetPose.getTranslation().minus(currentPose.getTranslation()).toVector())
            / driveErrorAbs);
    BreakerLog.log("DriveToPose/VelocitySetpoint", driveSetpoint.velocity);
    BreakerLog.log("DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
    BreakerLog.log("DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
    BreakerLog.log(
        "DriveToPose/Setpoint",
        new Pose2d[] {
          new Pose2d(
              lastSetpointTranslation,
              Rotation2d.fromRadians(thetaController.getSetpoint().position))
        });
    BreakerLog.log("DriveToPose/Goal", new Pose2d[] {targetPose});
  }

  private void resetProfile() {
    Pose2d currentPose = robot.get();
    Pose2d targetPose = target.get();
    ChassisSpeeds fieldVelocity = drivetrain.getLocalizer().getFieldRelativeSpeeds();
    Translation2d linearFieldVelocity =
        new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);

    driveProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(config.driveMaxVelocity.in(MetersPerSecond), config.driveMaxAcceleration.in(MetersPerSecondPerSecond))
        );

    driveController.reset(currentPose.getTranslation().getDistance(targetPose.getTranslation()));
    thetaController.reset(
        currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
    lastSetpointTranslation = currentPose.getTranslation();
    lastSetpointVelocity = linearFieldVelocity;
    lastGoalRotation = targetPose.getRotation();
    lastTime = Timer.getTimestamp();
  }

  @Override
  public void end(boolean interrupted) {
    driveRequest.VelocityX = 0;
    driveRequest.VelocityY = 0;
    driveRequest.RotationalRate = 0;

    drivetrain.setControl(driveRequest);

    running = false;
    // Clear logs
    BreakerLog.log("DriveToPose/Setpoint", new Pose2d[] {});
    BreakerLog.log("DriveToPose/Goal", new Pose2d[] {});
  }

/** Checks if the robot pose is within the allowed drive and theta tolerances. */
public boolean withinTolerance(Distance driveTolerance, Angle thetaTolerance) {
    return running
        && Math.abs(driveErrorAbs) < driveTolerance.in(Units.Meter)
        && Math.abs(thetaErrorAbs) < thetaTolerance.in(Units.Radian);
}

  @Override
  public boolean isFinished() {
      return withinTolerance(config.driveTolerence, config.thetaTolerence);
  }
}