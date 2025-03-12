package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.pathplanner.lib.config.PIDConstants;
public class DriveToPose extends Command {

  private final Drivetrain drivetrain;
  private final Supplier<Pose2d> target;

  private final ProfiledPIDController driveController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));

  private Translation2d lastSetpointTranslation = new Translation2d();
  private double driveErrorAbs = 0.0;
  private double thetaErrorAbs = 0.0;
  private boolean running = false;
  private Supplier<Pose2d> robot;

  private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
  private DoubleSupplier omegaFF = () -> 0.0;

  private final RobotCentric driveRequest;

  private final NavToPoseConfig config;

  public DriveToPose(Drivetrain drivetrain, Supplier<Pose2d> target) {
    this(drivetrain, target, new NavToPoseConfig(
        Meters.of(0.01), 
        Degrees.of(1.0), 
        MetersPerSecond.of(3.0), 
        DegreesPerSecond.of(360), 
        MetersPerSecondPerSecond.of(3.0), 
        RadiansPerSecondPerSecond.of(8), 
        Meters.of(0.05), 
        Meters.of(0.1), 
        new PIDConstants(0.8, 0.0, 0.0), 
        new PIDConstants(4, 0.0, 0.0)));
  }

  public DriveToPose(Drivetrain drivetrain, Supplier<Pose2d> target, NavToPoseConfig config) {
    this.drivetrain = drivetrain;
    this.target = target;

    // Enable continuous input for theta controller
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    driveRequest = new RobotCentric();
    driveRequest.DriveRequestType = DriveRequestType.Velocity;
    this.config = config;
    robot = drivetrain.getLocalizer()::getPose;

    driveController.setPID(config.drivePID.kP, config.drivePID.kI, config.drivePID.kD);
    thetaController.setPID(config.thetaPID.kP, config.thetaPID.kI, config.thetaPID.kD);
    BreakerLog.log("maxacc", config.driveMaxAcceleration);
    driveController.setConstraints(new TrapezoidProfile.Constraints(config.driveMaxVelocity.in(MetersPerSecond), config.driveMaxAcceleration.in(MetersPerSecondPerSecond)));
    thetaController.setConstraints(new TrapezoidProfile.Constraints(config.thetaMaxVelocity.in(RadiansPerSecond), config.thetaMaxAcceleration.in(RadiansPerSecondPerSecond)));

    addRequirements(drivetrain);
  }

  public DriveToPose(Drivetrain drivetrain, Supplier<Pose2d> target, Supplier<Pose2d> robot, NavToPoseConfig config) {
    this(drivetrain, target, config);
    this.robot = robot;
  }

  public DriveToPose(
      Drivetrain drivetrain,
      Supplier<Pose2d> target,
      Supplier<Pose2d> robot,
      Supplier<Translation2d> linearFF,
      DoubleSupplier omegaFF,
      NavToPoseConfig config
      ) {
    this(drivetrain, target, robot, config);
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

      Distance ffMinRadius,
      Distance ffMaxRadius,
    
      PIDConstants drivePID,
      PIDConstants thetaPID) {
  }

  @Override
  public void initialize() {
    Pose2d currentPose = robot.get();
    ChassisSpeeds fieldVelocity = drivetrain.getLocalizer().getFieldRelativeSpeeds();
    Translation2d linearFieldVelocity =
        new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
    driveController.reset(
        currentPose.getTranslation().getDistance(target.get().getTranslation()),
        Math.min(
            0.0,
            -linearFieldVelocity
                .rotateBy(
                    target
                        .get()
                        .getTranslation()
                        .minus(currentPose.getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    thetaController.reset(
        currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
    lastSetpointTranslation = currentPose.getTranslation();
  }

  @Override
  public void execute() {
    running = true;

    // Get current pose and target pose
    Pose2d currentPose = robot.get();
    Pose2d targetPose = target.get();

    // Calculate drive speed
    double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    double ffScaler =
        MathUtil.clamp(
            (currentDistance - config.ffMinRadius.in(Meters)) / (config.ffMaxRadius.in(Meters) - config.ffMinRadius.in(Meters)),
            0.0,
            1.0);
    driveErrorAbs = currentDistance;
    driveController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        driveController.getSetpoint().velocity);
        
    double calc = driveController.calculate(driveErrorAbs, 0.0);
    
    double driveVelocityScalar =
        driveController.getSetpoint().velocity * ffScaler
            + calc;
    if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;
    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(new Transform2d(driveController.getSetpoint().position, 0.0, new Rotation2d()))
            .getTranslation();

    // Calculate theta speed
    double thetaVelocity =
        thetaController.getSetpoint().velocity * ffScaler
            + thetaController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

    Translation2d driveVelocity =
        new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(new Transform2d(driveVelocityScalar, 0.0, new Rotation2d()))
            .getTranslation();
    
    // Scale feedback velocities by input ff
    final double linearS = linearFF.get().getNorm() * 3.0;
    final double thetaS = Math.abs(omegaFF.getAsDouble()) * 3.0;
    driveVelocity =
        driveVelocity.interpolate(linearFF.get().times(DriveConstants.MAXIMUM_TRANSLATIONAL_VELOCITY.in(MetersPerSecond)), linearS);
    thetaVelocity =
        MathUtil.interpolate(
            thetaVelocity, omegaFF.getAsDouble() * DriveConstants.MAXIMUM_ROTATIONAL_VELOCITY.in(RadiansPerSecond), thetaS);

    // Command speeds

    ChassisSpeeds requestedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation());


    driveRequest.VelocityX = requestedSpeeds.vxMetersPerSecond;
    driveRequest.VelocityY = requestedSpeeds.vyMetersPerSecond;
    driveRequest.RotationalRate = requestedSpeeds.omegaRadiansPerSecond;

    drivetrain.setControl(driveRequest);

    // Log data
    BreakerLog.log("DriveToPose/DistanceMeasured", currentDistance);
    BreakerLog.log("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
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

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    running = false;
    // Clear logs
    BreakerLog.log("DriveToPose/Setpoint", new Pose2d[] {});
    BreakerLog.log("DriveToPose/Goal", new Pose2d[] {});
  }

  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    return running && driveController.atGoal() && thetaController.atGoal();
  }

  /** Checks if the robot pose is within the allowed drive and theta tolerances. */
  public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return running
        && Math.abs(driveErrorAbs) < driveTolerance
        && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
  }
}