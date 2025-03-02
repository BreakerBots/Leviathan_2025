package frc.robot.commands;

import static com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue.BlueAlliance;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotation;

import java.io.ObjectInputFilter.Config;
import java.lang.annotation.ElementType;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BreakerLib.swerve.BreakerSwerveDrivetrain;
import frc.robot.BreakerLib.util.Localizer;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.Constants.ApriltagVisionConstants;
import frc.robot.subsystems.vision.ApriltagVision;
import frc.robot.subsystems.vision.ApriltagVision.EstimationType;

public class AutoPilot {
  private BreakerSwerveDrivetrain drivetrain;
  private Localizer localizer;
  private ApriltagVision vision;

  public AutoPilot(BreakerSwerveDrivetrain drivetrain, ApriltagVision apriltagVision, Localizer localizer) {
    this.drivetrain = drivetrain;
    this.localizer = localizer;
    vision = apriltagVision;
  }

  public record ProfiledPIDControllerConfig(double kP, double kI, double kD, TrapezoidProfile.Constraints constraints) {
    public ProfiledPIDControllerConfig withConstraints(TrapezoidProfile.Constraints newConstraints) {
      return new ProfiledPIDControllerConfig(kP, kI, kD, newConstraints);
    }
  }

  public record NavToPoseConfig(
      boolean allowTrigPoseEst,
      Pose2d positionTolerance,
      ChassisSpeeds velocityTolerance,
      ProfiledPIDControllerConfig posConfig,
      ProfiledPIDControllerConfig thetaConfig) {
  }

  public Command navigateToPose(Pose2d goal, NavToPoseConfig config) {
    return new NavToPose(goal, config);
  }

  private class NavToPose extends Command {
    private final Pose2d goal;
    private final Pose2d positionTolerance;
    private final ChassisSpeeds velocityTolerance;

    private double posError = 0;
    private double rotError = 0;
    private SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric();

    private ProfiledPIDController posController;
    private ProfiledPIDController thetaController;

    private boolean allowTrigPoseEst;

    private ProfiledPIDController fromConfig(ProfiledPIDControllerConfig config) {
      return new ProfiledPIDController(config.kP, config.kI, config.kD, config.constraints);
    }

    public NavToPose(Pose2d goal, NavToPoseConfig config) {
      this.goal = goal;
      this.positionTolerance = config.positionTolerance;
      this.velocityTolerance = config.velocityTolerance;
      request.ForwardPerspective = BlueAlliance;
      posController = fromConfig(config.posConfig);
      thetaController = fromConfig(config.thetaConfig);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);
      allowTrigPoseEst = config.allowTrigPoseEst;
      addRequirements(drivetrain);
    }

    /**
     * Updates the constraints of the position controller.
     * @param posConstraints The new constraints for the position controller.
     */
    public void updateConstraints(TrapezoidProfile.Constraints posConstraints) {
      posController.setConstraints(posConstraints);
    }

    /**
     * Returns true if the pose error is within tolerance of the goal.
     *
     * @return True if the pose error is within tolerance of the goal.
     */
    private boolean atGoal() {
      final var eTranslate = posError;
      final var eRotate = rotError;
      final var ptolTranslate = positionTolerance.getTranslation();
      final var ptolRotate = positionTolerance.getRotation();
      final var speeds = localizer.getSpeeds();

      // Check if position and velocity are within tolerances. Goal velocity is 0.
      final var translationAtGoal = Math.abs(eTranslate) < ptolTranslate.getNorm();
      final var rotationAtGoal = Math.abs(eRotate) < ptolRotate.getRadians();
      final var xSpeedAtGoal = Math.abs(speeds.vxMetersPerSecond) < velocityTolerance.vxMetersPerSecond;
      final var ySpeedAtGoal = Math.abs(speeds.vyMetersPerSecond) < velocityTolerance.vyMetersPerSecond;
      final var rotSpeedAtGoal = Math.abs(speeds.omegaRadiansPerSecond) < velocityTolerance.omegaRadiansPerSecond;
      BreakerLog.log("NavToPose/translationAtGoal", translationAtGoal);
      BreakerLog.log("NavToPose/rotationAtGoal", rotationAtGoal);
      BreakerLog.log("NavToPose/xSpeedAtGoal", xSpeedAtGoal);
      BreakerLog.log("NavToPose/ySpeedAtGoal", ySpeedAtGoal);
      BreakerLog.log("NavToPose/rotSpeedAtGoal", rotSpeedAtGoal);
      return translationAtGoal && rotationAtGoal && xSpeedAtGoal && ySpeedAtGoal && rotationAtGoal;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      // Read current pose from the localizer.
      final var currentPose = localizer.getPose();
      final var currentChassisSpeeds = localizer.getSpeeds();
      final var posDistance = currentPose.getTranslation().getDistance(goal.getTranslation());
      final var theta = currentPose.getRotation().getRadians();

      // If this is the first run, then we need to reset the controllers to the
      // current pose's position and heading.
      posController.reset(posDistance, new Translation2d(currentChassisSpeeds.vxMetersPerSecond, currentChassisSpeeds.vyMetersPerSecond).getNorm());
      thetaController.reset(theta, currentChassisSpeeds.omegaRadiansPerSecond);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      // Read current pose from the localizer.
      final var currentPose = localizer.getPose();
      final var posDistance = currentPose.getTranslation().getDistance(goal.getTranslation());
      final var theta = currentPose.getRotation().getRadians();

      // Calculate feedback velocities (based on position error).
      double posFB = posController.calculate(posDistance, 0);
      double thetaFB = thetaController.calculate(theta, goal.getRotation().getRadians());

      // Get feedforward velocities.
      double posFF = posController.getSetpoint().velocity;
      double thetaFF = thetaController.getSetpoint().velocity;

      // Return next output.
      final var posVel = posFB + posFF;
      request.VelocityX = posVel * Math.cos(goal.relativeTo(currentPose).getRotation().getRadians());
      request.VelocityY = posVel * Math.sin(goal.relativeTo(currentPose).getRotation().getRadians());
      request.RotationalRate = thetaFB + thetaFF;
      drivetrain.setControl(request);

      double ePos = goal.getTranslation().getDistance(currentPose.getTranslation());
      Rotation2d et = goal.getRotation().minus(currentPose.getRotation());

      if (allowTrigPoseEst && posError < ApriltagVisionConstants.kMaxTrigSolveTagDist.minus(Meters.of(0.5)).in(Meters)) {
        vision.setEstimationType(EstimationType.TRIG);
      } else {
        vision.setEstimationType(EstimationType.PNP);
      }

      posError = ePos;
      rotError = et.getRadians();

      BreakerLog.log("NavToPose/CurrentPose", currentPose);
      BreakerLog.log("NavToPose/posFB", posFB);
      BreakerLog.log("NavToPose/thetaFB", thetaFB);
      BreakerLog.log("NavToPose/posFF", posFF);
      BreakerLog.log("NavToPose/thetaFF", thetaFF);
      BreakerLog.log("NavToPose/xRequest", request.VelocityX);
      BreakerLog.log("NavToPose/yRequest", request.VelocityY);
      BreakerLog.log("NavToPose/thetaRequest", request.RotationalRate);
      BreakerLog.log("NavToPose/Goal", goal);
      BreakerLog.log("NavToPose/PosError", posError);
      BreakerLog.log("NavToPose/RotError", rotError);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      request.VelocityX = 0;
      request.VelocityY = 0;
      request.RotationalRate = 0;
      drivetrain.setControl(request);
      vision.setEstimationType(EstimationType.PNP);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return atGoal();
    }
  }
}
