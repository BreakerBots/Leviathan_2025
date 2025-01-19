package frc.robot.commands;

import static com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue.BlueAlliance;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static java.lang.Math.PI;

import java.util.concurrent.ThreadPoolExecutor.CallerRunsPolicy;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BreakerLib.swerve.BreakerSwerveDrivetrain;
import frc.robot.BreakerLib.util.Localizer;
import frc.robot.Constants.DriveConstants;

public class AutoPilot {
  public static class AutoPilotConfig {
  }

  private AutoPilotConfig autoPilotConfig;
  private BreakerSwerveDrivetrain drivetrain;
  private Localizer localizer;

  // TODO: change max accelerations to something realistic
  private static final Constraints translationalConstraints = new Constraints(
      DriveConstants.MAXIMUM_TRANSLATIONAL_VELOCITY.in(MetersPerSecond), 10.0);
  private static final Constraints rotationalConstraints = new Constraints(
      DriveConstants.MAXIMUM_ROTATIONAL_VELOCITY.in(RadiansPerSecond), PI);

  // TODO: use a single translational controller?
  private ProfiledPIDController xController;
  private ProfiledPIDController yController;
  private ProfiledPIDController thetaController;

  public AutoPilot(AutoPilotConfig autoPilotConfig, BreakerSwerveDrivetrain drivetrain, Localizer localizer) {
    this(autoPilotConfig, drivetrain, localizer,
        new ProfiledPIDController(1, 0, 0, translationalConstraints),
        new ProfiledPIDController(1, 0, 0, translationalConstraints),
        new ProfiledPIDController(1, 0, 0, rotationalConstraints));
  }

  public AutoPilot(AutoPilotConfig autoPilotConfig, BreakerSwerveDrivetrain drivetrain, Localizer localizer,
      ProfiledPIDController xController, ProfiledPIDController yController, ProfiledPIDController thetaController) {
    this.autoPilotConfig = autoPilotConfig;
    this.drivetrain = drivetrain;
    this.localizer = localizer;
    this.xController = xController;
    this.yController = yController;
    this.thetaController = thetaController;
    this.thetaController.enableContinuousInput(0, Degrees.of(360.0).in(Radians));
  }

  private class NavToPose extends Command {
    private final Pose2d goal;

    // TODO: position and velocity tolerance
    private final Pose2d tolerance;

    private Pose2d error = Pose2d.kZero;
    private SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric();

    // TODO: take: xy goal, v/a * xyt constraints, xyt * p/v tolerance
    public NavToPose(Pose2d goal, Pose2d tolerance) {
      this.goal = goal;
      this.tolerance = tolerance;
      request.ForwardPerspective = BlueAlliance;
      addRequirements(drivetrain);
    }

    /**
     * Returns true if the pose error is within tolerance of the goal.
     *
     * @return True if the pose error is within tolerance of the goal.
     */
    private boolean atGoal() {
      final var eTranslate = error.getTranslation();
      final var eRotate = error.getRotation();
      final var tolTranslate = tolerance.getTranslation();
      final var tolRotate = tolerance.getRotation();
      return Math.abs(eTranslate.getX()) < tolTranslate.getX()
          && Math.abs(eTranslate.getY()) < tolTranslate.getY()
          && Math.abs(eRotate.getRadians()) < tolRotate.getRadians();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      // Read current pose from the localizer.
      final var currentPose = localizer.getPose();
      final var currentChassisSpeeds = localizer.getSpeeds();
      final var x = currentPose.getX();
      final var y = currentPose.getY();
      final var theta = currentPose.getRotation().getRadians();

      // If this is the first run, then we need to reset the controllers to the
      // current pose's position and heading.
      xController.reset(x, currentChassisSpeeds.vxMetersPerSecond);
      yController.reset(y, currentChassisSpeeds.vyMetersPerSecond);
      thetaController.reset(theta, currentChassisSpeeds.omegaRadiansPerSecond);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      // TODO: logging

      // Read current pose from the localizer.
      final var currentPose = localizer.getPose();
      final var x = currentPose.getX();
      final var y = currentPose.getY();
      final var theta = currentPose.getRotation().getRadians();

      // Calculate feedback velocities (based on position error).
      double xFeedback = xController.calculate(x, goal.getX());
      double yFeedback = yController.calculate(y, goal.getY());
      double thetaFeedback = thetaController.calculate(theta, goal.getRotation().getRadians());

      // Get feedforward velocities.
      double xFF = xController.getSetpoint().velocity;
      double yFF = yController.getSetpoint().velocity;
      double thetaFF = thetaController.getSetpoint().velocity;

      // Return next output.
      request.VelocityX = xFeedback + xFF;
      request.VelocityY = yFeedback + yFF;
      request.RotationalRate = thetaFeedback + thetaFF;
      drivetrain.setControl(request);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      request.VelocityX = 0;
      request.VelocityY = 0;
      request.RotationalRate = 0;
      drivetrain.setControl(request);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
  }
}
