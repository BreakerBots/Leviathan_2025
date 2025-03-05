// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.BreakerLib.swerve.BreakerSwerveDrivetrain;
import frc.robot.BreakerLib.util.math.OdometryFusion;
import frc.robot.subsystems.vision.BreakerPoseEstimator;
public class Drivetrain extends BreakerSwerveDrivetrain {
  private SwerveDriveOdometry odometry;
 

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    super(DRIVETRAIN_CONSTANTS, FrontLeft, FrontRight, BackLeft, BackRight);
    odometry = new SwerveDriveOdometry(getKinematics(), getState().RawHeading, getState().ModulePositions, new Pose2d());
  }

  
  private void updatePoseEstimation(SwerveDriveState state) {
    odometryFusion.update(state.RawHeading, state.ModulePositions);
  }

  public OdometryFusion<SwerveModulePosition[]> getOdometryFusion() {
      return odometryFusion;
  }

  public BreakerPoseEstimator<SwerveModulePosition[]> getVisionFilter() {
      return visionFilter;
  }
  
  
  
  
  
  public static record DrivetrainKinematicLimits(LinearVelocity linearVelocity, LinearAcceleration linearAcceleration, AngularVelocity angularVelocity, AngularAcceleration angularAcceleration) {
    

    public DrivetrainKinematicLimits scale(double scalar) {
      return new DrivetrainKinematicLimits(linearVelocity.times(scalar), linearAcceleration.times(scalar), angularVelocity.times(scalar), angularAcceleration.times(scalar));
    }

    public DrivetrainKinematicLimits scale(double accelScalar, double velScalar) {
      return new DrivetrainKinematicLimits(linearVelocity.times(velScalar), linearAcceleration.times(accelScalar), angularVelocity.times(velScalar), angularAcceleration.times(accelScalar));
    }
  }
}
