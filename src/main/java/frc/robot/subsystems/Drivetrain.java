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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BreakerLib.swerve.BreakerSwerveDrivetrain;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.BreakerLib.util.math.OdometryFusion;
import frc.robot.subsystems.vision.BreakerPoseEstimator;
public class Drivetrain extends BreakerSwerveDrivetrain {
 

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    super(DRIVETRAIN_CONSTANTS, FrontLeft, FrontRight, BackLeft, BackRight);
  }
  
  
  public static record DrivetrainKinematicLimits(LinearVelocity linearVelocity, LinearAcceleration linearAcceleration, AngularVelocity angularVelocity, AngularAcceleration angularAcceleration) {
    

    public DrivetrainKinematicLimits scale(double scalar) {
      return new DrivetrainKinematicLimits(linearVelocity.times(scalar), linearAcceleration.times(scalar), angularVelocity.times(scalar), angularAcceleration.times(scalar));
    }

    public DrivetrainKinematicLimits scale(double accelScalar, double velScalar) {
      return new DrivetrainKinematicLimits(linearVelocity.times(velScalar), linearAcceleration.times(accelScalar), angularVelocity.times(velScalar), angularAcceleration.times(accelScalar));
    }
  }

  private class CharacterizeWheelRadiusCommand extends Command {
    private double gyroStartAng = 0;
    private Timer timer;
    private Time testLength;
    private double[] moduleStartPoses;
    CharacterizeWheelRadiusCommand(Time testLength) {
      timer = new Timer();
      moduleStartPoses = new double[getModules().length];
    }

    @Override
    public void initialize() {
      gyroStartAng = getPigeon2().getYaw().getValueAsDouble();
      timer.reset();
      timer.start();
      for (int i = 0; i < getModules().length; i++) {
        moduleStartPoses[i] = Math.abs(getModule(i).getCachedPosition().distanceMeters);
      }
    }

    @Override
    public void execute() {
      // if () {
       
      // }
    }

    @Override
    public void end(boolean interrupted) {
      timer.stop();
      for (int i = 0; i < moduleStartPoses.length; i++) {
       double wheelDelta = Math.abs(getModule(i).getCachedPosition().distanceMeters) - moduleStartPoses[i];
       double moduleDist = getModuleLocations()[i].getNorm();
       double circ = 2 * Math.PI * moduleDist;
       double rots = circ/wheelDelta;
      //  (rots * 360) / 
      }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(testLength.in(Units.Second));
    }
    
  }
}
