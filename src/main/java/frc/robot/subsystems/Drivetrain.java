// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static frc.robot.Constants.DriveConstants.BackLeft;
import static frc.robot.Constants.DriveConstants.BackRight;
import static frc.robot.Constants.DriveConstants.DRIVETRAIN_CONSTANTS;
import static frc.robot.Constants.DriveConstants.FrontLeft;
import static frc.robot.Constants.DriveConstants.FrontRight;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.BreakerLib.swerve.BreakerSwerveDrivetrain;
public class Drivetrain extends BreakerSwerveDrivetrain {

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    super(DRIVETRAIN_CONSTANTS, FrontLeft, FrontRight, BackLeft, BackRight);
  }

  public static record DrivetrainKinematicLimits(LinearVelocity linearVelocity, LinearAcceleration linearAcceleration, AngularVelocity angularVelocity, AngularAcceleration angularAcceleration) {
    

    public static class KinimaticLimitInterpolator implements Interpolator<DrivetrainKinematicLimits> {

      public KinimaticLimitInterpolator() {}

      @Override
      public DrivetrainKinematicLimits interpolate(DrivetrainKinematicLimits startValue,
          DrivetrainKinematicLimits endValue, double t) {
        double lv = MathUtil.interpolate(startValue.linearVelocity().in(MetersPerSecond), endValue.linearVelocity().in(MetersPerSecond), t);
        double la = MathUtil.interpolate(startValue.linearAcceleration().in(MetersPerSecondPerSecond), endValue.linearAcceleration().in(MetersPerSecondPerSecond), t);
        double av = MathUtil.interpolate(startValue.angularVelocity().in(RadiansPerSecond), endValue.angularVelocity().in(RadiansPerSecond), t);
        double aa = MathUtil.interpolate(startValue.angularAcceleration().in(RadiansPerSecondPerSecond), endValue.angularAcceleration().in(RadiansPerSecondPerSecond), t);
        return new DrivetrainKinematicLimits(MetersPerSecond.of(lv), MetersPerSecondPerSecond.of(la), RadiansPerSecond.of(av), RadiansPerSecondPerSecond.of(aa));
      }

    }

    public static class KinimaticLimitInverseInterpolator implements InverseInterpolator<DrivetrainKinematicLimits> {

      public KinimaticLimitInverseInterpolator() {}

      @Override
      public double inverseInterpolate(DrivetrainKinematicLimits startValue, DrivetrainKinematicLimits endValue,
          DrivetrainKinematicLimits q) {
            return MathUtil.inverseInterpolate(startValue.linearVelocity().in(MetersPerSecond), endValue.linearVelocity().in(MetersPerSecond), q.linearVelocity().in(MetersPerSecond));
      }

    }
  }
}
