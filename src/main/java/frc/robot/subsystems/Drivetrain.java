// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.HolonomicSlewRateLimiter;
import frc.robot.BreakerLib.driverstation.BreakerInputStream;
import frc.robot.BreakerLib.driverstation.BreakerInputStream2d;
import frc.robot.BreakerLib.swerve.BreakerSwerveDrivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.*;
public class Drivetrain extends BreakerSwerveDrivetrain {

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    super(DRIVETRAIN_CONSTANTS, FrontLeft, FrontRight, BackLeft, BackRight);
  }

  public static record DrivetrainKinimaticLimits(LinearVelocity linearVelocity, LinearAcceleration linearAcceleration, AngularVelocity angularVelocity, AngularAcceleration angularAcceleration) {
    

    public static class KinimaticLimitInterpolator implements Interpolator<DrivetrainKinimaticLimits> {

      public KinimaticLimitInterpolator() {}

      @Override
      public DrivetrainKinimaticLimits interpolate(DrivetrainKinimaticLimits startValue,
          DrivetrainKinimaticLimits endValue, double t) {
        double lv = MathUtil.interpolate(startValue.linearVelocity().in(MetersPerSecond), endValue.linearVelocity().in(MetersPerSecond), t);
        double la = MathUtil.interpolate(startValue.linearAcceleration().in(MetersPerSecondPerSecond), endValue.linearAcceleration().in(MetersPerSecondPerSecond), t);
        double av = MathUtil.interpolate(startValue.angularVelocity().in(RadiansPerSecond), endValue.angularVelocity().in(RadiansPerSecond), t);
        double aa = MathUtil.interpolate(startValue.angularAcceleration().in(RadiansPerSecondPerSecond), endValue.angularAcceleration().in(RadiansPerSecondPerSecond), t);
        return new DrivetrainKinimaticLimits(MetersPerSecond.of(lv), MetersPerSecondPerSecond.of(la), RadiansPerSecond.of(av), RadiansPerSecondPerSecond.of(aa));
      }

    }

    public static class KinimaticLimitInverseInterpolator implements InverseInterpolator<DrivetrainKinimaticLimits> {

      public KinimaticLimitInverseInterpolator() {}

      @Override
      public double inverseInterpolate(DrivetrainKinimaticLimits startValue, DrivetrainKinimaticLimits endValue,
          DrivetrainKinimaticLimits q) {
            return MathUtil.inverseInterpolate(startValue.linearVelocity().in(MetersPerSecond), endValue.linearVelocity().in(MetersPerSecond), q.linearVelocity().in(MetersPerSecond));
      }

    }
  }
}
