// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.BreakerLib.swerve.BreakerSwerveDrivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.units.measure.*;
public class Drivetrain extends BreakerSwerveDrivetrain {
  /** Creates a new Drivetrain. */
  public Drivetrain() {
    super(DRIVETRAIN_CONSTANTS, FrontLeft, FrontRight, BackLeft, BackRight);
  }

  public static record DrivetrainKinimaticLimits(LinearVelocity linearVelocity, LinearAcceleration linearAcceleration, AngularVelocity angularVelocity, AngularAcceleration rotationalAcceleration) {
    

    public static class KinimaticLimitInterpolator implements Interpolator<DrivetrainKinimaticLimits> {\

      public KinimaticLimitInterpolator() {}

      @Override
      public DrivetrainKinimaticLimits interpolate(DrivetrainKinimaticLimits startValue,
          DrivetrainKinimaticLimits endValue, double t) {
        double lv = MathUtil.interpolate(startValue.linearVelocity().in(MetersPerSecond), endValue.linearVelocity().in(MetersPerSecond), t);
        double la = MathUtil.interpolate(startValue.linearAcceleration().in(MetersPerSecondPerSecond), endValue.linearAcceleration().in(MetersPerSecondPerSecond), t);
        double av = MathUtil.interpolate(startValue.angularVelocity().in(RadiansPerSecondPerSecond), endValue.angularVelocity().in(RadiansPerSecondPerSecond), t);
        double aa = MathUtil.interpolate(startValue.linearAcceleration().in(MetersPerSecondPerSecond), endValue.linearAcceleration().in(MetersPerSecondPerSecond), t);
      }

    }
  }
}
