// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.driverstation;
import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.BreakerLib.util.logging.BreakerLog;

/** Add your docs here. */
public interface BreakerInputStream extends DoubleSupplier {
    
  /**
   * Creates an input stream from another.
   *
   * @param base The base stream.
   * @return A new input stream.
   */
  public static BreakerInputStream  of(DoubleSupplier base) {
    return base::getAsDouble;
  }

  public static BreakerInputStream  hypot(BreakerInputStream  x, BreakerInputStream  y) {
    return () -> Math.hypot(x.get(), y.get());
  }

  public static BreakerInputStream  atan(BreakerInputStream  x, BreakerInputStream  y) {
    return () -> Math.atan2(x.get(), y.get());
  }

  /**
   * Shorthand to return a double value.
   *
   * @return The value from {@link #getAsDouble()}.
   */
  public default double get() {
    return getAsDouble();
  }

  /**
   * Maps the stream outputs by an operator.
   *
   * @param operator A function that takes in a double input and returns a double output.
   * @return A mapped stream.
   */
  public default BreakerInputStream map(DoubleUnaryOperator operator) {
    return () -> operator.applyAsDouble(getAsDouble());
  }

  /**
   * Scales the stream outputs by a factor.
   *
   * @param factor A supplier of scaling factors.
   * @return A scaled stream.
   */
  public default BreakerInputStream scale(DoubleSupplier factor) {
    return map(x -> x * factor.getAsDouble());
  }

  /**
   * Scales the stream outputs by a factor.
   *
   * @param factor A scaling factor.
   * @return A scaled stream.
   */
  public default BreakerInputStream scale(double factor) {
    return scale(() -> factor);
  }

  /**
   * Negates the stream outputs.
   *
   * @return A stream scaled by -1.
   */
  public default BreakerInputStream negate() {
    return scale(-1);
  }

  /**
   * Offsets the stream by a factor.
   *
   * @param factor A supplier of offset values.
   * @return An offset stream.
   */
  public default BreakerInputStream add(DoubleSupplier offset) {
    return map(x -> x + offset.getAsDouble());
  }

  /**
   * Offsets the stream by a factor.
   *
   * @param factor An offset.
   * @return An offset stream.
   */
  public default BreakerInputStream add(double factor) {
    return add(() -> factor);
  }

  /**
   * Raises the stream outputs to an exponent.
   *
   * @param exponent The exponent to raise them to.
   * @return An exponentiated stream.
   */
  public default BreakerInputStream pow(double exponent) {
    return map(x -> Math.pow(x, exponent));
  }

  /**
   * Raises the stream outputs to an exponent and keeps their original sign.
   *
   * @param exponent The exponent to raise them to.
   * @return An exponentiated stream.
   */
  public default BreakerInputStream signedPow(double exponent) {
    return map(x -> Math.copySign(Math.pow(x, exponent), x));
  }

  /**
   * Filters the stream's outputs by the provided {@link LinearFilter}.
   *
   * @param filter The linear filter to use.
   * @return A filtered stream.
   */
  public default BreakerInputStream filter(LinearFilter filter) {
    return map(filter::calculate);
  }

  /**
   * Deadbands the stream outputs by a minimum bound and scales them from 0 to a maximum bound.
   *
   * @param bound The lower bound to deadband with.
   * @param max The maximum value to scale with.
   * @return A deadbanded stream.
   */
  public default BreakerInputStream deadband(double deadband, double max) {
    return map(x -> MathUtil.applyDeadband(x, deadband, max));
  }

  /**
   * Clamps the stream outputs by a maximum bound.
   *
   * @param magnitude The upper bound to clamp with.
   * @return A clamped stream.
   */
  public default BreakerInputStream clamp(double magnitude) {
    return map(x -> MathUtil.clamp(x, -magnitude, magnitude));
  }

  /**
   * Rate limits the stream outputs by a specified rate.
   *
   * @param rate The rate in units / s.
   * @return A rate limited stream.
   */
  public default BreakerInputStream rateLimit(double rate) {
    var limiter = new SlewRateLimiter(rate);
    return map(x -> limiter.calculate(x));
  }

  /**
   * publishes the output of this stream to networktables every time it is polled.
   *
   * <p>A new stream is returned that is identical to this stream, but publishes its output to
   * networktables every time it is polled.
   *
   * @param key The NetworkTables key to publish to.
   * @return A stream with the same output as this one.
   */
  public default BreakerInputStream log(String key) {
    return () -> {
      double val = this.get();
      BreakerLog.log(key, val);
      return val;
    };
  }
}
