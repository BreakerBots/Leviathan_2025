// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.driverstation;

import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.BreakerLib.physics.BreakerVector2;
import frc.robot.BreakerLib.util.logging.BreakerLog;

/** Add your docs here. */
public interface BreakerInputStream2d extends Supplier<BreakerVector2> {

  public static BreakerInputStream2d of(BreakerInputStream  x, BreakerInputStream  y) {
    return () -> new BreakerVector2(x.get(), y.get());
  }

  public default BreakerInputStream getX() {
    return () -> get().getX();
  }

  public default BreakerInputStream getY() {
    return () -> get().getY();
  }

  /**
   * Maps the stream outputs by an operator.
   *
   * @param operator A function that takes in a double input and returns a double output.
   * @return A mapped stream.
   */
  public default BreakerInputStream2d map(UnaryOperator<BreakerVector2> operator) {
    return () -> operator.apply(get());
  }

  /**
   * Maps the stream outputs by an operator.
   *
   * @param operator A function that takes in a double input and returns a double output.
   * @return A mapped stream.
   */
  public default BreakerInputStream2d mapToMagnitude(DoubleUnaryOperator operator) {
    return () -> {
        BreakerVector2 val = get();
        double mag = operator.applyAsDouble(val.getMagnitude());
        return new BreakerVector2(val.getAngle(), mag);
    };
  }

  /**
   * Scales the stream outputs by a factor.
   *
   * @param factor A supplier of scaling factors.
   * @return A scaled stream.
   */
  public default BreakerInputStream2d scale(DoubleSupplier factor) {
    return map(x -> x.times(factor.getAsDouble()));
  }

  /**
   * Scales the stream outputs by a factor.
   *
   * @param factor A scaling factor.
   * @return A scaled stream.
   */
  public default BreakerInputStream2d scale(double factor) {
    return scale(() -> factor);
  }

  /**
   * Negates the stream outputs.
   *
   * @return A stream scaled by -1.
   */
  public default BreakerInputStream2d negate() {
    return map(x -> x.unaryMinus());
  }

  public default BreakerInputStream getAngle() {
    return () -> get().getAngle().getRadians();
  }


  /**
   * Offsets the stream by a factor.
   *
   * @param factor A supplier of offset values.
   * @return An offset stream.
   */
  public default BreakerInputStream2d add(Supplier<BreakerVector2> offset) {
    return map(x -> x.plus(offset.get()));
  }

  /**
   * Offsets the stream by a factor.
   *
   * @param factor An offset.
   * @return An offset stream.
   */
  public default BreakerInputStream2d add(BreakerVector2 factor) {
    return add(() -> factor);
  }

  /**
   * Raises the stream outputs to an exponent.
   *
   * @param exponent The exponent to raise them to.
   * @return An exponentiated stream.
   */
  public default BreakerInputStream2d pow(double exponent) {
    return map(x -> x.pow(exponent));
  }

  /**
   * Filters the stream's outputs by the provided {@link LinearFilter}.
   *
   * @param filter The linear filter to use.
   * @return A filtered stream.
   */
  public default BreakerInputStream2d filter(LinearFilter filter) {
    return mapToMagnitude(filter::calculate);
  }

  /**
   * Deadbands the stream outputs by a minimum bound and scales them from 0 to a maximum bound.
   *
   * @param bound The lower bound to deadband with.
   * @param max The maximum value to scale with.
   * @return A deadbanded stream.
   */
  public default BreakerInputStream2d deadband(double deadband, double max) {

    return mapToMagnitude(x -> MathUtil.applyDeadband(x, deadband, max));
  }

  /**
   * Clamps the stream outputs by a maximum bound.
   *
   * @param magnitude The upper bound to clamp with.
   * @return A clamped stream.
   */
  public default BreakerInputStream2d clamp(double magnitude) {
    return map(x -> x.clampMagnitude(-magnitude, magnitude));
  }

  /**
   * Rate limits the stream outputs by a specified rate.
   *
   * @param rate The rate in units / s.
   * @return A rate limited stream.
   */
  public default BreakerInputStream2d rateLimit(double rate) {
    var limiter = new SlewRateLimiter(rate);
    return mapToMagnitude(x -> limiter.calculate(x));
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
public default BreakerInputStream2d log(String key) {
    return () -> {
      BreakerVector2 val = this.get();
      BreakerLog.log(key, val);
      return val;
    };
  }
}
