// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.sensors;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BreakerBeamBreak extends SubsystemBase implements BooleanSupplier, AutoCloseable {
  /** Creates a new BreakerBeamBreak. */
  private BooleanSupplier isBrokenSupplier;
  private Runnable closeIO;
  private boolean brokenOnTrue, prevRead, hasChanged;
  private final Timer timeSinceLastChange = new Timer();

  public BreakerBeamBreak(BooleanSupplier isBroken) {
    this(isBroken, () -> {});
  }


  private BreakerBeamBreak(BooleanSupplier isBroken, Runnable closeIO) {
    isBrokenSupplier = isBroken;
    this.closeIO = closeIO;
    prevRead = !brokenOnTrue;
    hasChanged = false;
  }

  public static BreakerBeamBreak fromDIO(int inputChannelDIO, boolean brokenOnTrue) {
    var dio = new DigitalInput(inputChannelDIO);
    return new BreakerBeamBreak(() -> dio.get() == brokenOnTrue, () -> dio.close());
  }

  public static BreakerBeamBreak fromCANrange(CANrange canrange) {
    return new BreakerBeamBreak(() -> canrange.getIsDetected().getValue(), () -> canrange.close());
  }

  public boolean isBroken() {
    return isBrokenSupplier.getAsBoolean();
  }

  public boolean hasChanged() {
    return hasChanged;
  }

  public double getTimeSinceLastChange() {
      return timeSinceLastChange.get();
  }

  @Override
  public void periodic() {
    hasChanged = prevRead != isBroken();
    if (hasChanged) {
      timeSinceLastChange.reset();
      timeSinceLastChange.start();
    }
  }

  @Override
  public boolean getAsBoolean() {
    return isBrokenSupplier.getAsBoolean();
  }

  @Override
  public void close() throws Exception {
    closeIO.run();
  }
}
