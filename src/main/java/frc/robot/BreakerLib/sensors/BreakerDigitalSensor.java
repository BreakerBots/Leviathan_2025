// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.sensors;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.ctre.phoenix6.hardware.CANdi;

public class BreakerDigitalSensor extends SubsystemBase implements BooleanSupplier, AutoCloseable {
  /** Creates a new BreakerBeamBreak. */
  private BooleanSupplier isTriggeredSupplier;
  private Runnable closeIO;
  private boolean triggeredOnHigh, prevRead, hasChanged;
  private final Timer timeSinceLastChange = new Timer();

  public BreakerDigitalSensor(BooleanSupplier isTriggered) {
    this(isTriggered, () -> {});
  }


  private BreakerDigitalSensor(BooleanSupplier isTriggered, Runnable closeIO) {
    isTriggeredSupplier = isTriggered;
    this.closeIO = closeIO;
    prevRead = !triggeredOnHigh;
    hasChanged = false;
  }

  public static BreakerDigitalSensor fromDIO(int inputChannelDIO, boolean triggeredOnTrue) {
    var dio = new DigitalInput(inputChannelDIO);
    return new BreakerDigitalSensor(() -> dio.get() == triggeredOnTrue, () -> dio.close());
  }

  public static BreakerDigitalSensor fromCANrange(CANrange canrange) {
    return new BreakerDigitalSensor(() -> canrange.getIsDetected().getValue(), () -> canrange.close());
  }

  public static BreakerDigitalSensor fromCANdiS1(CANdi candi) {
    return new BreakerDigitalSensor((BooleanSupplier)candi.getS1Closed().asSupplier(), () -> candi.close());
  }

  public static BreakerDigitalSensor fromCANdiS2(CANdi candi) {
    return new BreakerDigitalSensor((BooleanSupplier)candi.getS2Closed().asSupplier(), () -> candi.close());
  }


  public boolean isTriggered() {
    return isTriggeredSupplier.getAsBoolean();
  }

  public Trigger getAsTrigger() {
    return new Trigger(this);
  }

  public boolean hasChanged() {
    return hasChanged;
  }

  public double getTimeSinceLastChange() {
      return timeSinceLastChange.get();
  }

  @Override
  public void periodic() {
    hasChanged = prevRead != isTriggered();
    if (hasChanged) {
      timeSinceLastChange.reset();
      timeSinceLastChange.start();
    }
  }

  @Override
  public boolean getAsBoolean() {
    return isTriggeredSupplier.getAsBoolean();
  }

  @Override
  public void close() throws Exception {
    closeIO.run();
  }
}
