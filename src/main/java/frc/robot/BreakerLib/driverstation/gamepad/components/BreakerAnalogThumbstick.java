// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.driverstation.gamepad.components;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.BreakerLib.driverstation.BreakerInputStream;
import frc.robot.BreakerLib.driverstation.BreakerInputStream2d;
import frc.robot.BreakerLib.physics.BreakerVector2;

/** Class which represents an analog HID thumbstick. */
public class BreakerAnalogThumbstick implements BreakerInputStream2d  {
    private GenericHID hid;
    private int xAxisPort, yAxisPort;
    private boolean invertX, invertY;
    private BreakerInputStream streamX;
    private BreakerInputStream streamY;
    

    /**
     * Constructs an analog thumbstick with desired inverts.
     * 
     * @param hid Controller.
     * @param xAxisPort X-axis port #.
     * @param yAxisPort Y-axis port #.
     */
    public BreakerAnalogThumbstick(GenericHID hid, int xAxisPort, int yAxisPort) {
        this(hid, xAxisPort, false, yAxisPort, false);
    }

    /**
     * Constructs an analog thumbstick with desired inverts.
     * 
     * @param hid Controller.
     * @param xAxisPort X-axis port #.
     * @param invertX Invert X-axis.
     * @param yAxisPort Y-axis port #.
     * @param invertY Invert Y-axis.
     */
    public BreakerAnalogThumbstick(GenericHID hid, int xAxisPort, boolean invertX, int yAxisPort, boolean invertY) {
        this.hid = hid;
        this.xAxisPort = xAxisPort;
        this.yAxisPort = yAxisPort;
        this.invertX = invertX;
        this.invertY = invertY;

        }

    /** @return Raw X-axis value. */
    public double getRawX() {
        return hid.getRawAxis(xAxisPort);
    }

    /** @return Raw Y-axis value. */
    public double getRawY() {
        return hid.getRawAxis(yAxisPort);
    }

    public BreakerVector2 getRawVector() {
        return new BreakerVector2(getRawX(), getRawY());
    }

    /** @return If stick inputs outside of the deadband are detected. */
    public boolean isActive() {
        return (!MathUtil.isNear(getY().get(), 0.0, 1e-5) || !MathUtil.isNear(getY().get(), 0.0, 1e-5));
    }

    @Override
    public BreakerVector2 get() {
        return new BreakerVector2(getRawX() * (invertX ? -1 : 1), getRawY() * (invertY ? -1 : 1));
    }
}
