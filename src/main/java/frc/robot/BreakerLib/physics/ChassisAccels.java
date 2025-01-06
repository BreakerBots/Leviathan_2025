// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.physics;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Add your docs here. */
public class ChassisAccels {
    public final double axMetersPerSecondSquared, ayMetersPerSecondSquared, alphaRadiansPerSecondSquared;
    public ChassisAccels() {
        this(0.0, 0.0, 0.0);
    }

    public ChassisAccels(double axMetersPerSecondSquared, double ayMetersPerSecondSquared, double alphaRadiansPerSecondSquared) {
        this.axMetersPerSecondSquared = axMetersPerSecondSquared;
        this.ayMetersPerSecondSquared = ayMetersPerSecondSquared;
        this.alphaRadiansPerSecondSquared = alphaRadiansPerSecondSquared;
    }

    public static ChassisAccels fromDeltaSpeeds(ChassisSpeeds initialVels, ChassisSpeeds endVels, double dt) {
        ChassisSpeeds accels = endVels.minus(initialVels).times(dt);
        return new ChassisAccels(accels.vxMetersPerSecond, accels.vyMetersPerSecond, accels.omegaRadiansPerSecond);
    }

    public BreakerVector2 toLinearAccelerationVector() {
        return new BreakerVector2(axMetersPerSecondSquared, ayMetersPerSecondSquared);
    }

    public double getAlpha() {
        return alphaRadiansPerSecondSquared;
    }

    public double getX() {
        return axMetersPerSecondSquared;
    }

    public double getY() {
        return ayMetersPerSecondSquared;
    }
}
