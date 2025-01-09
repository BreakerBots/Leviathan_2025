// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.physics;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

    public static ChassisAccels fromRobotRelativeAccels(ChassisAccels accels, Rotation2d robotAngle) {
        return fromRobotRelativeAccels(accels.axMetersPerSecondSquared, accels.ayMetersPerSecondSquared, accels.alphaRadiansPerSecondSquared, robotAngle);
    }

    public static ChassisAccels fromRobotRelativeAccels(
        double axMetersPerSecondSquared, 
        double ayMetersPerSecondSquared, 
        double alphaRadiansPerSecondSquared,
        Rotation2d robotAngle) {
        // CCW rotation out of chassis frame
        var rotated = new Translation2d(axMetersPerSecondSquared, ayMetersPerSecondSquared).rotateBy(robotAngle);
        return new ChassisAccels(rotated.getX(), rotated.getY(), alphaRadiansPerSecondSquared);
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
