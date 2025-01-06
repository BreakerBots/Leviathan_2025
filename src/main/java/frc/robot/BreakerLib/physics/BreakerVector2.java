// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.physics;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import frc.robot.BreakerLib.util.math.interpolation.BreakerInterpolable;

/**
 * represents a 2 dimentional vector, a uantity with bolth magnatude and direction. Here representd as a total magnatude, angular direction, 
 * x manatude component, and y magnatude component
 */
public class BreakerVector2 implements BreakerInterpolable<BreakerVector2> {
    private final Rotation2d vectorRotation;
    private final double magnitude, x, y;


     /** creates an empty BreakerVector2 with 0's for all values */
     public BreakerVector2() {
        x = 0;
           y = 0;
           vectorRotation = Rotation2d.fromDegrees(0);
           magnitude = 0;
       }

    /**
     * creates a new BreakerVector2 from the magnatudes of the vector's X and Y
     * components
     */
    public BreakerVector2(double x, double y) {
        this.x = x;
        this.y = y;
        vectorRotation = new Rotation2d(Math.atan2(y, x));
        magnitude = Math.sqrt((x * x) + (y * y));
    }

    /**
     * creates a new BreakerVector2 from the vectors Magnatude and the vectors angle
     * in the Yaw angular axis
     */
    public BreakerVector2(Rotation2d vectorRotation, double magnatude) {
        this(magnatude * vectorRotation.getCos(),
                magnatude * vectorRotation.getSin(), magnatude, vectorRotation);
    }

    /** converts an instance of WPILib's Translation2d class into a vector.
     *  This exists because of the Tranlation2d classes suppport of various vector opperations */
    public BreakerVector2(Translation2d translationToVectorize) {
       this(translationToVectorize.getX(), translationToVectorize.getY(), translationToVectorize.getNorm(), translationToVectorize.getAngle());
    }

    private BreakerVector2(double x, double y, double magnitude, Rotation2d vectorRotation) {
        this.x = x;
        this.y = y;
        this.magnitude = magnitude;
        this.vectorRotation = vectorRotation;
    }

    public BreakerVector2(Vector<N2> vector) {
        this(vector.get(0), vector.get(1));
    }
    public static BreakerVector2 fromChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        return new BreakerVector2(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    }

    
    /** 
     * @return double
     */
    public double getMagnitude() {
        return magnitude;
    }

    
    /** 
     * @return Rotation2d
     */
    public Rotation2d getAngle() {
        return vectorRotation;
    }

    
    /** 
     * @return double
     */
    public double getX() {
        return x;
    }

    
    /** 
     * @return double
     */
    public double getY() {
        return y;
    }

    public BreakerVector2 clampMagnitude(double low, double high) {
        double clampedMag = MathUtil.clamp(magnitude, low, high);
        return new BreakerVector2(vectorRotation, clampedMag);
    }

    public BreakerVector2 abs() {
        return new BreakerVector2(Math.abs(x), Math.abs(y));
    }

    
    /** 
     * @param outher
     * @return BreakerVector2
     */
    public BreakerVector2 plus(BreakerVector2 outher) {
        return new BreakerVector2(x + outher.x, y + outher.y);
    }

    
    /** 
     * @param outher
     * @return BreakerVector2
     */
    public BreakerVector2 minus(BreakerVector2 outher) {
        return new BreakerVector2(x - outher.x, y - outher.y);
    }

    
    /** 
     * @return BreakerVector2
     */
    public BreakerVector2 unaryMinus()  {
        return new BreakerVector2(-x, -y);
    }

    
    /** 
     * @param scalar
     * @return BreakerVector2
     */
    public BreakerVector2 times(double scalar) {
        return new BreakerVector2(x * scalar, y * scalar);
    }

    
    /** 
     * @param scalar
     * @return BreakerVector2
     */
    public BreakerVector2 div(double scalar) {
        return new BreakerVector2(x / scalar,  y / scalar);
    }

    public BreakerVector2 pow(double exponent)  {
        return new BreakerVector2(vectorRotation, Math.pow(magnitude, exponent));
    }
    
    
    /** 
     * @return Translation2d
     */
    public Translation2d getAsTranslation() {
        return new Translation2d(x, y);
    }

    public Vector<N2> getVectorWPI() {
        return VecBuilder.fill(x, y);
    }
    
    /** 
     * @param rotation
     * @return BreakerVector2
     */
    public BreakerVector2 rotateBy(Rotation2d rotation) {
        double cos = Math.cos(rotation.getRadians());
        double sin = Math.sin(rotation.getRadians());
        return new BreakerVector2((this.x * cos) - (this.y * sin), (this.x * sin) + (this.y * cos));
    }

    
    /** 
     * @return BreakerVector2
     */
    public BreakerVector2 getUnitVector() {
        return new BreakerVector2(vectorRotation, 1.0);
    }

    
    /** 
     * @param obj
     * @return boolean
     */
    @Override
    public boolean equals(Object obj) {
        return (Math.abs(((BreakerVector2) obj).x - x) < 1E-9)
                && (Math.abs(((BreakerVector2) obj).y - y) < 1E-9);
    }

    
    /** 
     * @param endValue
     * @param t
     * @return BreakerVector2
     */
    @Override
    public BreakerVector2 interpolate(BreakerVector2 endValue, double t) {
        double interX = MathUtil.interpolate(x, endValue.getX(), t);
        double interY = MathUtil.interpolate(y, endValue.getY(), t);
        return new BreakerVector2(interX, interY);
    }

    /** [0] = X, [1] = Y */
    @Override
    public double[] getInterpolatableData() {
        return new double[] { x, y };
    }

    
    /** 
     * @param interpolatableData
     * @return BreakerVector2
     */
    @Override
    public BreakerVector2 fromInterpolatableData(double[] interpolatableData) {
        return new BreakerVector2(interpolatableData[0], interpolatableData[1]);
    }

    
    /** 
     * @return String
     */
    @Override
    public String toString() {
       return String.format("BreakerVector2(Magnatude: %.2f, X: %.2f, Y: %.2f, Angle: %s)", magnitude, x, y, vectorRotation.toString());
    }

}
