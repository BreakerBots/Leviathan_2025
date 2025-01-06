package frc.robot.BreakerLib.util.math;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.CoordinateAxis;
import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.BreakerLib.physics.BreakerVector2;
import frc.robot.BreakerLib.physics.BreakerVector3;

/** BreakerLib math util class. */
public class BreakerMath {


    public static double getCircumferenceFromDiameter(double diameter) {
        return diameter * Math.PI;
    }

    /**
     * Converts a fixed point notiation number into a double precision foating point
     * number
     * 
     * @param FixedPointVal    fixed point number represented as a non-fractional
     *                         integer
     * @param bitsAfterDicimal fixed point notation is generaly represented as Qx.y
     *                         where x
     *                         represents the number of bits before the decimal, and
     *                         y prepresents the number of bits
     *                         after the decimal (EX: Q2.14)
     */
    public static double fixedToFloat(int FixedPointVal, int bitsAfterDicimal) {
        return ((Double.valueOf(FixedPointVal)) / (Math.pow(2, bitsAfterDicimal)));
    }

    /**
     * Converts a fixed point notiation number into a double precision foating point
     * number
     * 
     * @param FixedPointVal    fixed point number represented as a non-fractional
     *                         long integer
     * @param bitsAfterDicimal fixed point notation is generaly represented as Qx.y
     *                         where x
     *                         represents the number of bits before the decimal, and
     *                         y prepresents the number of bits
     *                         after the decimal (EX: Q2.14)
     */
    public static double fixedToFloat(Long FixedPointVal, int bitsAfterDicimal) {
        return ((Double.valueOf(FixedPointVal)) / (Math.pow(2, bitsAfterDicimal)));
    }

    /**
     * Converts a fixed point notiation number into a double precision foating point
     * number
     * 
     * @param FixedPointVal    fixed point number represented as a non-fractional
     *                         short integer
     * @param bitsAfterDicimal fixed point notation is generaly represented as Qx.y
     *                         where x
     *                         represents the number of bits before the decimal, and
     *                         y prepresents the number of bits
     *                         after the decimal (EX: Q2.14)
     */
    public static double fixedToFloat(Short FixedPointVal, int bitsAfterDicimal) {
        return ((Double.valueOf(FixedPointVal)) / (Math.pow(2, bitsAfterDicimal)));
    }

    // /**
    //  * Checks if two numbers are sufficiently proximate.
    //  * 
    //  * @param val1         First number.
    //  * @param val2         Second number.
    //  * @param maxDeviation Absolute value difference between val1 and val2
    //  * 
    //  * @return true if within deviation, false otherwise.
    //  */
    // public static boolean epsilonEquals(double val1, double val2, double maxDeviation) {
    //     return ((val1 <= (val2 + maxDeviation)) && (val1 >= (val2 - maxDeviation)));
    //     MathUtil.isNear()
    // }

    public static Rotation2d getPointAngleRelativeToOtherPoint(Translation2d point1, Translation2d point2) {
        double x1 = point1.getX();
        double y1 = point1.getY();
        double x2 = point2.getX();
        double y2 = point2.getY();
        if (x2 >= x1) {
            return Rotation2d.fromRadians(Math.atan2(y2 - y1, x2 - x1));
        } else {
            if (y2 >= y1) {
                double a = Math.atan2(y2 - y1, x2 - x1);
                a *= -Math.signum(a);
                return Rotation2d.fromRadians(Math.PI - a);
            } else {
                double a = Math.atan2(y2 - y1, x2 - x1);
                a *= -Math.signum(a);
                return Rotation2d.fromRadians(-Math.PI + a);
            }
        }
    }

    /**
     * Linearly interpolates between 2 points to find y-val at given x.
     * 
     * @param queryX X-value to interpolate from.
     * @param lowX   X-val of low point.
     * @param highX  X-val of high point.
     * @param lowY   Y-val of low point.
     * @param highY  Y-val of high point.
     * @return Approximate Y-value at given X.
     */
    public static double interpolateLinear(double queryX, double lowX, double highX, double lowY, double highY) {
        return MathUtil.interpolate(lowY, highY, getLerpT(queryX, lowX, highX));
    }

    public static double getLerpT(double query, double low, double high) {
        return (query - low) / (high - low);
    }

    /**
     * Lagrange Polynomial interpolation of a Y value from an X value and a set of
     * known points. https://en.wikipedia.org/wiki/Lagrange_polynomial
     * 
     * @param queryX      X-value to interpolate a Y-value for.
     * @param knownPoints Known points in a 2D space.
     * @return The approximate Y-Value that would corespond to the given X-Value
     */
    public static double interpolateLagrange(double queryX, Translation2d... knownPoints) {
        double result = 0;
        for (int i = 0; i < knownPoints.length; i++) { // Goes through points.
            double term = knownPoints[i].getY(); // Y-value of selected point.
            for (int j = 0; j < knownPoints.length; j++) { // Loops through non-identical points.
                if (j != i) { // Avoids multiplication by 0.
                    // Interpolates between selected and point from data set.
                    term *= (queryX - knownPoints[j].getX()) / (knownPoints[i].getX() - knownPoints[j].getX());
                }
            }
            result += term; // Accumulated interpretation is added.
        }
        return result;
    }

    public static ChassisSpeeds fromRobotRelativeSpeeds(ChassisSpeeds robotRelativeSpeeds, Rotation2d robotAngle) {
        double cos = Math.cos(-robotAngle.getRadians());
        double sin = Math.sin(-robotAngle.getRadians());
        return new ChassisSpeeds(
                (robotRelativeSpeeds.vxMetersPerSecond * cos) - (robotRelativeSpeeds.vyMetersPerSecond * sin),
                (robotRelativeSpeeds.vxMetersPerSecond * sin) + (robotRelativeSpeeds.vyMetersPerSecond * cos),
                robotRelativeSpeeds.omegaRadiansPerSecond);
    }

    public static double getWeightedAvg(double[] valuesToAvg, double[] weights) {
        double numer = 0;
        double denom = 0;
        // Values multiplied by respective weights.
        for (int i = 0; i < valuesToAvg.length; i++) {
            double weight = i < weights.length ? weights[i] : 1.0;
            numer += valuesToAvg[i] * weight;
        }
        // Weights added up.
        for (int i = 0; i < valuesToAvg.length; i++) {
            denom += i < weights.length ? weights[i] : 1.0;
        }

        return numer / denom;
    }

    public static double getWeightedAvg(List<Double> valuesToAvg, List<Double> weights) {
        double numer = 0;
        double denom = 0;
        for (int i = 0; i < valuesToAvg.size(); i++) {
            double weight = i < weights.size() ? weights.get(i) : 1.0;
            numer += valuesToAvg.get(i) * weight;
        }

        for (int i = 0; i < valuesToAvg.size(); i++) {
            denom += i < weights.size() ? weights.get(i) : 1.0;
        }

        return numer / denom;
    }

    public static double root(double num, double root) {
        return Math.pow(num, 1.0 / root);
    }

    public static double absoluteAngleToContinuousRelativeAngleDegrees(double curRelativeAngle,
            Rotation2d curAbsoluteAngle, Rotation2d tgtAngle) {
        return curRelativeAngle + (tgtAngle.minus(curAbsoluteAngle).getDegrees());
    }

    public static Pose2d mirrorPose(Pose2d pose, double translationalAxisOfSymetry, MirrorSymetryAxis2d translationMirrorType, MirrorSymetryAxis2d rotationMirrorType) {
       return new Pose2d(mirrorTranslation(pose.getTranslation(), translationalAxisOfSymetry, translationMirrorType), mirrorRotation(pose.getRotation(), rotationMirrorType));
    }

    public static enum MirrorSymetryAxis2d {
        X,
        Y,
        X_AND_Y
    } 

    public static Translation2d mirrorTranslation(Translation2d translation, double axisOfSymetry, MirrorSymetryAxis2d mirrorType) {
        if (mirrorType == MirrorSymetryAxis2d.Y) {
            double distance = axisOfSymetry - translation.getX();
            return new Translation2d(axisOfSymetry + distance, translation.getY());
        } else if (mirrorType == MirrorSymetryAxis2d.X) {
            double distance = axisOfSymetry - translation.getY();
            return new Translation2d(translation.getX(), axisOfSymetry + distance);
        }
        double distanceX = axisOfSymetry - translation.getX();
        double distanceY = axisOfSymetry - translation.getY();
        return new Translation2d(axisOfSymetry + distanceX, axisOfSymetry + distanceY);
    }

    public static Rotation2d mirrorRotation(Rotation2d angle, MirrorSymetryAxis2d mirrorType) {
        if (mirrorType == MirrorSymetryAxis2d.Y) {
            return new Rotation2d(-angle.getCos(), angle.getSin());
        } else if (mirrorType == MirrorSymetryAxis2d.X) {
            return new Rotation2d(angle.getCos(), -angle.getSin());
        }
        return new Rotation2d(-angle.getCos(), -angle.getSin());
        
    }

    public static boolean epsilonEqualsPose2d(Pose2d pose0, Pose2d pose1, Pose2d maxDeviation) {
        return MathUtil.isNear(pose0.getX(), pose1.getX(), maxDeviation.getX()) && 
                MathUtil.isNear(pose0.getY(), pose1.getY(), maxDeviation.getY()) && 
                MathUtil.isNear(pose0.getRotation().getRadians(), pose1.getRotation().getRadians(), maxDeviation.getRotation().getRadians());
    }

    public static boolean epsilonEqualsChassisSpeeds(ChassisSpeeds chassisSpeeds0, ChassisSpeeds chassisSpeeds1, ChassisSpeeds maxDeviation) {
        return MathUtil.isNear(chassisSpeeds0.vxMetersPerSecond, chassisSpeeds1.vxMetersPerSecond, maxDeviation.vxMetersPerSecond) && 
                MathUtil.isNear(chassisSpeeds0.vyMetersPerSecond, chassisSpeeds1.vyMetersPerSecond, maxDeviation.vyMetersPerSecond) && 
                MathUtil.isNear(chassisSpeeds0.omegaRadiansPerSecond, chassisSpeeds1.omegaRadiansPerSecond, maxDeviation.omegaRadiansPerSecond);
    }

    public static ChassisSpeeds clampChassisSpeeds(ChassisSpeeds speedsToClamp, double maxLinearVel, double maxAngularVel) {
        BreakerVector2 linVelVec = new BreakerVector2(speedsToClamp.vxMetersPerSecond, speedsToClamp.vyMetersPerSecond).clampMagnitude(0.0, maxLinearVel);
        return new ChassisSpeeds(linVelVec.getX(), linVelVec.getY(), MathUtil.clamp(speedsToClamp.omegaRadiansPerSecond, -maxAngularVel, maxAngularVel));
    }


    /** https://www.desmos.com/calculator/ubkzzw4vrr */
    public static double linearizedConstrainedExponential(double x, double linearity, double exp, boolean preserveSign) {
        double output = Math.abs(x);
        linearity = MathUtil.clamp(linearity, 0.0, 1.0);
        exp = Math.max(exp, 1.0);
        output = ((1.0 - linearity) * Math.pow(output, exp)) + (linearity * output);
        if (preserveSign) {
            output = Math.abs(output) * Math.signum(x);
        }
        return output;
    }

    public static CoordinateSystem getCoordinateSystemFromRotation(Rotation3d rot) {
    BreakerVector3 x = new BreakerVector3(1.0, 0.0, 0.0);
    BreakerVector3 y = new BreakerVector3(0.0, 1.0, 0.0);
    BreakerVector3 z = new BreakerVector3(0.0, 0.0, 1.0);
    x = x.rotateBy(rot);
    y = y.rotateBy(rot);
    z = z.rotateBy(rot);
    return new CoordinateSystem(
      new CoordinateAxis(x.getX(), x.getY(), x.getZ()), 
      new CoordinateAxis(y.getX(), y.getY(), y.getZ()), 
      new CoordinateAxis(z.getX(), z.getY(), z.getZ()));
  }


}
