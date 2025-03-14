package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.LinearAcceleration;
import frc.robot.BreakerLib.driverstation.BreakerInputStream;
import frc.robot.BreakerLib.driverstation.BreakerInputStream2d;
import frc.robot.BreakerLib.physics.BreakerVector2;
import frc.robot.BreakerLib.util.BreakerSlewRateLimiter;

public class HolonomicSlewRateLimiter {
    private BreakerInputStream2d linearInputStream;
    private BreakerInputStream rotationalInputStream;
    private double linearLim;
    private double rotationalLim;
    private BreakerVector2 prevLinear;
    private double prevRot;

    private final BreakerSlewRateLimiter slrX = new BreakerSlewRateLimiter(0);
    private final BreakerSlewRateLimiter slrY = new BreakerSlewRateLimiter(0);

    private final BreakerSlewRateLimiter slr = new BreakerSlewRateLimiter(0);

    public HolonomicSlewRateLimiter(BreakerInputStream2d linearInputStream, BreakerInputStream rotationalInputStream) {
        this.linearInputStream = linearInputStream.map(this::updateLinear);
        this.rotationalInputStream = rotationalInputStream.map(this::updateRot);
        prevLinear = new BreakerVector2();
    }

    public BreakerInputStream2d getLinearInputStream() {
        return linearInputStream;
    }

    public BreakerInputStream getRotationalInputStream() {
        return rotationalInputStream;
    }

    public void setLimits(LinearAcceleration linear, AngularAcceleration rotational) {
        linearLim = linear.in(Units.MetersPerSecondPerSecond);
        rotationalLim = rotational.in(Units.RadiansPerSecondPerSecond);
    }

    private BreakerVector2 updateLinear(BreakerVector2 vec) {
        var delta = vec.minus(prevLinear);
        var deltaUnitVec = delta.getUnitVector();
        if (delta.getMagnitude() <= 1e-5) {
            deltaUnitVec = new BreakerVector2(1.0, 1.0).getUnitVector();
        }
        
        // var slrX = new SlewRateLimiter(linearLim * deltaUnitVec.getX());
        // var slrY = new SlewRateLimiter(linearLim * deltaUnitVec.getY());
        slrX.setRateLimit(Math.abs(linearLim * deltaUnitVec.getX()));
        slrY.setRateLimit(Math.abs(linearLim * deltaUnitVec.getY()));

        // slrX.reset(prevLinear.getX());
        // slrY.reset(prevLinear.getY());

        // slrX.setLastValue(prevLinear.getX());
        // slrY.setLastValue(prevLinear.getY());

        var x = slrX.calculate(vec.getX());
        var y = slrY.calculate(vec.getY());

        prevLinear = new BreakerVector2(x, y);
        return prevLinear;
    }

    private double updateRot(double rot) {
        // var slr = new SlewRateLimiter(rotationalLim);
        slr.setRateLimit(rotationalLim);
        // slr.reset(prevRot);
        // slr.setLastValue(prevRot);
        
        prevRot = slr.calculate(rot);
        return prevRot;
    }

    public double getLinearLim() {
        return linearLim;
    }

    public double getRotationalLim() {
        return rotationalLim;
    }

   
}
