package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.TipProtectionSystemConstants;
import frc.robot.HolonomicSlewRateLimiter;
import frc.robot.BreakerLib.driverstation.BreakerInputStream;
import frc.robot.BreakerLib.driverstation.BreakerInputStream2d;
import frc.robot.subsystems.Drivetrain.DrivetrainKinimaticLimits;

public class TipProtectionSystem {
    private HolonomicSlewRateLimiter limiter = null;

    public Pair<BreakerInputStream2d, BreakerInputStream> setStreams(BreakerInputStream2d linear, BreakerInputStream rotational) {
        limiter = new HolonomicSlewRateLimiter(linear, rotational);
        return new Pair<>(limiter.getLinearInputStream(), limiter.getRotationalInputStream());
    }

    public void update(Distance elevatorHeight) {
        DrivetrainKinimaticLimits limit = TipProtectionSystemConstants.kinematicLimitMap.get(elevatorHeight);
        limiter.setLimits(limit.linearAcceleration(), limit.angularAcceleration());
    }
}
