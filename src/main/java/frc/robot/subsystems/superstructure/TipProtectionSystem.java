package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Radian;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.TipProtectionSystemConstants;
import frc.robot.HolonomicSlewRateLimiter;
import frc.robot.BreakerLib.driverstation.BreakerInputStream;
import frc.robot.BreakerLib.driverstation.BreakerInputStream2d;
import frc.robot.BreakerLib.physics.BreakerVector2;
import frc.robot.subsystems.Drivetrain.DrivetrainKinematicLimits;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorSetpoint;

public class TipProtectionSystem {
    private HolonomicSlewRateLimiter limiter = null;

    private final Elevator elevator;
    private final Pigeon2 imu;
    
    public TipProtectionSystem(Elevator elevator, Pigeon2 imu) {
        this.elevator = elevator;
        this.imu = imu;
    }

    public Pair<BreakerInputStream2d, BreakerInputStream> setStreams(BreakerInputStream2d linear, BreakerInputStream rotational) {
        limiter = new HolonomicSlewRateLimiter(linear, rotational);
        return new Pair<>(limiter.getLinearInputStream(), limiter.getRotationalInputStream());
    }

    public void update() {
        var height = elevator.getHeight();
        DrivetrainKinematicLimits limit = TipProtectionSystemConstants.kinematicLimitMap.get(height);
        limiter.setLimits(limit.linearAcceleration(), limit.angularAcceleration());

        if (height.magnitude() > 0) {
            var angles = new BreakerVector2(imu.getPitch().getValue().in(Radian), imu.getRoll().getValue().in(Radian));
            if (angles.getMagnitude() > TipProtectionSystemConstants.tippingThreshold.in(Radian)) {
                CommandScheduler
                    .getInstance()
                    .schedule(elevator.set(ElevatorSetpoint.STOW, false).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
            }
        }
    }
}
