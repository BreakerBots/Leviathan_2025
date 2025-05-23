package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TipProtectionSystemConstants;
import frc.robot.Constants;
import frc.robot.HolonomicSlewRateLimiter;
import frc.robot.BreakerLib.driverstation.BreakerInputStream;
import frc.robot.BreakerLib.driverstation.BreakerInputStream2d;
import frc.robot.BreakerLib.physics.BreakerVector2;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.subsystems.Drivetrain.DrivetrainKinematicLimits;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorSetpoint;

public class TipProtectionSystem extends SubsystemBase {
    private HolonomicSlewRateLimiter limiter = null;

    private final Elevator elevator;
    private final Pigeon2 imu;

    private double linearVelLim = DriveConstants.MAXIMUM_TRANSLATIONAL_VELOCITY.in(Units.MetersPerSecond);
    private double angVelLim = DriveConstants.MAXIMUM_ROTATIONAL_VELOCITY.in(Units.RadiansPerSecond);
    private DrivetrainKinematicLimits kinematicLimits = Constants.TipProtectionSystemConstants.kBaseKinimaticLimits;
    
    public TipProtectionSystem(Elevator elevator, Pigeon2 imu) {
        this.elevator = elevator;
        this.imu = imu;
    }

    public Pair<BreakerInputStream2d, BreakerInputStream> setStreams(BreakerInputStream2d linear, BreakerInputStream rotational) {
        linear = linear.map((BreakerVector2 vec) -> vec.clampMagnitude(-linearVelLim, linearVelLim));
        rotational = rotational.map((double val) -> MathUtil.clamp(val, -angVelLim, angVelLim));
        limiter = new HolonomicSlewRateLimiter(linear, rotational);
        return new Pair<>(limiter.getLinearInputStream(), limiter.getRotationalInputStream());
    } 

    public Pair<BreakerInputStream2d, BreakerInputStream> getStreams() {
        return new Pair<>(limiter.getLinearInputStream(), limiter.getRotationalInputStream());
    }

    public DrivetrainKinematicLimits getLimits() {
        return kinematicLimits;
    }

    @Override
    public void periodic() {
        update();
    }

    private void update() {
        if (limiter != null) {
            var elevatorHeight = elevator.getHeight();
            
            if (elevatorHeight.in(Units.Meters) > TipProtectionSystemConstants.kHeightThreshold.in(Units.Meters)) {
                kinematicLimits = TipProtectionSystemConstants.kKinematicLimitMap.get(elevator.getSetpoint().getHeight());
                linearVelLim = kinematicLimits.linearVelocity().in(Units.MetersPerSecond);
                angVelLim = kinematicLimits.angularVelocity().in(Units.RadiansPerSecond);
                limiter.setLimits(kinematicLimits.linearAcceleration(), kinematicLimits.angularAcceleration());
                
                var angles = new BreakerVector2(imu.getPitch().getValue().in(Radian), imu.getRoll().getValue().in(Radian));
                BreakerLog.log("TipProtectionSystem/TipAngle", angles.getMagnitude());
                if (angles.getMagnitude() > TipProtectionSystemConstants.kTippingThreshold.in(Radian)) {
                    // CommandScheduler
                    //     .getInstance()
                    //     .schedule(elevator.forceStow().withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
                }
            } else {
                kinematicLimits = Constants.TipProtectionSystemConstants.kBaseKinimaticLimits;
                limiter.setLimits(MetersPerSecondPerSecond.of(10000), RotationsPerSecondPerSecond.of(10000));
                linearVelLim = DriveConstants.MAXIMUM_TRANSLATIONAL_VELOCITY.in(Units.MetersPerSecond);
                angVelLim = DriveConstants.MAXIMUM_ROTATIONAL_VELOCITY.in(Units.RadiansPerSecond);
            }
            BreakerLog.log("TipProtectionSystem/LinAccelLim", limiter.getLinearLim());
            BreakerLog.log("TipProtectionSystem/AngAccelLim", limiter.getRotationalLim());
        }
    }
}
