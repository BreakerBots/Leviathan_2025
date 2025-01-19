package frc.robot.subsystems;

import static frc.robot.Constants.EndEffectorConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.sensors.BreakerDigitalSensor;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.subsystems.Elevator.ElevatorSetpoint;
import frc.robot.subsystems.EndEffector.WristAngle.WristAngleType;

public class EndEffector extends SubsystemBase {
    private TalonSRX kicker;
    private TalonSRX rollers;
    private TalonFX wrist;
    private CANcoder pivotEncoder;
    private BreakerDigitalSensor coralSensor;
    private Canandcolor algaeSensor;
    private MotionMagicExpoVoltage wristRequest;
    private EndEffectorSetpoint setpoint;
    private EndEffectorRotationAllowence rotationAllowence;
    public EndEffector() {
        CANcoderConfiguration conf = new CANcoderConfiguration();
        setRotationAllowenceFunc(EndEffectorRotationAllowence.BOTTOM_LIMITED);
    }

    public Command setRotationAllowence(EndEffectorRotationAllowence rotationAllowence) {
        return Commands.runOnce(() -> setRotationAllowenceFunc(rotationAllowence), this);
    }

    private void setRotationAllowenceFunc(EndEffectorRotationAllowence rotationAllowence) {
        this.rotationAllowence = rotationAllowence;
        MagnetSensorConfigs magConfigs = new MagnetSensorConfigs();
        pivotEncoder.getConfigurator().refresh(magConfigs);
        magConfigs.AbsoluteSensorDiscontinuityPoint = rotationAllowence.getAngleType().discontinuity().getDiscontinuityEnd();
        pivotEncoder.getConfigurator().apply(magConfigs);
        wrist.getConfigurator().apply(rotationAllowence.getSoftLimits());
    }

    public Command set(EndEffectorSetpoint setpoint, boolean waitForSuccess) {
        return Commands.runOnce(() -> setControl(setpoint), this).andThen(Commands.waitUntil(() -> isAtSetpoint() || !waitForSuccess));
    }

    private void setControl(EndEffectorSetpoint setpoint) {
        this.setpoint = setpoint;
        setRollerState(setpoint.rollerState());
        setKicker(setpoint.kickerState());
        setWrist(setpoint.wristSetpoint().getSetpoint().convert(rotationAllowence.getAngleType()).getAngle());
    }
 
    private void setRollerState(RollerState rollerState) {
        rollers.configSupplyCurrentLimit(rollerState.getCurrentLimitConfig());
        rollers.set(ControlMode.PercentOutput, rollerState.getDutyCycle());
    }

    private void setKicker(KickerState kickerState) {
        kicker.configSupplyCurrentLimit(kickerState.getCurrentLimitConfig());
        kicker.set(ControlMode.PercentOutput, kickerState.getDutyCycle());
    }

    private void setWrist(Angle setpoint) {
        wrist.setControl(wristRequest.withPosition(setpoint));
    }


    public boolean hasCoral() {
        return coralSensor.getAsBoolean();
    }

    public boolean isAlgaeVisable() {
        return getColorDelta(algaeSensor.getColor().toWpilibColor(), kAlgaeColor) <= kMaxColorDelta;
    }

    private boolean isAlgaeProximityBelowThresh() {
        return algaeSensor.getProximity() <= kHasAlgaeProximityThresh;
    }

    public boolean hasAlgae() {
        return isAlgaeVisable() && isAlgaeProximityBelowThresh();
    }

    public Angle getWristAngle() {
        return pivotEncoder.getAbsolutePosition().getValue();
    }

    public AngularVelocity getWristVelocity() {
        return pivotEncoder.getVelocity().getValue();
    }

    public boolean isAtSetpoint() {
        return isAtAngleSetpoint() && isAtWristVelocitySetpoint();
    }

    public EndEffectorSetpoint getSetpoint() {
        return setpoint;
    }

    private boolean isAtAngleSetpoint() {
        return MathUtil.isNear(setpoint.wristSetpoint().getSetpoint().getNormal().getAngle().in(Rotations), getWristAngle().getNormal().getAngle().in(Rotations), setpoint.wristSetpoint().getTolerence().in(Rotations), -0.5, 0.5);
    }

    private boolean isAtWristVelocitySetpoint() {
        return MathUtil.isNear(0.0, getWristVelocity().in(RadiansPerSecond), setpoint.wristSetpoint().getVelocityTolerence().in(RadiansPerSecond));
    }


    public static double getColorDelta(Color a, Color b) {
        var at = new Translation3d(a.red, a.green, a.blue);
        var bt = new Translation3d(b.red, b.green, b.blue);
        return at.getDistance(bt);
    }

    @Override
    public void periodic() {
        if (RobotState.isDisabled()) {
            setControl(new EndEffectorSetpoint(new WristSetpoint(getWristAngle()), RollerState.NEUTRAL, KickerState.NEUTRAL));
        }

        BreakerLog.log("EndEffector/Wrist/Motor", wrist);
        BreakerLog.log("EndEffector/Wrist/Encoder", pivotEncoder);
        BreakerLog.log("EndEffector/Wrist/Setpoint/Angle", setpoint.wristSetpoint.setpoint.getNormal().getAngle().in(Degrees));
        BreakerLog.log("EndEffector/Wrist/Setpoint/Tolerence", setpoint.wristSetpoint.tolerence.in(Degrees));
        BreakerLog.log("EndEffector/Wrist/Setpoint/VelTolerence", setpoint.wristSetpoint.velocityTolerence.in(DegreesPerSecond));
        BreakerLog.log("EndEffector/Wrist/Setpoint/Error", Math.abs(getWristAngle()
        .getNormal().getAngle().in(Degrees)) - setpoint.wristSetpoint.setpoint.getNormal().getAngle().in(Degrees));

        BreakerLog.log("EndEffector/RollerMotor/SupplyCurrent", rollers.getSupplyCurrent());
        BreakerLog.log("EndEffector/RollerMotor/StatorCurrent", rollers.getStatorCurrent());
        BreakerLog.log("EndEffector/RollerMotor/Output", rollers.getMotorOutputPercent());
        BreakerLog.log("EndEffector/KickerMotor/SupplyCurrent", rollers.getSupplyCurrent());
        BreakerLog.log("EndEffector/KickerMotor/StatorCurrent", rollers.getStatorCurrent());
        BreakerLog.log("EndEffector/KickerMotor/Output", rollers.getMotorOutputPercent());

        BreakerLog.log("EndEffector/HasCoral", hasCoral());
        BreakerLog.log("EndEffector/AlgaeSensor/HasAlgae", hasAlgae());
        Color c = algaeSensor.getColor().toWpilibColor();
        double cd = getColorDelta(c, kAlgaeColor);
        BreakerLog.log("EndEffector/AlgaeSensor/SeesAlgae", cd <= kMaxColorDelta);
        BreakerLog.log("EndEffector/AlgaeSensor/Color/Delta", cd);
        BreakerLog.log("EndEffector/AlgaeSensor/Color/R", c.red);
        BreakerLog.log("EndEffector/AlgaeSensor/Color/G", c.green);
        BreakerLog.log("EndEffector/AlgaeSensor/Color/B", c.blue);
    }


    public static enum RollerState {
        INTAKE(-0.5, kNormalRollerCurrentLimitConfig),
        EXTAKE(1.0, kNormalRollerCurrentLimitConfig),
        INTAKE_ALGAE(-0.75, kNormalRollerCurrentLimitConfig),
        HOLD_ALGAE(-0.15, kAlgaeHoldRollerCurrentLimitConfig),
        NEUTRAL(0.0, kNormalRollerCurrentLimitConfig);
        private double dutyCycleOut;
        private SupplyCurrentLimitConfiguration currentLimitConfig;
        private RollerState(double dutyCycleOut, SupplyCurrentLimitConfiguration currentLimitConfig) {
            this.dutyCycleOut = dutyCycleOut;
            this.currentLimitConfig = currentLimitConfig;
        }

        public double getDutyCycle() {
            return dutyCycleOut;
        }

        public SupplyCurrentLimitConfiguration getCurrentLimitConfig() {
            return currentLimitConfig;
        }
    }

    public static enum EndEffectorRotationAllowence {
        RESTRICTED(kRestrictedSoftLimits),
        FLIPABLE(kFlippableSoftLimits);
        private SoftwareLimitSwitchConfigs softLimits;
        private EndEffectorRotationAllowence(SoftwareLimitSwitchConfigs softLimits) {
            this.softLimits = softLimits;
        }

        public SoftwareLimitSwitchConfigs getSoftLimits() {
            return softLimits;
        }
    }

    public static enum KickerState {
        KICK(-1.0, kNormalKickerCurrentLimitConfig),
        INTAKE(-0.8, kNormalKickerCurrentLimitConfig),
        EXTAKE(0.8, kNormalKickerCurrentLimitConfig),
        HOLD(-0.1, kAlgaeHoldKickerCurrentLimitConfig),
        NEUTRAL(0.0, kNormalKickerCurrentLimitConfig);

        private double dutyCycleOut;
        private SupplyCurrentLimitConfiguration currentLimitConfig;
        private KickerState(double dutyCycleOut, SupplyCurrentLimitConfiguration currentLimitConfig) {
            this.dutyCycleOut = dutyCycleOut;
            this.currentLimitConfig = currentLimitConfig;
        }

        public double getDutyCycle() {
            return dutyCycleOut;
        }
        
        public SupplyCurrentLimitConfiguration getCurrentLimitConfig() {
            return currentLimitConfig;
        }


    }

    public static class WristSetpoint {
        private Angle setpoint;
        private Angle tolerence;
        private AngularVelocity velocityTolerence;
        public WristSetpoint(Angle setpoint, Angle tolerence, AngularVelocity velocityTolerence) {
            this.setpoint = setpoint;
            this.tolerence = tolerence;
            this.velocityTolerence = velocityTolerence;
        }

        public WristSetpoint(Angle setpoint) {
            this(setpoint, kDefaultWristAngleTolerence, kDefaultWristVelocityTolerence);
        }

        public Angle getSetpoint() {
            return setpoint;
        }

        public Angle getTolerence() {
            return tolerence;
        }

        public AngularVelocity getVelocityTolerence() {
            return velocityTolerence;
        }
        
    }

    public static record EndEffectorSetpoint(WristSetpoint wristSetpoint, RollerState rollerState, KickerState kickerState) {
        public static final EndEffectorSetpoint INTAKE_HP = new EndEffectorSetpoint(new WristSetpoint(kMinWristAngle), RollerState.INTAKE, KickerState.INTAKE);
        public static final EndEffectorSetpoint NEUTRAL_L4 = new EndEffectorSetpoint(new WristSetpoint(Degrees.of(0)), RollerState.NEUTRAL, KickerState.NEUTRAL);
        public static final EndEffectorSetpoint EXTAKE_L4 = new EndEffectorSetpoint(new WristSetpoint(Degrees.of(0)), RollerState.EXTAKE, KickerState.EXTAKE);
    }


}
