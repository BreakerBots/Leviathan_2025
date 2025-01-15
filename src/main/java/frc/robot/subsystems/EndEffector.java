package frc.robot.subsystems;

import static frc.robot.Constants.EndEffectorConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.BreakerLib.sensors.BreakerDigitalSensor;

public class EndEffector {
    private TalonSRX kicker;
    private TalonSRX rollers;
    private TalonFX pivot;
    private CANcoder pivotEncoder;
    private BreakerDigitalSensor coralSensor;
    private Canandcolor algaeSensor;

    private EndEffectorSetpoint setpoint;
    public EndEffector() {

    }

    private void setRollerState(RollerState rollerState) {
        rollers.configSupplyCurrentLimit(rollerState.getCurrentLimitConfig());
        rollers.set(ControlMode.PercentOutput, rollerState.getDutyCycle());
    }

    private void setKicker(KickerState kickerState) {
        kicker.configSupplyCurrentLimit(kickerState.getCurrentLimitConfig());
        kicker.set(ControlMode.PercentOutput, kickerState.getDutyCycle());
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


    public static double getColorDelta(Color a, Color b) {
        var at = new Translation3d(a.red, a.green, a.blue);
        var bt = new Translation3d(b.red, b.green, b.blue);
        return at.getDistance(bt);
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
        private Rotation2d setpoint;
        private Angle tolerence;
        private AngularVelocity velocityTolerence;
        public WristSetpoint(Rotation2d setpoint, Angle tolerence, AngularVelocity velocityTolerence) {
            this.setpoint = setpoint;
            this.tolerence = tolerence;
            this.velocityTolerence = velocityTolerence;
        }

        public WristSetpoint(Rotation2d setpoint) {
            this(setpoint, kDefaultWristAngleTolerence, kDefaultWristVelocityTolerence);
        }

        public Rotation2d getSetpoint() {
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
        
    }


}
