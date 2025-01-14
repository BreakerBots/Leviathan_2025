package frc.robot.subsystems;

import static frc.robot.Constants.EndEffectorConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.BreakerLib.sensors.BreakerBeamBreak;

public class EndEffector {
    private TalonSRX kicker;
    private TalonSRX rollers;
    private TalonFX pivot;
    private CANcoder pivotEncoder;
    private BreakerBeamBreak coralSensor;
    private Canandcolor algaeSensor;
    public EndEffector() {

    }

    private void setRollerState(RollerState rollerState) {
        if (rollerState == RollerState.HOLD_ALGAE) {
            rollers.configSupplyCurrentLimit(kAlgaeHoldRollerCurrentLimitConfig);
        } else {
            rollers.configSupplyCurrentLimit(kNormalRollerCurrentLimitConfig);
        }
        rollers.set(ControlMode.PercentOutput, rollerState.getDutyCycle());
    }

    private void setKicker


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
        INTAKE(-0.5),
        EXTAKE(1.0),
        INTAKE_ALGAE(-0.75),
        HOLD_ALGAE(-0.15),
        NEUTRAL(0.0);
        private double dutyCycleOut;
        private RollerState(double dutyCycleOut) {
            this.dutyCycleOut = dutyCycleOut;
        }

        public double getDutyCycle() {
            return dutyCycleOut;
        }
    }

    public static enum KickerState {
        KICK(-1.0),
        INTAKE(-0.8),
        EXTAKE(0.8),
        HOLD(-0.1),
        NEUTRAL(0.0);

        private double dutyCycleOut;
        private KickerState(double dutyCycleOut) {
            this.dutyCycleOut = dutyCycleOut;
        }

        public double getDutyCycle() {
            return dutyCycleOut;
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
    
        
    }


}
