package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static frc.robot.Constants.ElevatorConstants.*;

public class Elevator extends SubsystemBase {
    private TalonFX left, right;
    private double setpointMeters;
    public Elevator() {
        left = new TalonFX(0);
        right = new TalonFX(0);
        configLeft();


    }

    private void configLeft() {
        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        leftConfig.Feedback.SensorToMechanismRatio = kRotationsPerMeter.getRatioToOne();

        leftConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        leftConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
        leftConfig.Slot0.kP = kP;
        leftConfig.Slot0.kI = kI;
        leftConfig.Slot0.kD = kD;
        leftConfig.Slot0.kS = kS;
        leftConfig.Slot0.kV = kV;
        leftConfig.Slot0.kA = kA;
        leftConfig.Slot0.kG = kG;
        leftConfig.MotionMagic.MotionMagicCruiseVelocity = kMotionMagicCruiseVelocity.in(Units.MetersPerSecond);
        leftConfig.MotionMagic.MotionMagicAcceleration = kMotionMagicAcceleration.in(Units.MetersPerSecondPerSecond);
        leftConfig.MotionMagic.MotionMagicJerk = kMotionMagicJerk.in(MetersPerSecondPerSecond.per(Second));


        leftConfig.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit.in(Units.Amps);
        leftConfig.CurrentLimits.SupplyCurrentLowerLimit = kSupplyLowerCurrentLimit.in(Units.Amps);
        leftConfig.CurrentLimits.SupplyCurrentLowerTime = kSupplyLowerCurrentLimitTime.in(Units.Seconds);
        leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        leftConfig.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit.in(Units.Amps);
        leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    }

    public Command setHeight(Distance height) {
        return null;
    }

    public boolean atSetpointHeight() {
        return true;
    }

    public boolean atSetpointVel() {
        return true;
    }

    public Distance getHeight() {
        return null;
    }

    public LinearVelocity getVelocity() {
        return null;
    }

    public LinearAcceleration getAcceleration() {
        return null;
    }

    
}
