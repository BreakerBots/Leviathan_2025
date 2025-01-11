package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
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
        leftConfig.MotorOutput.Inverted = kLeftMotorInverted;
        leftConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        leftConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        leftConfig.Slot0.kP = kRotationsToMeters.getInput(kP);
        leftConfig.Slot0.kI = kRotationsToMeters.getInput(kI);
        leftConfig.Slot0.kD = kRotationsToMeters.getInput(kD);
        leftConfig.Slot0.kS = kRotationsToMeters.getInput(kS);
        leftConfig.Slot0.kG = kRotationsToMeters.getInput(kG);
        leftConfig.MotionMagic.MotionMagicExpo_kA = kRotationsToMeters.getInput(kA);
        leftConfig.MotionMagic.MotionMagicExpo_kV = kRotationsToMeters.getInput(kV);
        leftConfig.MotionMagic.MotionMagicCruiseVelocity =  kRotationsToMeters.getInput(kMotionMagicCruiseVelocity.in(Units.MetersPerSecond));
        leftConfig.MotionMagic.MotionMagicAcceleration =  kRotationsToMeters.getInput(kMotionMagicAcceleration.in(Units.MetersPerSecondPerSecond));
        leftConfig.MotionMagic.MotionMagicJerk = kRotationsToMeters.getInput(kMotionMagicJerk.in(MetersPerSecondPerSecond.per(Second)));

        leftConfig.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit.in(Units.Amps);
        leftConfig.CurrentLimits.SupplyCurrentLowerLimit = kSupplyLowerCurrentLimit.in(Units.Amps);
        leftConfig.CurrentLimits.SupplyCurrentLowerTime = kSupplyLowerCurrentLimitTime.in(Units.Seconds);
        leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        leftConfig.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit.in(Units.Amps);
        leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    }

    private void configRight() {
        
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
