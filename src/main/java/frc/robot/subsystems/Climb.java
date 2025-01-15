// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.BreakerLib.sensors.BreakerDigitalSensor;
/** Add your docs here. */
public class Climb extends SubsystemBase {
    private TalonFXS fork;
    private TalonFX winch;
    private CANcoder forkCoder;
    private BreakerDigitalSensor forkSensor;
    private StateState currentStateState = StateState.NEUTRAL;
    
    public Climb() {
        fork = new TalonFXS(ClimbConstants.kForkMotorID);
        winch = new TalonFX(ClimbConstants.kWinchMotorID);
        forkCoder = new CANcoder(ClimbConstants.kForkCoder);
        forkSensor = BreakerDigitalSensor.fromDIO(ClimbConstants.kForkSensor, true);
        setupConfigs();
    }
    
    private void setupConfigs() {
        var config = new TalonFXSConfiguration();
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimbConstants.kReverseLimit.getMeasure().baseUnitMagnitude();
        config.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.FusedCANcoder;
        config.ExternalFeedback.FeedbackRemoteSensorID = ClimbConstants.kForkCoder;
        config.ExternalFeedback.RotorToSensorRatio = ClimbConstants.kForkSensorGearRatio.getRatioToOne();
        
        fork.getConfigurator().apply(config);
    }
    
    public void setState(StateState changeState) {
        currentStateState = changeState;
    }
    public Command changeStateState(StateState changeStateState) {
        return runOnce(() -> setState(changeStateState));
    }

    public enum ForkState {
        RETRACTED,
        EXTENDED;
    }
    
    public enum WinchState {
        ROLLED,
        UNROLLED;
    } 
    
    public enum StateState {
        EXTENDED(WinchState.UNROLLED, ForkState.EXTENDED),
        ROLLED_BACK(WinchState.ROLLED, ForkState.EXTENDED),
        NEUTRAL(WinchState.UNROLLED, ForkState.RETRACTED);

        private ForkState currentForkState = ForkState.RETRACTED;
        private WinchState currentWinchState = WinchState.UNROLLED;

        private StateState(WinchState winchState, ForkState forkState) {
            currentWinchState = winchState;
            currentForkState = forkState;
        }
    }

    public boolean isTargetState() {
        return switch (currentStateState) { // TODO fill rest of cases
            case ROLLED_BACK -> isWinchWound() && isForkExtended();
            default -> true;
        };
    }

    public boolean isInRolledBackState() {
        return currentStateState == StateState.ROLLED_BACK;
    }

    public boolean isWinchWound() {
        return false; // TODO
    }

    public boolean isForkExtended() {
        return false; // TODO
    }

    public StateState getState() {
        return currentStateState;
    }
    
    @Override
    public void periodic() {
         
        if (isInRolledBackState()) {
            
        }
    }
}
