// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.BreakerLib.sensors.BreakerDigitalSensor;
import frc.robot.BreakerLib.util.MechanismRatio;

/** Add your docs here. */
public class Climb extends SubsystemBase {
    private TalonFXS fork;
    private TalonFX winch;
    private CANcoder forkCoder;
    private BreakerDigitalSensor forkSensor;

    private StateState currentStateState = StateState.NEUTRAL;

    private DutyCycleOut forkDutyCycle = new DutyCycleOut(0);
    private DutyCycleOut winchDutyCycle = new DutyCycleOut(0);
    
    
    public Climb() {
        fork = new TalonFXS(ClimbConstants.kForkMotorID);
        winch = new TalonFX(ClimbConstants.kWinchMotorID);
        forkCoder = new CANcoder(ClimbConstants.kForkCoder);
        forkSensor = BreakerDigitalSensor.fromDIO(ClimbConstants.kForkSensor, true);
        setupConfigs();
    }
    
    private void setupConfigs() {
        var forkConfig = new TalonFXSConfiguration();
        forkConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        forkConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimbConstants.kForkReverseLimit.getMeasure().baseUnitMagnitude();
        forkConfig.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.FusedCANcoder;
        forkConfig.ExternalFeedback.FeedbackRemoteSensorID = ClimbConstants.kForkCoder;
        forkConfig.ExternalFeedback.RotorToSensorRatio = ClimbConstants.kForkSensorGearRatio.getRatioToOne();
        
        fork.getConfigurator().apply(forkConfig);

        var winchConfig = new TalonFXConfiguration();
        winchConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        winchConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimbConstants.kWinchReverseLimit.getRotations();
        winchConfig.Feedback.RotorToSensorRatio = ClimbConstants.kWinchRatio.getRatioToOne();
        
        winch.getConfigurator().apply(winchConfig);
    }
    
    public void setState(StateState changeState) {
        currentStateState = changeState;
    }

    public Command setStateCommand(StateState changeStateState) {
        return runOnce(() -> setState(changeStateState));
    }

    public Command setStateCommandAndWait(StateState changStateState) {
        return new FunctionalCommand(
            () -> setState(changStateState), 
            () -> {}, 
            (i) -> {}, 
            () -> isTargetState(), 
            this
        );
    }

    public enum ForkState {
        NEUTRAL(0),
        RETRACTED(-1),
        EXTENDED(1);

        private double motorDutyCycle;

        private ForkState(double motorDutyCycle) {
            this.motorDutyCycle = motorDutyCycle;
        }
        
        public double getMotorDutyCycle() {
            return motorDutyCycle;
        }
    }
    
    public enum WinchState {
        NEUTRAL(0),
        ROLLED(1),
        UNROLLED(-1);

        private double motorDutyCycle;

        private WinchState(double motorDutyCycle) {
            this.motorDutyCycle = motorDutyCycle;
        }

        public double getMotorDutyCycle() {
            return motorDutyCycle;
        }
    } 
    
    public enum StateState {
        EXTENDED(WinchState.UNROLLED, ForkState.EXTENDED),
        ROLLED_BACK(WinchState.ROLLED, ForkState.EXTENDED),
        NEUTRAL(WinchState.NEUTRAL, ForkState.NEUTRAL);

        private ForkState currentForkState = ForkState.RETRACTED;
        private WinchState currentWinchState = WinchState.UNROLLED;

        private StateState(WinchState winchState, ForkState forkState) {
            currentWinchState = winchState;
            currentForkState = forkState;
        }

        public WinchState getWinchState() {
            return currentWinchState;
        }

        public ForkState getForkState() {
            return currentForkState;
        }
        
    }

    public boolean isTargetState() {
        return switch (currentStateState) {
            case ROLLED_BACK -> isWinchWound() && isForkExtended();
            case EXTENDED -> !isWinchWound() && isForkExtended();
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
        return forkSensor.getAsBoolean();
    }

    public StateState getState() {
        return currentStateState;
    }
    
    @Override
    public void periodic() {
        WinchState winchState = currentStateState.getWinchState();
        ForkState forkState = currentStateState.getForkState();

        if (RobotState.isDisabled()) {
            setState(StateState.NEUTRAL);
        }

        if (isInRolledBackState()) {
            forkDutyCycle.withOutput(forkState.getMotorDutyCycle());
            winchDutyCycle.withOutput(winchState.getMotorDutyCycle());
        }

        fork.setControl(forkDutyCycle);
        winch.setControl(winchDutyCycle);
    }
}
