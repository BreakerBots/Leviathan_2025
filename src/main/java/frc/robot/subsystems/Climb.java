// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.BreakerLib.sensors.BreakerDigitalSensor;
import frc.robot.BreakerLib.util.logging.BreakerLog;

/** The Climb subsystem consisting of a winch and a fork. 🍴**/
public class Climb extends SubsystemBase {
    private TalonFXS fork;
    private TalonFX winch;
    private CANcoder forkCoder;
    private BreakerDigitalSensor forkSensor;

    private ClimbState currentStateState = ClimbState.NEUTRAL;

    private PositionVoltage forkPositionVolt = new PositionVoltage(0);
    private PositionVoltage winchPositionVolt = new PositionVoltage(0);
    
    
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
        forkConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimbConstants.kForkReverseLimit.getRotations();
        forkConfig.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.FusedCANcoder;
        forkConfig.ExternalFeedback.FeedbackRemoteSensorID = ClimbConstants.kForkCoder;
        forkConfig.ExternalFeedback.RotorToSensorRatio = ClimbConstants.kForkSensorGearRatio.getRatioToOne();
        
        fork.getConfigurator().apply(forkConfig);

        var winchConfig = new TalonFXConfiguration();
        winchConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        winchConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimbConstants.kWinchReverseLimit.getRotations();
        winchConfig.Feedback.RotorToSensorRatio = ClimbConstants.kWinchSpoolRatio.getRatioToOne();
        
        winch.getConfigurator().apply(winchConfig);
    }
    
    private void setState(ClimbState changeState) {
        currentStateState = changeState;
    }

    public Command setState(ClimbState state, boolean waitForSuccess) {
        return new FunctionalCommand(
            () -> setState(state), 
            () -> {}, 
            (i) -> {}, 
            () -> !waitForSuccess || isAtTargetState(), 
            this
        );
    }

    public enum ForkState {
        NEUTRAL(0),
        RETRACTED(ClimbConstants.kForkRetractedPosition),
        EXTENDED(ClimbConstants.kForkExtendedPosition);

        private double positionVoltage;

        private ForkState(double positionVoltage) {
            this.positionVoltage = positionVoltage;
        }
        
        public double getPositionVoltage() {
            return positionVoltage;
        }

        public boolean isNeutral() {
            return this == ForkState.NEUTRAL;
        }
    }
    
    public enum WinchState {
        NEUTRAL(0),
        ROLLED(ClimbConstants.kWinchWoundLimit.magnitude()),
        UNROLLED(ClimbConstants.kWinchUnwound.magnitude());

        private double motorDutyCycle;

        private WinchState(double motorDutyCycle) {
            this.motorDutyCycle = motorDutyCycle;
        }

        public double getPositionVoltage() {
            return motorDutyCycle;
        }

        public boolean isNeutral() {
            return this == WinchState.NEUTRAL;
        }
    } 
    
    public enum ClimbState {
        EXTENDED(WinchState.UNROLLED, ForkState.EXTENDED),
        ROLLED_BACK(WinchState.ROLLED, ForkState.EXTENDED),
        NEUTRAL(WinchState.NEUTRAL, ForkState.NEUTRAL);

        private ForkState currentForkState = ForkState.RETRACTED;
        private WinchState currentWinchState = WinchState.UNROLLED;

        private ClimbState(WinchState winchState, ForkState forkState) {
            currentWinchState = winchState;
            currentForkState = forkState;
        }

        public WinchState getWinchState() {
            return currentWinchState;
        }

        public ForkState getForkState() {
            return currentForkState;
        }

        public void setWinchState(WinchState state) {
            currentWinchState = state;
        }

        public void setForkState(ForkState state) {
            currentForkState = state;   
        }
        
    }

    public boolean isAtTargetState() {
        return switch (currentStateState) {
            case ROLLED_BACK -> isWinchWound() && isForkContacting();
            case EXTENDED -> !isWinchWound() && isForkContacting();
            default -> true;
        };
    }

    public boolean isInRolledBackState() {
        return currentStateState == ClimbState.ROLLED_BACK;
    }

    public double getSpoolDistanceCentimeter() {
        double winchPosition = winch.getPosition().getValue().in(Units.Rotations);
        return winchPosition / ClimbConstants.kWinchSpoolRatio.getRatioToOne();
    }

    public Distance getSpoolDistance() {
        return Units.Centimeters.of(getSpoolDistanceCentimeter());
    }

    public boolean isWinchWound() {
        return isWinchAtPoint(ClimbConstants.kWinchWoundLimit);
    }

    public boolean isWinchUnwound() {
        return isWinchAtPoint(0);
    }

    public boolean isWinchAtPoint(double cm) {
        return MathUtil.isNear(cm, getSpoolDistanceCentimeter(), ClimbConstants.kWinchTolerance.magnitude());
    }

    public boolean isWinchAtPoint(Distance point) {
        return isWinchAtPoint(point.in(Units.Centimeters));
    }

    public boolean isForkContacting() {
        return forkSensor.getAsBoolean();
    }

    public ClimbState getState() {
        return currentStateState;
    }
    
    @Override
    public void periodic() {
        WinchState winchState = currentStateState.getWinchState();
        ForkState forkState = currentStateState.getForkState();

        BreakerLog.log("Climb/Fork/Motor", fork);
        BreakerLog.log("Climb/Fork/Angle", forkCoder.getAbsolutePosition().getValueAsDouble());
        BreakerLog.log("Climb/Fork/Sensor", forkSensor.getAsBoolean());
        BreakerLog.log("Climb/Fork/Extended", isForkContacting());
        BreakerLog.log("Climb/Fork/PositionVoltage", forkPositionVolt.Position);

        BreakerLog.log("Climb/Winch/Motor", winch);
        BreakerLog.log("Climb/Winch/Wound", isWinchWound());
        BreakerLog.log("Climb/Winch/Distance", getSpoolDistance());
        BreakerLog.log("Climb/Winch/Position", winchPositionVolt.Position);
        BreakerLog.log("Climb/State", currentStateState);

        if (RobotState.isDisabled()) {
            setState(ClimbState.NEUTRAL);
        }
        
        if (!forkState.isNeutral()) forkPositionVolt.withPosition(forkState.getPositionVoltage());
        if (!forkState.isNeutral()) winchPositionVolt.withPosition(winchState.getPositionVoltage());

        fork.setControl(forkPositionVolt);
        winch.setControl(winchPositionVolt);
    }
}
