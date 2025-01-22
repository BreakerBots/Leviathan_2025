// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.ClimbConstants.kForkCoder;
import static frc.robot.Constants.ClimbConstants.kForkCurrentLimits;
import static frc.robot.Constants.ClimbConstants.kForkForwardLimit;
import static frc.robot.Constants.ClimbConstants.kForkMotorID;
import static frc.robot.Constants.ClimbConstants.kForkReverseLimit;
import static frc.robot.Constants.ClimbConstants.kForkSensor;
import static frc.robot.Constants.ClimbConstants.kForkSensorGearRatio;
import static frc.robot.Constants.ClimbConstants.kWinchCurrentLimits;
import static frc.robot.Constants.ClimbConstants.kWinchMotorID;
import static frc.robot.Constants.ClimbConstants.kWinchSpoolRatio;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.BreakerLib.sensors.BreakerDigitalSensor;
import frc.robot.BreakerLib.util.factory.BreakerCANCoderFactory;
import frc.robot.BreakerLib.util.logging.BreakerLog;

/** The Climb subsystem consisting of a winch and a fork. 🍴**/
public class Climb extends SubsystemBase {
    private TalonFXS fork;
    private TalonFX winch;
    private CANcoder forkEncoder;
    private BreakerDigitalSensor forkSensor;
    private Pigeon2 imu;

    private ClimbState currentClimbState = ClimbState.NEUTRAL;

    private PositionVoltage forkPositionVolt = new PositionVoltage(0);
    private PositionVoltage winchPositionVolt = new PositionVoltage(0);
    private DutyCycleOut forkDutyCycleRequest = new DutyCycleOut(0.0);
    private CoastOut coastOutRequest = new CoastOut();
   
    
    public Climb(Pigeon2 imu) {
        fork = new TalonFXS(kForkMotorID);
        winch = new TalonFX(kWinchMotorID);
        forkEncoder = BreakerCANCoderFactory.createCANCoder(kForkCoder, new CANBus(), 0.5, Rotations.of(0), SensorDirectionValue.CounterClockwise_Positive);
        forkSensor = BreakerDigitalSensor.fromDIO(kForkSensor, true);
        this.imu = imu;
        
        setupConfigs();
    }
    
    private void setupConfigs() {
        var forkConfig = new TalonFXSConfiguration();
        forkConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        forkConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kForkReverseLimit.getRotations();
        forkConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        forkConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kForkForwardLimit.getRotations();
        forkConfig.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Enabled;
        forkConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        forkConfig.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.FusedCANcoder;
        forkConfig.ExternalFeedback.FeedbackRemoteSensorID = kForkCoder;
        forkConfig.ExternalFeedback.RotorToSensorRatio = kForkSensorGearRatio.getRatioToOne();
        forkConfig.CurrentLimits = kForkCurrentLimits;
        forkConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        forkConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        fork.getConfigurator().apply(forkConfig);

        var winchConfig = new TalonFXConfiguration();
        winchConfig.Feedback.RotorToSensorRatio = kWinchSpoolRatio.getRatioToOne();
        winchConfig.CurrentLimits = kWinchCurrentLimits;
        winchConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        winchConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        winch.getConfigurator().apply(winchConfig);
    }
    
    private void setStateFunc(ClimbState changeState) {
        currentClimbState = changeState;
        setWinchState(currentClimbState.getWinchState());
        setForkState(currentClimbState.getForkState());
        
    }

    private void setWinchState(WinchState winchState) {
        if (!winchState.isNeutral()) {
            winchPositionVolt.withPosition(winchState.getPosition());
            winch.setControl(winchPositionVolt);
        } else {
            winch.setControl(coastOutRequest);
        }
    }

    private void setForkState(ForkState forkState) {
        if (!forkState.isNeutral()) {
            if (forkState == ForkState.EXTENDED) {
                fork.setControl(forkDutyCycleRequest.withOutput(0.9));
            } else {
                forkPositionVolt.withPosition(forkState.getPosition());
                fork.setControl(forkPositionVolt);
            }
        } else {
            fork.setControl(forkDutyCycleRequest.withOutput(0.0));
        }
        
    }

    public Command setState(ClimbState state, boolean waitForSuccess) {
        return Commands.runOnce(() -> setStateFunc(state)).andThen(Commands.waitUntil(() -> isAtTargetState() || !waitForSuccess));
    }

    public enum ForkState {
        NEUTRAL(Rotations.of(0), Rotations.of(0), RotationsPerSecond.of(0)),
        RETRACTED(Rotations.of(0), Rotations.of(0), RotationsPerSecond.of(0)),
        EXTENDED(Rotations.of(0), Rotations.of(0), RotationsPerSecond.of(0));

        private Angle forkPosition;
        private Angle tolerence;
        private AngularVelocity velocityTolerence;

        private ForkState(Angle forkPosition, Angle tolerence, AngularVelocity velocityTolerence) {
            this.forkPosition = forkPosition;
            this.tolerence = tolerence;
            this.velocityTolerence = velocityTolerence;
        }
        
        public Angle getPosition() {
            return forkPosition;
        }

        public Angle getTolerence() {
            return tolerence;
        }

        public AngularVelocity getVelocityTolerence() {
            return velocityTolerence;
        }

        public boolean isNeutral() {
            return this == ForkState.NEUTRAL;
        }
    }
    
    public enum WinchState {
        NEUTRAL(Rotations.of(0), Rotations.of(0), RotationsPerSecond.of(0)),
        ROLLED(Rotations.of(0), Rotations.of(0), RotationsPerSecond.of(0)),
        UNROLLED(Rotations.of(0), Rotations.of(0), RotationsPerSecond.of(0));

        private Angle motorPosition;
        private Angle tolerence;
        private AngularVelocity velocityTolerence;

        private WinchState(Angle motorPosition, Angle tolerence, AngularVelocity velocityTolerence) {
            this.motorPosition = motorPosition;
            this.tolerence = tolerence;
            this.velocityTolerence = velocityTolerence;
        }

        public Angle getPosition() {
            return motorPosition;
        }

        public Angle getTolerence() {
            return tolerence;
        }

        public AngularVelocity getVelocityTolerence() {
            return velocityTolerence;
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
        return isForkAtTargetState() && isWinchAtTargetState();
    }

    public boolean isWinchAtTargetState() {
        if (currentClimbState.getWinchState() == WinchState.NEUTRAL) {
            return true;
        }
        boolean atPos = MathUtil.isNear(
            currentClimbState.getWinchState().getPosition().in(Rotations),
            getWinchAngle().in(Rotations), 
            currentClimbState.getWinchState().getTolerence().in(Rotations));

        boolean atVel = MathUtil.isNear(
            0.0,
            getWinchVelocity().in(RotationsPerSecond), 
            currentClimbState.getWinchState().getVelocityTolerence().in(RotationsPerSecond));

        return atPos && atVel;
    }

    public boolean isForkAtTargetState() {
        if (currentClimbState.getWinchState() == WinchState.NEUTRAL) {
            return true;
        }
        boolean atPos = MathUtil.isNear(
            currentClimbState.getForkState().getPosition().in(Rotations),
            getForkAngle().in(Rotations), 
            currentClimbState.getWinchState().getTolerence().in(Rotations));

        boolean atVel = MathUtil.isNear(
            0.0,
            getForkVelocity().in(RotationsPerSecond), 
            currentClimbState.getForkState().getVelocityTolerence().in(RotationsPerSecond));

        return atPos && atVel;
    }

 
    public Angle getForkAngle() {
        return forkEncoder.getAbsolutePosition().getValue();
    }

    public AngularVelocity getForkVelocity() {
        return forkEncoder.getVelocity().getValue();
    }

    public Angle getWinchAngle() {
        return winch.getPosition().getValue();
    }

    public AngularVelocity getWinchVelocity() {
        return winch.getVelocity().getValue();
    }

    public boolean isForkContacting() {
        return forkSensor.getAsBoolean();
    }

    public ClimbState getState() {
        return currentClimbState;
    }
    
    @Override
    public void periodic() {
        pullWinch();
        
        BreakerLog.log("Climb/Fork/Motor", fork);
        BreakerLog.log("Climb/Fork/Angle", forkEncoder.getAbsolutePosition().getValueAsDouble());
        BreakerLog.log("Climb/Fork/Sensor", forkSensor.getAsBoolean());
        BreakerLog.log("Climb/Fork/Extended", isForkContacting());
        BreakerLog.log("Climb/Fork/PositionVoltage", forkPositionVolt.Position);

        BreakerLog.log("Climb/Winch/Motor", winch);
        BreakerLog.log("Climb/Winch/Position", winchPositionVolt.Position);
        BreakerLog.log("Climb/State", currentClimbState);

        if (RobotState.isDisabled()) {
            setStateFunc(ClimbState.NEUTRAL);
        }
    }

    private void pullWinch() {
        if (currentClimbState.getWinchState() != WinchState.ROLLED) return;

        Angle roll = imu.getRoll().getValue();
        winchPositionVolt.FeedForward = ClimbConstants.kG * Math.cos(roll.in(Radians));
    }
}
