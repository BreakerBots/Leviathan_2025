// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ClimbConstants.*;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.util.factory.BreakerCANCoderFactory;
import frc.robot.BreakerLib.util.logging.BreakerLog;

/** The Climb subsystem consisting of a winch and a fork. 🍴**/
public class Climb extends SubsystemBase {
    private TalonFX climbMotor;
    private CANcoder climbEncoder;
    private Pigeon2 imu;

    private ClimbState currentClimbState = ClimbState.NEUTRAL;

    private MotionMagicVoltage climbPosition = new MotionMagicVoltage(0);
    private CoastOut coastOutRequest = new CoastOut();
    
    public Climb(Pigeon2 imu) {
        this.imu = imu;
        climbMotor = new TalonFX(kClimbMotorID);
        climbEncoder = BreakerCANCoderFactory.createCANCoder(kClimbCoder, kClimbCoderAbsoluteSensorDiscontinuityPoint, kClimbCoderOffset, SensorDirectionValue.CounterClockwise_Positive);

        setupConfigs();
    }
    
    private void setupConfigs() {
        var climbConfig = new TalonFXConfiguration();
        climbConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        climbConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kClimbReverseLimit.getRotations();
        climbConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        climbConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kClimbForwardLimit.getRotations();
        // climbConfig.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Enabled;
        // climbConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        // climbConfig.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.FusedCANcoder;
        // climbConfig.ExternalFeedback.FeedbackRemoteSensorID = kClimbCoder;
        // climbConfig.ExternalFeedback.RotorToSensorRatio = kClimbGearRatio.getRatioToOne();
        climbConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        climbConfig.Feedback.FeedbackRemoteSensorID = kClimbCoder;
        climbConfig.Feedback.RotorToSensorRatio = kClimbGearRatio.getRatioToOne();

        climbConfig.CurrentLimits = kClimbCurrentLimits;
        climbConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climbConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        var magicConfig = new MotionMagicConfigs();
        magicConfig.MotionMagicAcceleration = kClimbMotionMagicAcceleration;
        magicConfig.MotionMagicCruiseVelocity = kClimbMotionMagicCruiseVelocity;

        climbConfig.MotionMagic = magicConfig;
        
        climbMotor.getConfigurator().apply(climbConfig);
    }
    
    private void setStateFunc(ClimbState changeState) {
        currentClimbState = changeState;

        if (currentClimbState == ClimbState.NEUTRAL) {
            climbMotor.setControl(coastOutRequest);
        } else {
            climbPosition.withPosition(currentClimbState.getAngle());
            climbMotor.setControl(climbPosition);
        }
    }

    public Command setState(ClimbState state, boolean waitForSuccess) {
        return Commands.runOnce(() -> setStateFunc(state)).andThen(Commands.waitUntil(() -> isAtTargetState() || !waitForSuccess));
    }

    public enum ClimbState {
        CLIMBING(kClimbingPosition),
        EXTENDED(kExtendedPosition),
        STOW(kStowPosition),
        NEUTRAL(kNeutralPosition);

        private Angle angle;

        private ClimbState(Angle angle) {
            this.angle = angle;
        }

        public Angle getAngle() {
            return angle;
        }
    }

    public boolean isAtTargetState() {
        return switch (currentClimbState) {
            case EXTENDED -> MathUtil.isNear(kExtendedThreshold.in(Rotations), getClimbCoderAngle().in(Rotations), 1e-5);
            // does not account for foot.
            case STOW -> MathUtil.isNear(0, getClimbCoderAngle().in(Rotations), 1e-5);
            
            case CLIMBING -> true;
            case NEUTRAL -> true;
        };
    }

    // public boolean isWinchAtTargetState() {
    //     if (currentClimbState.getWinchState() == WinchState.NEUTRAL) {
    //         return true;
    //     }
    //     boolean atPos = MathUtil.isNear(
    //         currentClimbState.getWinchState().getPosition().in(Rotations),
    //         getWinchAngle().in(Rotations), 
    //         currentClimbState.getWinchState().getTolerence().in(Rotations));

    //     boolean atVel = MathUtil.isNear(
    //         0.0,
    //         getWinchVelocity().in(RotationsPerSecond), 
    //         currentClimbState.getWinchState().getVelocityTolerence().in(RotationsPerSecond));

    //     return atPos && atVel;
    // }

    // public boolean isForkAtTargetState() {
    //     if (currentClimbState.getWinchState() == WinchState.NEUTRAL) {
    //         return true;
    //     }
    //     boolean atPos = MathUtil.isNear(
    //         currentClimbState.getForkState().getPosition().in(Rotations),
    //         getForkAngle().in(Rotations), 
    //         currentClimbState.getWinchState().getTolerence().in(Rotations));

    //     boolean atVel = MathUtil.isNear(
    //         0.0,
    //         getForkVelocity().in(RotationsPerSecond), 
    //         currentClimbState.getForkState().getVelocityTolerence().in(RotationsPerSecond));

    //     return atPos && atVel;
    // }

 
    public Angle getClimbCoderAngle() {
        return climbEncoder.getAbsolutePosition().getValue();
    }

    public AngularVelocity getForkVelocity() {
        return climbEncoder.getVelocity().getValue();
    }

    public ClimbState getState() {
        return currentClimbState;
    }
    
    @Override
    public void periodic() {
        BreakerLog.log("Climb/Motor", climbMotor);
        BreakerLog.log("Climb/Angle", climbEncoder.getAbsolutePosition().getValueAsDouble());
        BreakerLog.log("Climb/MotionMagicVoltagePosition", climbPosition.Position);

        BreakerLog.log("Climb/State", currentClimbState);

        if (RobotState.isDisabled()) {
            setStateFunc(ClimbState.NEUTRAL);
        }
    }

    // private void pullWinch() {
    //     if (currentClimbState.getWinchState() != WinchState.ROLLED) return;

    //     Angle roll = imu.getRoll().getValue();
    //     winchPositionVolt.FeedForward = ClimbConstants.kG * Math.cos(roll.in(Radians));
    // }
}
