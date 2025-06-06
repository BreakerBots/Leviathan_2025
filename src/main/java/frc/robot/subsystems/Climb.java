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
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.util.factory.BreakerCANCoderFactory;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.Constants.SuperstructureConstants;
public class Climb extends SubsystemBase {
    private TalonFX climbMotor;
    private CANcoder climbEncoder;
    private Pigeon2 imu;

    private ClimbState currentClimbState = ClimbState.STOW;

    private PIDController pid;
    private VoltageOut voltageOut = new VoltageOut(0);
    private CoastOut coastOutRequest = new CoastOut();

    private boolean emergencyLockout = false;
    
    public Climb(Pigeon2 imu) {
        this.imu = imu;
        climbMotor = new TalonFX(kClimbMotorID, SuperstructureConstants.kSuperstructureCANBus);
        climbEncoder = BreakerCANCoderFactory.createCANCoder(kClimbCoder,SuperstructureConstants.kSuperstructureCANBus, kClimbCoderAbsoluteSensorDiscontinuityPoint, kClimbCoderOffset, SensorDirectionValue.Clockwise_Positive);
        pid = new PIDController(200,0, 0.5);
        setupConfigs();
    }
    
    private void setupConfigs() {
        var climbConfig = new TalonFXConfiguration();

        climbConfig.CurrentLimits = kClimbCurrentLimits;
        climbConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climbConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        climbMotor.getConfigurator().apply(climbConfig);
    }
    
    private void setStateFunc(ClimbState changeState) {
        currentClimbState = changeState;
    }

    public Command setState(ClimbState state, boolean waitForSuccess) {
        return Commands.runOnce(() -> setStateFunc(state), this).andThen(Commands.waitUntil(() -> isAtTargetState() || !waitForSuccess));
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
            case EXTENDED -> MathUtil.isNear(getClimbCoderAngle().in(Rotations), kExtendedPosition.in(Rotations), kExtendedThreshold.in(Rotations));
            // does not account for foot.
            case STOW ->  MathUtil.isNear(getClimbCoderAngle().in(Rotations), kStowPosition.in(Rotations), kStowThreshold.in(Rotations));
            
            case CLIMBING -> true;
            case NEUTRAL -> true;
        };
    }

 
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
        BreakerLog.log("Climb/Angle", getClimbCoderAngle().in(Degrees));
        // BreakerLog.log("Climb/MotionMagicVoltagePosition", climbPosition.Position);

        BreakerLog.log("Climb/State/Name", currentClimbState);
        BreakerLog.log("Climb/State/Setpoint", currentClimbState.getAngle().in(Degrees));

        if (RobotState.isDisabled()) {
            setStateFunc(ClimbState.STOW);
        }

        double output = -pid.calculate(getClimbCoderAngle().in(Rotations), currentClimbState.getAngle().in(Rotations));
        double angle = getClimbCoderAngle().in(Rotations);


        boolean fwdLim = angle >= kClimbForwardLimit.in(Rotations);
        boolean revLim = angle <= kClimbReverseLimit.in(Rotations);
        boolean lockoutTrigger = angle <= kClimbReverseLimit.minus(Degrees.of(10)).in(Rotations);
        if (fwdLim) {
            output = MathUtil.clamp(output, 0, 16);
        }

        if (revLim) {
            output = MathUtil.clamp(output, -16, 0);
        }

        if (lockoutTrigger || emergencyLockout) {
            emergencyLockout = true;
            output = 0;
        }


        if (currentClimbState == ClimbState.NEUTRAL) {
            climbMotor.setControl(coastOutRequest);
        } else {
            climbMotor.setControl(voltageOut.withOutput(output));
        }
    }

    // private void pullWinch() {
    //     if (currentClimbState.getWinchState() != WinchState.ROLLED) return;

    //     Angle roll = imu.getRoll().getValue();
    //     winchPositionVolt.FeedForward = ClimbConstants.kG * Math.cos(roll.in(Radians));
    // }
}
