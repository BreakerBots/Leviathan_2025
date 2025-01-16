// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.sensors.BreakerDigitalSensor;
import frc.robot.BreakerLib.util.logging.BreakerLog;

import static frc.robot.Constants.IntakeConstants.*;

/** Add your docs here. */
public class Intake extends SubsystemBase{
    private TalonFX rollers;
    private TalonFX pivotLeft;
    private TalonFX pivotRight;
    private CANcoder encoder;
    private BreakerDigitalSensor coralSensor;
    private MotionMagicExpoVoltage pivotRequest;
    private Follower followRequest;
    private DutyCycleOut rollerRequest;

    private IntakeState setpoint;

    public Intake() {
        
    }

    public Command setState(IntakeState state, boolean waitForSuccess) {
        return Commands.runOnce(() -> setStateFunc(state), this).andThen(Commands.waitUntil(() -> atSetpoint() || !waitForSuccess));
    }

    private void setStateFunc(IntakeState state) {
        setAngle(state.getPivotState().getAngle());
        setRoller(state.getRollerState());
    }

    private void setAngle(Rotation2d angle) {
        pivotRequest.withPosition(angle.getRotations());
        pivotLeft.setControl(pivotRequest);
        pivotRight.setControl(followRequest);
    }

    private void setRoller(IntakeRollerState rollerState) {
        double dcOut = rollerState.getDutyCycleOut();
        rollers.setControl(rollerRequest.withOutput(dcOut));
    }

    public boolean atSetpoint() {
        return MathUtil.isNear(setpoint.getPivotState().getAngle().getRadians(), getPivotAngle().getRadians(), kPivotTolerence.in(Units.Radians));
    }

    public Rotation2d getPivotAngle() {
        return null;
    }

    public boolean hasCoral() {
        return coralSensor.isTriggered();
    }

    @Override
    public void periodic() {
        BreakerLog.log("Intake/Rollers/Motor", rollers);
        BreakerLog.log("Intake/Rollers/State", setpoint.getRollerState().toString());
        BreakerLog.log("Intake/Pivot/Motors/Left", pivotLeft);
        BreakerLog.log("Intake/Pivot/Motors/Right", pivotRight);
        BreakerLog.log("Intake/Pivot/Encoder", encoder);
        BreakerLog.log("Intake/Pivot/Setpoint/Name", setpoint.getPivotState().toString());
        BreakerLog.log("Intake/Pivot/Setpoint/Angle", setpoint.getPivotState().getAngle());
        BreakerLog.log("Intake/Pivot/Setpoint/AtSetpoint", atSetpoint());
        BreakerLog.log("Intake/Pivot/Angle", getPivotAngle());
    }

    public static enum IntakeState {
        INTAKE(IntakeRollerState.INTAKE, IntakePivotState.EXTENDED),
        EXTAKE(IntakeRollerState.EXTAKE, IntakePivotState.EXTENDED),
        EXTENDED_NEUTRAL(IntakeRollerState.NEUTRAL, IntakePivotState.EXTENDED),
        STOW(IntakeRollerState.NEUTRAL, IntakePivotState.RETRACTED),;
        private IntakeRollerState rollerState;
        private IntakePivotState pivotState;
        private IntakeState(IntakeRollerState rollerState, IntakePivotState pivotState) {
            this.rollerState = rollerState;
            this.pivotState = pivotState;
        }

        public IntakePivotState getPivotState() {
            return pivotState;
        }

        public IntakeRollerState getRollerState() {
            return rollerState;
        }
    }

    public static enum IntakeRollerState {
        INTAKE(-0.8),
        EXTAKE(0.8),
        NEUTRAL(0.0);
        private double dutyCycleOut;

        private IntakeRollerState(double dutyCycleOut) {
            this.dutyCycleOut = dutyCycleOut;
        }

        public double getDutyCycleOut() {
            return dutyCycleOut;
        }
    }

    public static enum IntakePivotState {
        EXTENDED(Rotation2d.fromDegrees(0)),
        RETRACTED(Rotation2d.fromDegrees(90));

        private Rotation2d angle;
        private IntakePivotState(Rotation2d angle) {
            this.angle = angle;
        }

        public Rotation2d getAngle() {
            return angle;
        }
    }
}
