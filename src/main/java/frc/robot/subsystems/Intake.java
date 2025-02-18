// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static frc.robot.Constants.IntakeConstants.kPivotEncoderOffset;
import static frc.robot.Constants.IntakeConstants.kPivotTolerence;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.BreakerLib.sensors.BreakerDigitalSensor;
import frc.robot.BreakerLib.util.factory.BreakerCANCoderFactory;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.subsystems.EndEffector.RollerState;

/** Add your docs here. */
public class Intake extends SubsystemBase{
    private TalonFX rollers;
    private TalonFX pivot;
    private CANcoder encoder;
    private BreakerDigitalSensor coralSensor;
    private MotionMagicVoltage pivotRequest;
    private DutyCycleOut rollerRequest;

    private IntakeState setpoint;

    public Intake() {
        rollers = new TalonFX(IntakeConstants.kIntakeRollersMotorID, SuperstructureConstants.kSuperstructureCANBus);
        pivot = new TalonFX(IntakeConstants.kIntakePivotMotorID, SuperstructureConstants.kSuperstructureCANBus);
        encoder = BreakerCANCoderFactory.createCANCoder(IntakeConstants.kIntakeCANCoderID, SuperstructureConstants.kSuperstructureCANBus, 0.5, kPivotEncoderOffset, SensorDirectionValue.CounterClockwise_Positive);
        coralSensor = BreakerDigitalSensor.fromDIO(0, true);
        setpoint = IntakeState.STOW;
        pivotRequest = new MotionMagicVoltage(IntakePivotState.RETRACTED.getAngle());
        rollerRequest = new DutyCycleOut(RollerState.NEUTRAL.getDutyCycle());
        configPivot();
        setClosestNeutral();
    }

    private void configPivot() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = IntakeConstants.kP;
        config.Slot0.kI = IntakeConstants.kI;
        config.Slot0.kD = IntakeConstants.kD;
        config.Slot0.kS = IntakeConstants.kS;
        config.Slot0.kA = IntakeConstants.kA;
        config.Slot0.kG = IntakeConstants.kG;
        config.Slot0.kV = IntakeConstants.kV;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.FeedbackRemoteSensorID = IntakeConstants.kIntakeCANCoderID;
        config.Feedback.RotorToSensorRatio = IntakeConstants.kPivotGearRatio.getRatioToOne();
        config.CurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(50)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(40)
            .withSupplyCurrentLowerLimit(40)
            .withSupplyCurrentLowerTime(0.1)
            .withSupplyCurrentLimitEnable(true);

        
        config.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.kMotionMagicCruiseVelocity.in(RotationsPerSecond);
        config.MotionMagic.MotionMagicAcceleration = IntakeConstants.kMotionMagicAcceleration.in(RotationsPerSecondPerSecond);

        pivot.getConfigurator().apply(config);
    }

    public Command setState(IntakeState state, boolean waitForSuccess) {
        return Commands.runOnce(() -> setStateFunc(state), this).andThen(Commands.waitUntil(() -> atSetpoint() || !waitForSuccess));
    }

    private void setStateFunc(IntakeState state) {
        setpoint = state;
        setAngle(state.getPivotState().getAngle());
        setRoller(state.getRollerState());
    }

    private void setAngle(Angle angle) {
        pivotRequest.withPosition(angle);
        pivot.setControl(pivotRequest);
    }

    private void setRoller(IntakeRollerState rollerState) {
        double dcOut = rollerState.getDutyCycleOut();
        rollers.setControl(rollerRequest.withOutput(dcOut));
    }

    public boolean atSetpoint() {
        return MathUtil.isNear(setpoint.getPivotState().getAngle().in(Radians), getPivotAngle().in(Radians), kPivotTolerence.in(Units.Radians));
    }

    public Angle getPivotAngle() {
        return encoder.getAbsolutePosition().getValue();
    }

    public boolean hasCoral() {
        return coralSensor.isTriggered();
    }

    private void setClosestNeutral() {
        double curAng = getPivotAngle().in(Degrees);
        double extDelta = Math.abs(IntakePivotState.EXTENDED.getAngle().in(Degrees) - curAng);
        double retDelta = Math.abs(IntakePivotState.RETRACTED.getAngle().in(Degrees) - curAng);
        if (extDelta > retDelta) {
            setStateFunc(IntakeState.STOW);
        } else {
            setStateFunc(IntakeState.EXTENDED_NEUTRAL);
        }
    }

    @Override
    public void periodic() {
        if (RobotState.isDisabled()) {
            setClosestNeutral();
        }
        BreakerLog.log("Intake/Rollers/Motor", rollers);
        BreakerLog.log("Intake/Rollers/State", setpoint.getRollerState().toString());
        BreakerLog.log("Intake/Pivot/Motor", pivot);
        BreakerLog.log("Intake/Pivot/Encoder", encoder);
        BreakerLog.log("Intake/Pivot/Setpoint/Name", setpoint.getPivotState().toString());
        BreakerLog.log("Intake/Pivot/Setpoint/Angle", setpoint.getPivotState().getAngle().in(Degrees));
        BreakerLog.log("Intake/Pivot/Setpoint/AtSetpoint", atSetpoint());
        BreakerLog.log("Intake/Pivot/Angle", getPivotAngle().in(Degrees));
    }

    public static enum IntakeState {
        INTAKE(IntakeRollerState.INTAKE, IntakePivotState.EXTENDED),
        EXTAKE(IntakeRollerState.EXTAKE, IntakePivotState.EXTENDED),
        EXTENDED_NEUTRAL(IntakeRollerState.NEUTRAL, IntakePivotState.EXTENDED),
        CLIMB(IntakeRollerState.NEUTRAL, IntakePivotState.CLIMB),
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
        INTAKE(-1),
        EXTAKE(1),
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
        EXTENDED(Rotations.of(0.035)),
        CLIMB(Degrees.of(45)),
        RETRACTED(Rotations.of(0.4));

        private Angle angle;
        private IntakePivotState(Angle angle) {
            this.angle = angle;
        }

        public Angle getAngle() {
            return angle;
        }
    }
}
