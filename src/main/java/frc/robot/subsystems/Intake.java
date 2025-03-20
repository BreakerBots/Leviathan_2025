// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.IntakeConstants.*;

import java.util.concurrent.locks.ReentrantLock;

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

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Robot;
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
    private DigitalInput rawCoralSensor;
    private MotionMagicVoltage pivotRequest;
    private DutyCycleOut rollerRequest;

    private IntakeState setpoint;

    public Intake() {
        rollers = new TalonFX(IntakeConstants.kIntakeRollersMotorID, SuperstructureConstants.kSuperstructureCANBus);
        pivot = new TalonFX(IntakeConstants.kIntakePivotMotorID, SuperstructureConstants.kSuperstructureCANBus);
        encoder = BreakerCANCoderFactory.createCANCoder(IntakeConstants.kIntakeCANCoderID, SuperstructureConstants.kSuperstructureCANBus, 0.5, kPivotEncoderOffset, SensorDirectionValue.CounterClockwise_Positive);
        rawCoralSensor = new DigitalInput(0);
        coralSensor = BreakerDigitalSensor.fromDIO(rawCoralSensor, false);
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

    public Command waitForCoralGroundIntakeL1AndStopRollers() {
        return this.new WaitForCoralGroundIntakeL1AndStopRollersCommand();
    }

    public Command setState(IntakeState state, boolean waitForSuccess) {
        if (Robot.isSimulation()) return Commands.runOnce(() -> setStateFunc(state), this).andThen(Commands.waitSeconds(0.2));
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

    private Intake getSelf() {
        return this;
    }

    @Override
    public void periodic() {
        if (RobotState.isDisabled()) {
            setClosestNeutral();
        }
        BreakerLog.log("Intake/HasCoral", hasCoral());
        BreakerLog.log("Intake/Rollers/Motor", rollers);
        BreakerLog.log("Intake/Rollers/State", setpoint.getRollerState().toString());
        BreakerLog.log("Intake/Pivot/Motor", pivot);
        BreakerLog.log("Intake/Pivot/Encoder", encoder);
        BreakerLog.log("Intake/Pivot/Setpoint/Name", setpoint.getPivotState().toString());
        BreakerLog.log("Intake/Pivot/Setpoint/Angle", setpoint.getPivotState().getAngle().in(Degrees));
        BreakerLog.log("Intake/Pivot/Setpoint/AtSetpoint", atSetpoint());
        BreakerLog.log("Intake/Pivot/Angle", getPivotAngle().in(Degrees));
    }

    private class WaitForCoralGroundIntakeL1AndStopRollersCommand extends Command {
        private AsynchronousInterrupt interrupt;
        private boolean endFlag;
        private ReentrantLock lock;
        private Timer timer;
        public WaitForCoralGroundIntakeL1AndStopRollersCommand() {
            lock = new ReentrantLock();
            interrupt = new AsynchronousInterrupt(rawCoralSensor, this::internalCallback);
            interrupt.setInterruptEdges(false, true);
            timer = new Timer();
            addRequirements(getSelf());
        }

        private void internalCallback(Boolean isRiseing, Boolean isFalling) {
            try {
                lock.lock();
                if (isFalling && timer.hasElapsed(0.2)) {
                    endFlag = true;
                    setStateFunc(IntakeState.EXTENDED_NEUTRAL);
                }
            } finally {
                lock.unlock();
            }
        }

        @Override
        public void initialize() {
            timer.reset();
            timer.start();
            endFlag = false;
            interrupt.enable();
        }

        @Override
        public void end(boolean interrupted) {
            timer.stop();
            endFlag = false;
            interrupt.disable();
        }

        @Override
        public boolean isFinished() {
            return endFlag;
        }
    }

    public static record IntakeState(IntakeRollerState rollerState, IntakePivotState pivotState) {
        public static final IntakeState INTAKE = new IntakeState(IntakeRollerState.INTAKE, IntakePivotState.EXTENDED);
        public static final IntakeState EXTAKE = new IntakeState(IntakeRollerState.EXTAKE, IntakePivotState.EXTENDED);

        public static final IntakeState EXTENDED_NEUTRAL = new IntakeState(IntakeRollerState.NEUTRAL, IntakePivotState.EXTENDED);
        public static final IntakeState CLEAR = new IntakeState(IntakeRollerState.NEUTRAL, IntakePivotState.CLEAR);


        public static final IntakeState L1_INTAKE = new IntakeState(IntakeRollerState.INTAKE_L1, IntakePivotState.EXTENDED);
        public static final IntakeState L1_NEUTRAL = new IntakeState(IntakeRollerState.NEUTRAL, IntakePivotState.L1);
        public static final IntakeState L1_EXTAKE = new IntakeState(IntakeRollerState.EXTAKE, IntakePivotState.L1);

        public static final IntakeState CLIMB = new IntakeState(IntakeRollerState.NEUTRAL, IntakePivotState.CLIMB);
        public static final IntakeState STOW = new IntakeState(IntakeRollerState.NEUTRAL, IntakePivotState.RETRACTED);
        public static final IntakeState STOW2 = new IntakeState(IntakeRollerState.NEUTRAL, IntakePivotState.RETRACTED2);

        public IntakePivotState getPivotState() {
            return pivotState;
        }

        public IntakeRollerState getRollerState() {
            return rollerState;
        }
    }

    public static enum IntakeRollerState {
        INTAKE(-0.9),
        INTAKE_L1(-0.9),
        // INTAKE_ALGAE(0.5),
        // HOLD_ALGAE(0.1),
        // EXTAKE_ALGAE(-0.8),
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
        EXTENDED(Rotations.of(0.035).minus(Degrees.of(3))),
        // ALGAE(Rotations.of(0.22)),
        // ALGAE_HOLD(Rotations.of(0.34)),
        CLEAR(Degrees.of(0.25)),
        CLIMB(Degrees.of(45)),
        RETRACTED(Rotations.of(0.35)),
        RETRACTED2(Rotations.of(0.4)),
        L1(Rotations.of(0.3));


        private Angle angle;
        private IntakePivotState(Angle angle) {
            this.angle = angle;
        }

        public Angle getAngle() {
            return angle;
        }
    }
}
