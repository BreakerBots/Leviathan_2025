package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ElevatorConstants.*;

public class Elevator extends SubsystemBase {
    private TalonFX left, right;
    private ElevatorSetpoint setpoint;
    private MotionMagicExpoVoltage motionMagicRequest;
    private Follower followRequest;
    public Elevator() {
        left = new TalonFX(kLeftMotorID);
        right = new TalonFX(kRightMotorID);
        configLeft();
        configRight();
        motionMagicRequest = new MotionMagicExpoVoltage(getNativePosition()).withEnableFOC(true);
        followRequest = new Follower(kLeftMotorID, kLeftMotorInverted != kRightMotorInverted);
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
        leftConfig.MotionMagic.MotionMagicCruiseVelocity =  kRotationsToMeters.getInput(kMotionMagicCruiseVelocity.in(MetersPerSecond));
        leftConfig.MotionMagic.MotionMagicAcceleration =  kRotationsToMeters.getInput(kMotionMagicAcceleration.in(MetersPerSecondPerSecond));
        leftConfig.MotionMagic.MotionMagicJerk = kRotationsToMeters.getInput(kMotionMagicJerk.in(MetersPerSecondPerSecond.per(Second)));

        leftConfig.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit.in(Amps);
        leftConfig.CurrentLimits.SupplyCurrentLowerLimit = kSupplyLowerCurrentLimit.in(Amps);
        leftConfig.CurrentLimits.SupplyCurrentLowerTime = kSupplyLowerCurrentLimitTime.in(Seconds);
        leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        leftConfig.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit.in(Amps);
        leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        left.getConfigurator().apply(leftConfig);
    }

    private void configRight() {
        TalonFXConfiguration rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.Inverted = kRightMotorInverted;

        rightConfig.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit.in(Amps);
        rightConfig.CurrentLimits.SupplyCurrentLowerLimit = kSupplyLowerCurrentLimit.in(Amps);
        rightConfig.CurrentLimits.SupplyCurrentLowerTime = kSupplyLowerCurrentLimitTime.in(Second);
        rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        rightConfig.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit.in(Amps);
        rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        right.getConfigurator().apply(rightConfig);

    }

    public Command set(ElevatorSetpoint setpoint) {
        this.setpoint = setpoint;
        return Commands.runOnce(() -> setControl(setpoint.getNativeSetpoint()), this)
        .andThen(Commands.waitUntil(this::atSetpoint));
    }

    private void setControl(Angle nativeSetpoint) {
        motionMagicRequest.withPosition(nativeSetpoint);
        left.setControl(motionMagicRequest);
        right.setControl(followRequest);
    }

    public boolean atSetpoint() {
        return atSetpointHeight() && atSetpointVel();
    }


    public boolean atSetpointHeight() {
        return MathUtil.isNear(setpoint.getHeight().in(Meters), getHeight().in(Meters), setpoint.getTolerence().in(Meters));
    }

    public boolean atSetpointVel() {
        return MathUtil.isNear(0.0, getVelocity().in(MetersPerSecond), setpoint.getVelocityTolerence().in(MetersPerSecond));
    }

    private Angle getNativePosition() {
        return left.getPosition().getValue();
    }

    public Distance getHeight() {
        return Meters.of(kRotationsToMeters.getOutput(getNativePosition().in(Rotations)));
    }

    public LinearVelocity getVelocity() {
        return MetersPerSecond.of(kRotationsToMeters.getOutput(left.getVelocity().getValue().in(RotationsPerSecond)));
    }

    public LinearAcceleration getAcceleration() {
        return MetersPerSecondPerSecond.of(kRotationsToMeters.getOutput(left.getAcceleration().getValue().in(RotationsPerSecondPerSecond)));
    }

    public static class ElevatorSetpoint {
        private Distance height;
        private Distance tolerence;
        private LinearVelocity velocityTolerence;
        public ElevatorSetpoint(Distance height, Distance tolerence, LinearVelocity velocityTolerence) {
            this.height = Meters.of(MathUtil.clamp(height.in(Meters), kMinHeight.in(Meters), kMaxHeight.in(Meters)));
            this.tolerence = tolerence;
            this.velocityTolerence = velocityTolerence;
        }
        
        public Distance getHeight() {
            return height;
        }

        public Distance getTolerence() {
            return tolerence;
        }

        public LinearVelocity getVelocityTolerence() {
            return velocityTolerence;
        }

        public ElevatorSetpoint(Distance height) {
            this(height, kDefaultHeightTolerence, kDefaultVelocityTolerence);
        }

        public Angle getNativeSetpoint() {
            return Rotations.of(kRotationsToMeters.getInput(height.in(Meters)));
        }

        public static final ElevatorSetpoint L1 = new ElevatorSetpoint(Meters.of(0.0));
        public static final ElevatorSetpoint L2 = new ElevatorSetpoint(Meters.of(0.0));
        public static final ElevatorSetpoint L3 = new ElevatorSetpoint(Meters.of(0.0));
        public static final ElevatorSetpoint L4 = new ElevatorSetpoint(Meters.of(0.0));
        public static final ElevatorSetpoint HANDOFF = new ElevatorSetpoint(Meters.of(0.0));
        public static final ElevatorSetpoint STOW = new ElevatorSetpoint(Meters.of(0.0));
    }

    
}
