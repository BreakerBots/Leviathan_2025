package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.EndEffectorConstants.*;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S1FloatStateValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.sensors.BreakerDigitalSensor;
import frc.robot.BreakerLib.util.factory.BreakerCANCoderFactory;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffector extends SubsystemBase {
    private TalonSRX kicker;
    private TalonSRX rollers;
    private TalonFX wrist;
    private CANcoder wristEncoder;
    private CANdi candi;
    private BreakerDigitalSensor coralSensor;
    private Canandcolor algaeSensor;
    private MotionMagicVoltage wristRequest;
    private EndEffectorSetpoint setpoint;
    private EndEffectorWristLimits wristLimits;
    
    public EndEffector() {
        wristEncoder = BreakerCANCoderFactory.createCANCoder(EndEffectorConstants.kEndEffectorCANCoderID, kWristDiscontinuityPoint, kWristEncoderOffset, SensorDirectionValue.Clockwise_Positive);
        wrist = new TalonFX(EndEffectorConstants.kEndEffectorPivotMotorID);
        candi = new CANdi(EndEffectorConstants.kEndEffectorCANdiID);
        kicker = new TalonSRX(kEndEffectorKickerID);
        rollers = new TalonSRX(kEndEffectorRollerID);
        configCandi();
        wristRequest = new MotionMagicVoltage(getWristAngle());

        configWrist();
    }

    private void configCandi() {
        CANdiConfiguration config = new CANdiConfiguration();
        config.DigitalInputs.S1CloseState = S1CloseStateValue.CloseWhenHigh;
        config.DigitalInputs.S1FloatState = S1FloatStateValue.FloatDetect;
        candi.getConfigurator().apply(config);

        coralSensor = BreakerDigitalSensor.fromCANdiS1(candi);
    }
 
    private void configWrist() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Feedback.FeedbackRemoteSensorID = 51;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.RotorToSensorRatio = kWristRatio.getRatioToOne();

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kV = kV;
        config.Slot0.kS = kS;
        config.Slot0.kA = kA;
        config.Slot0.kG = kG;

        config.MotionMagic.MotionMagicCruiseVelocity = kMotionMagicCruiseVelocity.in(RotationsPerSecond);
        config.MotionMagic.MotionMagicAcceleration = kMotionMagicAcceleration.in(RotationsPerSecondPerSecond);

        config.CurrentLimits = kWristCurrentLimits;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
    }


    public void setWristLimits(EndEffectorWristLimits limits) {
        wristLimits = limits;
        wrist.getConfigurator().apply(wristLimits.getSoftLimits());
    }

    public Command set(EndEffectorSetpoint setpoint, boolean waitForSuccess) {
        return Commands.runOnce(() -> setControl(setpoint), this).andThen(Commands.waitUntil(() -> isAtSetpoint() || !waitForSuccess));
    }

    private void setControl(EndEffectorSetpoint setpoint) {
        this.setpoint = setpoint;
        setRollerState(setpoint.rollerState());
        setKicker(setpoint.kickerState());
        setWrist(setpoint.wristSetpoint().getSetpoint());
    }
 
    private void setRollerState(RollerState rollerState) {
        rollers.configSupplyCurrentLimit(rollerState.getCurrentLimitConfig());
        rollers.set(ControlMode.PercentOutput, rollerState.getDutyCycle());
    }

    private void setKicker(KickerState kickerState) {
        kicker.configSupplyCurrentLimit(kickerState.getCurrentLimitConfig());
        kicker.set(ControlMode.PercentOutput, kickerState.getDutyCycle());
    }

    private void setWrist(Angle setpoint) {
        wrist.setControl(wristRequest.withPosition(setpoint));
    }


    public boolean hasCoral() {
        return coralSensor.getAsBoolean();
    }

    public boolean isAlgaeVisable() {
        return getColorDelta(algaeSensor.getColor().toWpilibColor(), kAlgaeColor) <= kMaxColorDelta;
    }

    private boolean isAlgaeProximityBelowThresh() {
        return algaeSensor.getProximity() <= kHasAlgaeProximityThresh;
    }

    public boolean hasAlgae() {
        return isAlgaeVisable() && isAlgaeProximityBelowThresh();
    }

    public Angle getWristAngle() {
        return wristEncoder.getAbsolutePosition().getValue();
    }

    public AngularVelocity getWristVelocity() {
        return wristEncoder.getVelocity().getValue();
    }

    public boolean isAtSetpoint() {
        return isAtAngleSetpoint() && isAtWristVelocitySetpoint();
    }

    public EndEffectorSetpoint getSetpoint() {
        return setpoint;
    }

    private boolean isAtAngleSetpoint() {
        return MathUtil.isNear(setpoint.wristSetpoint().getSetpoint().in(Rotations), getWristAngle().in(Rotations), setpoint.wristSetpoint().getTolerence().in(Rotations), -0.5, 0.5);
    }

    private boolean isAtWristVelocitySetpoint() {
        return MathUtil.isNear(0.0, getWristVelocity().in(RadiansPerSecond), setpoint.wristSetpoint().getVelocityTolerence().in(RadiansPerSecond));
    }


    public static double getColorDelta(Color a, Color b) {
        var at = new Translation3d(a.red, a.green, a.blue);
        var bt = new Translation3d(b.red, b.green, b.blue);
        return at.getDistance(bt);
    }

    @Override
    public void periodic() {
        if (RobotState.isDisabled()) {
            setControl(new EndEffectorSetpoint(new WristSetpoint(getWristAngle()), RollerState.NEUTRAL, KickerState.NEUTRAL));
        }

        BreakerLog.log("EndEffector/Wrist/Motor", wrist);
        BreakerLog.log("EndEffector/Wrist/Encoder", wristEncoder);
        BreakerLog.log("EndEffector/Wrist/Setpoint/Angle", setpoint.wristSetpoint.setpoint.in(Degrees));
        BreakerLog.log("EndEffector/Wrist/Setpoint/Tolerence", setpoint.wristSetpoint.tolerence.in(Degrees));
        BreakerLog.log("EndEffector/Wrist/Setpoint/VelTolerence", setpoint.wristSetpoint.velocityTolerence.in(DegreesPerSecond));
        BreakerLog.log("EndEffector/Wrist/Setpoint/Error", Math.abs(getWristAngle()
       .in(Degrees)) - setpoint.wristSetpoint.setpoint.in(Degrees));

        BreakerLog.log("EndEffector/RollerMotor/SupplyCurrent", rollers.getSupplyCurrent());
        BreakerLog.log("EndEffector/RollerMotor/StatorCurrent", rollers.getStatorCurrent());
        BreakerLog.log("EndEffector/RollerMotor/Output", rollers.getMotorOutputPercent());
        BreakerLog.log("EndEffector/KickerMotor/SupplyCurrent", rollers.getSupplyCurrent());
        BreakerLog.log("EndEffector/KickerMotor/StatorCurrent", rollers.getStatorCurrent());
        BreakerLog.log("EndEffector/KickerMotor/Output", rollers.getMotorOutputPercent());

        BreakerLog.log("EndEffector/HasCoral", hasCoral());
        BreakerLog.log("EndEffector/AlgaeSensor/HasAlgae", hasAlgae());
        Color c = algaeSensor.getColor().toWpilibColor();
        double cd = getColorDelta(c, kAlgaeColor);
        BreakerLog.log("EndEffector/AlgaeSensor/SeesAlgae", cd <= kMaxColorDelta);
        BreakerLog.log("EndEffector/AlgaeSensor/Color/Delta", cd);
        BreakerLog.log("EndEffector/AlgaeSensor/Color/R", c.red);
        BreakerLog.log("EndEffector/AlgaeSensor/Color/G", c.green);
        BreakerLog.log("EndEffector/AlgaeSensor/Color/B", c.blue);
    }


    public static enum RollerState {
        INTAKE(-0.5, kNormalRollerCurrentLimitConfig),
        EXTAKE(1.0, kNormalRollerCurrentLimitConfig),
        INTAKE_ALGAE(-0.75, kNormalRollerCurrentLimitConfig),
        HOLD_ALGAE(-0.15, kAlgaeHoldRollerCurrentLimitConfig),
        NEUTRAL(0.0, kNormalRollerCurrentLimitConfig);
        private double dutyCycleOut;
        private SupplyCurrentLimitConfiguration currentLimitConfig;
        private RollerState(double dutyCycleOut, SupplyCurrentLimitConfiguration currentLimitConfig) {
            this.dutyCycleOut = dutyCycleOut;
            this.currentLimitConfig = currentLimitConfig;
        }

        public double getDutyCycle() {
            return dutyCycleOut;
        }

        public SupplyCurrentLimitConfiguration getCurrentLimitConfig() {
            return currentLimitConfig;
        }
    }

    public static enum EndEffectorWristLimits {
        ELEVATOR_EXTENDED(kElevatorExtendedLimits),
        FLOOR_RESTRICTED(kFloorRestrictedLimits),
        NORMAL(kNormalLimits);
        private SoftwareLimitSwitchConfigs softLimits;
        private EndEffectorWristLimits(SoftwareLimitSwitchConfigs softLimits) {
            this.softLimits = softLimits;
        }

        public SoftwareLimitSwitchConfigs getSoftLimits() {
            return softLimits;
        }
    }

    public static enum KickerState {
        KICK(-1.0, kNormalKickerCurrentLimitConfig),
        INTAKE(-0.8, kNormalKickerCurrentLimitConfig),
        EXTAKE(0.8, kNormalKickerCurrentLimitConfig),
        HOLD(-0.1, kAlgaeHoldKickerCurrentLimitConfig),
        NEUTRAL(0.0, kNormalKickerCurrentLimitConfig);

        private double dutyCycleOut;
        private SupplyCurrentLimitConfiguration currentLimitConfig;
        private KickerState(double dutyCycleOut, SupplyCurrentLimitConfiguration currentLimitConfig) {
            this.dutyCycleOut = dutyCycleOut;
            this.currentLimitConfig = currentLimitConfig;
        }

        public double getDutyCycle() {
            return dutyCycleOut;
        }
        
        public SupplyCurrentLimitConfiguration getCurrentLimitConfig() {
            return currentLimitConfig;
        }


    }

    public static class WristSetpoint {
        private Angle setpoint;
        private Angle tolerence;
        private AngularVelocity velocityTolerence;
        public WristSetpoint(Angle setpoint, Angle tolerence, AngularVelocity velocityTolerence) {
            this.setpoint = setpoint;
            this.tolerence = tolerence;
            this.velocityTolerence = velocityTolerence;
        }

        public WristSetpoint(Angle setpoint) {
            this(setpoint, kDefaultWristAngleTolerence, kDefaultWristVelocityTolerence);
        }

        public Angle getSetpoint() {
            return setpoint;
        }

        public Angle getTolerence() {
            return tolerence;
        }

        public AngularVelocity getVelocityTolerence() {
            return velocityTolerence;
        }

        public boolean requiresFlip() {
            return setpoint.in(Degrees) >= kMaxElevatorRestrictedSafeAngle.in(Degrees);
        }
        
    }

    public static record EndEffectorSetpoint(WristSetpoint wristSetpoint, RollerState rollerState, KickerState kickerState) {

        public static final EndEffectorSetpoint STOW = 
            new EndEffectorSetpoint(
                new WristSetpoint(Rotations.of(0.43)), 
                RollerState.NEUTRAL, 
                KickerState.NEUTRAL
        );

        public static final EndEffectorSetpoint CORAL_GROUND_INTAKE_HANDOFF = 
            new EndEffectorSetpoint(
                new WristSetpoint(Degrees.of(30)), 
                RollerState.INTAKE, 
                KickerState.NEUTRAL
        );

        public static final EndEffectorSetpoint CORAL_GROUND_INTAKE_HANDOFF_REVERSE = 
            new EndEffectorSetpoint(
                new WristSetpoint(Degrees.of(180)), 
                RollerState.INTAKE, 
                KickerState.NEUTRAL
        );

        public static final EndEffectorSetpoint ALGAE_GROUND_INTAKE_NEUTRAL = 
            new EndEffectorSetpoint(
                new WristSetpoint(Degrees.of(-15)), 
                RollerState.INTAKE, 
                KickerState.INTAKE
        );

        public static final EndEffectorSetpoint ALGAE_GROUND_INTAKE = 
            new EndEffectorSetpoint(
                new WristSetpoint(Degrees.of(-15)), 
                RollerState.INTAKE, 
                KickerState.INTAKE
        );

        public static final EndEffectorSetpoint ALGAE_HOLD_GROUND = 
            new EndEffectorSetpoint(
                new WristSetpoint(Degrees.of(-15)), 
                RollerState.HOLD_ALGAE, 
                KickerState.HOLD
        );

        public static final EndEffectorSetpoint INTAKE_HUMAN_PLAYER_NEUTRAL = 
        new EndEffectorSetpoint(
            new WristSetpoint(Degrees.of(55)), 
            RollerState.NEUTRAL, 
            KickerState.NEUTRAL
        );

        public static final EndEffectorSetpoint INTAKE_HUMAN_PLAYER = 
        new EndEffectorSetpoint(
            new WristSetpoint(Degrees.of(55)), 
            RollerState.INTAKE, 
            KickerState.INTAKE
        );

        public static final EndEffectorSetpoint CLIMB = 
        new EndEffectorSetpoint(
            new WristSetpoint(Degrees.of(15)), 
            RollerState.NEUTRAL, 
            KickerState.NEUTRAL
        );

        public static final EndEffectorSetpoint L1_NEUTRAL = 
        new EndEffectorSetpoint(
            new WristSetpoint(Degrees.of(-65)), 
            RollerState.NEUTRAL, 
            KickerState.NEUTRAL
        );

        public static final EndEffectorSetpoint L2_NEUTRAL = 
        new EndEffectorSetpoint(
            new WristSetpoint(Degrees.of(-45)), 
            RollerState.NEUTRAL, 
            KickerState.NEUTRAL
        );
        public static final EndEffectorSetpoint L3_NEUTRAL = 
        new EndEffectorSetpoint(
            new WristSetpoint(Degrees.of(-45)), 
            RollerState.NEUTRAL, 
            KickerState.NEUTRAL
        );

        public static final EndEffectorSetpoint L4_NEUTRAL = 
        new EndEffectorSetpoint(
            new WristSetpoint(Degrees.of(-65)), 
            RollerState.NEUTRAL, 
            KickerState.NEUTRAL
        );

        public static final EndEffectorSetpoint L1_EXTAKE_CORAL = 
        new EndEffectorSetpoint(
            new WristSetpoint(Degrees.of(-65)), 
            RollerState.EXTAKE, 
            KickerState.NEUTRAL
        );

        public static final EndEffectorSetpoint L2_EXTAKE_CORAL = 
        new EndEffectorSetpoint(
            new WristSetpoint(Degrees.of(-45)), 
            RollerState.EXTAKE, 
            KickerState.NEUTRAL
        );
        public static final EndEffectorSetpoint L3_NEXTAKE_CORAL = 
        new EndEffectorSetpoint(
            new WristSetpoint(Degrees.of(-45)), 
            RollerState.EXTAKE, 
            KickerState.NEUTRAL
        );

        public static final EndEffectorSetpoint L4_EXTAKE_CORAL = 
        new EndEffectorSetpoint(
            new WristSetpoint(Degrees.of(-65)), 
            RollerState.EXTAKE, 
            KickerState.NEUTRAL
        );

        // // public 

        public static EndEffectorFlipDirection getFlipDirection(Angle from, Angle to) {
            double f = from.in(Rotations);
            double t = to.in(Rotations);
            
            final double minFlip = kMinFlipAngle.in(Rotations);
            final double maxFlip = kMaxFlipAngle.in(Rotations);
            
            if (f > t) { // if hell
                if (f < minFlip) {
                    return EndEffectorFlipDirection.NONE;
                } else if (f > maxFlip) {
                    return EndEffectorFlipDirection.FRONT_TO_BACK;
                } else {
                    // from angle is inside of the elevator?
                    return EndEffectorFlipDirection.FRONT_TO_BACK;
                }
            } else {
                if (f < minFlip) {
                    return EndEffectorFlipDirection.BACK_TO_FRONT;
                } else if (f > maxFlip) {
                    return EndEffectorFlipDirection.NONE;
                } else { // this branch is a little strange
                    return EndEffectorFlipDirection.BACK_TO_FRONT;
                }
            }
        }

        public static enum EndEffectorFlipDirection {
            FRONT_TO_BACK,
            BACK_TO_FRONT,
            NONE
        }
        
    }


}
