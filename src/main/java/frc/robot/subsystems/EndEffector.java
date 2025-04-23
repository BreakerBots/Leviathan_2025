package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.EndEffectorConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S1FloatStateValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.BreakerLib.sensors.BreakerDigitalSensor;
import frc.robot.BreakerLib.util.factory.BreakerCANCoderFactory;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.subsystems.EndEffector.EndEffectorSetpoint.EndEffectorFlipDirection;

public class EndEffector extends SubsystemBase {
    private TalonFXS rollers;
    private TalonFXS wrist;
    private CANcoder wristEncoder;
    private CANdi candi;
    private BreakerDigitalSensor coralSensor;
    // private Canandcolor algaeSensor;
    private MotionMagicVoltage wristRequest;
    private DutyCycleOut rollerRequest;
    private EndEffectorSetpoint setpoint;
    private EndEffectorWristLimits wristLimits;

    private Timer logRefreshTimer = new Timer();
    
    public EndEffector() {
        wristEncoder = BreakerCANCoderFactory.createCANCoder(EndEffectorConstants.kEndEffectorCANCoderID, kWristDiscontinuityPoint, kWristEncoderOffset, SensorDirectionValue.CounterClockwise_Positive);
        wrist = new TalonFXS(EndEffectorConstants.kEndEffectorPivotMotorID);
        candi = new CANdi(EndEffectorConstants.kEndEffectorCANdiID);
        // kicker = new TalonSRX(kEndEffectorKickerID);
        rollers = new TalonFXS(kEndEffectorRollerID);
        configCandi();
        wristRequest = new MotionMagicVoltage(getWristAngle());
        rollerRequest = new DutyCycleOut(0);
        logRefreshTimer.start();
        configWrist();
        configRollers();
    }

    private void configCandi() {
        CANdiConfiguration config = new CANdiConfiguration();
        config.DigitalInputs.S1CloseState = S1CloseStateValue.CloseWhenNotHigh;
        config.DigitalInputs.S1FloatState = S1FloatStateValue.FloatDetect;
        candi.getConfigurator().apply(config);

        coralSensor = BreakerDigitalSensor.fromCANdiS1(candi);
    }
 
    private void configWrist() {
        TalonFXSConfiguration config = new TalonFXSConfiguration();

        config.ExternalFeedback.FeedbackRemoteSensorID = 51;
        config.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.FusedCANcoder;
        config.ExternalFeedback.RotorToSensorRatio = kWristRatio.getRatioToOne();

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

        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        config.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Enabled;
        
        wrist.getConfigurator().apply(config);
    }

    private void configRollers() {
        TalonFXSConfiguration config = new TalonFXSConfiguration();
        config.CurrentLimits = kNormalRollerCurrentLimitConfig;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        rollers.getConfigurator().apply(config);
    }


    public void setWristLimits(EndEffectorWristLimits limits) {
        if (limits != wristLimits) {
            wristLimits = limits;
            wrist.getConfigurator().apply(wristLimits.getSoftLimits());
        }
    }

    public Command set(EndEffectorSetpoint setpoint, boolean waitForSuccess) {
        return Commands.runOnce(() -> setControl(setpoint), this).andThen(Commands.waitUntil(() -> isAtSetpoint() || !waitForSuccess));
    }


    private void setControl(EndEffectorSetpoint setpoint) {
        
        setRollerState(setpoint.rollerState());
        // setKicker(setpoint.kickerState());
        setWrist(setpoint.wristSetpoint().getSetpoint());
        this.setpoint = setpoint;
    }
 
    private void setRollerState(RollerState rollerState) {
        if (setpoint == null || rollerState != setpoint.rollerState()) {
            rollers.getConfigurator().apply(rollerState.getCurrentLimitConfig());
        }
        rollers.setControl(rollerRequest.withOutput(rollerState.getDutyCycle()));
    }

    // private void setKicker(KickerState kickerState) {
    //     kicker.configSupplyCurrentLimit(kickerState.getCurrentLimitConfig());
    //     kicker.set(ControlMode.PercentOutput, kickerState.getDutyCycle());
    // }

    private void setWrist(Angle setpoint) {
        wrist.setControl(wristRequest.withPosition(setpoint));
    }


    public boolean hasCoral() {
        return coralSensor.getAsBoolean();
    }

    public boolean isAlgaeVisable() {
        return false; //getColorDelta(algaeSensor.getColor().toWpilibColor(), kAlgaeColor) <= kMaxColorDelta;
    }

    private boolean isAlgaeProximityBelowThresh() {
        return false; //algaeSensor.getProximity() <= kHasAlgaeProximityThresh;
    }

    public boolean hasAlgae() {
        return false;//isAlgaeVisable() && isAlgaeProximityBelowThresh();
    }

    public Angle getWristAngle() {
        return wristEncoder.getAbsolutePosition(false).getValue();
    }

    public AngularVelocity getWristVelocity() {
        return wristEncoder.getVelocity(false).getValue();
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

    private void refreshLogs() {
        BreakerLog.log("EndEffector/Wrist/Motor", wrist);
        
        BreakerLog.log("EndEffector/RollerMotor", rollers);
        
        BreakerLog.log("EndEffector/Wrist/Setpoint/Satisifed", isAtSetpoint());
        BreakerLog.log("EndEffector/Wrist/Angle", getWristAngle().in(Degrees));

        BreakerLog.log("EndEffector/HasCoral", hasCoral());
        BreakerLog.log("EndEffector/AlgaeSensor/HasAlgae", hasAlgae());
        BreakerLog.log("EndEffector/Wrist/Setpoint/Angle", setpoint.wristSetpoint.setpoint.in(Degrees));
        BreakerLog.log("EndEffector/Wrist/Setpoint/Tolerence", setpoint.wristSetpoint.tolerence.in(Degrees));
        BreakerLog.log("EndEffector/Wrist/Setpoint/VelTolerence", setpoint.wristSetpoint.velocityTolerence.in(DegreesPerSecond));
    }
    
    @Override
    public void periodic() {
        if (RobotState.isDisabled()) {
            setControl(new EndEffectorSetpoint(new WristSetpoint(getWristAngle()), RollerState.NEUTRAL));
        }
        
        if (logRefreshTimer.advanceIfElapsed(MiscConstants.kInfrequentLogRate)) {
            refreshLogs();
        }
        
        BreakerLog.log("EndEffector/Wrist/Encoder", wristEncoder);
        BreakerLog.log("EndEffector/Wrist/Setpoint/Error", Math.abs(getWristAngle()
       .in(Degrees) - setpoint.wristSetpoint.setpoint.in(Degrees)));
       
        // BreakerLog.log("EndEffector/RollerMotor/Output", rollers.getMotorOutputPercent());
        // BreakerLog.log("EndEffector/KickerMotor/Output", rollers.getMotorOutputPercent());

        // Color c = algaeSensor.getColor().toWpilibColor();
        // double cd = getColorDelta(c, kAlgaeColor);
        // BreakerLog.log("EndEffector/AlgaeSensor/SeesAlgae", cd <= kMaxColorDelta);
        // BreakerLog.log("EndEffector/AlgaeSensor/Color/Delta", cd);
        // BreakerLog.log("EndEffector/AlgaeSensor/Color/R", c.red);
        // BreakerLog.log("EndEffector/AlgaeSensor/Color/G", c.green);
        // BreakerLog.log("EndEffector/AlgaeSensor/Color/B", c.blue);
    }


    public static enum RollerState {
        INTAKE(1, kNormalRollerCurrentLimitConfig),
        EXTAKE(-1.0, kNormalRollerCurrentLimitConfig),
        EXTAKE_L1(-0.5, kNormalRollerCurrentLimitConfig),
        INTAKE_ALGAE(0.75, kNormalRollerCurrentLimitConfig),
        HOLD_ALGAE(1, kAlgaeHoldRollerCurrentLimitConfig),
        NEUTRAL(0.0, kNormalRollerCurrentLimitConfig);
        private double dutyCycleOut;
        private CurrentLimitsConfigs currentLimitConfig;
        private RollerState(double dutyCycleOut, CurrentLimitsConfigs currentLimitConfig) {
            this.dutyCycleOut = dutyCycleOut;
            this.currentLimitConfig = currentLimitConfig;
        }

        public double getDutyCycle() {
            return dutyCycleOut;
        }

        public CurrentLimitsConfigs getCurrentLimitConfig() {
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

    // public static enum KickerState {
    //     KICK(1.0, kNormalKickerCurrentLimitConfig),
    //     INTAKE(1, kNormalKickerCurrentLimitConfig),
    //     EXTAKE(-0.8, kNormalKickerCurrentLimitConfig),
    //     HOLD(0.1, kAlgaeHoldKickerCurrentLimitConfig),
    //     NEUTRAL(0.0, kNormalKickerCurrentLimitConfig);

    //     private double dutyCycleOut;
    //     private SupplyCurrentLimitConfiguration currentLimitConfig;
    //     private KickerState(double dutyCycleOut, SupplyCurrentLimitConfiguration currentLimitConfig) {
    //         this.dutyCycleOut = dutyCycleOut;
    //         this.currentLimitConfig = currentLimitConfig;
    //     }

    //     public double getDutyCycle() {
    //         return dutyCycleOut;
    //     }
        
    //     public SupplyCurrentLimitConfiguration getCurrentLimitConfig() {
    //         return currentLimitConfig;
    //     }


    // }

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

        public EndEffectorFlipDirection getFlipDirectionFrom(Angle from) {
            return EndEffectorSetpoint.getFlipDirection(from, setpoint);
        }
        
    }

    public static record EndEffectorSetpoint(WristSetpoint wristSetpoint, RollerState rollerState) {

        public static final EndEffectorSetpoint STOW = 
            new EndEffectorSetpoint(
                new WristSetpoint(Rotations.of(0.3)), 
                RollerState.NEUTRAL
        );

        public static final EndEffectorSetpoint EXTENDED_STOW = 
            new EndEffectorSetpoint(
                new WristSetpoint(Degrees.of(5)), 
                RollerState.NEUTRAL
        );

        public static final EndEffectorSetpoint FORCE_EJECT = 
            new EndEffectorSetpoint(
                new WristSetpoint(Degrees.of(5)), 
                RollerState.EXTAKE
        );

        public static final EndEffectorSetpoint CORAL_GROUND_HANDOFF_INTAKE = 
            new EndEffectorSetpoint(
                new WristSetpoint(Degrees.of(185)), 
                RollerState.INTAKE
        );

        public static final EndEffectorSetpoint CORAL_GROUND_HANDOFF_EXTAKE = 
            new EndEffectorSetpoint(
                new WristSetpoint(Degrees.of(185)), 
                RollerState.EXTAKE
        );

        public static final EndEffectorSetpoint INTAKE_HUMAN_PLAYER = 
        new EndEffectorSetpoint(
            new WristSetpoint(Degrees.of(43)), 
            RollerState.INTAKE
        );

        public static final EndEffectorSetpoint CLIMB = 
        new EndEffectorSetpoint(
            new WristSetpoint(Degrees.of(70)), 
            RollerState.NEUTRAL
        );

        public static final EndEffectorSetpoint L1_EXTAKE_CORAL = 
        new EndEffectorSetpoint(
            new WristSetpoint(Degrees.of(-20)), 
            RollerState.EXTAKE_L1
        );

        public static final EndEffectorSetpoint L2_EXTAKE_CORAL = 
        new EndEffectorSetpoint(
            new WristSetpoint(Degrees.of(-32.0)), 
            RollerState.EXTAKE
        );

        public static final EndEffectorSetpoint L3_EXTAKE_CORAL = 
        new EndEffectorSetpoint(
            new WristSetpoint(Degrees.of(-35)), 
            RollerState.EXTAKE
        );

        public static final EndEffectorSetpoint L4_EXTAKE_CORAL = 
        new EndEffectorSetpoint(
            new WristSetpoint(Degrees.of(-40)), 
            RollerState.EXTAKE
        );

        public static final EndEffectorSetpoint REEF_ALGAE_LOW_INTAKE = 
        new EndEffectorSetpoint(
            new WristSetpoint(Degrees.of(-5)), 
            RollerState.INTAKE_ALGAE
        );

        public static final EndEffectorSetpoint REEF_ALGAE_HIGH_INTAKE = 
        new EndEffectorSetpoint(
            new WristSetpoint(Degrees.of(-5)), 
            RollerState.INTAKE_ALGAE
        );


        public static EndEffectorFlipDirection getFlipDirection(Angle from, Angle to) {
            double f = from.in(Rotations);
            double t = to.in(Rotations);
            
            final double minFlip = kMinFlipAngle.in(Rotations);
            final double maxFlip = kMaxFlipAngle.in(Rotations);
            
            if (f > t) { // if hell
                if (f < minFlip && t < minFlip) {
                    return EndEffectorFlipDirection.NONE;
                } else if (f > maxFlip && t < minFlip) {
                    return EndEffectorFlipDirection.FRONT_TO_BACK;
                } else if (f > maxFlip && t > maxFlip) {
                    return EndEffectorFlipDirection.NONE;
                }
                return EndEffectorFlipDirection.BACK_TO_FRONT;
            } else {
                if (f < minFlip && t < minFlip) {
                    return EndEffectorFlipDirection.NONE;
                } else if (f > maxFlip && t > maxFlip) {
                    return EndEffectorFlipDirection.NONE;
                } else if (f < minFlip && t > maxFlip) {
                    return EndEffectorFlipDirection.BACK_TO_FRONT;
                }
                return EndEffectorFlipDirection.FRONT_TO_BACK;
            }
        }

        public static enum EndEffectorFlipDirection {
            FRONT_TO_BACK,
            BACK_TO_FRONT,
            NONE
        }
        
    }


}
