package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.SuperstructureConstants.kSuperstructureCANBus;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MiscConstants;
import frc.robot.BreakerLib.util.commands.TimedWaitUntilCommand;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.BreakerLib.util.logging.LoggedAlert;

public class Elevator extends SubsystemBase {
    private TalonFX left, right;
    private ElevatorSetpoint setpoint;
    private MotionMagicVoltage motionMagicRequest;
    private VoltageOut voltageRequest;
    private Follower followRequest;
    private NeutralOut neutralRequest;
    private LoggedAlert homeingFailedAlert;
    private LoggedAlert isHomeingAlert;
    private boolean forceStoped;

    private Timer logRefreshTimer = new Timer();

    public Elevator() {
        left = new TalonFX(kLeftMotorID, kSuperstructureCANBus);
        right = new TalonFX(kRightMotorID, kSuperstructureCANBus);
        configLeft();
        configRight();
        homePosition();
        voltageRequest = new VoltageOut(0);
        motionMagicRequest = new MotionMagicVoltage(getNativePosition()).withEnableFOC(true);
        followRequest = new Follower(kLeftMotorID, true);
        neutralRequest = new NeutralOut();
        var sp = new ElevatorSetpoint(getHeight());
        setFunc(sp);

        logRefreshTimer.start();

        homeingFailedAlert = new LoggedAlert("Elevator/Errors/HomeingFailed", "Failed to home elevator, setpoints will be inacurate", AlertType.kError);
        isHomeingAlert = new LoggedAlert("Elevator/Errors/IsHomeing", "Elevator Is Homeing, Please Wait", AlertType.kInfo);
    }

    public void forceStop(boolean forceStop) {
        if (forceStop) {
            left.setControl(neutralRequest);
            right.setControl(neutralRequest);
        } else if (forceStoped && !forceStop) {
            setControl(setpoint.getNativeSetpoint());
        }


        if (forceStop && !forceStoped) {

        }
        forceStoped = forceStop;
    }

    private void configLeft() {
        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        leftConfig.MotorOutput.Inverted = kLeftMotorInverted;
        leftConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        leftConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
        leftConfig.Slot0.kP = kP;
        leftConfig.Slot0.kI = kI;
        leftConfig.Slot0.kD = kD;
        leftConfig.Slot0.kV = kV;
        leftConfig.Slot0.kA = kA;
        leftConfig.Slot0.kS = kS;
        leftConfig.Slot0.kG = kG;
        // leftConfig.MotionMagic.MotionMagicExpo_kA = kA;
        // leftConfig.MotionMagic.MotionMagicExpo_kV = kV;
        leftConfig.MotionMagic.MotionMagicCruiseVelocity =  kMotionMagicCruiseVelocity.in(RotationsPerSecond);
        leftConfig.MotionMagic.MotionMagicAcceleration = kMotionMagicAcceleration.in(RotationsPerSecondPerSecond);
        leftConfig.MotionMagic.MotionMagicJerk = 0.0;

        leftConfig.CurrentLimits = kNormalCurrentLimits;

        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        left.getConfigurator().apply(leftConfig);
    }

    private void configRight() {
        TalonFXConfiguration rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.Inverted = kRightMotorInverted;

        rightConfig.CurrentLimits = kNormalCurrentLimits;

        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        right.getConfigurator().apply(rightConfig);

    }

    public Command home() {
        return Commands.sequence(
            Commands.runOnce(() -> isHomeingAlert.set(true)),
            Commands.runOnce(() -> setHomeingCurrents(true)),
            // set(ElevatorSetpoint.HOMEING, true).withTimeout(3),
            Commands.runOnce(() -> setVoltageOut(kHomeingVoltage)),
            new TimedWaitUntilCommand(this::detectHome, 0.8)
                .andThen(() -> homeingFailedAlert.set(true))
                .raceWith(Commands.waitSeconds(8)
                    .andThen(() -> homeingFailedAlert.set(true))),
            Commands.runOnce(() -> setVoltageOut(0.0)),
            Commands.waitSeconds(0.2),
            Commands.runOnce(this::homePosition),
            set(ElevatorSetpoint.STOW, false),
            Commands.runOnce(() -> isHomeingAlert.set(false))

        ).finallyDo((boolean interupted) -> setHomeingCurrents(false));
    }

    private void setHomeingCurrents(boolean isHomeing) {
        left.getConfigurator().apply(isHomeing ? kHomeingCurrentLimits : kNormalCurrentLimits);
        right.getConfigurator().apply(isHomeing ? kHomeingCurrentLimits : kNormalCurrentLimits);
    }

    private void homePosition() {
        left.setPosition(0);
        right.setPosition(0);
    }

    private boolean detectHome() {
        return Math.abs(left.getSupplyCurrent().getValue().in(Amps)) >= kHomeDetectCurrentThreshold.in(Units.Amps);
    }

    public Command set(ElevatorSetpoint setpoint, boolean waitForSuccess) { 
        return Commands.runOnce(() -> setFunc(setpoint), this)
        .andThen(Commands.waitUntil(() -> atSetpoint() || !waitForSuccess));
    }

    private void setFunc(ElevatorSetpoint setpoint) {
        this.setpoint = setpoint;
        setControl(setpoint.getNativeSetpoint());
    }

    private void setControl(Angle nativeSetpoint) {
        motionMagicRequest.withPosition(nativeSetpoint);
        left.setControl(motionMagicRequest);
        right.setControl(followRequest);
    }

    private void setVoltageOut(double voltageOut) {
        left.setControl(voltageRequest.withOutput(voltageOut));
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
        return left.getPosition(false).getValue();
    }

    public Distance getHeight() {
        return Meters.of(kRotationsToMeters.getOutput(getNativePosition().in(Rotations)));
    }

    public LinearVelocity getVelocity() {
        return MetersPerSecond.of(kRotationsToMeters.getOutput(left.getVelocity(false).getValue().in(RotationsPerSecond)));
    }

    // public LinearAcceleration getAcceleration() {
    //     return MetersPerSecondPerSecond.of(kRotationsToMeters.getOutput(left.getAcceleration().getValue().in(RotationsPerSecondPerSecond)));
    // }

    private void refreshLogs() {
        BreakerLog.log("Elevator/Left/StatorCurrent", left.getStatorCurrent().getValueAsDouble());
        BreakerLog.log("Elevator/Left/SupplyCurrent", left.getSupplyCurrent().getValueAsDouble());
        
        BreakerLog.log("Elevator/Motors/Right", right);

        BreakerLog.log("Elevator/Setpoint/Value", setpoint.getHeight().in(Meters));
        BreakerLog.log("Elevator/Setpoint/ValueNative", setpoint.getNativeSetpoint().in(Rotations));
        BreakerLog.log("Elevator/Setpoint/Tolerence", setpoint.getTolerence().in(Units.Meters));
        BreakerLog.log("Elevator/Setpoint/Satisfied", atSetpoint());
    }

    public ElevatorSetpoint getSetpoint() {
        return setpoint;
    }

    @Override
    public void periodic() {
        if (RobotState.isDisabled()) {
            setFunc(new ElevatorSetpoint(getHeight()));
        }

        if (logRefreshTimer.advanceIfElapsed(MiscConstants.kInfrequentLogRate)) {
            refreshLogs();
        }

        BreakerLog.log("Elevator/Motors/Left/Positoin", left.getPosition().getValueAsDouble());
        BreakerLog.log("Elevator/Motors/Left/Velocity", left.getVelocity().getValueAsDouble());
        BreakerLog.log("Elevator/Setpoint/Error", Math.abs(getHeight().in(Meters)) -  setpoint.getHeight().in(Meters));

        BreakerLog.log("Elevator/State/NativePosition", getNativePosition());
        BreakerLog.log("Elevator/State/Height", getHeight().in(Meters));
        BreakerLog.log("Elevator/State/Velocity", getVelocity().in(MetersPerSecond));
        // BreakerLog.log("Elevator/State/Acceleration", getAcceleration().in(MetersPerSecondPerSecond));
        homeingFailedAlert.log();
    }

    public static class ElevatorSetpoint {
        private Distance height;
        private Distance tolerence;
        private LinearVelocity velocityTolerence;
        public ElevatorSetpoint(Distance height, Distance tolerence, LinearVelocity velocityTolerence) {
           // this.height = Meters.of(MathUtil.clamp(height.in(Meters), kMinHeight.in(Meters), kMaxHeight.in(Meters)));
           this.height = height;
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

        public static final ElevatorSetpoint L1 = new ElevatorSetpoint(Meters.of(0.6));
        public static final ElevatorSetpoint L2 = new ElevatorSetpoint(Inches.of(29.183).plus(Inches.of(0.75)));
        public static final ElevatorSetpoint L3 = new ElevatorSetpoint(Meters.of(0.92).plus(Inches.of(16.5).minus(Inches.of(7))));
        public static final ElevatorSetpoint L4 = new ElevatorSetpoint(kMaxHeight.minus(Inches.of(4.25)));
        public static final ElevatorSetpoint HUMAN_PLAYER = new ElevatorSetpoint(Meters.of(0.45).plus(Inches.of(3.5)));
        public static final ElevatorSetpoint HANDOFF = new ElevatorSetpoint(Meters.of(0.0), Centimeters.of(2), kDefaultVelocityTolerence);
        public static final ElevatorSetpoint GROUND_ALGAE = new ElevatorSetpoint(Meters.of(0.0));
        public static final ElevatorSetpoint STOW = new ElevatorSetpoint(Meters.of(0.0), Centimeters.of(4), kDefaultVelocityTolerence);

        public static final ElevatorSetpoint LOW_REEF_ALGAE = new ElevatorSetpoint(Inches.of(23));
        public static final ElevatorSetpoint HIGH_REEF_ALGAE = new ElevatorSetpoint(Meters.of(1.05));

        public static final ElevatorSetpoint BARGE = new ElevatorSetpoint(kMaxHeight.plus(Inches.of(6)));


        private static final ElevatorSetpoint HOMEING = new ElevatorSetpoint(Meters.of(0.04));
    }

    
}
