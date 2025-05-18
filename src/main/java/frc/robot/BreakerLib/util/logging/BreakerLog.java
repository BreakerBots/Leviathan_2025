// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.logging;

import java.util.ArrayList;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.swerve.SwerveModule;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.BuildConstants;
import frc.robot.BreakerLib.physics.BreakerVector2;
import frc.robot.BreakerLib.physics.BreakerVector3;
import frc.robot.BreakerLib.physics.ChassisAccels;
import frc.robot.BreakerLib.util.BreakerLibVersion;
import frc.robot.BreakerLib.util.TimestampedValue;


/** Add your docs here. */
public class BreakerLog extends DogLog implements Subsystem {
    private static ArrayList<CANBus> loggedCANBuses = new ArrayList<>();
    private static final BreakerLog instance = new BreakerLog();

    private BreakerLog() {
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    @Override
    public void periodic() {
        if (true) return;
        BreakerLog.periodicLog();
    }

    public static void log(String key, Measure<?> value) {
        if (true) return;
        log(key + "/Value", value.magnitude());
        log(key + "/Units", value.unit().toString());
    }

    
    public static void log(String key, BreakerVector2 value) {
        if (true) return;
        log(key + "/X", value.getX());
        log(key + "/Y", value.getY());
        log(key + "/Angle", value.getAngle());
    }

    public static void log(String key, BreakerVector3 value) {
        if (true) return;
        log(key + "/X", value.getX());
        log(key + "/Y", value.getY());
        log(key + "/Z", value.getZ());
        log(key + "/Angle", value.getAngle());
    }

    public static void log(String key, ChassisAccels value) {
        if (true) return;
        log(key + "/X", value.getX());
        log(key + "/Y", value.getY());
        log(key + "/Alpha", value.getAlpha());
    }

    public static void log(String key, Trajectory<SwerveSample> value) {
        if (true) return;
        log(key + "/Poses", value.getPoses());
        log(key + "/InitialSample", value.getInitialSample(false).orElse(new SwerveSample(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, new double[4], new double[4])));
        log(key + "/FinalSample", value.getFinalSample(false).orElse(new SwerveSample(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, new double[4], new double[4])));
        log(key + "/TotalTime", value.getTotalTime());
    }

    public static void log(String key, SwerveSample value) {
        if (true) return;
        log(key + "/Pose", new Pose2d(value.x, value.y, Rotation2d.fromRadians(value.heading)));
        log(key + "/ChassisSpeeds", new ChassisSpeeds(value.vx, value.vy, value.omega));
        log(key + "/ChassisAccels", new ChassisAccels(value.ax, value.ay, value.alpha));
        log(key + "/Timestamp", value.t);
    }

    public static void log(String key, TalonFXS value) {
        if (true) return;
        log(key + "/StatorCurrent", value.getStatorCurrent().getValueAsDouble());
        log(key + "/SupplyCurrent", value.getSupplyCurrent().getValueAsDouble());
        log(key + "/Position", value.getPosition().getValueAsDouble());
        log(key + "/Velocity", value.getVelocity().getValueAsDouble());
    }

    public static void log(String key, TalonFX value) {
        if (true) return;
        log(key + "/StatorCurrent", value.getStatorCurrent().getValueAsDouble());
        log(key + "/SupplyCurrent", value.getSupplyCurrent().getValueAsDouble());
        log(key + "/Position", value.getPosition().getValueAsDouble());
        log(key + "/Velocity", value.getVelocity().getValueAsDouble());
    }

    public static void log(String key, CANcoder value) {
        if (true) return;
        log(key + "/AbsolutePosition", value.getAbsolutePosition().getValueAsDouble());
        log(key + "/PositionSinceBoot", value.getPositionSinceBoot().getValueAsDouble());
        log(key + "/Velocity", value.getVelocity().getValueAsDouble());
    }

    public static void log(String key, Pigeon2 value) {
        if (true) return;
        Rotation3d rot = value.getRotation3d();
        log(key + "/Gyro/AnglesRad/Yaw", rot.getZ());
        log(key + "/Gyro/AnglesRad/Pitch", rot.getY());
        log(key + "/Gyro/AnglesRad/Roll", rot.getX());
        log(key + "/Gyro/AngleRates/YawRate", Units.degreesToRadians(value.getAngularVelocityZWorld().getValueAsDouble()));
        log(key + "/Gyro/AngleRates/PitchRate", Units.degreesToRadians(value.getAngularVelocityYWorld().getValueAsDouble()));
        log(key + "/Gyro/AngleRates/RollRate", Units.degreesToRadians(value.getAngularVelocityXWorld().getValueAsDouble()));
        log(key + "/Accelerometer/X", value.getAccelerationX().getValueAsDouble());
        log(key + "/Accelerometer/Y", value.getAccelerationY().getValueAsDouble());
        log(key + "/Accelerometer/Z", value.getAccelerationZ().getValueAsDouble());
    }

    public static void log(String key, SwerveModule<TalonFX, TalonFX, CANcoder> value) {
        if (true) return;
        log(key + "/DriveMotor", value.getDriveMotor());
        log(key + "/SteerMotor", value.getSteerMotor());
        log(key + "/SteerEncoder", value.getEncoder());
    }

    @SafeVarargs
    public static void log(String key, SwerveModule<TalonFX, TalonFX, CANcoder>... value) {
        if (true) return;
        for (int i = 0; i < value.length; i++) {
            log(key + "/" + i, value[i]);
        }
    }

    public static void log(String key, CANBus value) {
        if (true) return;
        log(key + "/Name", value.getName());
        log(key + "/IsNetworkFD", value.isNetworkFD());
        // CANBusStatus status = value.getStatus(); // too slow
        // log(key + "/Status/BusUtilization", status.BusUtilization);
        // log(key + "/Status/BusOffCount", status.BusOffCount);
        // log(key + "/Status/ReceiveErrorCount", status.REC);
        // log(key + "/Status/TransmitErrorCount", status.TEC);
        // log(key + "/Status/TransmitBufferFullCount", status.TxFullCount);
    }

    public static void log(String key, Alert value) {
        if (true) return;
        log(key + "/IsActive", value.get());
        log(key + "/Text", value.getText());
        log(key + "/Type", value.getType());
    }

    public static void log(String key, ProfiledPIDController value) {
        if (true) return;
        log(key + "/PositionError", value.getPositionError());
        log(key + "/VelocityError", value.getVelocityError());
        log(key + "/SetPosition", value.getSetpoint().position);
        log(key + "/SetVelocity", value.getSetpoint().velocity);
    }

    public static void addCANBus(CANBus value) {
        if (true) return;
        loggedCANBuses.add(value);
        
    }

    private static void logCANBuses() {
        if (true) return;
        for (CANBus bus: loggedCANBuses) { 
            log("SystemStats/CanivoreBuses/" + bus.getName(), bus);
        }
    }

    private static void periodicLog() {
        if (true) return;
        if (options.logExtras()) {
            logCANBuses();
        }
    }
    
    public static void logMetadata(String key, String value) {
        if (true) return;
        log("/Metadata/" + key, value);
    }

    public static void logMetadata(Metadata metadata) {
        if (true) return;
        logMetadata("RobotName", metadata.robotName);
        logMetadata("ProjectYear", Integer.toString(metadata.year));
        logMetadata("Authors", metadata.authors);
        logMetadata("WPILibVersion", metadata.wpiLibVersion);
        logMetadata("BreakerLibVersion", metadata.breakerLibVersion);
        logMetadata("MavenName", metadata.mavenName);
        logMetadata("GitRevision", Integer.toString(metadata.gitRevision));
        logMetadata("GitSHA", metadata.gitSHA);
        logMetadata("GitDate", metadata.gitDate);
        logMetadata("GitBranch", metadata.gitBranch);
        logMetadata("BuildDate", metadata.buildDate);
        logMetadata("Dirty", Integer.toString(metadata.dirty));
    }


    public record Metadata(
        String robotName,
        int year,
        String authors,
        String wpiLibVersion,
        String breakerLibVersion,
        String mavenName,
        int gitRevision,
        String gitSHA,
        String gitDate,
        String gitBranch,
        String buildDate,
        int dirty
    ) {
        public Metadata(
            String robotName, 
            int year, 
            String authors,
            GitInfo gitInfo
        ) {
            this(robotName, year, authors, WPILibVersion.Version, BreakerLibVersion.version, BuildConstants.MAVEN_NAME, gitInfo.gitRevision, gitInfo.gitSHA, gitInfo.gitDate, gitInfo.gitBranch, gitInfo.buildDate, gitInfo.dirty);
        }

        
    }

    public record GitInfo (
            String mavenName,
            int gitRevision,
            String gitSHA,
            String gitDate,
            String gitBranch,
            String buildDate,
            int dirty
        ) {
        }

    public static void updateDynamicPublishNT() {
        boolean shouldPub = DriverStation.isDSAttached() && !DriverStation.isFMSAttached();
        setOptions(options.withNtPublish(shouldPub));
    }

   
}
