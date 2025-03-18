// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.fasterxml.jackson.databind.deser.std.ArrayBlockingQueueDeserializer;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.drivers.ZED.LocalizationResults;
import frc.robot.BreakerLib.drivers.ZED.LocalizationResults.OdometryRecord;
import frc.robot.BreakerLib.drivers.gtsam.GTSAM;
import frc.robot.BreakerLib.physics.ChassisAccels;
import frc.robot.BreakerLib.util.Localizer;
import frc.robot.BreakerLib.util.TimestampedValue;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.BreakerLib.util.math.OdometryFusion;
import frc.robot.Constants.DepthVisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.vision.ApriltagVision2.CameraResult;
import frc.robot.subsystems.vision.ApriltagVision2.EstimationStrategy;

import static frc.robot.Constants.LocalizationConstants.*;

/** Add your docs here. */
public class Localization extends SubsystemBase implements Localizer {
    private GTSAM gtsam;
    private ApriltagVision2 apriltagVision;
    // private BreakerOdometry<SwerveModulePosition[]> wheelOdometry;
    private OdometryFusion<SwerveModulePosition[]> odometryFusion;
    private BreakerPoseEstimator<SwerveModulePosition[]> visionFilter;
    private DepthVision depthVision;
    private boolean hasOdometryBeenSeeded;
    private Pose2d lastOdometryValue;
    private Drivetrain drivetrain;
    private boolean useTrigStrat;
    public Localization(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        SwerveDriveOdometry wheelOdometry = new SwerveDriveOdometry(
            drivetrain.getKinematics(), drivetrain.getState().RawHeading, 
            drivetrain.getState().ModulePositions, 
            new Pose2d()
        );

        odometryFusion = new OdometryFusion<SwerveModulePosition[]>(
            drivetrain.getKinematics(), 
            wheelOdometry,
            VecBuilder.fill(0.1, 0.1, 0.1),
            VecBuilder.fill(0.9, 0.9, 0.9) 
        );

        visionFilter = new BreakerPoseEstimator<>(
            drivetrain.getKinematics(),
            odometryFusion,
            VecBuilder.fill(0.1, 0.1, 0.1),
            VecBuilder.fill(0.9, 0.9, 0.9) 
        );

        apriltagVision = new ApriltagVision2(this);
        depthVision = new DepthVision(() -> new TimestampedValue<Pose3d>(new Pose3d(getPose()), Timer.getTimestamp()));

        lastOdometryValue = drivetrain.getStateCopy().Pose;
        drivetrain.registerTelemetry(this::updateWheelOdometry);

        ArrayList<String> camNames = new ArrayList<>();
        for (var cam : apriltagVision.getCameras()) {
            camNames.add(cam.getName());
        }
        gtsam = new GTSAM(camNames);

        drivetrain.setLocalizer(this);
    }

    private void updateWheelOdometry(SwerveDriveState state) {
        Pose2d guess = visionFilter.update(state.RawHeading, state.ModulePositions);
        if (kUseGTSAM) {
            Pose2d currentOdom = odometryFusion.getEstimatedPosition();
            Twist2d odomTwist = lastOdometryValue.log(currentOdom);
            lastOdometryValue = currentOdom;


            double phoenixTimestamp = state.Timestamp;
            double fpgaTime = VisionUtils.phoenixTimeToFPGA(phoenixTimestamp);
            long timeMcrs = (long) (fpgaTime * 1e6);
            gtsam.sendOdomUpdate(timeMcrs, VisionUtils.toTwist3d(odomTwist), new Pose3d(guess));
        }
    }

    public void addExternalOdometry(Twist2d robotTwistMeters, double startTime, double endTime, Matrix<N3, N1> stdDevs) {
        odometryFusion.addExternalOdometryMeasurment(robotTwistMeters, startTime, endTime, stdDevs);
    }

    @Override
    public void periodic() {
        update();
        BreakerLog.log("Localization/Estimates/KalmanPoseEstimator", visionFilter.getEstimatedPosition());
        BreakerLog.log("Localization/Estimates/OdometryFusion", odometryFusion.getEstimatedPosition());
        BreakerLog.log("Localization/Estimates/PureOdometry", odometryFusion.getPureOdometryPose());
        if (kUseGTSAM) {
            BreakerLog.log("Localization/Estimates/GTSAM/AtomicPose", gtsam.getAtomicPoseEstimate().getValue());
            BreakerLog.log("Localization/Estimates/GTSAM/LatencyCompensatedPose", gtsam.getLatencyCompensatedPoseEstimate());
        }
    }

    private void update() {
        ChassisSpeeds speeds = getFieldRelativeSpeeds();
        Optional<LocalizationResults> zedVIOOpt = depthVision.getUnreadLocalizationResults();
        if (zedVIOOpt.isPresent()) {
            var zedVIO = zedVIOOpt.get();
            boolean confGood = zedVIO.getConfidance() >= DepthVisionConstants.kMinConfidanceVIO;
            boolean linVelGood = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) <= DepthVisionConstants.kMaxLinearVelVIO.in(Units.MetersPerSecond);
            boolean angVelGood = speeds.omegaRadiansPerSecond <= DepthVisionConstants.kMaxAngularVelVIO.in(Units.RadiansPerSecond);
            if (confGood && linVelGood && angVelGood) {
                var devs = VisionUtils.estimateStdDevsZedVIO(zedVIO, speeds);
                OdometryRecord odoRec = zedVIO.getFieldRelativeDelta();
                addExternalOdometry(VisionUtils.toTwist2d(odoRec.delta()), odoRec.startTime(), odoRec.endTime(), devs);
            }
        }
        
        EstimationStrategy strat = EstimationStrategy.kDefaultNoSeed;
        if (hasOdometryBeenSeeded) {
            if (!useTrigStrat) {
                strat = EstimationStrategy.kDefault;
            } else {
                strat = EstimationStrategy.kTrig;
            }
        }
        List<CameraResult> apriltagResults = apriltagVision.update(strat);
        if (!hasOdometryBeenSeeded && apriltagResults.size() > 0) {
            CameraResult best = apriltagResults.get(apriltagResults.size() - 1);
            visionFilter.resetPose(best.est().estimatedPose.toPose2d());
            hasOdometryBeenSeeded = true;
        }
        Pose3d[] poses = new Pose3d[apriltagResults.size()];
        int i = 0;
        for (CameraResult res : apriltagResults) {
            poses[i++] = res.est().estimatedPose;
            visionFilter.addVisionMeasurement(res);
            if (kUseGTSAM) {
                gtsam.setCamIntrinsics(res.camera().getName(), res.camera().getCameraMatrix(), res.camera().getDistanceCoeffs());
                gtsam.sendVisionUpdate(res.camera().getName(), res.est().timestampSeconds, VisionUtils.photonTrackedTargetsToGTSAM(res.est().targetsUsed), res.camera().getRobotTCam());
            }
        }
        drivetrain.resetPose(getPose());
    }

    public OdometryFusion<SwerveModulePosition[]> getOdometryFusion() {
        return odometryFusion;
    }

    public BreakerPoseEstimator<SwerveModulePosition[]> getVisionFilter() {
        return visionFilter;
    }

    public void useTrigApriltagStragey(boolean useTrigStrat) {
        this.useTrigStrat = useTrigStrat;
    }

    @Override
    public TimestampedValue<Pose2d> getAtomicPose() {
        if (kUseGTSAM) {
            var val = gtsam.getAtomicPoseEstimate();
            return new TimestampedValue<Pose2d>(val.getValue().toPose2d(), val.getTimestamp());
        }
        return new TimestampedValue<>(visionFilter.getEstimatedPosition(), Timer.getTimestamp());
    }


    @Override
    public Pose2d getPose() {
        if (kUseGTSAM) {
            return gtsam.getLatencyCompensatedPoseEstimate().toPose2d();
        }
        return visionFilter.getEstimatedPosition();
    }
    @Override
    public ChassisSpeeds getSpeeds() {
       return drivetrain.getChassisSpeeds();
    }
    @Override
    public ChassisAccels getAccels() {
      return drivetrain.getChassisAccels();
    }
    @Override
    public void resetPose(Pose2d newPose) {
        visionFilter.resetPose(newPose);
    }
} 
