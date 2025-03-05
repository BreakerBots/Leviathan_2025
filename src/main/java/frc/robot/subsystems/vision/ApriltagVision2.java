package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.BreakerLib.drivers.gtsam.GTSAM;
import frc.robot.BreakerLib.util.Localizer;
import frc.robot.BreakerLib.util.TimestampedValue;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.vision.ApriltagVision.BreakerPoseEstimationStandardDeviationCalculator;
import frc.robot.subsystems.vision.ApriltagVision.EstimationType;

import static frc.robot.Constants.ApriltagVisionConstants.*;

public class ApriltagVision2 {
    private GTSAM gtsam;
    private VisionSystemSim sim;
    private Camera[] cameras;
    private Localizer localizer;
    private Drivetrain drivetrain;

    private void update() {
        FrameContext context = new FrameContext(localizer.getPose());
        if (RobotBase.isSimulation()) {
            sim.update(drivetrain.getOdometryFusion().getPureOdometryPose());
        }
        List<CameraResult> allCameraResults = new ArrayList<>();
        for (var cam : cameras) {
            List<CameraResult> results = cam.update(context);
            for (var r : results) {
                allCameraResults.add(r);
            }
        }
    }
    

    public static Matrix<N3, N1> calculateTrigDevs(EstimatedRobotPose est) {
        PhotonTrackedTarget tgt = est.targetsUsed.get(0);
        Optional<Pose3d> tagPoseOpt = FieldConstants.kAprilTagFieldLayout.getTagPose(tgt.fiducialId);
        if (tagPoseOpt.isPresent()) {
            Pose2d tagPose = tagPoseOpt.get().toPose2d();
            double dist = tagPose.getTranslation().getDistance(est.estimatedPose.getTranslation().toTranslation2d());
            if (dist > kMaxTrigSolveTagDist.in(Units.Meters)) {
                return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            }
            return kTrigBaseStdDevs.times(1 + (dist * dist / kTrigDevScaleFactor));
        } else {
            return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        }
        
    }

    private static double phoenixTimeToFPGA(double phoenixTime) {
        return (Timer.getFPGATimestamp() - Utils.getCurrentTimeSeconds()) + phoenixTime;
    }

    private static class Camera {

        private PhotonCamera camera;
        private PhotonCameraSim sim;
        private PhotonPoseEstimator poseEstimator;
        private Transform3d robotTcam;
        private double stdDevScalar;
        
        public Camera(String cameraName, VisionSystemSim systemSim, Transform3d robotTcam, double stdDevScalar) {
            camera = new PhotonCamera(cameraName);
            sim = new PhotonCameraSim(camera);
            systemSim.addCamera(sim, robotTcam);
            poseEstimator = new PhotonPoseEstimator(FieldConstants.kAprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotTcam);
            this.stdDevScalar = stdDevScalar;


        }

        public List<CameraResult> update(FrameContext context) {
            var rawResults = camera.getAllUnreadResults();
            poseEstimator.setPrimaryStrategy(context.strategy.primaryStrategy);
            poseEstimator.setMultiTagFallbackStrategy(context.strategy.fallbackStrategy);
            poseEstimator.setReferencePose(context.prevPose.getValue().transformBy(robotTcam));
            poseEstimator.addHeadingData(phoenixTimeToFPGA(context.prevPose.getTimestamp().in(Seconds)), context.prevPose.getValue().getRotation());

            List<CameraResult> results = new ArrayList<>();
            for (PhotonPipelineResult res : rawResults) {
                Optional<EstimatedRobotPose> estOpt =  poseEstimator.update(res);
                if (estOpt.isPresent()) {
                    var pos = estOpt.get();
                    List<PhotonTrackedTarget> targets = pos.targetsUsed;
                        if (targets.size() > 0) {
                            PhotonTrackedTarget tgt = targets.get(0);
                            if (targets.size() == 1 && tgt.getPoseAmbiguity() >= 0.15) {
                                continue;
                            }
                            
                            final Pose3d actual = pos.estimatedPose;
                            final double fieldBorderMargin = 0.5;
                            final double zMargin = 0.75;

                            if (actual.getX() < -fieldBorderMargin
                                || actual.getX() > Constants.FieldConstants.kAprilTagFieldLayout.getFieldLength() + fieldBorderMargin
                                || actual.getY() < -fieldBorderMargin
                                || actual.getY() >  Constants.FieldConstants.kAprilTagFieldLayout.getFieldWidth() + fieldBorderMargin
                                || actual.getZ() < -zMargin
                                || actual.getZ() > zMargin) {
                                    continue;
                            }

                            var stdDevs = context.strategy.stdDevCalc.apply(pos);
                            stdDevs = stdDevs.times(stdDevScalar);
                            results.add(new CameraResult(pos, stdDevs));
                        } else {
                            continue;
                        }
                }
            }
            return results;

        }


    }

    private record FrameContext(
        TimestampedValue<Pose3d> prevPose,
        ChassisSpeeds prevSpeeds,
        EstimationStrategy strategy
    ) {}

    private record EstimationStrategy(
        PoseStrategy primaryStrategy,
        PoseStrategy fallbackStrategy,
        Function<EstimatedRobotPose, Matrix<N3, N1>> stdDevCalc
    ) {
        public static final EstimationStrategy kDefault = new EstimationStrategy(
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
            new BreakerPoseEstimationStandardDeviationCalculator(
            VecBuilder.fill(3.5, 3.5, 10), 
            VecBuilder.fill(0.5, 0.5, 1), 
            4.5, 
            6.5, 
            5.0)
        );

        public static final EstimationStrategy kDefaultNoSeed = new EstimationStrategy(
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            PoseStrategy.LOWEST_AMBIGUITY,
            new BreakerPoseEstimationStandardDeviationCalculator(
            VecBuilder.fill(3.5, 3.5, 10), 
            VecBuilder.fill(0.5, 0.5, 1), 
            4.5, 
            6.5, 
            5.0)
        );

        public static final EstimationStrategy kTrig = new EstimationStrategy(
            PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
            PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
            (EstimatedRobotPose est) -> calculateTrigDevs(est)
        );

        public static final EstimationStrategy kConstrained = new EstimationStrategy(
            PoseStrategy.CONSTRAINED_SOLVEPNP,
            PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
            new BreakerPoseEstimationStandardDeviationCalculator(
                VecBuilder.fill(3.5, 3.5, 10), 
                VecBuilder.fill(0.5, 0.5, 1), 
                4.5, 
                6.5, 
                5.0)
        );


    }

    private record CameraResult (
        EstimatedRobotPose est,
        Matrix<N3, N1> stdDevs
        ) {
    }
}
