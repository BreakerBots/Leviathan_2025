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
import org.photonvision.PhotonPoseEstimator.ConstrainedSolvepnpParams;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.BreakerLib.drivers.gtsam.GTSAM;
import frc.robot.BreakerLib.util.Localizer;
import frc.robot.BreakerLib.util.TimestampedValue;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.vision.ApriltagVision.BreakerPoseEstimationStandardDeviationCalculator;
import frc.robot.subsystems.vision.ApriltagVision.EstimationType;

import static frc.robot.Constants.ApriltagVisionConstants.*;

public class ApriltagVision2 {
    private VisionSystemSim sim;
    private Camera[] cameras;
    private Localization localization;

    public ApriltagVision2(Localization localization) {
        this.localization = localization;
        sim = new VisionSystemSim("ApriltagVision");
        sim.addAprilTags(FieldConstants.kAprilTagFieldLayout);
        var bottomLeftCam = new Camera(kBottomLeftCameraName, sim, kBottomLeftCameraSimProperties, kBottomLeftCameraTransform, 1);
        var bottomRightCam = new Camera(kBottomRightCameraName, sim, kBottomRightCameraSimProperties, kBottomRightCameraTransform, 1);
        var topLeftCam = new Camera(kTopLeftCameraName, sim, kTopLeftCameraSimProperties, kTopLeftCameraTransform, 1.2);
        var topFrontCam = new Camera(kTopFrontCameraName, sim, kTopFrontCameraSimProperties, kTopFrontCameraTransform, 1.2);
        cameras = new Camera[]{bottomLeftCam, bottomRightCam, topLeftCam, topFrontCam};
    }

    public List<CameraResult> update(EstimationStrategy strategy) {
        FrameContext context = new FrameContext(localization.getAtomicPose(), localization.getSpeeds(), strategy);
        if (RobotBase.isSimulation()) {
            sim.update(localization.getPose());
        }
        ArrayList<CameraResult> allCameraResults = new ArrayList<>();
        for (var cam : cameras) {
            List<CameraResult> results = cam.update(context);
            for (var r : results) {
                if (strategy.primaryStrategy == PoseStrategy.PNP_DISTANCE_TRIG_SOLVE) {
                    if (cam.getName().equals(kTopFrontCameraName) || cam.getName().equals(kTopLeftCameraName)) {
                        continue;
                    }
                }
                allCameraResults.add(r);
            }
        }
        VisionUtils.sortByStandardDeviation(allCameraResults);
        return allCameraResults;
    }

    public Camera[] getCameras() {
        return cameras;
    }

    

    public static class Camera {

        private PhotonCamera camera;
        private PhotonCameraSim sim;
        private PhotonPoseEstimator poseEstimator;
        private Transform3d robotTcam;
        private double stdDevScalar;
        
        public Camera(String cameraName, VisionSystemSim systemSim, SimCameraProperties simCameraProperties, Transform3d robotTcam, double stdDevScalar) {
            camera = new PhotonCamera(cameraName);
            sim = new PhotonCameraSim(camera, simCameraProperties, FieldConstants.kAprilTagFieldLayout);
            sim.enableDrawWireframe(true);
            systemSim.addCamera(sim, robotTcam);
            poseEstimator = new PhotonPoseEstimator(FieldConstants.kAprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotTcam);
            this.stdDevScalar = stdDevScalar;
            this.robotTcam = robotTcam;
            
        }

        public String getName() {
            return camera.getName();
        }

        public Optional<Matrix<N3, N3>> getCameraMatrix() {
            return camera.getCameraMatrix();
        }

        
        public Optional<Matrix<N8, N1>> getDistanceCoeffs() {
            return camera.getDistCoeffs();
        }

        public Transform3d getRobotTCam() {
            return robotTcam;
        }

        public List<CameraResult> update(FrameContext context) {
            var rawResults = camera.getAllUnreadResults();
            poseEstimator.setPrimaryStrategy(context.strategy.primaryStrategy);
            poseEstimator.setMultiTagFallbackStrategy(context.strategy.fallbackStrategy);
            poseEstimator.setReferencePose(new Pose3d(context.prevPose.getValue()).transformBy(robotTcam));
            poseEstimator.addHeadingData(context.prevPose.getTimestamp().in(Seconds), context.prevPose.getValue().getRotation());

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

                            if (stdDevs.get(0, 0) > 25 || stdDevs.get(1, 0) > 25) {
                                continue;
                            }

                            results.add(new CameraResult(this, pos, stdDevs));
                        } else {
                            continue;
                        }
                }
            }
            Pose3d[] poses = new Pose3d[results.size()];
            for (int i = 0; i < results.size(); i++) {
                poses[i] = results.get(i).est.estimatedPose;
            }
            BreakerLog.log("ApriltagVision/" + getName() + "/Poses", poses);
            return results;

        }


    }

    private record FrameContext(
        TimestampedValue<Pose2d> prevPose,
        ChassisSpeeds prevSpeeds,
        EstimationStrategy strategy
    ) {}

    public record EstimationStrategy(
        PoseStrategy primaryStrategy,
        PoseStrategy fallbackStrategy,
        Optional<ConstrainedSolvepnpParams> ConstrainedSolvepnpParams,
        Function<EstimatedRobotPose, Matrix<N3, N1>> stdDevCalc
    ) {

        public EstimationStrategy(
            PoseStrategy primaryStrategy,
            PoseStrategy fallbackStrategy,
            Function<EstimatedRobotPose, Matrix<N3, N1>> stdDevCalc
        ) {
            this(primaryStrategy, fallbackStrategy, Optional.empty(), stdDevCalc);
        }
        public static final EstimationStrategy kDefault = new EstimationStrategy(
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
            (EstimatedRobotPose est) -> 
                VisionUtils.calculateStdDevs(est,
                    VecBuilder.fill(3.5, 3.5, 10), 
                    VecBuilder.fill(0.5, 0.5, 1), 
                    4.5, 
                    6.5, 
                    5.0
                )
        );

        public static final EstimationStrategy kDefaultNoSeed = new EstimationStrategy(
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            PoseStrategy.LOWEST_AMBIGUITY,
            (EstimatedRobotPose est) -> 
                VisionUtils.calculateStdDevs(est,
                    VecBuilder.fill(3.5, 3.5, 10), 
                    VecBuilder.fill(0.5, 0.5, 1), 
                    4.5, 
                    6.5, 
                    5.0
                )
        );

        public static final EstimationStrategy kTrig = new EstimationStrategy(
            PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
            PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
            (EstimatedRobotPose est) -> VisionUtils.calculateTrigDevs(est)
        );

        public static final EstimationStrategy kConstrained = new EstimationStrategy(
            PoseStrategy.CONSTRAINED_SOLVEPNP,
            PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
            Optional.of(new ConstrainedSolvepnpParams(true, 2.0)),
            (EstimatedRobotPose est) -> 
                VisionUtils.calculateStdDevs(est,
                    VecBuilder.fill(3.5, 3.5, 10), 
                    VecBuilder.fill(0.5, 0.5, 1), 
                    4.5, 
                    6.5, 
                    5.0
                )
        );

        @Override
        public final boolean equals(Object arg0) {
            if (arg0 instanceof EstimationStrategy) {
                var strat = (EstimationStrategy) arg0;
                return strat.primaryStrategy == this.primaryStrategy && strat.fallbackStrategy == this.fallbackStrategy;
            }
            return false;
        }
    }

    public record CameraResult (
        Camera camera,
        EstimatedRobotPose est,
        Matrix<N3, N1> stdDevs
        ) {

    }
}
