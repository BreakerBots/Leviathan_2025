package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.proto.Translation2dProto;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.BreakerLib.physics.ChassisAccels;
import frc.robot.BreakerLib.util.Localizer;
import frc.robot.BreakerLib.util.TimestampedValue;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.subsystems.Drivetrain;
import org.photonvision.EstimatedRobotPose;

import org.ejml.simple.SimpleMatrix;
import org.opencv.osgi.OpenCVInterface;

import static frc.robot.Constants.ApriltagVisionConstants.*;

/** Add your docs here. */
public class ApriltagVision extends SubsystemBase {
    private PhotonCamera[] cameras;
    private Transform3d[] camOffsets;
    private PhotonPoseEstimator photonPoseEstimator;
    private boolean odometryHasBeenSeededCashed;
    private ArrayList<BreakerEstimatedPose> estimatedPoses;
    private BreakerPoseEstimationStandardDeviationCalculator stdDevCalculator;
    private Drivetrain drivetrain;

    private EstimationType estimationType = EstimationType.PNP;

    public ApriltagVision(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        // PhotonCamera topLeftCam = new PhotonCamera(kTopLeftCameraName);
        PhotonCamera bottomLeftCam = new PhotonCamera(kBottomLeftCameraName);
        PhotonCamera topRightCam = new PhotonCamera(kTopRightCameraName);
        PhotonCamera bottomRightCam = new PhotonCamera(kBottomRightCameraName);

        cameras = new PhotonCamera[] {/*topLeftCam,*/ topRightCam, bottomLeftCam, bottomRightCam};
        camOffsets = new Transform3d[] {kTopRightCameraTransform, kBottomLeftCameraTransform, kBottomRightCameraTransform};

        photonPoseEstimator = new PhotonPoseEstimator(FieldConstants.kAprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d());
        estimatedPoses = new ArrayList<>();
        stdDevCalculator = new BreakerPoseEstimationStandardDeviationCalculator(
            VecBuilder.fill(3.5, 3.5, 10), 
            VecBuilder.fill(0.5, 0.5, 1), 
            4.5, 
            6.5, 
            5.0
        );
        odometryHasBeenSeededCashed = false;
    }
 
    private double phoenixTimeToFPGA(double phoenixTime) {
        return (Timer.getFPGATimestamp() - Utils.getCurrentTimeSeconds()) + phoenixTime;
    }

        public void setEstimationType(EstimationType type) {
            estimationType = type;
        }

        @Override
        public void periodic() {
                    var driveState = drivetrain.getState();
                    Pose2d odometryRefPos = driveState.Pose;


            if (odometryHasBeenSeededCashed) {
                photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
                photonPoseEstimator.setReferencePose(odometryRefPos);
                photonPoseEstimator.addHeadingData(phoenixTimeToFPGA(driveState.Timestamp), odometryRefPos.getRotation());
            } else {
                photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            } 

            if (estimationType == EstimationType.PNP) {
                photonPoseEstimator.setPrimaryStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
            } else {
                photonPoseEstimator.setPrimaryStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
            }

            estimatePose();

            sortByStandardDeviation(estimatedPoses);

            for (BreakerEstimatedPose estPos: estimatedPoses) {
                drivetrain.addVisionMeasurement(estPos.poseEst.estimatedPose.toPose2d(), estPos.poseEst.timestampSeconds, estPos.stdDevs);
            }

            estimatedPoses.clear();

    }

    private void estimatePose() {
        for (int i = 0; i < cameras.length; i++) {
            var cam = cameras[i];
            
            var allResults = cam.getAllUnreadResults();
            if (allResults.size() > 0) {
                photonPoseEstimator.setRobotToCameraTransform(camOffsets[i]);
                for (var result : allResults) {
                    Optional<EstimatedRobotPose> posOpt = photonPoseEstimator.update(result, cam.getCameraMatrix(), cam.getDistCoeffs());
                    BreakerLog.log("fsdfsd", posOpt.isPresent());
                    if (posOpt.isPresent()) {
                    EstimatedRobotPose pos = posOpt.get();
                        List<PhotonTrackedTarget> targets = pos.targetsUsed;
                        if (!odometryHasBeenSeededCashed) {
                            drivetrain.resetPose(pos.estimatedPose.toPose2d());
                            odometryHasBeenSeededCashed = true;
                        }
                        if (targets.size() == 1) {
                            PhotonTrackedTarget tgt = targets.get(0);
                            if (tgt.getPoseAmbiguity() >= 0.15 && estimationType == EstimationType.PNP) {
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
                        } 
                        Matrix<N3, N1> devs = null;
                        if (pos.strategy == PoseStrategy.PNP_DISTANCE_TRIG_SOLVE)  {
                            devs = calculateTrigDevs(pos, result);
                        } else {   
                            devs = stdDevCalculator.apply(pos);   
                        }
                    
                        if (devs.get(0, 0) > 15 || devs.get(1, 0) > 15) {
                            continue;
                        }
                        BreakerLog.log("ApriltagVision/RawEsts/" + cam.getName(), pos.estimatedPose);
                        estimatedPoses.add(new BreakerEstimatedPose(pos, devs));
                    }
                }
            }
        }
    }

    public Matrix<N3, N1> calculateTrigDevs(EstimatedRobotPose est, PhotonPipelineResult result) {
        PhotonTrackedTarget tgt = result.getBestTarget();
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

    public static record BreakerEstimatedPose(EstimatedRobotPose poseEst, Matrix<N3, N1> stdDevs) {}

    
    public static class BreakerPoseEstimationStandardDeviationCalculator implements Function<EstimatedRobotPose, Matrix<N3, N1>> {

        private Matrix<N3, N1> singleTagStdDevs;
        private Matrix<N3, N1> multiTagStdDevs;
        private double maxSingleTagDist, maxMultiTagDist, distanceScaleFactor;

        public BreakerPoseEstimationStandardDeviationCalculator() {
            this(VecBuilder.fill(2, 2, 10), VecBuilder.fill(0.5, 0.5, 1));
        }

        public BreakerPoseEstimationStandardDeviationCalculator(Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevs) {
            this(singleTagStdDevs, multiTagStdDevs, 3.5, 6.5, 5.0);
        }

        public BreakerPoseEstimationStandardDeviationCalculator(double maxSingleTagDist, double maxMultiTagDist, double distanceScaleFactor) {
            this(VecBuilder.fill(6, 6, 8), VecBuilder.fill(0.5, 0.5, 1), maxSingleTagDist, maxMultiTagDist, distanceScaleFactor);
        }
        public BreakerPoseEstimationStandardDeviationCalculator(Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevs, double maxSingleTagDist, double maxMultiTagDist, double distanceScaleFactor) {
            this.multiTagStdDevs = multiTagStdDevs;
            this.singleTagStdDevs = singleTagStdDevs;
            this.maxSingleTagDist = maxSingleTagDist;
            this.maxMultiTagDist = maxMultiTagDist;
            this.distanceScaleFactor = distanceScaleFactor;
        }
        @Override
        public Matrix<N3, N1> apply(EstimatedRobotPose t) {
            var estStdDevs = singleTagStdDevs;
            var targets = t.targetsUsed;
            int numTags = 0;
            double avgDist = 0;
            for (var tgt : targets) {
                var tagPose = FieldConstants.kAprilTagFieldLayout.getTagPose(tgt.fiducialId).get();
                numTags++;
                avgDist +=
                        tagPose.getTranslation().toTranslation2d().getDistance(t.estimatedPose.getTranslation().toTranslation2d());
            }
            if (numTags == 0) return estStdDevs;
            avgDist /= numTags;
            // Decrease std devs if multiple targets are visible
            if (numTags > 1) estStdDevs = multiTagStdDevs;
            // Increase std devs based on (average) distance
            if ((numTags == 1 && avgDist > maxSingleTagDist) || (numTags > 1 && avgDist > maxMultiTagDist) )//4
                estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / distanceScaleFactor)); //30

            return estStdDevs;
        }
    }

    private void sortByStandardDeviation(ArrayList<BreakerEstimatedPose> poses) {
        poses.sort((pos1, pos2) -> {
            var firstDeviation = pos1.stdDevs;
            var secondDeviation = pos2.stdDevs;
            // if (firstDeviation. && secondDeviation.isEmpty()) return 0;
            // if (firstDeviation.isEmpty() && secondDeviation.isPresent()) return -1;
            // if (firstDeviation.isPresent() && secondDeviation.isEmpty()) return 1;

            double firstValue = firstDeviation.get(2, 0);
            double secondValue = secondDeviation.get(2, 0);

            return firstValue == secondValue 
                ? 0 
                : (firstValue < secondValue ? -1 : 1);
        });
    }

    public enum EstimationType {
        PNP,
        TRIG
    }
    
}