package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.vision.ApriltagVision.BreakerEstimatedPose;
import frc.robot.subsystems.vision.ApriltagVision2.CameraResult;
import frc.robot.BreakerLib.drivers.ZED.LocalizationResults;
import frc.robot.BreakerLib.drivers.gtsam.TagDetection;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Constants.ApriltagVisionConstants.*;
import static frc.robot.Constants.DepthVisionConstants.*;

public class VisionUtils {
    public static double phoenixTimeToFPGA(double phoenixTime) {
        return (Timer.getFPGATimestamp() - Utils.getCurrentTimeSeconds()) + phoenixTime;
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

    public static Matrix<N3, N1> calculateStdDevs(EstimatedRobotPose est, Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevs, double maxSingleTagDist, double maxMultiTagDist, double distanceScaleFactor) {
         var estStdDevs = singleTagStdDevs;
            var targets = est.targetsUsed;
            int numTags = 0;
            double avgDist = 0;
            for (var tgt : targets) {
                var tagPose = FieldConstants.kAprilTagFieldLayout.getTagPose(tgt.fiducialId).get();
                numTags++;
                avgDist +=
                        tagPose.getTranslation().toTranslation2d().getDistance(est.estimatedPose.getTranslation().toTranslation2d());
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

    public static void sortByStandardDeviation(ArrayList<CameraResult> poses) {
        poses.sort((pos1, pos2) -> {
            var firstDeviation = pos1.stdDevs();
            var secondDeviation = pos2.stdDevs();
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

    public static Twist3d toTwist3d(Twist2d twist) {
        return new Pose3d().log(new Pose3d(new Pose2d().exp(twist)));
    }

    public static Twist2d toTwist2d(Twist3d twist) {
        return new Pose2d().log(new Pose3d().exp(twist).toPose2d());
    }

    public static List<TagDetection> photonTrackedTargetsToGTSAM(List<PhotonTrackedTarget> targetsUsed) {
        List<TagDetection> dets = new ArrayList<>();
        for (var tgt : targetsUsed) {
            dets.add(new TagDetection(tgt.getFiducialId(), tgt.detectedCorners));
        }
        return dets;
    }

    public static Matrix<N3, N1> estimateStdDevsZedVIO(LocalizationResults results, ChassisSpeeds speeds) {
        Matrix<N3, N1> devs = kBaseStdDevsVIO.div(results.getConfidance() * kConfidanceScalarVIO);
        devs = devs.times((Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)) * kLinearVelStdDevScalarVIO);
        devs = devs.times((speeds.omegaRadiansPerSecond) * kAngularVelStdDevScalarVIO);
        return devs;
    }
}
