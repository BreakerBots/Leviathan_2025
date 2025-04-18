package frc.robot.BreakerLib.drivers.gtsam;

import java.security.CryptoPrimitive;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.TermCriteria;
import org.photonvision.estimation.OpenCVHelp;
import org.photonvision.targeting.TargetCorner;

import java.util.stream.Collectors;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.networktables.TimestampedObject;
import frc.robot.BreakerLib.util.TimestampedValue;

public class GTSAM {

    public static class CameraCalibration {
        Mat cameraMat;
        Mat distCoeffs;

        public CameraCalibration(Matrix<N3, N3> camMat, Matrix<?, N1> distCoeffs) {
            OpenCVHelp.forceLoadOpenCV();

            this.cameraMat = OpenCVHelp.matrixToMat(camMat.getStorage());
            this.distCoeffs = OpenCVHelp.matrixToMat(distCoeffs.getStorage());
        }

        public void release() {
            cameraMat.release();
            distCoeffs.release();
        }
    }

    private static class CameraInterface {
        StructArrayPublisher<TagDetection> tagPub;
        DoubleArrayPublisher camIntrinsicsPublisher;
        StructPublisher<Transform3d> robotTcamPub;
        private String name;

        private CameraCalibration cameraCal = null;

        public TagDetection undistort(TagDetection distorted) {
            if (this.cameraCal == null) {
                //System.err.println("Camera cal still null -- is your camera connected?");
                return distorted;
            }

            Point[] distPts = new Point[distorted.corners.size()];
            for (int i = 0; i < distPts.length; i++) {
                distPts[i] = new Point(distorted.corners.get(i).x, distorted.corners.get(i).y);
            }

            var mat = new MatOfPoint2f(distPts);
            Calib3d.undistortImagePoints(mat, mat, cameraCal.cameraMat, cameraCal.distCoeffs, new TermCriteria(3, 30, 1e-6));
            ArrayList<TargetCorner> corners = new ArrayList<>();
            for (var pt : mat.toList()) {
                corners.add(new TargetCorner(pt.x, pt.y));
            }
            return new TagDetection(distorted.id,
                    corners);
        }

        public CameraInterface(String name) {
            this.name = name;
            tagPub = NetworkTableInstance.getDefault()
                    .getStructArrayTopic("/gtsam_meme/" + name + "/input/tags", TagDetection.struct)
                    .publish(PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));
            robotTcamPub = NetworkTableInstance.getDefault()
                    .getStructTopic("/gtsam_meme/" + name + "/input/robotTcam", Transform3d.struct)
                    .publish(PubSubOption.sendAll(false), PubSubOption.keepDuplicates(false));
            camIntrinsicsPublisher = NetworkTableInstance.getDefault()
                    .getDoubleArrayTopic("/gtsam_meme/" + name + "/input/cam_intrinsics")
                    .publish(PubSubOption.sendAll(false), PubSubOption.keepDuplicates(false));
        }
    }

    StructPublisher<Twist3d> odomPub;
    StructPublisher<Pose3d> guessPub;
    Map<String, CameraInterface> cameras = new HashMap<>();
    StructSubscriber<Pose3d> optimizedPoseSub;

    // Estimated odom-only location relative to robot boot. We assume zero slip here
    Pose3d localOdometryPose = new Pose3d();
    TimeInterpolatableBuffer<Pose3d> odometryBuffer = TimeInterpolatableBuffer.createBuffer(5);

    public GTSAM(List<String> cameraNames) {
        odomPub = NetworkTableInstance.getDefault()
                .getStructTopic("/gtsam_meme/input/odom_twist", Twist3d.struct)
                .publish(PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));
        guessPub = NetworkTableInstance.getDefault()
                .getStructTopic("/gtsam_meme/input/pose_initial_guess", Pose3d.struct)
                .publish(PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));
        optimizedPoseSub = NetworkTableInstance.getDefault()
                .getStructTopic("/gtsam_meme/output/optimized_pose", Pose3d.struct)
                .subscribe(null, PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));

        cameraNames.stream().map(CameraInterface::new).forEach(it -> cameras.put(it.name, it));
    }

    /**
     * Update the core camera intrinsic parameters. The localizer will apply these
     * as soon as reasonable, and makes no attempt to latency compensate this.
     * 
     * @param camName    The name of the camera
     * @param intrinsics Camera intrinsics in standard OpenCV format. See:
     *                   https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html
     * @param distCoeffs Camera distortion coefficients, of length 4, 5 or 8
     */
    public void setCamIntrinsics(String camName, Optional<Matrix<N3, N3>> intrinsics,
            Optional<Matrix<N8, N1>> distCoeffs) {
        if (intrinsics.isEmpty() || distCoeffs.isEmpty()) {
            return;
        }

        var cam = cameras.get(camName);
        if (cam == null) {
            throw new RuntimeException("Camera " + camName + " not in map!");
        }

        cam.camIntrinsicsPublisher.set(new double[] {
                intrinsics.get().get(0, 0),
                intrinsics.get().get(1, 1),
                intrinsics.get().get(0, 2),
                intrinsics.get().get(1, 2),
        });

        cam.cameraCal = new CameraCalibration(intrinsics.get(), distCoeffs.get());
    }

    /**
     * Update the localizer with new info from this robot loop iteration.
     * 
     * @param odomTime The time that the odometry twist from last iteration
     *                 was collected at, in microseconds. WPIUtilJNI::now is
     *                 what I used
     * @param odom     The twist encoding chassis movement from last
     *                 timestamp to now. I use
     *                 SwerveDriveKinematics::toTwist2d
     * @param guess    An (optional, possibly null) initial guess at robot
     *                 pose from solvePNP or prior knowledge.
     */
    public void sendOdomUpdate(long odomTime, Twist3d odom,
            Pose3d guess) {

        odomPub.set(odom, odomTime);

        if (guess != null) {
            guessPub.set(guess, odomTime);
        }

        localOdometryPose = localOdometryPose.exp(odom);
        odometryBuffer.addSample(odomTime / 1e6, localOdometryPose);
    }

    /**
     * Update the localizer with new info from this robot loop iteration.
     * 
     * @param camName         The name of the camera
     * @param tagDetTime      The time that the frame encoding detected tags
     *                        collected at, in microseconds.
     * @param camDetectedTags The list of tags this camera could see
     * @param robotTcam       Transform from robot to camera optical focal point.
     *                        This is not latency compensated yet, so maybe don't
     *                        put your camera on a turret
     */
    public void sendVisionUpdate(String camName, double captureTimestamp, List<TagDetection> camDetectedTags,
            Transform3d robotTcam) {

        var cam = cameras.get(camName);
        if (cam == null) {
            throw new RuntimeException("Camera " + camName + " not in map!");
        }

        long tagDetTime = (long) (captureTimestamp * 1e6);

        camDetectedTags.forEach((TagDetection det) -> cam.undistort(det));

        cam.tagPub.set(camDetectedTags.toArray(new TagDetection[camDetectedTags.size()]), tagDetTime);
        cam.robotTcamPub.set(robotTcam, tagDetTime);
    }

    public Pose3d getLatencyCompensatedPoseEstimate() {
        var poseEst = optimizedPoseSub.getAtomic();
        if (poseEst.timestamp != 0) {
            var poseAtSample = odometryBuffer.getSample(poseEst.timestamp / 1e6);
            var poseNow = localOdometryPose;

            if (poseAtSample.isEmpty()) {
                // huh
                //System.err.println("pose outside buffer?");
                return new Pose3d();
            }

            var poseDelta = poseNow.minus(poseAtSample.get());
            return poseEst.value.transformBy(poseDelta);
        } else {
            //System.err.println("No pose estimate yet");
            return new Pose3d();
        }
    }

    public TimestampedValue<Pose3d> getAtomicPoseEstimate() {
        TimestampedObject<Pose3d> atomicVal = optimizedPoseSub.getAtomic();
        if (atomicVal.timestamp != 0) {
            return new TimestampedValue<Pose3d>(atomicVal.value, atomicVal.timestamp);
        } else {
            //System.err.println("No pose estimate yet");
            return new TimestampedValue<Pose3d>(new Pose3d(),0);
        }
        
    }
}