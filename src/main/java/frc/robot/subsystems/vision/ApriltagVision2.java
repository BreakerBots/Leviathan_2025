package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.BreakerLib.drivers.gtsam.GTSAM;
import frc.robot.BreakerLib.util.Localizer;
import frc.robot.Constants.FieldConstants;

public class ApriltagVision2 {
    private GTSAM gtsam;
    private VisionSystemSim sim;
    private Camera[] cameras;
    private Localizer localizer;

    private void update() {
        FrameContext context = new FrameContext(localizer.getPose());
        for (var cam : cameras) {
            cam.update(loc);
        }
    }
    

    private static class Camera {

        private PhotonCamera camera;
        private PhotonCameraSim sim;
        private PhotonPoseEstimator poseEstimator;
        
        public Camera(String cameraName, VisionSystemSim systemSim, Transform3d robotTcam) {
            camera = new PhotonCamera(cameraName);
            sim = new PhotonCameraSim(camera);
            systemSim.addCamera(sim, robotTcam);
            poseEstimator = new PhotonPoseEstimator(FieldConstants.kAprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotTcam);
            

        }

        public List<CameraResult> update(FrameContext context) {
            var rawResults = camera.getAllUnreadResults();

            for (PhotonPipelineResult res : rawResults) {
                Optional<EstimatedRobotPose> estOpt =  poseEstimator.update(res);
                if (estOpt.isPresent()) {

                }
            }
        }


    }

    private record FrameContext(
        Pose2d prevPose,
        PoseStrategy primaryStrategy
    ) {}

    private record CameraResults(
        EstimatedRobotPose est,
        Matrix<N3, N1> stdDevs
        ) {
    }
}
