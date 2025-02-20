package frc.robot.subsystems.vision;

import java.security.spec.PSSParameterSpec;
import java.util.ArrayList;
import java.util.Vector;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.util.TimestampedValue;
import frc.robot.Constants.FieldConstants;

public class ApriltagVision extends SubsystemBase {
    private PhotonCamera[] cameras;
    private PhotonPoseEstimator poseEstimator;

    


    public ApriltagVision() {
        var frontLeftCam = new PhotonCamera("frontLeft");
        var frontRightCam = new PhotonCamera("frontRight");
        var backLeftCam = new PhotonCamera("backLeft");
        var backRightCam = new PhotonCamera("backRight");
        cameras = new PhotonCamera[]{frontLeftCam, frontRightCam, backLeftCam, backRightCam};

      

    }

    @Override
    public void periodic() {
        
        for (var cam : cameras) {
            var unreadResults = cam.getAllUnreadResults();
            var cameraMatrix = cam.getCameraMatrix();
            var distCoeffs = cam.getDistCoeffs();
            
            poseEstimator.update(null);
        }
    }

    private static record RawPoseEstimate() {

    }
    
}
