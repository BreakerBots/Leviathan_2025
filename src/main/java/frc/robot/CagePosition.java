package frc.robot;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public enum CagePosition {
    FAR(),
    MID(),
    CLOSE();
    public CagePosition(Pose2d poseBlue) {
        this.pose = pose;
    }

    public Pose2d getClimbPose() {
        return pose;
    }

    private Pose2d getLinearOffsetPose(Alliance alliance, double offset) {
        allyPose = MathUtil.
        return new Pose2d(pose.getTranslation().plus(new Translation2d(offset, )), pose.getRotation());
    }

    public Pose2d getAllignPose() {
        return new Pose2d(pose.getTranslation().minus(new Translation2d(0.3, pose.getRotation())), pose.getRotation());
    }
}
