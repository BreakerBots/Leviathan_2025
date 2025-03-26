package frc.robot;

import com.pathplanner.lib.util.GeometryUtil;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public enum CagePosition {
    FAR(new Pose2d(8.7765884, 7.25860023, Rotation2d.fromDegrees(0))),
    MID(new Pose2d(8.7765884, 6.15860023, Rotation2d.fromDegrees(0))),
    CLOSE(new Pose2d(8.7765884, 5.06598472, Rotation2d.fromDegrees(0)));

    private final Pose2d pose;

    private CagePosition(Pose2d poseBlue) {
        this.pose = poseBlue;
    }

    public Pose2d getClimbPose(Alliance alliance) {
        if (alliance == Alliance.Blue) return pose;
        return new Pose2d(pose.getX(), ChoreoAllianceFlipUtil.flipY(pose.getY()), pose.getRotation());
    }

    private Pose2d getLinearOffsetPose(Alliance alliance, double offset) {
        double desiredOffset = alliance == Alliance.Blue 
            ? -offset 
            : offset;
        final var desiredAngleOffset = alliance == Alliance.Blue
            ? Rotation2d.fromDegrees(0)
            : Rotation2d.fromDegrees(180);
        final var trans = getClimbPose(alliance).getTranslation().plus(new Translation2d(desiredOffset, desiredAngleOffset));
        return new Pose2d(trans, trans.getAngle());
    }

    public Pose2d getAlignPose(Alliance alliance) {
        return getLinearOffsetPose(alliance, 0.3);
    }

    public static CagePosition getClosest(Pose2d robotPose, Alliance alliance) {
        var closest = (Pair<CagePosition, Double>)null; // i know you'll hate this one, Roman.
        for (final var cage : CagePosition.values()) {
            final double dist = cage.getAlignPose(alliance).getTranslation().getDistance(robotPose.getTranslation());
            if (closest == null || dist < closest.getSecond()) {
                closest = new Pair<>(cage, dist);
            }
        }
        return closest.getFirst();
    }
}
