package frc.robot.BreakerLib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.BreakerLib.physics.ChassisAccels;

public interface Localizer {
    public TimestampedValue<Pose2d> getAtomicPose();
    public Pose2d getPose();
    public ChassisSpeeds getSpeeds();
    public default ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getSpeeds(), getPose().getRotation());
    }
    public ChassisAccels getAccels();
    public default ChassisAccels getFieldRelativeAccels() {
        return ChassisAccels.fromRobotRelativeAccels(getAccels(), getPose().getRotation());
    }
    public void resetPose(Pose2d newPose);
    public default void resetRotation(Rotation2d newRotation) {
        resetPose(new Pose2d(getPose().getTranslation(), newRotation));
    }
    public default void resetTranslation(Translation2d newTranslation) {
        resetPose(new Pose2d(newTranslation, getPose().getRotation()));
    }

}
