package frc.robot;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.BreakerLib.physics.BreakerVector2;
import frc.robot.Constants.FieldConstants;

public enum CoralHumanPlayerStation {
    UPPER(13, 2),
    LOWER(12, 1);

    private final int blueTag;
    private final int redTag;

    CoralHumanPlayerStation(int blueTag, int redTag) {
        this.blueTag = blueTag;
        this.redTag = redTag;
    }

    public CoralHumanPlayerStation swap() {
        return switch (this) {
            case UPPER -> CoralHumanPlayerStation.LOWER;
            case LOWER -> CoralHumanPlayerStation.UPPER;
        };
    }

    public Pose2d getAlignPose(Alliance alliance) {
        Pose2d tagPose = FieldConstants.kAprilTagFieldLayout.getTagPose(alliance == Alliance.Blue ? blueTag : redTag).get().toPose2d();
        
        final var pos = new BreakerVector2(tagPose.getTranslation());
        final Distance offset = Meter.of(0.7);

        final var finalTranslation = pos.plus(new BreakerVector2(tagPose.getRotation(), offset.in(Meter)));
        return new Pose2d(finalTranslation.getAsTranslation(), tagPose.getRotation());
    }
}
