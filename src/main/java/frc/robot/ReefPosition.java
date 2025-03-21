package frc.robot;

import java.security.AllPermission;
import java.util.HashMap;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.AutoPilotConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.superstructure.Superstructure.MastState;
import frc.robot.subsystems.superstructure.Superstructure2.SuperstructureState;

public record ReefPosition(ReefLevel level, ReefBranch branch) {
    public static enum ReefLevel {
        L1(MastState.L1_NEUTRAL, MastState.L1_EXTAKE, SuperstructureState.L2),
        L2(MastState.L2_NEUTRAL, MastState.L2_EXTAKE, SuperstructureState.L2),
        L3(MastState.L3_NEUTRAL, MastState.L3_EXTAKE, SuperstructureState.L3),
        L4(MastState.L4_NEUTRAL, MastState.L4_EXTAKE, SuperstructureState.L4);
        private MastState neutral, extake;
        private SuperstructureState extakeState;
        private ReefLevel(MastState neutral, MastState extake, SuperstructureState extakeState) {
            this.extake = extake;
            this.neutral = neutral;
            this.extakeState = extakeState;
        }

        public SuperstructureState getExtakeSuperstructureState() {
            return extakeState;
        }

        public MastState getExtakeMastState() {
            return extake;
        }

        public MastState getNeutralMastState() {
            return neutral;
        }
    }

    public static enum ReefBranch {
        A(18, 7, !true),
        B(18, 7, !false),
        C(17, 8, !true),
        D(17, 8, !false),
        E(22, 9, !true),
        F(22, 9, !false),
        G(21, 10, !true),
        H(21, 10, !false),
        I(20, 11, !true),
        J(20, 11, !false),
        K(19, 6, !true),
        L(19, 6, !false);

        private int blueReefFaceApriltagID, redReefFaceApriltagID;
        private boolean isLeft;
        private ReefBranch(int blueReefFaceApriltagID, int redReefFaceApriltagID, boolean isLeft) {
            this.blueReefFaceApriltagID = blueReefFaceApriltagID;
            this.redReefFaceApriltagID = redReefFaceApriltagID;
            this.isLeft = isLeft;
        }

        public Pose2d getAlignPose(Alliance alliance) {
            Pose2d tagPose = FieldConstants.kAprilTagFieldLayout.getTagPose(alliance == Alliance.Blue ? blueReefFaceApriltagID : redReefFaceApriltagID).get().toPose2d();
            Distance endEffectorOffset = Units.Inches.of(0.5);
            Translation2d allignOffsetRel = new Translation2d(
                -AutoPilotConstants.kReefAutoAllignOffsetFromReefFace.in(Units.Meters), 
                ((isLeft ? 1 : -1) * -FieldConstants.kReefBranchOffsetFromFaceApriltagStrafe.in(Units.Meters) + endEffectorOffset.in(Units.Meter)));

            Translation2d allignOffset = allignOffsetRel.rotateBy(tagPose.getRotation().minus(new Rotation2d(Math.PI)));

            Translation2d allignTrans = tagPose.getTranslation().plus(allignOffset);

            Pose2d allignPose = new Pose2d(allignTrans, tagPose.getRotation());
            

            return allignPose;

        }

        public static ReefBranch getClosest(Pose2d robotPose, Alliance alliance) {
            Pair<ReefBranch, Double> closestBranchAndDist = null;
            for (ReefBranch branch : ReefBranch.values()) {
                var pose = branch.getAlignPose(alliance);
                double dist = pose.getTranslation().getDistance(robotPose.getTranslation());
                if (closestBranchAndDist == null) {
                    closestBranchAndDist = new Pair<ReefPosition.ReefBranch,Double>(branch, dist);
                } else if (closestBranchAndDist.getSecond() > dist) {
                    closestBranchAndDist = new Pair<ReefPosition.ReefBranch,Double>(branch, dist);
                }
            }
            return closestBranchAndDist.getFirst();

        }
    }
}
