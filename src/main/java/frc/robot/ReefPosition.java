package frc.robot;

import java.security.AllPermission;
import java.util.HashMap;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.AutoPilotConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.superstructure.Superstructure.MastState;

public record ReefPosition(ReefLevel level, ReefBranch branch) {
    public static enum ReefLevel {
        L1(MastState.L1_NEUTRAL, MastState.L1_EXTAKE),
        L2(MastState.L2_NEUTRAL, MastState.L2_EXTAKE),
        L3(MastState.L3_NEUTRAL, MastState.L3_EXTAKE),
        L4(MastState.L4_NEUTRAL, MastState.L4_EXTAKE);
        private MastState neutral, extake;
        private ReefLevel(MastState neutral, MastState extake) {
            this.extake = extake;
            this.neutral = neutral;
        }

        public MastState getExtakeMastState() {
            return extake;
        }

        public MastState getNeutralMastState() {
            return neutral;
        }
    }

    public static enum ReefBranch {
        A(18, 0, !true),
        B(18, 0, !false),
        C(17, 0, !true),
        D(17, 0, !false),
        E(22, 0, !true),
        F(22, 0, !false),
        G(21, 0, !true),
        H(21, 0, !false),
        I(20, 0, !true),
        J(20, 0, !false),
        K(19, 0, !true),
        L(19, 0, !false);

        private int blueReefFaceApriltagID, redReefFaceApriltagID;
        private boolean isLeft;
        private ReefBranch(int blueReefFaceApriltagID, int redReefFaceApriltagID, boolean isLeft) {
            this.blueReefFaceApriltagID = blueReefFaceApriltagID;
            this.redReefFaceApriltagID = redReefFaceApriltagID;
            this.isLeft = isLeft;
        }

        public Pose2d getAllignPose(Alliance alliance) {
            Pose2d tagPose = FieldConstants.kAprilTagFieldLayout.getTagPose(alliance == Alliance.Blue ? blueReefFaceApriltagID : redReefFaceApriltagID).get().toPose2d();

            Translation2d allignOffsetRel = new Translation2d(
                (alliance == Alliance.Blue ? -1 : 1) * AutoPilotConstants.kReefAutoAllignOffsetFromReefFace.in(Units.Meters), 
                (isLeft ? 1 : -1) * (alliance == Alliance.Blue ? -1 : 1) * FieldConstants.kReefBranchOffsetFromFaceApriltagStrafe.in(Units.Meters));

            Translation2d allignOffset = allignOffsetRel.rotateBy(tagPose.getRotation().minus(new Rotation2d(Math.PI)));

            Translation2d allignTrans = tagPose.getTranslation().plus(allignOffset);

            Pose2d allignPose = new Pose2d(allignTrans, tagPose.getRotation());
            

            return allignPose;

        }

        public static ReefBranch getClosest(Pose2d robotPose, Alliance alliance) {
            Pair<ReefBranch, Double> closestBranchAndDist = null;
            for (ReefBranch branch : ReefBranch.values()) {
                var pose = branch.getAllignPose(alliance);
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
