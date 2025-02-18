package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.AutoPilotConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.superstructure.Superstructure.MastState;

public class ReefPosition {
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
        A(0, 0, true),
        B(0, 0, false),
        C(0, 0, true),
        D(0, 0, false),
        E(0, 0, true),
        F(0, 0, false),
        G(0, 0, true),
        H(0, 0, false),
        I(0, 0, true),
        J(0, 0, false),
        K(0, 0, true),
        L(0, 0, false);

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
                FieldConstants.kReefBranchOffsetFromFaceApriltag.getX() - AutoPilotConstants.kReefAutoAllignOffsetFromBranch.in(Units.Meters), 
                isLeft ? FieldConstants.kReefBranchOffsetFromFaceApriltag.getY() : -FieldConstants.kReefBranchOffsetFromFaceApriltag.getY());
            
            Translation2d allignOffset = allignOffsetRel.rotateBy(tagPose.getRotation().minus(new Rotation2d(Math.PI)));

            Translation2d allignTrans = tagPose.getTranslation().plus(allignOffset);

            Pose2d allignPose = new Pose2d(allignTrans, tagPose.getRotation());
            

            return allignPose;

        }
    }

    

    
}
