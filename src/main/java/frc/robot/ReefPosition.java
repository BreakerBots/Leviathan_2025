package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
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
        A(new Pose2d()),
        B(new Pose2d()),
        C(new Pose2d()),
        D(new Pose2d()),
        E(new Pose2d()),
        F(new Pose2d()),
        G(new Pose2d()),
        H(new Pose2d()),
        I(new Pose2d()),
        J(new Pose2d()),
        K(new Pose2d()),
        L(new Pose2d());

        private ReefBranch(Pose2d blueAllignPose) {

        }
    }

    

    
}
