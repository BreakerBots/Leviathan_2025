package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Elevator.ElevatorSetpoint;
import frc.robot.subsystems.EndEffector.EndEffectorSetpoint;

public class ReefPosition {
    public static enum ReefLevel {
        L1,
        L2,
        L3,
        L4;
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
