// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.BreakerLib.drivers.gtsam.GTSAM;
import frc.robot.BreakerLib.physics.ChassisAccels;
import frc.robot.BreakerLib.util.Localizer;
import frc.robot.BreakerLib.util.TimestampedValue;
import frc.robot.BreakerLib.util.math.OdometryFusion;

/** Add your docs here. */
public class Localization implements Localizer {
    // private GTSAM gtsam;
    private ApriltagVision apriltagVision;
    // private DepthVision depthVision;
    // private OdometryFusion<SwerveModulePosition[]> fusedOdometry;
    // private Pair<Pose2d, Double> prevPoseFrontZED;
    // private Pair<Pose2d, Double> prevPoseBackZED;  
    public Localization() {
    }



    @Override
    public TimestampedValue<Pose2d> getAtomicPose() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getAtomicPose'");
    }
    @Override
    public Pose2d getPose() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPose'");
    }
    @Override
    public ChassisSpeeds getSpeeds() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getSpeeds'");
    }
    @Override
    public ChassisAccels getAccels() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getAccels'");
    }
    @Override
    public void resetPose(Pose2d newPose) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetPose'");
    }
} 
