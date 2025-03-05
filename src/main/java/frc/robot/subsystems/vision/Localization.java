// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.BreakerLib.drivers.gtsam.GTSAM;
import frc.robot.BreakerLib.physics.ChassisAccels;
import frc.robot.BreakerLib.util.Localizer;
import frc.robot.BreakerLib.util.TimestampedValue;
import frc.robot.BreakerLib.util.math.OdometryFusion;
import frc.robot.subsystems.Drivetrain;

/** Add your docs here. */
public class Localization implements Localizer {
    // private GTSAM gtsam;
    private ApriltagVision apriltagVision;
    private OdometryFusion<SwerveModulePosition[]> odometryFusion;
    private BreakerPoseEstimator<SwerveModulePosition[]> visionFilter;
    private DepthVision depthVision;
    public Localization(Drivetrain drivetrain) {
        odometryFusion = new OdometryFusion<>(
            drivetrain.getKinematics(), 
            new SwerveDriveOdometry(
                drivetrain.getKinematics(),
                drivetrain.getState().RawHeading, 
                drivetrain.getState().ModulePositions, 
                new Pose2d()
            ),
            VecBuilder.fill(0, 0, 0),
            VecBuilder.fill(0, 0, 0)
        );

        visionFilter = new BreakerPoseEstimator<>(
            drivetrain.getKinematics(),
            odometryFusion,
            VecBuilder.fill(0, 0, 0),
            VecBuilder.fill(0, 0, 0)
        );
    }

    public updateWheelOdometry(SwerveDriveState state) {
        
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
