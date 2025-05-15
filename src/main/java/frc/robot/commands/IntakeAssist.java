package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.BreakerLib.drivers.ZED.DetectionResults;
import frc.robot.BreakerLib.drivers.ZED.ObjectDimensions;
import frc.robot.BreakerLib.drivers.ZED.TrackedObject;
import frc.robot.BreakerLib.physics.BreakerVector2;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.DepthVision;

public class IntakeAssist extends Command{
    private Superstructure superstructure;
    private Supplier<Optional<TrackedObject>> targetSupplier;
    private DriveToPose driveToPose;
    private Pose2d targetPose;
    

    public IntakeAssist(Supplier<Optional<TrackedObject>> targetSupplier, Superstructure superstructure) {
        this.superstructure = superstructure;
        this.targetSupplier = targetSupplier;
        targetPose = superstructure.getLocalization().getPose();
        driveToPose = new DriveToPose(superstructure.getDrivetrain(), superstructure.getTipProtectionSystem(), this::getTargetPose);
        addRequirements(driveToPose.getRequirements());
    }

    public IntakeAssist(Superstructure superstructure) {
        this(()->getBestCoral(superstructure), superstructure);
    }
    
    private Pose2d getTargetPose() {
        return targetPose;
    }

    @Override
    public void initialize() {
        targetPose = superstructure.getLocalization().getPose();
        driveToPose.initialize();
    }
    
    public void execute() {
        Pose2d pose = superstructure.getLocalization().getPose();
        if (targetSupplier.get().isPresent()) {
            TrackedObject target = targetSupplier.get().get();
            Translation2d targetTrans = target.position().getPositionFieldSpace(false).toTranslation2d();
            Rotation2d targetAngleCamera = new Rotation2d(target.customValue());
            var tar = targetAngleCamera.minus(new Rotation2d(Constants.DepthVisionConstants.kCameraTransform.getRotation().getMeasureZ()));
            var taf = tar.rotateBy(pose.getRotation().unaryMinus());
            var offset = new BreakerVector2(taf, 0.5);
            var tgtA = targetTrans.plus(offset.getAsTranslation());
            var tgtB = targetTrans.minus(offset.getAsTranslation());
            if (tgtA.getDistance(pose.getTranslation()) > tgtB.getDistance(pose.getTranslation())) {
                targetPose = new Pose2d(tgtB, taf.plus(Rotation2d.k180deg));
            } else {
                targetPose = new Pose2d(tgtA, taf);
            }
            driveToPose.execute();
            
        } else {
            this.cancel();
        }
    }

    private static final Distance maxDistance = Meters.of(3.5);
    private static final double maxAngle = Math.toRadians(70);   
    private static final double distWeight = 0.5;
    private static final double angWeight = 0.5;


    private static Optional<TrackedObject> getBestCoral(Superstructure superstructure) {
        Pose2d robotPose = superstructure.getLocalization().getPose();
        DetectionResults detectionResults = superstructure.getLocalization().getDepthVision().getLatestDetectionResults();
        Pair<TrackedObject, Double> best = null;

        for (TrackedObject obj : detectionResults.getTrackedObjects()) {
            Translation2d objectTrans = obj.position().getPositionFieldSpace(true).toTranslation2d();
            double distance = robotPose.getTranslation().getDistance(objectTrans);
            // if (distance > maxDistance.in(Meters)) {
            //     continue;
            // }   

            // if (Math.abs(obj.customValue()) > maxAngle) {
            //     continue;
            // }

            double distScore = MathUtil.clamp(1 - (distance / maxDistance.in(Meters)), 0, 1) * distWeight;
            double angScore = MathUtil.clamp(1 - (Math.abs(obj.customValue()) / maxAngle), 0, 1) * angWeight;
            double compositeScore = distScore + angScore;
            if (best == null || best.getSecond() >= compositeScore) {
                best = new Pair<TrackedObject,Double>(obj, compositeScore);
                continue;
            }

        }
        if (best == null) {
            return Optional.empty();
        }
        return Optional.of(best.getFirst());
    }
    
    public void end(boolean interrupted) {
        // Logic to run when the command ends
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }
    
}
