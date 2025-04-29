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
import frc.robot.BreakerLib.drivers.ZED.DetectionResults;
import frc.robot.BreakerLib.drivers.ZED.ObjectDimensions;
import frc.robot.BreakerLib.drivers.ZED.TrackedObject;
import frc.robot.BreakerLib.physics.BreakerVector2;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.subsystems.superstructure.Superstructure;

public class IntakeAssist extends Command{
    // This class is a placeholder for the actual implementation of the IntakeAssist command.
    // The actual implementation would include methods and logic to assist with the intake process.
    private Superstructure superstructure;
    private Supplier<Optional<TrackedObject>> targetSupplier;
     // Placeholder for the target ID
    public IntakeAssist(Supplier<Optional<TrackedObject>> targetSupplier, Superstructure superstructure) {
        // Constructor logic here
    }
    
    public void execute() {
        if (targetSupplier.get().isPresent()) {
            TrackedObject target = targetSupplier.get().get();
            Translation2d targetTrans = target.position().getPositionFieldSpace(false).toTranslation2d();
            
        } else {
            // Logic when no target is detected
        }
    }

    private final ObjectDimensions coralDims = new ObjectDimensions(new Translation3d(Inches.of(4.5), Inches.of(4.5), Inches.of(11.875))); // Example dimensions
    private final Distance maxDistance = Meters.of(3.5);
    private final double maxAngle = Math.toRadians(70);   
    private final double distWeight = 0.5;
    private final double aproachAngleWeight = 0.5;


    private Optional<TrackedObject> getBestCoral() {
        Pose2d robotPose = superstructure.getLocalization().getPose();
        ChassisSpeeds robotSpeeds = superstructure.getLocalization().getFieldRelativeSpeeds();
        BreakerVector2 robotVelVec = new BreakerVector2(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);
        DetectionResults detectionResults = superstructure.getLocalization().getDepthVision().getLatestDetectionResults();
        for (TrackedObject obj : detectionResults.getTrackedObjects()) {
            Translation2d objectTrans = obj.position().getPositionFieldSpace(true).toTranslation2d();
            double distance = robotPose.getTranslation().getDistance(objectTrans);
            if (distance > maxDistance.in(Meters)) {
                continue;
            }   

            ObjectDimensions dims = obj.cameraRelitiveDimensions();
            double box_x = MathUtil.inverseInterpolate(coralDims.length(), coralDims.width(), dims.width());
            double box_y = MathUtil.inverseInterpolate(coralDims.length(), coralDims.width(), dims.length());
            BreakerVector2 coralVec = new BreakerVector2(box_x, box_y);
            Rotation2d coralAngle = coralVec.getAngle();
            if (coralAngle.getRadians() > maxAngle) {
                continue;
            }

            Rotation2d coralAngleToBot = objectTrans.minus(robotPose.getTranslation()).getAngle();
            
            double aproachAngleDif = BreakerMath.getDifferenceBetweenWrapedValues(coralAngleToBot.getRadians(), robotVelVec.getAngle().getRadians(), -Math.PI, Math.PI);

            double distWeight = MathUtil.clamp(1 - (distance / maxDistance.in(Meters)), 0, 1);

            

        }
        return null;
    }
    
    public void end(boolean interrupted) {
        // Logic to run when the command ends
    }
    
}
