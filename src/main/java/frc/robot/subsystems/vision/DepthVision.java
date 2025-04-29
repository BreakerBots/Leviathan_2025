package frc.robot.subsystems.vision;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.BreakerLib.drivers.ZED;
import frc.robot.BreakerLib.drivers.ZED.DetectionResults;
import frc.robot.BreakerLib.drivers.ZED.LocalizationResults;
import frc.robot.BreakerLib.drivers.ZED.TrackedObject;
import frc.robot.BreakerLib.util.TimestampedValue;

public class DepthVision {
    private ZED zed;

    public DepthVision(Supplier<TimestampedValue<Pose3d>> atomicRobotPoseSupplier) {
        zed = new ZED("", atomicRobotPoseSupplier, new Transform3d());
    }

    public Optional<LocalizationResults> getUnreadLocalizationResults() {
        return zed.getUnreadLocalizationResults();
    }

    public DetectionResults getLatestDetectionResults() {
        return zed.getDetectionResults();
    }

}
