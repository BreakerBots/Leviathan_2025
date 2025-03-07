package frc.robot.subsystems.vision;

import java.util.Optional;

import frc.robot.BreakerLib.drivers.ZED;
import frc.robot.BreakerLib.drivers.ZED.LocalizationResults;

public class DepthVision {
    private ZED zed;

    public DepthVision() {

    }

    public Optional<LocalizationResults> getUnreadLocalizationResults() {
        return zed.getUnreadLocalizationResults();
    }

}
