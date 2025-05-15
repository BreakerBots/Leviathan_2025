package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.LedConstants.*;

import java.util.function.Supplier;

public class LED extends SubsystemBase {
    private final CANdle led;
    private Supplier<ControlRequest> requestSupplier;

    public LED() {
        led = new CANdle(7);
    }

    public Command set(Supplier<ControlRequest> requestSupplier) {
        return Commands.runOnce(() -> setFunctional(requestSupplier));
    }


    public void setFunctional(Supplier<ControlRequest> requestSupplier) {
        this.requestSupplier = requestSupplier;
    }

    @Override
    public void periodic() {
        var req = requestSupplier.get();
        if (!req.equals(led.getAppliedControl())) {
            led.setControl(req);
        }
    }

    

    public static class Animations {
        public static final LarsonAnimation kEnabledTeleop = new LarsonAnimation(0, kOverallMaxIndex).withColor(kRed).withBounceMode(LarsonBounceValue.Front).withSize(8).withFrameRate(50);
        public static final LarsonAnimation kEnabledAuto =  new LarsonAnimation(0, kOverallMaxIndex).withColor(kLimeGreen).withBounceMode(LarsonBounceValue.Front).withSize(8).withFrameRate(50);
        public static final RainbowAnimation kDisabled = new RainbowAnimation(0, kOverallMaxIndex).withBrightness(1.0);
        
        public static final StrobeAnimation kHasCoralAuto = new StrobeAnimation(0, kOverallMaxIndex).withColor(kWhite).withFrameRate(7);
        public static final StrobeAnimation kHasCoralTeleop = new StrobeAnimation(0, kOverallMaxIndex).withColor(kWhite).withFrameRate(7);
        public static final SingleFadeAnimation kAutoAllign = new SingleFadeAnimation(0, kOverallMaxIndex).withColor(kMidDeepBlue).withFrameRate(7);
    }
    
}
