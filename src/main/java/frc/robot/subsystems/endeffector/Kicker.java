package frc.robot.subsystems.endeffector;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ElevatorConstants.kA;
import static frc.robot.Constants.KickerConstants.*;

import java.awt.color.ColorSpace;

public class Kicker extends SubsystemBase{
    private Canandcolor algaeSensor;
    private TalonSRX kickerMotor;
    
    public Kicker() {

    }

    public boolean hasAlgae() {
        return getColorDelta(algaeSensor.getColor().toWpilibColor(), kAlgaeColor) <= kMaxColorDelta;
    }

    public static double getColorDelta(Color a, Color b) {
        var at = new Translation3d(a.red, a.green, a.blue);
        var bt = new Translation3d(b.red, b.green, b.blue);
        return at.getDistance(bt);
    }

    public static enum KickerState {
        NEUTRAL,
        RUNNING
    }

    
}
