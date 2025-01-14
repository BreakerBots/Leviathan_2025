// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import frc.robot.BreakerLib.sensors.BreakerBeamBreak;
/** Add your docs here. */
public class Climb {
    private TalonFX fork;
    private TalonFX winch;
    private CANcoder forkCoder;
    private BreakerBeamBreak forkSensor;
    public Climb(){
        fork = new TalonFX(0);
        winch = new TalonFX(0);
        forkCoder = new CANcoder(0);
        forkSensor = BreakerBeamBreak.fromDIO(0, true);
    };
    

    public enum ClimbForkState{
        RETRACTED,
        EXTENDED;
    }

    

}
