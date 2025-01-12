// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;

import frc.robot.BreakerLib.sensors.BreakerBeamBreak;

/** Add your docs here. */
public class EndEffector {
    private TalonSRX rollers;
    private TalonFX pivot;
    private CANcoder pivotEncoder;
    private BreakerBeamBreak coralSensor;
    public EndEffector() {
        
    }
 

}
