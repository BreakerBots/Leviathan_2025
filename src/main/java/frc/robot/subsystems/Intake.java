// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.BreakerLib.sensors.BreakerBeamBreak;

/** Add your docs here. */
public class Intake {
    private TalonFX rollers;
    private TalonFX pivot;
    private CANcoder encoder;
    private BreakerBeamBreak coral;

    public static enum IntakeRollerState {
        INTAKE,
        EXTAKE,
        NEUTRAL
    }

    public static enum IntakePiviotState {
        EXTENDED,
        RETRACTED
    }
}
