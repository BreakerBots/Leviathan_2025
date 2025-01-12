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
    private TalonFX piviot;
    private CANcoder encoder;
    private BreakerBeamBreak coral;
    private BreakerBeamBreak algae;

    public static enum IntakeRollerState {
        INTAKE_CORAL_EXTAKE_ALGAE,
        EXTAKE_CORAL_INTAKE_ALGAE,
        NEUTRAL
    }

    public static enum IntakePiviotState {
        EXTENDED,
        RETRACTED
    }
}
