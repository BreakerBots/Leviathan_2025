// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.ElevatorSetpoint;
import frc.robot.subsystems.EndEffector.EndEffectorSetpoint;

/** Add your docs here. */
public class Superstructure extends SubsystemBase {
    private Elevator elevator;
    private Indexer indexer;
    private EndEffector endEffector;
    private Intake intake;
    private Climb climb;
    private Drivetrain drivetrain;

    public Superstructure() {
        
    }

    public Command extendToScoreInL4() {
        return Commands.sequence(
            elevator.set(ElevatorSetpoint.HANDOFF, true);
            endEffector.set(EndEffectorSetpoint., false)
        );
    }

    private void constrainEndEffectorWrist() {
        double 
    }

    @Override
    public void periodic() {

    }
}
