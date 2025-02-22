// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.ReefPosition;
import frc.robot.ReefPosition.ReefBranch;
import frc.robot.ReefPosition.ReefLevel;
import frc.robot.subsystems.superstructure.Superstructure;

/** Add your docs here. */
public class Autos {
    private final Superstructure superstructure;
    private final AutoFactory autoFactory;

    public Autos(Superstructure superstructure) {
        this.superstructure = superstructure;
        autoFactory = superstructure.getDrivetrain().getAutoFactory();
    }

    public Command testPath() {
        final var routine = autoFactory.newRoutine("testPath");
        final var startTraj = routine.trajectory("Start 1 to Reef J");
        final var toCoralPS = routine.trajectory("Reef J to Coral PS");

        startTraj
            .done()
            .onTrue(Commands.sequence( // note: scoreOnReef is not implemented
                superstructure.scoreOnReef(new ReefPosition(ReefLevel.L4, ReefBranch.L)),
                toCoralPS.cmd()
            ));


        routine.active().onTrue(Commands.sequence(
            startTraj.resetOdometry(),
            startTraj.cmd()
        ));

        return routine.cmd();
    }
}
