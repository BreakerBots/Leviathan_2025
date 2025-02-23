// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import choreo.auto.AutoFactory;
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

    public static enum StartPosition {
        ONE,
        TWO,
        THREE,
    }

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

    /**
     * Given a starting point, it will then go to
     * Reef J -> K -> L -> A, each time it will also visit the Coral Player Station
     * to restock on Coral.
     */
    public Command startAnywhereThenJKLA(StartPosition startPos) {
        final var routine = autoFactory.newRoutine("Start1Full");

        final var startToReefJ = switch (startPos) {
            case ONE -> routine.trajectory("Start 1 to Reef J");
            case TWO -> routine.trajectory("Start 2 to Reef J");
            case THREE -> routine.trajectory("Start 3 to Reef J");
        };

        final var reefJToCoralPS = routine.trajectory("Reef J to Coral PS");
        final var coralPSToReefK = routine.trajectory("Coral PS to Reef K");
        final var reefKToCoralPS = routine.trajectory("Reef K to Coral PS");
        final var coralPSToReefL = routine.trajectory("Coral PS to Reef L");
        final var reefLToCoralPS = routine.trajectory("Reef L to Coral PS");
        final var coralPSToReefA = routine.trajectory("Coral PS to Reef A");
        
        
        startToReefJ.done().onTrue(Commands.sequence( // note: scoreOnReef is not implemented
            superstructure.scoreOnReef(new ReefPosition(ReefLevel.L4, ReefBranch.J)),
            // scoreOnReef should stow
            reefJToCoralPS.cmd()
        ));
                
                
        reefJToCoralPS.done().onTrue(Commands.sequence(
            superstructure.intakeCoralFromHumanPlayer(),
            coralPSToReefK.cmd()
        ));

        coralPSToReefK.done().onTrue(Commands.sequence(
            superstructure.scoreOnReef(new ReefPosition(ReefLevel.L4, ReefBranch.K)),
            reefKToCoralPS.cmd()
        ));

        reefKToCoralPS.done().onTrue(Commands.sequence(
            superstructure.intakeCoralFromHumanPlayer(),
            coralPSToReefL.cmd()
        ));

        coralPSToReefL.done().onTrue(Commands.sequence(
            superstructure.scoreOnReef(new ReefPosition(ReefLevel.L4, ReefBranch.L)),
            reefLToCoralPS.cmd()
        ));

        reefLToCoralPS.done().onTrue(Commands.sequence(
            superstructure.intakeCoralFromHumanPlayer(),
            coralPSToReefA.cmd()
        ));


        routine.active().onTrue(Commands.sequence(
            startToReefJ.resetOdometry(),
            startToReefJ.cmd()
        ));

        return routine.cmd();
    }
}
