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
    public Command startAnywhereThenJKLA(StartPosition startPosition) {
        final var routine = autoFactory.newRoutine("Start->JKLA");

        final var startToReefJ = switch (startPosition) {
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

    public Command startThenGFED(StartPosition startPosition) { // I would like to put all these strings in a nice enum eventually to avoid typos, sounds like a job for the freshies.
        final var routine = autoFactory.newRoutine("Start2->GFED");
        
        final var startToG = switch (startPosition) {
            case ONE -> routine.trajectory("Start 1 to Reef G");
            case TWO -> routine.trajectory("Start 2 to Reef G");
            case THREE -> routine.trajectory("Start 3 to Reef G");
        };

        final var reefGToPS2 = routine.trajectory("Reef G to Coral PS2");
        final var ps2ToReefF = routine.trajectory("Coral PS2 to Reef F");
        final var reefFToPS2 = routine.trajectory("Reef F to Coral PS2");
        final var ps2ToReefE = routine.trajectory("Coral PS2 to Reef E");
        final var reefEToPS2 = routine.trajectory("Reef E to Coral PS2");
        final var ps2ToReefD = routine.trajectory("Coral PS2 to Reef D");

        startToG.done().onTrue(Commands.sequence(
            superstructure.scoreOnReef(new ReefPosition(ReefLevel.L4, ReefBranch.G)),
            reefGToPS2.cmd()
        ));

        reefGToPS2.done().onTrue(Commands.sequence(
            superstructure.intakeCoralFromHumanPlayer(),
            ps2ToReefF.cmd()
        ));

        ps2ToReefF.done().onTrue(Commands.sequence(
            superstructure.scoreOnReef(new ReefPosition(ReefLevel.L4, ReefBranch.F)),
            reefFToPS2.cmd()
        ));

        reefFToPS2.done().onTrue(Commands.sequence(
            superstructure.intakeCoralFromHumanPlayer(),
            ps2ToReefE.cmd()
        ));

        ps2ToReefE.done().onTrue(Commands.sequence(
            superstructure.scoreOnReef(new ReefPosition(ReefLevel.L4, ReefBranch.E)),
            reefEToPS2.cmd()
        ));

        reefEToPS2.done().onTrue(Commands.sequence(
            superstructure.intakeCoralFromHumanPlayer(),
            ps2ToReefD.cmd()
        ));

        routine.active().onTrue(Commands.sequence(
            startToG.resetOdometry(),
            startToG.cmd()
        ));

        return routine.cmd();
    }

    // This autopath is a little strange IMO, however I couldn't think of any more interesting paths for coral.
    public Command start3ThenABC() {
        final var routine = autoFactory.newRoutine("Start3->ABC");
        
        final var startToA = routine.trajectory("Start 3 to Reef A");

        final var reefAToPS2 = routine.trajectory("Reef A to Coral PS2");
        final var ps2ToReefB = routine.trajectory("Coral PS2 to Reef B");
        final var reefBToPS2 = routine.trajectory("Reef B to Coral PS2");
        final var ps2ToReefC = routine.trajectory("Coral PS2 to Reef C");

        startToA.done().onTrue(Commands.sequence(
            superstructure.scoreOnReef(new ReefPosition(ReefLevel.L4, ReefBranch.A)),
            reefAToPS2.cmd()
        ));

        reefAToPS2.done().onTrue(Commands.sequence(
            superstructure.intakeCoralFromHumanPlayer(),
            ps2ToReefB.cmd()
        ));

        ps2ToReefB.done().onTrue(Commands.sequence(
            superstructure.scoreOnReef(new ReefPosition(ReefLevel.L4, ReefBranch.B)),
            reefBToPS2.cmd()
        ));

        reefBToPS2.done().onTrue(Commands.sequence(
            superstructure.intakeCoralFromHumanPlayer(),
            ps2ToReefC.cmd()
        ));

        ps2ToReefC.done().onTrue(Commands.sequence(
            superstructure.scoreOnReef(new ReefPosition(ReefLevel.L4, ReefBranch.C))
        ));

        return routine.cmd();
    }
}
