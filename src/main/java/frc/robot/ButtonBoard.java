package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.Constants.ButtonBoardConstants;

public class ButtonBoard {
    private final GenericHID hid;

    private final ButtonBoardLevelButtons levelButtons;
    private final ButtonBoardReefButtons reefButtons;
    private final ButtonBoardRightButtons rightButtons;

    private boolean potentiallyShorted = false;

    public ButtonBoard(int port) {
        hid = new GenericHID(port);
        levelButtons = new ButtonBoardLevelButtons(hid);
        reefButtons = new ButtonBoardReefButtons(hid);
        rightButtons = new ButtonBoardRightButtons(hid);

        new JoystickButton(hid, ButtonBoardConstants.POTENTIAL_SHORT_BUTTON)
            .onTrue(Commands.runOnce(() -> {
                potentiallyShorted = true;
                BreakerLog.log("ButtonBoard/Shorted", potentiallyShorted);
            }));
    }
    
    public ButtonBoardLevelButtons getLevelButtons() {
        return levelButtons;
    }

    public ButtonBoardReefButtons getReefButtons() {
        return reefButtons;
    }

    public ButtonBoardRightButtons getRightButtons() {
        return rightButtons;
    }

    public GenericHID getBaseHID() {
        return hid;
    }

    public boolean isPotentiallyShorted() {
        return potentiallyShorted;
    }
}
