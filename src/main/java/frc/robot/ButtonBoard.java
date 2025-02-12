package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;

public class ButtonBoard {
    private final GenericHID hid;

    private final ButtonBoardLevelButtons levelButtons;
    private final ButtonBoardReefButtons reefButtons;
    private final ButtonBoardRightButtons rightButtons;

    public ButtonBoard(int port) {
        hid = new GenericHID(port);
        levelButtons = new ButtonBoardLevelButtons(hid);
        reefButtons = new ButtonBoardReefButtons(hid);
        rightButtons = new ButtonBoardRightButtons(hid);
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
}
