package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;

public class ButtonBoard {
    private GenericHID hid;

    private ButtonBoardLevelButtons levelButtons;
    private ButtonBoardReefButtons reefButtons;

    public ButtonBoard(GenericHID hid) {
        this.hid = hid;
        levelButtons = new ButtonBoardLevelButtons(hid);
        reefButtons = new ButtonBoardReefButtons(hid);
    }
    
    public ButtonBoardLevelButtons getLevelButtons() {
        return levelButtons;
    }

    public ButtonBoardReefButtons getReefButtons() {
        return reefButtons;
    }

    public GenericHID getBaseHID() {
        return hid;
    }
}
