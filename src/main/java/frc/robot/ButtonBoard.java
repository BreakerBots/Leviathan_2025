package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.BreakerLib.util.ReefPosition.ReefBranch;
import frc.robot.BreakerLib.util.ReefPosition.ReefLevel;

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

    public JoystickButton getButtonByLevel(ReefLevel level) {
        var buttons = getLevelButtons();
        return switch (level) {
            case L1 -> buttons.getL1Button();
            case L2 -> buttons.getL2Button();
            case L3 -> buttons.getL3Button();
            case L4 -> buttons.getL4Button();
        };
    }

    public JoystickButton getButtonByBranch(ReefBranch branch) {
        var buttons = getReefButtons();
        return switch (branch) {
            case A -> buttons.getReefButtonA();
            case B -> buttons.getReefButtonB();
            case C -> buttons.getReefButtonC();
            case D -> buttons.getReefButtonD();
            case E -> buttons.getReefButtonE();
            case F -> buttons.getReefButtonF();
            case G -> buttons.getReefButtonG();
            case H -> buttons.getReefButtonH();
            case I -> buttons.getReefButtonI();
            case J -> buttons.getReefButtonJ();
            case K -> buttons.getReefButtonK();
            case L -> buttons.getReefButtonL();
        };
    }


    public GenericHID getBaseHID() {
        return hid;
    }
}
