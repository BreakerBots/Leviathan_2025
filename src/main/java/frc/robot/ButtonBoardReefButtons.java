package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ButtonBoardConstants;
import frc.robot.ReefPosition.ReefBranch;

public class ButtonBoardReefButtons {
    private final JoystickButton reefAButton;
    private final JoystickButton reefBButton;
    private final JoystickButton reefCButton;
    private final JoystickButton reefDButton;
    private final JoystickButton reefEButton;
    private final JoystickButton reefFButton;
    private final JoystickButton reefGButton;
    private final JoystickButton reefHButton;
    private final JoystickButton reefIButton;
    private final JoystickButton reefJButton;
    private final JoystickButton reefKButton;
    private final JoystickButton reefLButton;

    
    public ButtonBoardReefButtons(GenericHID hid) {
        reefAButton = new JoystickButton(hid, ButtonBoardConstants.REEF_A_BUTTON);
        reefBButton = new JoystickButton(hid, ButtonBoardConstants.REEF_B_BUTTON);
        reefCButton = new JoystickButton(hid, ButtonBoardConstants.REEF_C_BUTTON);
        reefDButton = new JoystickButton(hid, ButtonBoardConstants.REEF_D_BUTTON);
        reefEButton = new JoystickButton(hid, ButtonBoardConstants.REEF_E_BUTTON);
        reefFButton = new JoystickButton(hid, ButtonBoardConstants.REEF_F_BUTTON);
        reefGButton = new JoystickButton(hid, ButtonBoardConstants.REEF_G_BUTTON);
        reefHButton = new JoystickButton(hid, ButtonBoardConstants.REEF_H_BUTTON);
        reefIButton = new JoystickButton(hid, ButtonBoardConstants.REEF_I_BUTTON);
        reefJButton = new JoystickButton(hid, ButtonBoardConstants.REEF_J_BUTTON);
        reefKButton = new JoystickButton(hid, ButtonBoardConstants.REEF_K_BUTTON);
        reefLButton = new JoystickButton(hid, ButtonBoardConstants.REEF_L_BUTTON);
    }

    public JoystickButton getReefButtonA() {
        return reefAButton;
    }
    
    public JoystickButton getReefButtonB() {
        return reefBButton;
    }
    
    public JoystickButton getReefButtonC() {
        return reefCButton;
    }
    
    public JoystickButton getReefButtonD() {
        return reefDButton;
    }
    
    public JoystickButton getReefButtonE() {
        return reefEButton;
    }
    
    public JoystickButton getReefButtonF() {
        return reefFButton;
    }
    
    public JoystickButton getReefButtonG() {
        return reefGButton;
    }
    
    public JoystickButton getReefButtonH() {
        return reefHButton;
    }
    
    public JoystickButton getReefButtonI() {
        return reefIButton;
    }
    
    public JoystickButton getReefButtonJ() {
        return reefJButton;
    }
    
    public JoystickButton getReefButtonK() {
        return reefKButton;
    }
    
    public JoystickButton getReefButtonL() {
        return reefLButton;
    }

    public JoystickButton getButtonByBranch(ReefBranch branch) {
        return switch (branch) {
            case A -> getReefButtonA();
            case B -> getReefButtonB();
            case C -> getReefButtonC();
            case D -> getReefButtonD();
            case E -> getReefButtonE();
            case F -> getReefButtonF();
            case G -> getReefButtonG();
            case H -> getReefButtonH();
            case I -> getReefButtonI();
            case J -> getReefButtonJ();
            case K -> getReefButtonK();
            case L -> getReefButtonL();
        };
    }
}
