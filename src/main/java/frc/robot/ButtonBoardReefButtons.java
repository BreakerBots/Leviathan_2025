package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ButtonBoardConstants;

public class ButtonBoardReefButtons {
    private JoystickButton reefAButton;
    private JoystickButton reefBButton;
    private JoystickButton reefCButton;
    private JoystickButton reefDButton;
    private JoystickButton reefEButton;
    private JoystickButton reefFButton;
    private JoystickButton reefGButton;
    private JoystickButton reefHButton;
    private JoystickButton reefIButton;
    private JoystickButton reefJButton;
    private JoystickButton reefKButton;
    private JoystickButton reefLButton;

    
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
}
