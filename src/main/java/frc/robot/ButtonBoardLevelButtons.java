package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ButtonBoardConstants;

public class ButtonBoardLevelButtons {
    private JoystickButton l1Button;
    private JoystickButton l2Button;
    private JoystickButton l3Button;
    private JoystickButton l4Button;

    public ButtonBoardLevelButtons(GenericHID hid) {
        l1Button = new JoystickButton(hid, ButtonBoardConstants.L1_BUTTON);
        l2Button = new JoystickButton(hid, ButtonBoardConstants.L2_BUTTON);
        l3Button = new JoystickButton(hid, ButtonBoardConstants.L3_BUTTON);
        l4Button = new JoystickButton(hid, ButtonBoardConstants.L4_BUTTON);
    }

    public JoystickButton getL1Button() {
        return l1Button;
    }
    
    public JoystickButton getL2Button() {
        return l2Button;
    }
    
    public JoystickButton getL3Button() {
        return l3Button;
    }
    
    public JoystickButton getL4Button() {
        return l4Button;
    }
}
