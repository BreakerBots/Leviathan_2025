package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ButtonBoardConstants;

public class ButtonBoardRightButtons {
    private final JoystickButton lowRightButton;
    private final JoystickButton lowRightSwitch;
    private final JoystickButton highRightSwitch;
    private final JoystickButton highRightButton;
    
    public ButtonBoardRightButtons(GenericHID hid) {
        lowRightButton = new JoystickButton(hid, ButtonBoardConstants.LOW_RIGHT_BUTTON);
        lowRightSwitch = new JoystickButton(hid, ButtonBoardConstants.LOW_RIGHT_SWITCH);
        highRightSwitch = new JoystickButton(hid, ButtonBoardConstants.HIGH_RIGHT_SWITCH);
        highRightButton = new JoystickButton(hid, ButtonBoardConstants.HIGH_RIGHT_BUTTON);
    }

    public JoystickButton getLowRightButton() {
        return lowRightButton;
    }

    public JoystickButton getLowRightSwitch() {
        return lowRightSwitch;
    }

    public JoystickButton getHighRightSwitch() {
        return highRightSwitch;
    }

    public JoystickButton getHighRightButton() {
        return highRightButton;
    }
}
