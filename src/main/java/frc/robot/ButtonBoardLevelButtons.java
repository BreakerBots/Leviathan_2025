package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ButtonBoardConstants;
import frc.robot.ReefPosition.ReefBranch;
import frc.robot.ReefPosition.ReefLevel;

public class ButtonBoardLevelButtons {
    private final JoystickButton l1Button;
    private final JoystickButton l2Button;
    private final JoystickButton l3Button;
    private final JoystickButton l4Button;
    private final Trigger anyPressed;

    public ButtonBoardLevelButtons(GenericHID hid) {
        l1Button = new JoystickButton(hid, ButtonBoardConstants.L1_BUTTON);
        l2Button = new JoystickButton(hid, ButtonBoardConstants.L2_BUTTON);
        l3Button = new JoystickButton(hid, ButtonBoardConstants.L3_BUTTON);
        l4Button = new JoystickButton(hid, ButtonBoardConstants.L4_BUTTON);
        anyPressed = l1Button.or(l2Button).or(l3Button).or(l4Button);
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

    public Trigger isAnyPressed() {
        return anyPressed ;
    }

    public Optional<ReefLevel> getSelectedLevel() {
        for (ReefLevel level : ReefLevel.values()) {
            if (getButtonByLevel(level).getAsBoolean()) {
                return Optional.of(level);
            }
        }
        return Optional.empty();
    }

    public JoystickButton getButtonByLevel(ReefLevel level) {
        return switch (level) {
            case L1 -> getL1Button();
            case L2 -> getL2Button();
            case L3 -> getL3Button();
            case L4 -> getL4Button();
        };
    }
}
