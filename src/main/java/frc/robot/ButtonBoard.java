package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.BreakerLib.util.logging.Elastic;
import frc.robot.BreakerLib.util.logging.LoggedAlert;
import frc.robot.BreakerLib.util.logging.Elastic.Notification;
import frc.robot.BreakerLib.util.logging.Elastic.Notification.NotificationLevel;
import frc.robot.Constants.ButtonBoardConstants;
import frc.robot.ReefPosition.ReefBranch;
import frc.robot.ReefPosition.ReefLevel;

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
                
                Elastic.sendNotification(new Notification(NotificationLevel.WARNING, "ButtonBoard Potentially Shorted", "The button board may be potentially shorted, replug it in and if the problem persists check the wires."));
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
