package frc.robot.BreakerLib.util.logging;

import edu.wpi.first.wpilibj.Alert;

public class LoggedAlert extends Alert {
    private String logKey;
    
    /**
   * Creates a new alert in the default group - "Alerts". If this is the first to be instantiated,
   * the appropriate entries will be added to NetworkTables.
   *
   * @param text Text to be displayed when the alert is active.
   * @param type Alert urgency level.
   */
  public LoggedAlert(String logKey, String text, AlertType type) {
    super(text, type);
    this.logKey = logKey;
    log();
  }

  /**
   * Creates a new alert. If this is the first to be instantiated in its group, the appropriate
   * entries will be added to NetworkTables.
   *
   * @param group Group identifier, used as the entry name in NetworkTables.
   * @param text Text to be displayed when the alert is active.
   * @param type Alert urgency level.
   */
  @SuppressWarnings("this-escape")
  public LoggedAlert(String logKey, String group, String text, AlertType type) {
    super(group, text, type);
    this.logKey = logKey;
    log();
  }

  @Override
  public void set(boolean active) {
      super.set(active);
      log();
  }

  @Override
  public void setText(String text) {
      super.setText(text);
      log();
  }

  public void log() {
    BreakerLog.log(logKey, this);
  }

  

}
