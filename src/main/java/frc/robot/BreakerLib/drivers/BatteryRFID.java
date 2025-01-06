// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.drivers;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.BreakerLib.util.logging.LoggedAlert;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Optional;
/** Add your docs here. */
public class BatteryRFID extends SubsystemBase {
    private Optional<BatteryInfo> batteryInfo = Optional.empty();
    private boolean batteryIdChecked = false;
    private boolean batteryIdWritten = false;
    private LoggedAlert sameBatteryAlert = new LoggedAlert("LoggedBattery/SameBattery","Same Battery As Previous Match!", AlertType.kWarning);
    private LoggedAlert nonCompBatteryAlert = new LoggedAlert("LoggedBattery/NonCompetitionBattery", "A Test Battery Is Being Used In Competition!", AlertType.kWarning);
    private LoggedAlert readFailAlert = new LoggedAlert("LoggedBattery/BatteryReadFailure","Failed To Read Battery RFID!", AlertType.kError);
    private NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private NetworkTable batteryDatatable = inst.getTable("Battery");
    private BooleanSubscriber readBattPub = batteryDatatable.getBooleanTopic("has_tag").subscribe(false);
    private IntegerSubscriber tagIDPub = batteryDatatable.getIntegerTopic("tag_id").subscribe(0);
    private IntegerSubscriber battIDPub = batteryDatatable.getIntegerTopic("data/id").subscribe(0);
    private IntegerSubscriber yearPub = batteryDatatable.getIntegerTopic("data/year").subscribe(0000);
    private DoubleSubscriber capPub = batteryDatatable.getDoubleTopic("data/cap").subscribe(0.0);
    private StringSubscriber typePub = batteryDatatable.getStringTopic("data/type").subscribe("NONE");
    private StringSubscriber mfgPub = batteryDatatable.getStringTopic("data/mfg").subscribe("NONE");
    private final String HISTORY_FILE_PATH;
    private final BatteryInfo DEFAULT_BATTERY_INFO = new BatteryInfo(-1, -1, -1, 0.0, BatteryType.UNKNOWN, "NONE");

    public BatteryRFID(String historyFilePath) {
        HISTORY_FILE_PATH = historyFilePath;
    }

    public Optional<BatteryInfo> getBatteryName() {
        return batteryInfo;
    }

    private BatteryType strToBatteryType(String str) {
        switch (str) {
            case "COMP":
                return BatteryType.COMP;
            case "TEST":
                return BatteryType.TEST;
            default:
                return BatteryType.UNKNOWN;
        }
    }

    @Override
    public void periodic() {
        if (RobotState.isEnabled()) {
            readFailAlert.set(batteryInfo.isEmpty());
        }

        if (batteryInfo.isEmpty()) {
            boolean hasTag = readBattPub.get();
            int batID = (int) battIDPub.get();
            String typeStr = typePub.get();
            boolean isValid = hasTag && (batID > 0) &&  (!typeStr.equals("NONE"));
            if (isValid) {
                batteryInfo = Optional.of(new BatteryInfo(tagIDPub.get(), batID, (int) yearPub.get(), capPub.get(), strToBatteryType(typeStr), mfgPub.get()));
            }
        }

        if (batteryInfo.isPresent()) {
            batteryInfo.get().log();
        } else {
            DEFAULT_BATTERY_INFO.log();
        }

        BreakerLog.log("LoggedBattery/HasRead", batteryInfo.isPresent());

            
        if (Robot.isReal() && batteryInfo.isPresent()) {
            // Check for battery alert
            if (!batteryIdChecked) {
                batteryIdChecked = true;
                File file = new File(HISTORY_FILE_PATH);
                if (file.exists()) {
                    // Read previous battery name
                    int previousBatteryId = -1;
                    try {
                        previousBatteryId =
                            Integer.parseInt(new String(Files.readAllBytes(Paths.get(HISTORY_FILE_PATH)), StandardCharsets.UTF_8));
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                    if (previousBatteryId == batteryInfo.get().batteryID()) {
                        // Same battery, set alert
                        sameBatteryAlert.set(true);
                        DriverStation.reportWarning("BATTERY NOT CHANGED SINCE LAST MATCH", false);
                    } else {
                        // New battery, delete file
                        sameBatteryAlert.set(false);
                        file.delete();
                    }
                }
            }

            // Write battery name if connected to FMS
            if (DriverStation.isFMSAttached()) {
                if (!batteryIdWritten) {
                    batteryIdWritten = true;
                    try {
                        FileWriter fileWriter = new FileWriter(HISTORY_FILE_PATH);
                        fileWriter.write(Integer.valueOf(batteryInfo.orElse(DEFAULT_BATTERY_INFO).batteryID()).toString());
                        fileWriter.close();
                    } catch (IOException e) {
                        e.printStackTrace();
                        
                    }
                }
                if (batteryInfo.isPresent()) {
                    if (batteryInfo.get().type() != BatteryType.COMP) {
                        nonCompBatteryAlert.set(true);
                        DriverStation.reportWarning("NON-COMP BATTERY IN ROBOT", false);
                    } else {
                        nonCompBatteryAlert.set(false);
                    }
                }
            }
            
        }
    }

    public static record BatteryInfo(long tagID, int batteryID, int purchaseYear, double chargeAh, BatteryType type, String manufacturer) {
        public void log() {
            BreakerLog.log("LoggedBattery/BatteryInfo/TagID", tagID);
            BreakerLog.log("LoggedBattery/BatteryInfo/BatteryID", batteryID);
            BreakerLog.log("LoggedBattery/BatteryInfo/PurchaseYear", purchaseYear);
            BreakerLog.log("LoggedBattery/BatteryInfo/ChargeAh", chargeAh);
            BreakerLog.log("LoggedBattery/BatteryInfo/BatteryType", type);
            BreakerLog.log("LoggedBattery/BatteryInfo/Manufacturer", manufacturer);
        }
    }

    public static enum BatteryType {
        COMP,
        TEST,
        UNKNOWN
    }
}
