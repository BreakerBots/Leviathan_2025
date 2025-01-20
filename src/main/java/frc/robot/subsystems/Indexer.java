package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {
    private TalonSRX indexer;
    private IndexerState currentState = IndexerState.NEUTRAL;

    public Indexer() {
        indexer = new TalonSRX(IndexerConstants.kIndexerMotorID);
    }

    private void set(IndexerState state) {
        currentState = state;
    }

    public Command setState(IndexerState state) {
        return Commands.runOnce(() -> set(state), this);
    }

    public static enum IndexerState {
        INDEXING(-1.0),
        REVERSE(1.0),
        NEUTRAL(0.0);

        private double dutyCycleOut;
        private IndexerState(double dutyCycleOut) {
            this.dutyCycleOut = dutyCycleOut;
        }

        public double getDutyCycleOut() {
            return dutyCycleOut;
        }
    }

    @Override
    public void periodic() {
        if (RobotState.isDisabled()) {
            setState(IndexerState.NEUTRAL);
        }

        double cycle = currentState.getDutyCycleOut();

        BreakerLog.log("Indexer/State", currentState);
        BreakerLog.log("Indexer/DutyCycle", cycle);
        BreakerLog.log("Indexer/Motor/OutputPercent", indexer.getMotorOutputPercent());
        BreakerLog.log("Indexer/Motor/StatorCurrent", indexer.getStatorCurrent());
        BreakerLog.log("Indexer/Motor/SupplyCurrent", indexer.getSupplyCurrent());

        indexer.set(TalonSRXControlMode.PercentOutput, cycle);
    }

}
