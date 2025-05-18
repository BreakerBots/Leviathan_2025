package frc.robot.subsystems;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.BreakerLib.util.logging.BreakerLog;

public class Indexer extends SubsystemBase {
    private TalonFX indexer;
    private IndexerState currentState = IndexerState.NEUTRAL;
    private DutyCycleOut dutyCycleOut;

    public Indexer() {
        indexer = new TalonFX(IndexerConstants.kIndexerMotorID, SuperstructureConstants.kSuperstructureCANBus);
        dutyCycleOut = new DutyCycleOut(0);
    }

    private void set(IndexerState state) {
        currentState = state;
    }

    public Command setState(IndexerState state) {
        return Commands.runOnce(() -> set(state), this);
    }

    public static enum IndexerState {
        INDEXING(-0.8),
        REVERSE(0.8),
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
            set(IndexerState.NEUTRAL);
        }

        double cycle = currentState.getDutyCycleOut();

        
        
        

        indexer.setControl(dutyCycleOut.withOutput(cycle));
    }

}
