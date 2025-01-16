package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
public class Indexer {
    private TalonSRX indexer;

    public Indexer() {
        
    }


    public static enum IndexerState {
        INDEXING(-1.0),
        REVERSE(1.0),
        NEUTRAL(0.0);

        private double ductyCycleOut;
        private IndexerState(double dutyCycleOut) {
            this.ductyCycleOut = dutyCycleOut;
        }

        public double getDuctyCycleOut() {
            return ductyCycleOut;
        }
    }

}
