package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
public class Indexer {
    private TalonSRX indexer;


    public static enum IndexerState {
        INDEXING,
        REVERSE,
        NEUTRAL
    }

}
