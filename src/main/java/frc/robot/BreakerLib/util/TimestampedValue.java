package frc.robot.BreakerLib.util;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;

public class TimestampedValue<T> {
    private T value;
    private Time timestamp;
    public TimestampedValue(T value, Time timestamp) {
        this.value = value;
        this.timestamp = timestamp;
    }

    public TimestampedValue(T value, double timestampSeconds) {
        this(value, Units.Seconds.of(timestampSeconds));
    }

    public Time getTimestamp() {
        return timestamp;
    }

    public T getValue() {
        return value;
    }
}
