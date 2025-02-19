package frc.robot.BreakerLib.util;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
/**
 * Similar to SlewRateLimiter but the rate limits can be mutated after construction.
 * And also timestamps are only reset in the calculate() method.
 */
public class BreakerSlewRateLimiter {
    private double positiveRateLimit;
    private double negativeRateLimit;
    private double prevTime;
    private double prevVal;

    public BreakerSlewRateLimiter(double positiveRateLimit, double negativeRateLimit, double initialValue) {
        this.positiveRateLimit = positiveRateLimit;
        this.negativeRateLimit = negativeRateLimit;
        this.prevVal = initialValue;
        prevTime = MathSharedStore.getTimestamp();
    }

    public BreakerSlewRateLimiter(double rateLimit) {
        this(rateLimit, -rateLimit, 0);
    }

    public double calculate(double input) {
        double currentTime = MathSharedStore.getTimestamp();
        double elapsedTime = currentTime - prevTime;
        prevVal +=
            MathUtil.clamp(
                input - prevVal,
                negativeRateLimit * elapsedTime,
                positiveRateLimit * elapsedTime);
        prevTime = currentTime;
        return prevVal;
    }

    public void resetTimestamp() {
        prevTime = MathSharedStore.getTimestamp();
    }

    public void setRateLimit(double limit) {
        setRateLimit(limit, -limit);
    }

    public void setRateLimit(double positiveLimit, double negativeLimit) {
        this.positiveRateLimit = positiveLimit;
        this.negativeRateLimit = negativeLimit;
    }

    /**
     * Set the 'last' value without changing the timestamp.
     * 
     */
    public void setLastValue(double value) {
        prevVal = value;
    }

    public double getLastValue() {
        return prevVal;
    }
}
