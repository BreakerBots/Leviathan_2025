package frc.robot.BreakerLib.util.math.functions;
import java.util.function.DoubleUnaryOperator;

import frc.robot.BreakerLib.util.math.BreakerMath;
public class BreakerLinearizedConstrainedExponential implements DoubleUnaryOperator {
    /** https://www.desmos.com/calculator/ubkzzw4vrr */
    private double lin, exp;
    private boolean preserveSign;
    public BreakerLinearizedConstrainedExponential(double linearity, double baseExponent, boolean preserveSign) {
        lin = linearity;
        exp = baseExponent;
        this.preserveSign = preserveSign;
    }

    /** https://www.desmos.com/calculator/ubkzzw4vrr */
    public BreakerLinearizedConstrainedExponential() {
        this(0.3, 3.0, true);
    }

    @Override
    public double applyAsDouble(double operand) {
        return BreakerMath.linearizedConstrainedExponential(operand, lin, exp, preserveSign);
    }
}