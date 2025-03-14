package frc.robot.BreakerLib.util.commands;

import java.util.concurrent.locks.ReentrantLock;

import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj2.command.Command;

public class AsyncronousAwaitCommand extends Command {
    private AsynchronousInterrupt interrupt;
    private Runnable riseingEdgeCallback;
    private Runnable fallingEdgeCallback;
    private boolean endFlag;
    private ReentrantLock lock;

    public AsyncronousAwaitCommand(DigitalSource source) {
        this(source, () -> {}, () -> {});
    }

    public AsyncronousAwaitCommand(DigitalSource source, Runnable riseingEdgeCallback) {
        this(source, riseingEdgeCallback, () -> {});
    }

    public AsyncronousAwaitCommand(DigitalSource source, Runnable riseingEdgeCallback, Runnable fallingEdgeCallback) {
        lock = new ReentrantLock();
        interrupt = new AsynchronousInterrupt(source, this::internalCallback);
        this.riseingEdgeCallback = riseingEdgeCallback;
        this.fallingEdgeCallback = fallingEdgeCallback;
    }

    private void internalCallback(Boolean isRiseing, Boolean isFalling) {
        try {
            lock.lock();
            if (isRiseing) {
                endFlag = true;
                riseingEdgeCallback.run();
            } else if (isFalling) {
                endFlag = true;
                fallingEdgeCallback.run();
            }
        } finally {
            lock.unlock();
        }
        
        
    }

    @Override
    public void initialize() {
        endFlag = false;
        interrupt.enable();
    }

    @Override
    public void end(boolean interrupted) {
        interrupt.disable();
        interrupt.close();
    }

    @Override
    public boolean isFinished() {
        return endFlag;
    }
}
