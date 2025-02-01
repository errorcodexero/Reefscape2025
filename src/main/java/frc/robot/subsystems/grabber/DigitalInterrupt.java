package frc.robot.subsystems.grabber;

import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;

public class DigitalInterrupt {
    private AtomicBoolean rising_seen_;
    private AtomicBoolean falling_seen_;
    private DigitalInput sensor_;
    private AsynchronousInterrupt interrupt_;

    public DigitalInterrupt(int kChannel) {
        sensor_ = new DigitalInput(kChannel);

        rising_seen_ = new AtomicBoolean();
        falling_seen_ = new AtomicBoolean();

        interrupt_ = new AsynchronousInterrupt(sensor_, (rising, falling) -> {interruptHandler(rising, falling);});

        interrupt_.enable();
    }

    private void interruptHandler(boolean rising, boolean falling) {
        if (rising) {
            if (RobotBase.isReal()) {
                rising_seen_.set(true);
            }
            else {
                falling_seen_.set(true);
            }
        }

        if (falling) {
            if (RobotBase.isReal()) {
                falling_seen_.set(true);
            }
            else {
                rising_seen_.set(true);
            }
        }
    }

    public boolean getRising() {
        return rising_seen_.get();
    }

    public void setRising(boolean b) {
        rising_seen_.set(b);
    }

    public boolean getFalling() {
        return falling_seen_.get();
    }

    public void setFalling(boolean b) {
        falling_seen_.set(b);
    }

    public boolean getSensor() {
        return sensor_.get();
    }
}
