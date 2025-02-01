package org.xerosw.util;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;

public class DigitalInterrupt {
    private DigitalInput input_ ;
    private AsynchronousInterrupt interrupt_ ;
    private AtomicBoolean rising_seen_ ;
    private AtomicBoolean falling_seen_ ;
    private AtomicInteger count_at_interrupt_ ;
    private Supplier<Integer> count_supplier_ ;

    public DigitalInterrupt(int channel, boolean risingEdge, boolean fallingEdge) {
        this(new DigitalInput(channel), risingEdge, fallingEdge) ;
    }
    
    public DigitalInterrupt(DigitalInput input, boolean risingEdge, boolean fallingEdge) {
        input_ = input ;
        interrupt_ = new AsynchronousInterrupt(input_, (rising, falling) -> { handler(rising, falling) ; }) ;
        interrupt_.setInterruptEdges(risingEdge, fallingEdge) ;
        rising_seen_ = new AtomicBoolean(false) ;
        falling_seen_ = new AtomicBoolean(false) ;
        count_at_interrupt_ = null ;
    }

    public boolean hasCountValue() {
        return count_at_interrupt_ != null ;
    }

    public DigitalInput getInput() {
        return input_ ;
    }

    public void enable() {
        interrupt_.enable() ;
    }

    public void disable() {
        interrupt_.disable() ;
    }

    public boolean risingEdge() {
        return rising_seen_.getAndSet(false) ;
    }

    public boolean fallingEdge() {
        return falling_seen_.getAndSet(false) ;
    }
    
    public int getCount() {
        return count_supplier_.get() ;
    }

    public void setCountSupplier(Supplier<Integer> supplier) {
        count_supplier_ = supplier ;
        count_at_interrupt_ = new AtomicInteger(supplier.get()) ;
    }

    private void handler(boolean rising, boolean falling) {
        if (rising) {
            rising_seen_.set(true) ;
        }
        if (falling) {
            falling_seen_.set(true) ;
        }

        if (count_at_interrupt_ != null) {
            count_at_interrupt_.set(count_supplier_.get()) ;
        }
    }
}
