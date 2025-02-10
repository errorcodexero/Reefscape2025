package org.xerosw.util ;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;

public class XeroTimer {
    private boolean running_ ;
    private double duration_ ;
    private double endtime_ ;
    private double start_ ;

    public XeroTimer(Time duration) {
        duration_ = duration.in(Seconds) ;
        running_ = false ;
        endtime_ = 0.0 ;        
    }

    public double getDuration() {
        return duration_ ;
    }

    public void setDuration(double dur) {
        duration_ = dur ;
    }

    public double elapsed() {
        return Timer.getFPGATimestamp() - start_ ;
    }

    public void start() {
        running_ = true ;
        start_ = Timer.getFPGATimestamp() ;
        endtime_ = start_ + duration_ ;
    }

    public boolean isRunning() {
        return running_ ;
    }

    public boolean isExpired() {
        boolean ret = false ;

        if (running_ == false)
            return true ;

        if (running_ && Timer.getFPGATimestamp() > endtime_) {
            running_ = false ;
            ret = true ;
        }

        return ret ;
    }
}