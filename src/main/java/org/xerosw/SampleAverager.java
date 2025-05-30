package org.xerosw;

public class SampleAverager {
    private double [] last_pos_ ;
    private int count_ ;
    private boolean complete_ ;

    public SampleAverager(int count) {
        count_ = 0 ;
        complete_ = false ;
        last_pos_ = new double[count] ;
    }

    public void addSample(double s) {
        last_pos_[count_++] = s ;
        if (count_ == last_pos_.length) {
            count_ = 0 ;
            complete_ = true ;
        }
    }

    public void reset() {
        count_ = 0 ;
        complete_ = false ;
    }

    public boolean isComplete() {
        return complete_ ;
    }

    public double maxDeviationFromAverage() {
        double sum = 0.0 ;
        for (int i = 0 ; i < last_pos_.length ; i++) {
            sum += last_pos_[i] ;
        }
        double avg = sum / last_pos_.length ;

        double max = 0.0 ;
        for (int i = 0 ; i < last_pos_.length ; i++) {
            double diff = Math.abs(last_pos_[i] - avg) ;
            if (diff > max)
                max = diff ;
        }

        return max ;
    }
}
