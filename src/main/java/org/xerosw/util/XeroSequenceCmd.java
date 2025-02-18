package org.xerosw.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public abstract class XeroSequenceCmd extends Command {
    private SequentialCommandGroup sequence_;
    private boolean complete_ ;

    public XeroSequenceCmd() {
    }

    public boolean isComplete() {
        return complete_ ;
    }

    public abstract void initSequence(SequentialCommandGroup sequence) ;

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        complete_ = false ;
        sequence_ = new SequentialCommandGroup() ;
        initSequence(sequence_) ;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (sequence_ != null) {
            sequence_.execute() ;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted && sequence_ != null) {
            sequence_.cancel();
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        boolean ret = false ;
        if (sequence_ != null && sequence_.isFinished()) {
            ret = true ;
            complete_ = true ;
        }
        return ret;
    }
    
}
