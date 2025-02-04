package frc.robot.subsystems.grabber ;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;

public class SetGrabberVelocityCmd extends Command {
    GrabberSubsystem g_ ;
    private Voltage v_ ;

    public SetGrabberVelocityCmd(GrabberSubsystem g, Voltage v) {
        g_ = g ;
        v_ = v ;
    }

    @Override
    public void initialize() {
        g_.setGrabberVelocity(v_) ;
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return true ;
    }

    @Override
    public void end(boolean interrupt) {
    }
}
