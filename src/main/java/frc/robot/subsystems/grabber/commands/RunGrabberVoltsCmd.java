package frc.robot.subsystems.grabber.commands;

import org.xerosw.util.XeroTimer;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.grabber.GrabberSubsystem;

public class RunGrabberVoltsCmd extends Command {
    private GrabberSubsystem g_ ;
    private XeroTimer timer_ ;
    private boolean done_ ;

    public RunGrabberVoltsCmd(GrabberSubsystem grabber, Time duration) {
        g_ = grabber ;
        addRequirements(g_) ;
        timer_ = new XeroTimer(duration) ;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        done_ = false ;
        g_.setGrabberMotorVoltage(12.0) ;
        timer_.start() ;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (timer_.isExpired()) {
            g_.setGrabberMotorVoltage(0.0) ;
            done_ = true ;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            g_.setGrabberMotorVoltage(0.0) ;
            done_ = true ;
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return done_ ;
    }
}
