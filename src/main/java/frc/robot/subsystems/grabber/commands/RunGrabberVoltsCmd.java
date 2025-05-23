package frc.robot.subsystems.grabber.commands;

import static edu.wpi.first.units.Units.Volts;

import org.xerosw.util.XeroTimer;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.grabber.GrabberSubsystem;

public class RunGrabberVoltsCmd extends Command {
    private GrabberSubsystem g_ ;
    private XeroTimer timer_ ;
    private boolean done_ ;
    private Voltage volts_ ;

    public RunGrabberVoltsCmd(GrabberSubsystem grabber, Time duration) {
        this(grabber, duration, Volts.of(12.0)) ;
    }

    public RunGrabberVoltsCmd(GrabberSubsystem grabber, Time duration, Voltage volts) {
        g_ = grabber ;
        addRequirements(g_) ;
        timer_ = new XeroTimer(duration) ;
        volts_ = volts ;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        done_ = false ;
        g_.setGrabberMotorVoltage(volts_) ;
        timer_.start() ;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (timer_.isExpired()) {
            g_.setGrabberMotorVoltage(Volts.zero()) ;
            done_ = true ;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            g_.setGrabberMotorVoltage(Volts.zero()) ;
            done_ = true ;
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return done_ ;
    }
}
