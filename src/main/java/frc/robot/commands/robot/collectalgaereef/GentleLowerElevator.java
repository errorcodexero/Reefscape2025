package frc.robot.commands.robot.collectalgaereef;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class GentleLowerElevator extends Command {
    private ManipulatorSubsystem m_ ;
    private Distance target_ ;
    private boolean done_ ;
    private static final Voltage volts_ = Volts.of(-4.0) ;

    public GentleLowerElevator(ManipulatorSubsystem m, Distance d) {
        m_ = m ;
        target_ = d ;
    }

    @Override
    public void initialize() {
        done_ = false ;
        m_.setElevatorVoltage(volts_);
    }

    @Override
    public void execute() {
        if (m_.getElevatorPosition().lte(target_)) {
            done_ = true ;
            m_.setElevatorVoltage(Volts.zero());
        }
    }

    @Override
    public boolean isFinished() {
        return done_ ;
    }

    @Override
    public void end(boolean interrupted) {
        m_.setElevatorVoltage(Volts.of(0.0));
    }
}
