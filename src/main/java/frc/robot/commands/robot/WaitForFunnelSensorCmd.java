package frc.robot.commands.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class WaitForFunnelSensorCmd extends Command {
    private ManipulatorSubsystem m_ ;
    private boolean seen_ ;

    public WaitForFunnelSensorCmd(ManipulatorSubsystem sys) {
        m_ = sys ;
    }

    @Override
    public void initialize() {
        seen_ = false ;
    }

    @Override
    public void execute() {
        if (m_.sawFunnelCoral()) {
            seen_ = true ;
        }
    }

    @Override
    public boolean isFinished() {
        return seen_ ;
    }
}
