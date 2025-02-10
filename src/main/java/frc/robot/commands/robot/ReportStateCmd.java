package frc.robot.commands.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;

public class ReportStateCmd extends Command {
    private final String category_ ;
    private final String state_ ;

    public ReportStateCmd(String category, String state) {
        setName("ReportStateCmd");

        category_ = category ;
        state_ = state ;
    }

    @Override
    public void initialize() {
        Logger.recordOutput("state/" + category_, state_) ;
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
