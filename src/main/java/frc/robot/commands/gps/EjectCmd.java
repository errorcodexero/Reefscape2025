package frc.robot.commands.gps;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.manipulator.GoToCmd;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class EjectCmd extends Command {

    private BrainSubsystem brain_ ;
    private ManipulatorSubsystem m_ ;
    private GrabberSubsystem g_ ;

    private Command eject_cmd_ ;
    private Command stow_cmd_ ;

    public EjectCmd(BrainSubsystem b, ManipulatorSubsystem m, GrabberSubsystem g) {
        brain_ = b ;
        m_ = m;
        g_ = g ;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        brain_.lock() ;
        brain_.clearRobotActions();

        switch(brain_.gp()) {
            case CORAL:
                eject_cmd_ = new EjectCoralCmd(m_, g_) ;
                break ;
            case ALGAE_HIGH:
            case ALGAE_LOW:
                eject_cmd_ = new EjectAlgaeCmd(m_, g_) ;
                break ;
            default:
                eject_cmd_ = null ;
                break ;
        }

        if (eject_cmd_ != null) {
            eject_cmd_.schedule();
        }
        else {
            stow_cmd_ = new GoToCmd(m_, ManipulatorConstants.Positions.kStowedHeight, ManipulatorConstants.Positions.kStowedAngle) ;
            stow_cmd_.schedule();
        }
    }

    @Override
    public void execute() {
        if (eject_cmd_ != null && eject_cmd_.isFinished()) {
            stow_cmd_ = new GoToCmd(m_, ManipulatorConstants.Positions.kStowedHeight, ManipulatorConstants.Positions.kStowedAngle) ;
            stow_cmd_.schedule();
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return stow_cmd_ != null && stow_cmd_.isFinished() ;
    } 

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            if (eject_cmd_ != null) {
                eject_cmd_.cancel();
            }

            if (stow_cmd_ != null) {
                stow_cmd_.cancel();
            }
        }

        brain_.unlock() ;
    }
}
