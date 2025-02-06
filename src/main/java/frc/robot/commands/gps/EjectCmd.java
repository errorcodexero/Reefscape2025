package frc.robot.commands.gps;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorGotoCmd;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class EjectCmd extends Command {

    private ManipulatorSubsystem m_ ;
    private GrabberSubsystem g_ ;
    private ManipulatorSubsystem manipulator_ ;
    private GrabberSubsystem grabber_ ;

    private Command goto_ ;
    private Command eject_gp_ ;

    public EjectCmd(ManipulatorSubsystem m, GrabberSubsystem g) {
        manipulator_ = m;
        grabber_ = g;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.getRobotContainer().getExecutor().lock() ;
        RobotContainer.getRobotContainer().getExecutor().clearRobotActions();

        switch(grabber_.gp()) {
            case Coral:
                eject_gp_ = new EjectCoralCmd(m_, g_) ;
                goto_ = null ;
                eject_gp_.schedule();
                break ;
            case AlgaeHigh:
            case AlgaeLow:
                eject_gp_ = new EjectAlgaeCmd(m_, g_) ;
                goto_ = null ;
                eject_gp_.schedule();
                break ;
            default:
                eject_gp_ = null ;
                goto_ = new ManipulatorGotoCmd(manipulator_, ManipulatorConstants.Positions.kStowedHeight, ManipulatorConstants.Positions.kStowedAngle) ;
                goto_.schedule();
                break ;
        }
    }

    @Override
    public void execute() {
        if (eject_gp_ != null) {
            if (eject_gp_.isFinished()) {
                eject_gp_ = null ;
                goto_ = new ManipulatorGotoCmd(manipulator_, ManipulatorConstants.Positions.kStowedHeight, ManipulatorConstants.Positions.kStowedAngle) ;
                goto_.schedule();
            }
        }
        else if (goto_ != null) {
            if (goto_.isFinished()) {
                goto_ = null ;
            }
        }
        else {
            RobotContainer.getRobotContainer().getExecutor().unlock() ;
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return eject_gp_ == null && goto_ == null ;
    }    
}
