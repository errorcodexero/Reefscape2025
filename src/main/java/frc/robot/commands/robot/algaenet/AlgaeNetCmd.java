package frc.robot.commands.robot.algaenet;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Volts;

import org.xerosw.util.XeroTimer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.manipulator.commands.GoToCmdDirect;

public class AlgaeNetCmd extends Command {
    private enum State {
        GoingUp,
        Shooting,
        Done
    } ;

    private ManipulatorSubsystem m_ ;
    private GrabberSubsystem g_ ;
    private Command goto_ ;
    private Command eject_ ;
    private XeroTimer timer_ ;
    private State state_ ;

    public AlgaeNetCmd(ManipulatorSubsystem m, GrabberSubsystem g) {
        m_ = m ;
        g_ = g ;

        addRequirements(m_, g_) ;

         eject_ = g_.setVoltageCommand(Volts.of(12.0)) ;
        timer_ = new XeroTimer(Milliseconds.of(1000)) ;
    }

    @Override
    public void initialize() {
        goto_ = new GoToCmdDirect(m_, ManipulatorConstants.Elevator.Positions.kShootAlgae, ManipulatorConstants.Arm.Positions.kShootAlgae) ;
        goto_.initialize();
        state_ = State.GoingUp ;
    }

    @Override
    public void execute() {
        switch(state_) {
            case GoingUp:
                goto_.execute();
                if (m_.getElevatorPosition().gte(ManipulatorConstants.Elevator.Positions.kShootAlgaeEject)) {
                    state_ = State.Shooting ;
                    eject_.initialize() ;
                    timer_.start() ;
                }
                break ;

            case Shooting:
                eject_.execute() ;
                if (timer_.isExpired()) {
                    state_ = State.Done ;
                }
                break ;

            case Done:
                break ;
        }
    }

    @Override
    public boolean isFinished() {
        return state_ == State.Done ;
    }
    
}
