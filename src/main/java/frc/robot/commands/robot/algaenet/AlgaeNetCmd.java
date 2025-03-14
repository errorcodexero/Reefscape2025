package frc.robot.commands.robot.algaenet;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;
import org.xerosw.util.XeroTimer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.brain.GamePiece;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.manipulator.commands.GoToCmd;
import frc.robot.subsystems.manipulator.commands.GoToCmdDirect;

public class AlgaeNetCmd extends Command {
    private enum State {
        GoingUp,
        Shooting,
        Down,
        Done
    } ;

    private ManipulatorSubsystem m_ ;
    private GrabberSubsystem g_ ;
    private BrainSubsystem b_ ;
    private Command goto_ ;
    private Command eject_ ;
    private XeroTimer timer_ ;
    private State state_ ;

    public AlgaeNetCmd(BrainSubsystem b, ManipulatorSubsystem m, GrabberSubsystem g) {
        m_ = m ;
        g_ = g ;
        b_ = b ;

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
        Logger.recordOutput("AlgaeNetCmd/state", state_.toString()) ;
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
                    g_.setGrabberMotorVoltage(Volts.zero()) ;
                    goto_ = new GoToCmd(m_, ManipulatorConstants.Elevator.Positions.kStow, ManipulatorConstants.Arm.Positions.kStow) ;
                    goto_.initialize() ;
                    state_ = State.Down ;
                }
                break ;

            case Down:
                goto_.execute() ;
                if (goto_.isFinished()) {
                    b_.setGp(GamePiece.NONE) ;
                    b_.clearRobotActions();
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
