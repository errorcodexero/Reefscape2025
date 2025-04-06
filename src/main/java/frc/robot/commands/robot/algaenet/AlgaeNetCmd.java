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
import frc.robot.subsystems.manipulator.commands.GoToCmdDirect;

public class AlgaeNetCmd extends Command {
    private enum State {
        GoingUp,
        Shooting,
        Dropping,
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
        timer_ = new XeroTimer(Milliseconds.of(500)) ;
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
                b_.setGp(GamePiece.NONE);
                if (timer_.isExpired()) {
                    m_.setElevatorTarget(ManipulatorConstants.Elevator.Positions.kStow) ;
                    g_.setGrabberMotorVoltage(Volts.zero()) ;
                    state_ = State.Dropping ;
                }
                break ;

            case Dropping:
                if (m_.getElevatorPosition().lte(ManipulatorConstants.Elevator.Positions.kShootRotateArm)) {
                    m_.setArmTarget(ManipulatorConstants.Arm.Positions.kStow);
                }

                if (m_.isElevAtTarget()) {
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
