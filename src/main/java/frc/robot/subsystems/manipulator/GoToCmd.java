package frc.robot.subsystems.manipulator;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;

public class GoToCmd extends Command {

    private enum State {
        MoveArmToRaise,
        MoveElevator,
        MoveArm,
        Direct,
        Done
    }

    private ManipulatorSubsystem m_ ;
    private Distance targetElevPos_ ;
    private Angle targetArmPos_ ;
    private State state_ ;
    private boolean direct_ ;

    public GoToCmd(ManipulatorSubsystem m, Distance targetElevPos, Angle targetArmPos, boolean direct) {
        m_ = m ;
        targetElevPos_ = targetElevPos ;
        targetArmPos_ = targetArmPos ;
        direct_ = direct ;
    }

    public GoToCmd(ManipulatorSubsystem m, Distance targetElevPos, Angle targetArmPos) {
        this(m, targetElevPos, targetArmPos, false) ;
    }

    @Override
    public void initialize() {
        if (direct_) {
            m_.setElevatorPosition(targetElevPos_);
            m_.setArmPosition(targetArmPos_);
            state_ = State.Direct ;
        }
        else {
            if (m_.getArmPosition().isNear(ManipulatorConstants.Arm.Positions.kRaiseAngle, ManipulatorConstants.Arm.kPosTolerance)) {
                m_.setElevatorPosition(targetElevPos_);
                state_ = State.MoveElevator ;
            }
            else {
                m_.setArmPosition(ManipulatorConstants.Arm.Positions.kRaiseAngle);
                state_ = State.MoveArmToRaise ;
            }
        }
    }

    @Override
    public void execute() {
        switch(state_) {
            case MoveArmToRaise:
                if (m_.isArmAtTarget()) {
                    m_.setElevatorPosition(targetElevPos_);
                    state_ = State.MoveElevator ;
                }
                break ;

            case MoveElevator:
                if (m_.isElevAtTarget()) {
                    m_.setArmPosition(targetArmPos_);
                    state_ = State.MoveArm ;
                }
                break ;

            case MoveArm:
                if (m_.isArmAtTarget()) {
                    state_ = State.Done ;
                }
                break ;

            case Direct:
                if (m_.isArmAtTarget() && m_.isElevAtTarget()) {
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

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            m_.setArmPosition(m_.getArmPosition()) ;
            m_.setElevatorPosition(m_.getElevatorPosition()) ;
        }
    }
}
