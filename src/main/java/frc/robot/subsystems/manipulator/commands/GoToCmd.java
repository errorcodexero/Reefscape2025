package frc.robot.subsystems.manipulator.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class GoToCmd extends Command {

    private enum State {
        MoveArmToRaise,
        MoveElevator,
        MoveArm,
        Done
    }

    private ManipulatorSubsystem m_ ;
    private Distance targetElevPos_ ;
    private Angle targetArmPos_ ;
    private State state_ ;

    public GoToCmd(ManipulatorSubsystem m, Distance targetElevPos, Angle targetArmPos) {
        m_ = m ;
        targetElevPos_ = targetElevPos ;
        targetArmPos_ = targetArmPos ;
    }

    @Override
    public void initialize() {
        if (m_.getArmPosition().isNear(ManipulatorConstants.Arm.Positions.kRaiseAngle, ManipulatorConstants.Arm.kPosTolerance)) {
            m_.setElevatorTarget(targetElevPos_);
            state_ = State.MoveElevator ;
        }
        else {
            m_.setArmTarget(ManipulatorConstants.Arm.Positions.kRaiseAngle);
            state_ = State.MoveArmToRaise ;
        }
    }

    @Override
    public void execute() {
        switch(state_) {
            case MoveArmToRaise:
                if (m_.isArmAtTarget()) {
                    m_.setElevatorTarget(targetElevPos_);
                    state_ = State.MoveElevator ;
                }
                break ;

            case MoveElevator:
                if (m_.isElevAtTarget()) {
                    m_.setArmTarget(targetArmPos_);
                    state_ = State.MoveArm ;
                }
                break ;

            case MoveArm:
                if (m_.isArmAtTarget()) {
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
            m_.setArmTarget(m_.getArmPosition()) ;
            m_.setElevatorTarget(m_.getElevatorPosition()) ;
        }
    }
}
