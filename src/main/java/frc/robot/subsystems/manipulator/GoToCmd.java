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
    private Distance elev_tolerance_ ;
    private Angle arm_tolerance_ ;

    public GoToCmd(ManipulatorSubsystem m, Distance targetElevPos, Distance elevtol, Angle targetArmPos, Angle armtol, boolean direct) {
        addRequirements(m) ;

        m_ = m ;
        targetElevPos_ = targetElevPos ;
        targetArmPos_ = targetArmPos ;
        direct_ = direct ;
        arm_tolerance_ = armtol ;
        elev_tolerance_ = elevtol ;
    }

    public GoToCmd(ManipulatorSubsystem m, Distance targetElevPos, Distance elevtol, Angle targetArmPos, Angle armtol) {
        this(m, targetElevPos, elevtol, targetArmPos, armtol, false) ;
    }

    public GoToCmd(ManipulatorSubsystem m, Distance targetElevPos, Angle targetArmPos, boolean direct) {
        this(m, targetElevPos, ManipulatorConstants.Elevator.kPosTolerance, targetArmPos, ManipulatorConstants.Arm.kPosTolerance, direct) ;
    }

    public GoToCmd(ManipulatorSubsystem m, Distance targetElevPos, Angle targetArmPos) {
        this(m, targetElevPos, ManipulatorConstants.Elevator.kPosTolerance, targetArmPos, ManipulatorConstants.Arm.kPosTolerance, false) ;
    }

    @Override
    public void initialize() {
        if (direct_) {
            m_.setElevatorTarget(targetElevPos_, elev_tolerance_);
            m_.setArmTarget(targetArmPos_, arm_tolerance_);
            state_ = State.Direct ;
        }
        else {
            if (m_.getArmPosition().isNear(ManipulatorConstants.Arm.Positions.kRaiseAngle, arm_tolerance_)) {
                m_.setElevatorTarget(targetElevPos_, elev_tolerance_);
                state_ = State.MoveElevator ;
            }
            else {
                m_.setArmTarget(ManipulatorConstants.Arm.Positions.kRaiseAngle, ManipulatorConstants.Arm.kPosTolerance);
                state_ = State.MoveArmToRaise ;
            }
        }
    }

    @Override
    public void execute() {
        switch(state_) {
            case MoveArmToRaise:
                if (m_.isArmAtTarget()) {
                    m_.setElevatorTarget(targetElevPos_, elev_tolerance_);
                    state_ = State.MoveElevator ;
                }
                break ;

            case MoveElevator:
                if (m_.isElevAtTarget()) {
                    m_.setArmTarget(targetArmPos_, arm_tolerance_);
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
            m_.setArmTarget(m_.getArmPosition(), arm_tolerance_) ;
            m_.setElevatorTarget(m_.getElevatorPosition(), elev_tolerance_) ;
        }
    }
}
