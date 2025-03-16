package frc.robot.subsystems.manipulator.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
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
    private Distance elev_pos_tolerance_ ;
    private LinearVelocity elev_vel_tolerance_ ;

    private Angle targetArmPos_ ;
    private Angle arm_pos_tolerance_ ;
    private AngularVelocity arm_vel_tolerance_ ;

    private State state_ ;

    public GoToCmd(ManipulatorSubsystem m, Distance targetElevPos, Angle targetArmPos) {
        this(m, targetElevPos, null, null, targetArmPos, null, null) ;
    }

    public GoToCmd(ManipulatorSubsystem m,                             
                    Distance targetElevPos, Distance elevPosTol, LinearVelocity elevVelTol,
                    Angle targetArmPos, Angle armPosTol, AngularVelocity armVelTol) {
        m_ = m ;
        
        targetElevPos_ = targetElevPos ;
        elev_pos_tolerance_ = elevPosTol ;
        elev_vel_tolerance_ = elevVelTol ;

        targetArmPos_ = targetArmPos ;
        arm_pos_tolerance_ = armPosTol ;
        arm_vel_tolerance_ = armVelTol ;

    }

    @Override
    public void initialize() {
        if (m_.getArmPosition().isNear(ManipulatorConstants.Arm.Positions.kRaiseAngle, ManipulatorConstants.Arm.kPosTolerance)) {
            m_.setElevatorTarget(targetElevPos_, elev_pos_tolerance_, elev_vel_tolerance_) ;
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
                    m_.setElevatorTarget(targetElevPos_, elev_pos_tolerance_, elev_vel_tolerance_);
                    state_ = State.MoveElevator ;
                }
                break ;

            case MoveElevator:
                if (m_.isElevAtTarget()) {
                    m_.setArmTarget(targetArmPos_, arm_pos_tolerance_, arm_vel_tolerance_) ;
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
