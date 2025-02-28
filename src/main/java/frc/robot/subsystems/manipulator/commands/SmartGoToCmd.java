package frc.robot.subsystems.manipulator.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

//
// Here are the constraints
//

public class SmartGoToCmd extends Command {

    private enum State {
        FinalTravel,
        MoveToRaiseAngle,
        MovingUpThroughKeepout,
        MovingDownThroughKeepout,
        Done,
        Error,
    }

    private ManipulatorSubsystem m_ ;
    private Distance height_ ;
    private Distance height_tolerance_ ;
    private Angle angle_ ;
    private Angle angle_tolerance ;
    private State state_ ;

    public SmartGoToCmd(ManipulatorSubsystem m, Distance d, Angle a) {
        this(m, d, ManipulatorConstants.Elevator.kPosTolerance, a, ManipulatorConstants.Arm.kPosTolerance) ;
    }

    public SmartGoToCmd(ManipulatorSubsystem m, Distance d, Distance dtol, Angle a, Angle atol) {
        m_ = m ;
        height_ = d ;
        angle_ = a ;
        height_tolerance_ = dtol ;
        angle_tolerance = atol ;
    }

    @Override
    public void initialize() {
        if (!isValid()) {
            System.out.println("SmartGoToCmd: Invalid target angle and height") ;
            state_ = State.Error ;
        }
        else if (m_.getArmPosition().isNear(angle_, angle_tolerance) && m_.getElevatorPosition().isNear(height_, height_tolerance_)) {
            state_ = State.Done ;
        }
        else if (targetIsInTravelZone(height_)) {
            //
            // Our target is in the travel zone we must ...
            //
            // If we are at the raise angle, just go to the height desired
            // Otherwise, move to the raise angle and then go to the height desired
            //

            if (!angle_.isNear(ManipulatorConstants.Arm.Positions.kRaiseAngle, ManipulatorConstants.Arm.kPosTolerance)) {
                //
                // We cannot stop in the travel zone except at the raise angle
                //
                state_ = State.Error ;
            }
            else if (m_.getArmPosition().isNear(ManipulatorConstants.Arm.Positions.kRaiseAngle, ManipulatorConstants.Arm.kPosTolerance)) {
                //
                // The arm is already at the raise angle, move to the desired height 
                //
                m_.setElevatorTarget(height_) ;
                state_ = State.FinalTravel ;
            }
            else {
                //
                // Move to the raise angle, after the arm is in the right position, we will move the
                // elevator to the right height
                //
                m_.setArmTarget(ManipulatorConstants.Arm.Positions.kRaiseAngle) ;
                state_ = State.MoveToRaiseAngle ;
            }
        }
        else if (travelThroughKeepout(height_)) {
            //
            // We need to move through the keepout zone
            //

            if (m_.getArmPosition().isNear(ManipulatorConstants.Arm.Positions.kRaiseAngle, ManipulatorConstants.Arm.kPosTolerance)) {
                //
                // We are already at the raise angle, start moving the elevator through the keepout
                //
                if (height_.gt(m_.getElevatorPosition())) {
                    state_ = State.MovingUpThroughKeepout ;
                }
                else {
                    state_ = State.MovingDownThroughKeepout ;
                }
                m_.setElevatorTarget(height_) ;
            }
            else {
                //
                // Not at the raise angle, move to the raise angle
                //
                m_.setArmTarget(ManipulatorConstants.Arm.Positions.kRaiseAngle) ;
                state_ = State.MoveToRaiseAngle ;
            }
        }
        else if (height_.lt(ManipulatorConstants.SmartGoToValues.kLowerRotateHeight)) {
            if (angle_.gte(ManipulatorConstants.SmartGoToValues.kLowerRotateMinAngle) && angle_.lte(ManipulatorConstants.SmartGoToValues.kLowerRotateMaxAingle)) {
                m_.setArmTarget(angle_) ;
                m_.setElevatorTarget(height_) ;
                state_ = State.FinalTravel ;
            }
            else {
                state_ = State.Error ;
            }
        }
        else if (height_.gt(ManipulatorConstants.SmartGoToValues.kUpperRotateHeight)) {
            if (angle_.gte(ManipulatorConstants.SmartGoToValues.kUpperRotateMinAngle) && angle_.lte(ManipulatorConstants.SmartGoToValues.kUpperRotateMaxAingle)) {
                m_.setArmTarget(angle_) ;
                m_.setElevatorTarget(height_) ;
                state_ = State.FinalTravel ;
            }
            else {
                state_ = State.Error ;
            }
        }
    }

    @Override
    public void execute() {
        switch(state_) {
            case FinalTravel:
                if (m_.isArmAtTarget() && m_.isElevAtTarget()) {
                    state_ = State.Done ;
                }
                break ;

            case MovingUpThroughKeepout:
                if (m_.getElevatorPosition().gte(ManipulatorConstants.SmartGoToValues.kUpperRotateHeight)) {
                }
                break ;

            case MovingDownThroughKeepout:
                break ;

            case MoveToRaiseAngle:
                if (m_.isArmAtTarget()) {
                    if (m_.getElevatorPosition().isNear(height_, ManipulatorConstants.Elevator.kPosTolerance)) {
                        state_ = State.Done ;
                    }
                    else {
                        //
                        // We are already at the raise angle, start moving the elevator through the keepout
                        //
                        if (height_.gt(m_.getElevatorPosition())) {
                            state_ = State.MovingUpThroughKeepout ;
                        }
                        else {
                            state_ = State.MovingDownThroughKeepout ;
                        }
                        m_.setElevatorTarget(height_) ;                        
                    }
                }
                break ;

            case Done:
                break ;

            case Error:
                break ;
        }
    }

    private boolean isValid() {
        boolean ret = false ;
        if (height_.lt(ManipulatorConstants.SmartGoToValues.kLowerRotateHeight)) {
            ret = angle_.gte(ManipulatorConstants.SmartGoToValues.kLowerRotateMinAngle) && angle_.lte(ManipulatorConstants.SmartGoToValues.kLowerRotateMaxAingle) ;
        }
        else if (height_.gt(ManipulatorConstants.SmartGoToValues.kUpperRotateHeight)) {
            ret = angle_.gte(ManipulatorConstants.SmartGoToValues.kUpperRotateMinAngle) && angle_.lte(ManipulatorConstants.SmartGoToValues.kUpperRotateMaxAingle) ;
        }
        else {
            ret = angle_.isNear(ManipulatorConstants.Arm.Positions.kRaiseAngle, ManipulatorConstants.Arm.kPosTolerance) ;
        }

        return ret ;
    }

    @Override
    public boolean isFinished() {
        return state_ == State.Done || state_ == State.Error ;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            m_.setArmTarget(m_.getArmPosition()) ;
            m_.setElevatorTarget(m_.getElevatorPosition()) ;
        }
    }

    private boolean travelThroughKeepout(Distance target) {
        return (target.lt(ManipulatorConstants.SmartGoToValues.kLowerRotateHeight) && m_.getElevatorPosition().gt(ManipulatorConstants.SmartGoToValues.kUpperRotateHeight)) ||
               (target.gt(ManipulatorConstants.SmartGoToValues.kUpperRotateHeight) && m_.getElevatorPosition().lt(ManipulatorConstants.SmartGoToValues.kLowerRotateHeight)) ;
    }

    private boolean targetIsInTravelZone(Distance target) {
        return target.gte(ManipulatorConstants.SmartGoToValues.kLowerRotateHeight) && target.lte(ManipulatorConstants.SmartGoToValues.kUpperRotateHeight) ;
    }
}
