package frc.robot.subsystems.manipulator;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;

public class ManipulatorGotoCmd extends Command {
    private enum State {
        GoToTarget,                     // Going directly to targets, action is done when destinations are hit
        FinishHeight,                   // The arm is where it needs to be
        GoToKeepoutHeight,              // Move the arm to its destination
        MovingArm,                      // We are at the keepout height, move the arm to its destination
        WatchForKeepoutHeight,          // We are moving to a height greater than the keepout, start the arm as soon as we pass the keepout height
        Done,
        Interrupted,
        InvalidRequest,
    }

    private ManipulatorSubsystem manip_ ;
    private Distance elev_ ;
    private Angle arm_ ;
    private State state_ ;
    private boolean direct_ ;

    public ManipulatorGotoCmd(ManipulatorSubsystem manip, Distance elev, Angle arm) {
        this(manip, elev, arm, false) ;
    }   
    
    public ManipulatorGotoCmd(ManipulatorSubsystem manip, Distance elev, Angle arm, boolean direct) {
        manip_ = manip ;
        elev_ = elev ;
        arm_ = arm ;        
        direct_ = direct ;
    }   

    @Override
    public void initialize() {
        if (direct_) {
            initializeDirect() ;
        }
        else {
            initializeNonDirect() ;
        }
    }

    private void initializeDirect() {
        manip_.setArmAngleTarget(arm_);
        manip_.setElevatorHeightTarget(elev_);
        state_ = State.FinishHeight ;
    }

    private void initializeNonDirect() {
        Angle curarm = manip_.getCurrentArmAngle() ;
        Distance curelev = manip_.getCurrentElevatorHeight() ;

        if (isTargetInvalid()) {
            state_ = State.InvalidRequest ;
            return ;
        }

        if (curelev.gt(ManipulatorConstants.Goto.kElevatorKeepOutHeight)) {
            //
            // We are above the keepout height to start
            //
            if (doesArmCrossKeepout()) {
                //
                // We go to the keep height and move the arm
                //
                manip_.setArmAngleTarget(arm_);
                manip_.setElevatorHeightTarget(ManipulatorConstants.Goto.kElevatorKeepOutHeight);
                state_ = State.FinishHeight ;
            }
            else {
                //
                // The arm does not cross the keepout so we can just go to the desired target height
                //
                manip_.setArmAngleTarget(arm_);
                manip_.setElevatorHeightTarget(elev_);
                state_ = State.GoToTarget ;                
            }
        }
        else if (curarm.lt(ManipulatorConstants.Goto.kArmKeepOutMinAngle) && arm_.gt(ManipulatorConstants.Goto.kArmKeepOutMaxAngle)) {
            // 
            // We are crossing the keepout region from min to max
            //
            if (elev_.gt(ManipulatorConstants.Goto.kElevatorKeepOutHeight)) {
                manip_.setElevatorHeightTarget(elev_);
                state_ = State.WatchForKeepoutHeight ;
            }
            else {
                manip_.setElevatorHeightTarget(ManipulatorConstants.Goto.kElevatorKeepOutHeight);
                state_ = State.GoToKeepoutHeight ;
            }
        }
        else if (curarm.gt(ManipulatorConstants.Goto.kArmKeepOutMaxAngle) && arm_.lt(ManipulatorConstants.Goto.kArmKeepOutMinAngle)) {
            //
            // We are crossing the keepout region from max to min
            //
            if (elev_.gt(ManipulatorConstants.Goto.kElevatorKeepOutHeight)) {
                manip_.setElevatorHeightTarget(elev_);
                state_ = State.WatchForKeepoutHeight ;
            }
            else {
                manip_.setElevatorHeightTarget(ManipulatorConstants.Goto.kElevatorKeepOutHeight);
                state_ = State.GoToKeepoutHeight ;
            }
        }
        else {
            //
            // There is no interference with the keepout zone
            // go straight to the target positions
            //
            manip_.setArmAngleTarget(arm_);
            manip_.setElevatorHeightTarget(elev_);
            state_ = State.GoToTarget ;
        }
    }

    @Override
    public void execute() {
        switch(state_) {
            case GoToTarget:
                if (manip_.isArmAtTarget() && manip_.isElevatorAtTarget()) {
                    state_ = State.Done ;
                }
                break ;

            case FinishHeight:
                if (manip_.isArmAtTarget()) {
                    //
                    // The arm have finished its movement.  Now we can safely finish
                    // our height adjustments
                    //
                    manip_.setElevatorHeightTarget(elev_);
                    state_ = State.GoToTarget ;                        
                }
                break ;

            case GoToKeepoutHeight:
                if (manip_.isElevatorAtTarget()) {
                    manip_.setArmAngleTarget(arm_);
                    state_ = State.MovingArm ;
                }
                break ;

            case MovingArm:
                if (manip_.isArmAtTarget()) {
                    state_ = State.Done ;
                }
                break ;

            case WatchForKeepoutHeight:
                if (manip_.getCurrentElevatorHeight().gte(ManipulatorConstants.Goto.kElevatorKeepOutHeight)) {
                    manip_.setArmAngleTarget(arm_);
                    state_ = State.GoToTarget ;
                }
                break ;

            case Interrupted:
            case Done:
            case InvalidRequest:
                break ;
        }
    }

    @Override
    public boolean isFinished() {
        return state_ == State.Done || state_ == State.InvalidRequest ;
    }
    
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            state_ = State.Interrupted ;
            manip_.setArmAngleTarget(manip_.getCurrentArmAngle());
            manip_.setElevatorHeightTarget(manip_.getCurrentElevatorHeight()) ;
        }
    }

    private boolean isTargetInvalid() {
        return (arm_.gt(ManipulatorConstants.Goto.kArmKeepOutMinAngle) && 
                arm_.lt(ManipulatorConstants.Goto.kArmKeepOutMaxAngle) &&
                elev_.lt(ManipulatorConstants.Goto.kElevatorKeepOutHeight)) ||
                elev_.lt(ManipulatorConstants.Elevator.kMinHeight) ||
                elev_.gt(ManipulatorConstants.Elevator.kMaxHeight) ;
    }

    private boolean doesArmCrossKeepout() {
        Angle curarm = manip_.getCurrentArmAngle() ;

        return (curarm.lt(ManipulatorConstants.Goto.kArmKeepOutMinAngle) && arm_.gt(ManipulatorConstants.Goto.kArmKeepOutMaxAngle)) || 
                (curarm.gt(ManipulatorConstants.Goto.kArmKeepOutMaxAngle) && arm_.lt(ManipulatorConstants.Goto.kArmKeepOutMinAngle)) ;
    }
}
