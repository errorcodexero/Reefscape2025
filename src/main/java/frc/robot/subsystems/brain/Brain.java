package frc.robot.subsystems.brain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.oi.CoralSide;
import frc.robot.subsystems.oi.OICommandSupplier;
import frc.robot.subsystems.oi.OISubsystem;
import frc.robot.subsystems.oi.RobotAction;

public class Brain extends SubsystemBase {
    // The currently executing action, can be null if nothing is being executed
    private RobotAction current_action_ ;

    // The next action to be executed, can be null if no action is pending
    private RobotAction next_action_ ;

    // If true, we do not accept any robot actions
    private boolean locked_ ;   

    // The command associated with the current robot action
    private OICommandSupplier.Pair<Command, Command> current_robot_action_command_ ;

    // The command we are running
    private Command current_cmd_ ;

    // The last command we ran
    private Command last_cmd_ ;

    // The OI subsystem, for LED control
    private OISubsystem oi_ ;

    // The level of coral to place
    private int coral_level_ ;

    // The side of the coral to place
    private CoralSide coral_side_ ;

    //
    // This supplier provides a pair of commands that must be executed in order to perform a given
    // robot action.  The second command may be null indicating the robot action can complete with a 
    // single command.
    //
    private OICommandSupplier robot_action_command_supplier_ ;

    public Brain(OISubsystem oi, OICommandSupplier robotActionCommandSupplier) {
        oi_ = oi ;
        robot_action_command_supplier_ = robotActionCommandSupplier ;
        current_action_ = null ;
        next_action_ = null ;
        current_robot_action_command_ = null ;

        CommandScheduler.getInstance().onCommandFinish(this::cmdFinished) ;
    }

    private void cmdFinished(Command c) {
        if (current_cmd_ == c) {
            last_cmd_ = c ;
            current_cmd_ = null ;
        }
    }

    public void setCoralLevel(int l) {
        coral_level_ = l ;
    }

    public int coralLevel() {
        return coral_level_ ;
    }

    public void setCoralSide(CoralSide s) {
        coral_side_ = s ;
    }

    public CoralSide coralSide() {
        return coral_side_ ;
    }

    public boolean readyForAction() {
        return current_action_ == null || next_action_ == null ;
    }

    public void queueRobotAction(RobotAction action) {
        if (!locked_ && RobotState.isEnabled() && RobotState.isTeleop()) {
            next_action_ = action ;
            if (current_action_ == null) {
                periodic();
            }
        }
    }    

    //
    // This clears the state of the OI to a basic default, no actions 
    // scheduled state.
    // 
    public void clearRobotActions() {
        if (current_cmd_ != null) {
            current_cmd_.cancel() ;
        }

        current_action_ = null ;
        next_action_ = null ;

        oi_.clearAllActionLEDs() ;
    }

    public void lock() {
        locked_ = true ;
    }

    public void unlock() {
        locked_ = false ;
    }

    public void execute() {
        if (current_cmd_ == null && current_robot_action_command_ != null && current_robot_action_command_.two() != null) {
            Logger.recordOutput("oi/execute", false) ;
            current_cmd_ = current_robot_action_command_.two() ;
            current_cmd_.schedule();
        }
    }

    @Override
    public void periodic() {
        String status = "" ;

        if (current_action_ == null) {
            //
            // We are executing nothing, see if there are things to run queued
            //
            if (next_action_ != null) {
                current_action_ = next_action_ ;
                next_action_ = null ;
                current_robot_action_command_ = robot_action_command_supplier_.get(current_action_, coral_level_, coral_side_) ;
                if (current_robot_action_command_ == null) {
                    status = current_action_.toString() + ":no command" ;
                    current_action_ = null ;
                    current_cmd_ = null ;
                }
                else {
                    current_cmd_ = current_robot_action_command_.one() ;
                    status = current_action_.toString() + ":" + current_cmd_.getName() ;
                    current_cmd_.schedule() ;
                }
            }
            else {
                status = "idle" ;
            }
        }
        else {
            if (current_cmd_ == null && last_cmd_ == null) {
                status = current_action_.toString() + ":waiting" ;
            } 
            else if (current_cmd_ == null && last_cmd_ != null) {
                if (last_cmd_ == current_robot_action_command_.two() || current_robot_action_command_.two() == null) {
                    current_action_ = null ;
                    last_cmd_ = null ;
                    status = "idle" ;


                    //
                    // We are done with the current command.  We call recursively back into this code to be sure the next
                    // command starts in the same robot loop
                    //
                    periodic();
                }
                else {
                    current_cmd_ = null ;
                    last_cmd_ = null ;
                    status = "finished" ;
                }
            }
            else {
                status = current_action_.toString() + ":" + current_cmd_.getName() ;
            }
        }

        Logger.recordOutput("oi/status", status) ;
        Logger.recordOutput("oi/locked", locked_) ;
        Logger.recordOutput("oi/current_action", (current_action_ != null) ? current_action_.toString() : "none") ; 
        Logger.recordOutput("oi/next_action", next_action_ != null ? next_action_.toString() : "none") ;
    }

} 
