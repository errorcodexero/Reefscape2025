package frc.robot.subsystems.brain;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.robot.collectcoral.CollectCoralCmd;
import frc.robot.commands.robot.placecoral.PlaceCoralTwoStepOne;
import frc.robot.commands.robot.placecoral.PlaceCoralTwoStepTwo;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.oi.CoralSide;
import frc.robot.subsystems.oi.OISubsystem;
import frc.robot.subsystems.oi.OISubsystem.LEDState;
import frc.robot.util.ReefUtil;

public class BrainSubsystem extends SubsystemBase {
    // The currently executing action, can be null if nothing is being executed
    private RobotAction current_action_ ;

    // The next action to be executed, can be null if no action is pending
    private RobotAction next_action_ ;

    // If true, we do not accept any robot actions
    private boolean locked_ ;

    // The command associated with the current robot action
    private RobotActionCommandList current_robot_action_command_ ;
    private int current_robot_action_command_index_ ;

    // The command we are running
    private Command current_cmd_ ;

    // The OI subsystem, for LED control
    private OISubsystem oi_ ;

    // The level of coral to place
    private int coral_level_ ;

    // The side of the coral to place
    private CoralSide coral_side_ ;

    // The game piece we are holding
    private GamePiece gp_ ;

    //
    // Subsystems used to implement the robot actions that are
    // managed by the brain subsystem.  Remove th suppress warnings when
    // we get the code to generate the commands required for the various
    // robot actions.
    //
    @SuppressWarnings("unused")
    private Drive db_ ;

    @SuppressWarnings("unused")
    private ManipulatorSubsystem m_ ;

    @SuppressWarnings("unused")
    private GrabberSubsystem g_ ;   

    public BrainSubsystem(OISubsystem oi, Drive db, ManipulatorSubsystem m, GrabberSubsystem g) {
        oi_ = oi ;
        db_ = db ;
        m_ = m ;
        g_ = g ;
        locked_ = false ;

        current_action_ = null ;
        next_action_ = null ;
        current_robot_action_command_ = null ;
        current_robot_action_command_index_ = 0 ;
        gp_ = GamePiece.NONE ;

        CommandScheduler.getInstance().onCommandFinish(this::cmdFinished) ;
    }

    public GamePiece gp() {
        return gp_ ;
    }

    public void setGp(GamePiece gp) {
        gp_ = gp ;
        switch(gp) {
            case NONE:
                oi_.setLEDState(OISubsystem.OILed.HoldingCoral, LEDState.Off) ;
                oi_.setLEDState(OISubsystem.OILed.HoldingAlgaeHigh, LEDState.Off) ;
                oi_.setLEDState(OISubsystem.OILed.HoldingAlgaeLow, LEDState.Off) ;
                break ;

            case CORAL:
                oi_.setLEDState(OISubsystem.OILed.HoldingCoral, LEDState.On) ;
                oi_.setLEDState(OISubsystem.OILed.HoldingAlgaeHigh, LEDState.Off) ;
                oi_.setLEDState(OISubsystem.OILed.HoldingAlgaeLow, LEDState.Off) ;
                break ;
                
            case ALGAE_HIGH:
                oi_.setLEDState(OISubsystem.OILed.HoldingCoral, LEDState.Off) ;
                oi_.setLEDState(OISubsystem.OILed.HoldingAlgaeHigh, LEDState.On) ;
                oi_.setLEDState(OISubsystem.OILed.HoldingAlgaeLow, LEDState.Off) ;
                break ;

            case ALGAE_LOW:
                oi_.setLEDState(OISubsystem.OILed.HoldingCoral, LEDState.Off) ;
                oi_.setLEDState(OISubsystem.OILed.HoldingAlgaeHigh, LEDState.Off) ;
                oi_.setLEDState(OISubsystem.OILed.HoldingAlgaeLow, LEDState.On) ;
                break ;
        }
    }    

    private void cmdFinished(Command c) {
        if (current_cmd_ == c) {
            current_cmd_ = null ;
            if (current_action_ != null && current_robot_action_command_index_ >= current_robot_action_command_.size()) {
                completedAction() ;
            }
        }
    }

    public void setCoralLevel(int l) {
        coral_level_ = l ;
    }

    public int level() {
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
            //
            // We can override the next action, until it becomes the current action
            //
            if (next_action_ != null) {
                oi_.setRobotActionLEDState(next_action_, LEDState.Off) ;
            }
            next_action_ = action ;
            if (next_action_ != null) {
                oi_.setRobotActionLEDState(next_action_, LEDState.On) ;
            }

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

        current_cmd_ = null ;
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

    private void completedAction() {
        current_action_ = null ;
        current_robot_action_command_ = null ;
        current_robot_action_command_index_ = 0 ;
        current_cmd_ = null ;
        if (next_action_ != null) {
            startNewAction() ;
        }
    }

    public void execute() {
        boolean cond = true ;
        if (current_robot_action_command_.getCondition(current_robot_action_command_index_) != null) {
            cond = current_robot_action_command_.getCondition(current_robot_action_command_index_).getAsBoolean() ;
        }

        if (cond && current_cmd_ == null && current_action_ != null && current_robot_action_command_ != null) {
            current_cmd_ = current_robot_action_command_.getCommand(current_robot_action_command_index_++) ;
            current_cmd_.schedule();
        }
    }

    private String startNewAction() {
        String status ;

        //
        // We are executing nothing, see if there are things to run queued
        //
        if (next_action_ != null) {
            current_robot_action_command_index_ = 0 ;
            current_action_ = next_action_ ;
            next_action_ = null ;
            oi_.setRobotActionLEDState(current_action_, LEDState.Fast) ;
            current_robot_action_command_ = this.getRobotActionCommand(current_action_, coral_level_, coral_side_) ;

            if (current_robot_action_command_ == null || current_robot_action_command_.size() == 0) {
                status = current_action_.toString() + ":no command" ;
                current_action_ = null ;
                current_cmd_ = null ;
            }
            else {
                current_cmd_ = current_robot_action_command_.getCommand(current_robot_action_command_index_++) ;
                status = current_action_.toString() + ":" + current_cmd_.getName() ;
                current_cmd_.schedule() ;
            }
        }
        else {
            status = "idle" ;
        }
        return status ;
    }

    @Override
    public void periodic() {
        String status = "" ;

        if (current_action_ == null) {
            if (next_action_ != null) {
                status = startNewAction() ;
            }
            else {
                status = "idle" ;
            }
        }
        else {
            if (current_cmd_ == null) {
                //
                // The last command has finished and we are waiting for the execute button to schedule
                // the next phase of this robot action.
                //
                status = current_action_.toString() + ":waiting" ;
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

    static final boolean PlaceCoralTwoStep = true ;

    private RobotActionCommandList getRobotActionCommand(RobotAction action, int level, CoralSide side) {
        List<Command> list = new ArrayList<Command>() ;
        List<BooleanSupplier> conds = new ArrayList<BooleanSupplier>() ;

        switch(action) {
            case CollectCoral:
                list.add(new CollectCoralCmd(this, m_, g_)) ;
                conds.add(null) ;
                break ;

            case PlaceCoral:
                    list.add(new PlaceCoralTwoStepOne(m_)) ;
                    conds.add(null) ;

                    // We only execute this step if we are in a position that the target face is valid
                    list.add(new PlaceCoralTwoStepTwo(this, db_, m_, g_, true)) ;
                    conds.add(() -> { return ReefUtil.getTargetedReefFace(db_.getPose()).isPresent() ; }) ;
                break ;

            case PlaceAlgae:
                // TODO: write me
                break ;

            case CollectAlgaeReef:
                // TODO: write me
                break ;

            case CollectAlgaeGround:
                // TODO: write me
                break ;
        }

        return new RobotActionCommandList(action, list, conds) ;
    }    
} 
