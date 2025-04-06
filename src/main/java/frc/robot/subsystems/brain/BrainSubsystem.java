package frc.robot.subsystems.brain;

import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;
import org.xerosw.util.XeroSequenceCmd;
import org.xerosw.util.XeroTimer;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.funnel.FunnelSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.oi.CoralSide;
import frc.robot.subsystems.oi.OIConstants.LEDState;
import frc.robot.subsystems.oi.OIConstants.OILed;
import frc.robot.subsystems.oi.OISubsystem;
import frc.robot.util.ReefFaceInfo;
import frc.robot.util.ReefUtil;
import frc.robot.commands.robot.NullCmd ;
import frc.robot.commands.robot.collectalgaereef.CollectAlgaeReefCmd;
import frc.robot.commands.robot.collectalgaereef.CollectAlgaeReefGotoCmd;
import frc.robot.commands.robot.collectcoral.CollectCoralCmd;
import frc.robot.commands.robot.placecoral.PlaceCoralCmd;
import frc.robot.commands.robot.scorealgae.ScoreAlgaeAfter;
import frc.robot.Constants.ReefLevel;
import frc.robot.RobotContainer;

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
    private XeroSequenceCmd current_cmd_ ;

    // The OI subsystem, for LED control
    private OISubsystem oi_ ;

    // The level of coral to place
    private ReefLevel coral_level_ ;

    // The side of the coral to place
    private CoralSide coral_side_ ;

    // If true, there is algae on the reef where placing
    private boolean algae_on_reef_ ;

    // The game piece we are holding
    private GamePiece gp_ ;

    // If true, the left/right side switch has been initialized
    private boolean leds_inited_ ;

    // The number of times periodic has been called
    private int periodic_count_ ;

    // If true, we are clearing state
    private boolean clearing_state_ ;

    private boolean climb_signaled_ ;

    private boolean going_down_ ;

    private XeroTimer flash_timer_ ;

    //
    // Subsystems used to implement the robot actions that are
    // managed by the brain subsystem.  Remove th suppress warnings when
    // we get the code to generate the commands required for the various
    // robot actions.
    //
    private Drive db_ ;
    private ManipulatorSubsystem m_ ;
    private GrabberSubsystem g_ ;   
    private ClimberSubsystem c_ ;
    private FunnelSubsystem f_ ;

    private boolean placed_ok_ ;

    public BrainSubsystem(OISubsystem oi, Drive db, ManipulatorSubsystem m, GrabberSubsystem g, ClimberSubsystem c, FunnelSubsystem f) {
        oi_ = oi ;
        db_ = db ;
        m_ = m ;
        g_ = g ;
        c_ = c ;
        f_ = f ;
        locked_ = false ;

        current_action_ = null ;
        next_action_ = null ;
        current_robot_action_command_ = null ;
        current_robot_action_command_index_ = 0 ;
        gp_ = GamePiece.NONE ;
        leds_inited_ = false ;
        periodic_count_ = 0 ;
        climb_signaled_ = false ;
        placed_ok_ = false ;

        flash_timer_ = null ;
    }

    public void setGoingDown(boolean b) {
        going_down_ = b ;
    }

    public GamePiece gp() {
        return gp_ ;
    }

    public boolean placedOk() {
        return placed_ok_ ;
    }

    public void setPlacedOk(boolean v) {
        placed_ok_ = v ;
    }

    public void setGp(GamePiece gp) {
        gp_ = gp ;
        switch(gp) {
            case NONE:
                oi_.setLEDState(OILed.HoldingCoral, LEDState.Off) ;
                oi_.setLEDState(OILed.HoldingAlgaeHigh, LEDState.Off) ;
                break ;

            case CORAL:
                oi_.setLEDState(OILed.HoldingCoral, LEDState.On) ;
                oi_.setLEDState(OILed.HoldingAlgaeHigh, LEDState.Off) ;
                break ;
                
            case ALGAE_HIGH:
                oi_.setLEDState(OILed.HoldingCoral, LEDState.Off) ;
                oi_.setLEDState(OILed.HoldingAlgaeHigh, LEDState.On) ;
                break ;
        }
    }    

    public boolean doesReefHaveAlgae() {
        return algae_on_reef_ ;
    }

    public void toggleAlgaeOnReef() {
        algae_on_reef_ = !algae_on_reef_ ;
        oi_.setLEDState(OILed.AlgaeOnReef, algae_on_reef_ ? LEDState.On : LEDState.Off) ;
    }

    public void clearAlgaeOnReef() {
        algae_on_reef_ = false ;
        oi_.setLEDState(OILed.AlgaeOnReef, algae_on_reef_ ? LEDState.On : LEDState.Off) ;
    }

    public void setCoralLevel(ReefLevel height) {
        if (flash_timer_ != null) {
            flash_timer_ = null ;
        }
        
        coral_level_ = height ;
        oi_.setLevelLED(height);

        //
        // This is a HACK but its the safest way to do this
        //
        if ((current_action_ == RobotAction.CollectAlgaeReefKeep || current_action_ == RobotAction.CollectAlgaeReefEject) && 
                        current_cmd_ == null && current_robot_action_command_index_ == 1) {
            current_cmd_ = new CollectAlgaeReefGotoCmd(this, m_, height) ;
            current_cmd_.schedule() ;
        }
    }

    public ReefLevel coralLevel() {

        return coral_level_ ;
    }

    public ReefLevel algaeLevel() {
        if (coral_level_ == ReefLevel.L1 || coral_level_ == ReefLevel.L2)
            return ReefLevel.L2 ;

        return ReefLevel.L3 ;
    }

    public void setCoralSide(CoralSide s) {
        coral_side_ = s ;
        oi_.setSideLED(s);
    }

    public CoralSide coralSide() {
        return coral_side_ ;
    }

    private boolean isNextActionLegal(RobotAction act) {
        boolean ret = true ;

        if (current_action_ != null) {
            switch(current_action_) {
                case CollectCoral:
                    ret = (act == RobotAction.PlaceCoral) ;
                    break ;

                case PlaceCoral:
                    ret = (act != RobotAction.PlaceCoral && act != RobotAction.ScoreAlgae) ;
                    break ;

                case ScoreAlgae:
                    ret = (act == RobotAction.PlaceCoral) ;
                    break ;

                case CollectAlgaeReefKeep:
                    ret = (act != RobotAction.PlaceCoral && act != RobotAction.CollectCoral) ;
                    break ;

                case CollectAlgaeReefEject:
                    ret = (act != RobotAction.PlaceCoral && act != RobotAction.CollectCoral) ;
                    break ;

                default:
                    break ;
            }
        }

        return ret;
    }

    public void queueRobotAction(RobotAction action) {
        if (!locked_ && RobotState.isEnabled() && RobotState.isTeleop()) {
            //
            // We can override the next action, until it becomes the current action
            //
            if (next_action_ != null) {
                //
                // Turn off the button for the current next action
                //
                oi_.setRobotActionLEDState(next_action_, LEDState.Off) ;
            }

            if (!isNextActionLegal(action)) {
                oi_.flashDisplay();
                return ;
            }
            else {
                next_action_ = action ;
                if (action != null) {
                    oi_.setRobotActionLEDState(next_action_, LEDState.On) ;
                }
            }

            //
            // If the current action is null, start the next action now
            //
            if (current_action_ == null) {
                startNewAction() ;
            }
        }
    }    

    public void coralOnFloor() {
        flash_timer_ = new XeroTimer(Seconds.of(3.0)) ;
        flash_timer_.start() ;
        oi_.setLEDState(OILed.CoralL1, LEDState.Fast) ;
        oi_.setLEDState(OILed.CoralL2, LEDState.Fast) ;
        oi_.setLEDState(OILed.CoralL3, LEDState.Fast) ;
        oi_.setLEDState(OILed.CoralL4, LEDState.Fast) ;
    }

    //
    // This clears the state of the OI to a basic default, no actions 
    // scheduled state.
    //
    public void clearRobotActions() {
        clearing_state_ = true ;

        if (current_cmd_ != null) {
            //
            // If a current command is running, cancel it
            //
            current_cmd_.cancel() ;
        }

        current_cmd_ = null ;
        current_action_ = null ;
        next_action_ = null ;

        oi_.clearAllActionLEDs() ;

        clearing_state_ = false ;
    }

    public void lock() {
        locked_ = true ;
    }

    public void unlock() {
        locked_ = false ;
    }

    private void completedAction() {
        oi_.setRobotActionLEDState(current_action_, LEDState.Off) ;
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

        oi_.setLEDState(OILed.Execute, LEDState.Off) ;

        if (current_robot_action_command_ == null || current_robot_action_command_index_ >= current_robot_action_command_.size()) {
            //
            // Gunner hit the execute button with nothing to execute
            //
            return ;
        }

        if (current_robot_action_command_.getCondition(current_robot_action_command_index_) != null) {
            cond = current_robot_action_command_.getCondition(current_robot_action_command_index_).getAsBoolean() ;
        }

        if (cond && current_cmd_ == null && current_action_ != null && current_robot_action_command_ != null) {
            current_cmd_ = current_robot_action_command_.getCommand(current_robot_action_command_index_++) ;
            current_cmd_.schedule();
        }
    }

    private boolean isCurrentActionLegal() {
        boolean ret = true ;

        switch(current_action_) {
            case CollectCoral:
                ret = (gp_ == GamePiece.NONE) ;
                break ;

            case PlaceCoral:
                ret = (gp_ == GamePiece.CORAL) ;
                break ;

            case ScoreAlgae:
                ret = (gp_ == GamePiece.ALGAE_HIGH) ;
                break ;

            case CollectAlgaeReefKeep:
                ret = (gp_ == GamePiece.NONE) ;
                break ;

            case CollectAlgaeReefEject:
                ret = (gp_ == GamePiece.NONE) ;
                break ;

            default:
                break ;
        }

        return ret;
    }

    private String startNewAction() {
        String status ;

        going_down_ = false ;

        //
        // We are executing nothing, see if there are things to run queued
        //
        if (next_action_ != null) {
            current_robot_action_command_index_ = 0 ;
            current_action_ = next_action_ ;
            next_action_ = null ;

            if (isCurrentActionLegal()) {
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
                current_action_ = null ;
                next_action_ = null ;
                status = "illegal action" ;
                oi_.flashDisplay();
                oi_.clearAllActionLEDs();
            }
        }
        else {
            status = "idle" ;
        }

        return status ;
    }

    private void initLEDs() {
        if (oi_.sideSwitch()) {
            coral_side_ = CoralSide.Right ;
        }
        else {
            coral_side_ = CoralSide.Left ;
        }
        coral_level_ = ReefLevel.L4 ;

        oi_.setLevelLED(coral_level_);
        oi_.setSideLED(coral_side_);
    }

    @Override
    public void periodic() {
        String status = "" ;

        if (flash_timer_ != null) {
            if (flash_timer_.isExpired()) {
                flash_timer_ = null ;
                oi_.setLEDState(OILed.CoralL1, LEDState.Off) ;
                oi_.setLEDState(OILed.CoralL2, LEDState.Off) ;
                oi_.setLEDState(OILed.CoralL3, LEDState.Off) ;
                oi_.setLEDState(OILed.CoralL4, LEDState.Off) ;
                if (coral_level_ != null) {
                    oi_.setLevelLED(coral_level_);
                }
            }
        }

        if (current_action_ == RobotAction.PlaceCoral && going_down_) {
            if (m_.getElevatorPosition().lte(ManipulatorConstants.Elevator.Positions.kReleaseGamePad)) {
                RobotContainer.getInstance().gamepad().setLocked(false) ;
            }
        }

        Logger.recordOutput("brain/holding", gp_.toString()) ;
        Logger.recordOutput("brain/placedok", placed_ok_) ;
        trackReefPlace() ;

        if (c_.readyToClimb() && !climb_signaled_) {
            //
            // This does not really belong in the brain, but it is a good place to put it for now
            //
            oi_.setLEDState(OILed.ReadyToClimb, LEDState.Fast) ;
            oi_.rumble(Seconds.of(2.0));
            climb_signaled_ = true ;
        }

        if (current_cmd_ != null && current_cmd_.isComplete() && !clearing_state_) {
            current_cmd_ = null ;
            if (current_action_ != null && current_robot_action_command_index_ >= current_robot_action_command_.size()) {
                //
                // This was the last command in the current action, we are done so complete this action and move to
                // the next action if there is one
                //
                completedAction() ;
            }            
        }

        if (!RobotState.isEnabled() || !RobotState.isTeleop()) {
            clearRobotActions() ;
            return ;
        }

        if (!leds_inited_ && periodic_count_ > 2) {
            initLEDs() ;
            leds_inited_ = true ;
        }
        periodic_count_++ ;

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
                oi_.setLEDState(OILed.Execute, LEDState.Fast) ;
                status = current_action_.toString() + ":waiting" ;
            }
            else {
                status = current_action_.toString() + ":" + current_cmd_.getName() ;
            }
        }


        Logger.recordOutput("brain/status", status) ;
        Logger.recordOutput("brain/locked", locked_) ;
        Logger.recordOutput("brain/current_action", (current_action_ != null) ? current_action_.toString() : "none") ; 
        Logger.recordOutput("brain/next_action", next_action_ != null ? next_action_.toString() : "none") ;
;
    }

    private void trackReefPlace() {
        Optional<ReefFaceInfo> info = ReefUtil.getTargetedReefFace(db_.getPose()) ;
        if (info.isPresent()) {
            Logger.recordOutput("brain/reefplace", info.get().getLeftScoringPose()) ;
        }
    }

    private RobotActionCommandList getRobotActionCommand(RobotAction action, ReefLevel level, CoralSide side) {
        List<XeroSequenceCmd> list = new ArrayList<XeroSequenceCmd>() ;
        List<BooleanSupplier> conds = new ArrayList<BooleanSupplier>() ;

        switch(action) {
            case CollectCoral:
                list.add(new CollectCoralCmd(this, oi_, m_, f_, g_, true)) ;
                conds.add(null) ;
                break ;

            case PlaceCoral:
                list.add(new NullCmd()) ;
                conds.add(null) ;

                list.add(new PlaceCoralCmd(this, db_, m_, g_, ReefLevel.AskBrain, CoralSide.AskBrain, true)) ;
                conds.add(() -> { return ReefUtil.getTargetedReefFace(db_.getPose()).isPresent() ; }) ;
                break ;

            case ScoreAlgae:
                list.add(new NullCmd()) ;
                conds.add(null) ;

                list.add(new ScoreAlgaeAfter(db_, this, m_, g_)) ;
                conds.add(null) ;
                break ;

            case CollectAlgaeReefEject:
                list.add(new CollectAlgaeReefGotoCmd(this, m_, ReefLevel.AskBrain)) ;
                conds.add(null) ;
                
                list.add(new CollectAlgaeReefCmd(this, db_, m_, g_, ReefLevel.AskBrain, true)) ;
                conds.add(() -> { return ReefUtil.getTargetedReefFace(db_.getPose()).isPresent() ; }) ;
                break ;

            case CollectAlgaeReefKeep:
                list.add(new CollectAlgaeReefGotoCmd(this, m_, ReefLevel.AskBrain)) ;
                conds.add(null) ;
                
                list.add(new CollectAlgaeReefCmd(this, db_, m_, g_, ReefLevel.AskBrain, false)) ;
                conds.add(() -> { return ReefUtil.getTargetedReefFace(db_.getPose()).isPresent() ; }) ;
                break ;
        }

        return new RobotActionCommandList(action, list, conds) ;
    }    
} 
