package frc.robot.subsystems.oi;

import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OISubsystem extends SubsystemBase {
    
    //
    // The LEDs on the driver station
    //
    public enum OILed {
        Eject(0),
        CoralL1(1),
        CoralL2(2),
        CoralL3(3),
        CoralL4(4),
        CoralCollect(5),
        CoralPlace(6),
        AlgaeGround(7),
        AlgaeScore(8),
        AlgaeCollectL2(9),
        AlgaeCollectL3(10),
        ClimbDeploy(11),
        ClimbExecute(12),
        CoralLeft(13),
        CoralRight(14),
        HoldingCoral(15),
        HoldingAlgaeLow(16),
        HoldingAlgaeHigh(17);

        public final Integer value ;

        private OILed(int value) {
            this.value = value ;
        }
    }

    //
    // The states of the LEDs
    //
    public enum LEDState {
        On,
        Off,
        Slow,
        Fast
    }

    // The IO layer for the OI
    private OIIO ios_ ;

    // The inputs from the OI IO layer during the last robot loop
    private OIIosInputsAutoLogged inputs_ ;

    // The currently executing action, can be null if nothing is being executed
    private RobotAction current_action_ ;

    // The next action to be executed, can be null if no action is pending
    private RobotAction next_action_ ;
    
    // The gamepad controller attached for driving the robot
    private CommandXboxController gamepad_ ;

    // If true, we are rumbling the xbox controller
    private boolean rumbling_ ;

    // The time when the rumbling will end
    private double end_time_ ;

    // The current coral level
    private int coral_level_ ;

    // The current left/right side coral placement
    private CoralSide coral_side_ ;

    // If true, we do not accept any robot actions
    private boolean locked_ ;

    // Triggers
    private Trigger abort_trigger_ ;
    private Trigger eject_trigger_ ;
    private Trigger climb_lock_trigger_ ;
    private Trigger climb_exec_trigger_ ;
    private Trigger coral_place_trigger_ ;
    private Trigger coral_collect_trigger_ ;
    private Trigger algae_ground_trigger_ ;
    private Trigger algae_collect_l2_ ;
    private Trigger algae_collect_l3_ ;
    private Trigger algae_score_ ;

    // The command associated with the current robot action
    private OICommandSupplier.Pair<Command, Command> current_robot_action_command_ ;

    // The command we are running
    private Command current_cmd_ ;

    //
    // This supplier provides a pair of commands that must be executed in order to perform a given
    // robot action.  The second command may be null indicating the robot action can complete with a 
    // single command.
    //
    private OICommandSupplier robot_action_command_supplier_ ;

    public OISubsystem(OIIO ios, CommandXboxController ctrl, OICommandSupplier robotActionCommandSupplier) {
        this.ios_ = ios ;
        this.inputs_ = new OIIosInputsAutoLogged() ;
        gamepad_ = ctrl ;
        robot_action_command_supplier_ = robotActionCommandSupplier ;

        current_action_ = null ;
        next_action_ = null ;
        current_robot_action_command_ = null ;

        // Create the action triggers
        abort_trigger_ = new Trigger(() -> inputs_.abort) ;
        eject_trigger_ = new Trigger(() -> inputs_.eject) ;
        climb_lock_trigger_ = new Trigger(() -> inputs_.climb_lock) ;
        climb_exec_trigger_ = new Trigger(()-> inputs_.climb_execute) ;
        coral_place_trigger_  = new Trigger(()-> inputs_.coral_place) ;
        coral_collect_trigger_ = new Trigger(()-> inputs_.coral_collect) ;
        algae_ground_trigger_ = new Trigger(()-> inputs_.algae_ground) ;
        algae_collect_l2_ = new Trigger(()-> inputs_.algae_collect_l2) ;
        algae_collect_l3_ = new Trigger(()-> inputs_.algae_collect_l3) ;
        algae_score_ = new Trigger(()-> inputs_.algae_score) ;

        // Initialize the LEDs
        for (OILed led : OILed.values()) {
            ios_.setLED(led.value, LEDState.Off) ;
        }
    }

    public Trigger abort() {
        return abort_trigger_ ;
    }

    public Trigger eject() {
        return eject_trigger_ ;
    }    

    public Trigger climbLock() {
        return climb_lock_trigger_ ;
    }

    public Trigger climbExecute() {
        return climb_exec_trigger_ ;
    }

    public Trigger coralPlace() {
        return coral_place_trigger_ ;
    }

    public Trigger coralCollect() {
        return coral_collect_trigger_ ;
    }

    public Trigger algaeGround() {
        return algae_ground_trigger_ ;
    }

    public Trigger algaeCollectL2() {
        return algae_collect_l2_ ;
    }

    public Trigger algaeCollectL3() {
        return algae_collect_l3_ ;
    }

    public Trigger algaeScore() {
        return algae_score_ ;
    }

    public boolean readyForAction() {
        return current_action_ == null || next_action_ == null ;
    }

    public void queueRobotAction(RobotAction action) {
        if (!locked_ && RobotState.isEnabled() && RobotState.isTeleop()) {
            next_action_ = action ;
            if (current_action_ == null) {
                commandProcessing();
            }
        }
    }
    public void rumble(Time duration) {
        end_time_ = Timer.getFPGATimestamp() + duration.in(Seconds) ;
        rumbling_ = true ;
        gamepad_.setRumble(RumbleType.kBothRumble, 1.0);
    }

    public void setLEDState(OILed led, LEDState st) {
        ios_.setLED(led.value, st) ;
    }    

    public void clearAllActionLEDs() {
        setLEDState(OILed.CoralPlace, LEDState.Off) ;
        setLEDState(OILed.CoralCollect, LEDState.Off) ;
        setLEDState(OILed.AlgaeGround, LEDState.Off) ;
        setLEDState(OILed.AlgaeCollectL2, LEDState.Off) ;
        setLEDState(OILed.AlgaeCollectL3, LEDState.Off) ;
        setLEDState(OILed.AlgaeScore, LEDState.Off) ;
        setLEDState(OILed.ClimbDeploy, LEDState.Off) ;
        setLEDState(OILed.ClimbExecute, LEDState.Off) ;
    }

    public void setRobotActionLEDState(RobotAction a, LEDState st) {
        switch(a) {
            case PlaceCoral:
                setLEDState(OILed.CoralPlace, st) ;
                break ;

            case CollectCoral:
                setLEDState(OILed.CoralCollect, st) ;
                break ;

            case CollectAlgaeGround:
                setLEDState(OILed.AlgaeGround, st) ;
                break ;

            case CollectAlgaeReefL2:
                setLEDState(OILed.AlgaeCollectL2, st) ;
                break ;

            case CollectAlgaeReefL3:
                setLEDState(OILed.AlgaeCollectL3, st) ;
                break ;

            case PlaceAlgae:
                setLEDState(OILed.AlgaeScore, st) ;
                break ;
        }
    }

    @Override
    public void periodic() {
        ios_.updateInputs(inputs_) ;
        Logger.processInputs("OI", inputs_) ;

        if (rumbling_ && Timer.getFPGATimestamp() > end_time_) {
            gamepad_.setRumble(RumbleType.kBothRumble, 0);
            rumbling_ = false ;
        }

        //
        // Process the left versus right status information
        //
        if (inputs_.coral_side) {
            setLEDState(OILed.CoralLeft, LEDState.On) ;
            setLEDState(OILed.CoralRight, LEDState.Off) ;
            coral_side_ = CoralSide.Left ;
        }
        else {
            setLEDState(OILed.CoralLeft, LEDState.Off) ;
            setLEDState(OILed.CoralRight, LEDState.On) ;
            coral_side_ = CoralSide.Right ;
        }

        //
        // Process the L1 through L4 coral location
        //
        if (inputs_.coral_l1) {
            setLEDState(OILed.CoralL1, LEDState.On) ;
            coral_level_ = 1 ;
        } else {
            setLEDState(OILed.CoralL1, LEDState.Off) ;
        }

        if (inputs_.coral_l2) {
            setLEDState(OILed.CoralL2, LEDState.On) ;
            coral_level_ = 2 ;
        } else {
            setLEDState(OILed.CoralL2, LEDState.Off) ;
        }

        if (inputs_.coral_l3) {
            setLEDState(OILed.CoralL3, LEDState.On) ;
            coral_level_ = 3 ;
        } else {
            setLEDState(OILed.CoralL3, LEDState.Off) ;
        }

        if (inputs_.coral_l4) {
            setLEDState(OILed.CoralL4, LEDState.On) ;
            coral_level_ = 4 ;
        } else {
            setLEDState(OILed.CoralL4, LEDState.Off) ;
        }

        if (RobotState.isEnabled() && RobotState.isTeleop()) {
            commandProcessing() ;
        }

        Logger.recordOutput("oi/buttons", getPressedString()) ; 
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
        clearAllActionLEDs();
    }

    public void lock() {
        locked_ = true ;
    }

    public void unlock() {
        locked_ = false ;
    }

    public void execute() {
        if (current_cmd_ != null && current_cmd_.isFinished()) {
            Logger.recordOutput("oi/execute", true) ;
            current_cmd_ = current_robot_action_command_.two() ;
            current_cmd_.schedule();
        }
    }

    private void commandProcessing() {
        String status = "" ;

        Logger.recordOutput("oi/execute", false) ;
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
            if (current_cmd_ == null) {
                status = current_action_.toString() + ":waiting" ;
            } 
            else if (current_cmd_.isFinished()) {
                if (current_cmd_ == current_robot_action_command_.two() || current_robot_action_command_.two() == null) {
                    current_action_ = null ;
                    current_cmd_ = null ;

                    //
                    // We are done with the current command.  We call recursively back into this code to be sure the next
                    // command starts in the same robot loop
                    //
                    commandProcessing();
                }
                else {
                    current_cmd_ = null ;
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

    public String getPressedString() {
        String str = "" ;

        if (inputs_.eject) {
            if (str.length() > 0)
                str += "," ;
            str += "eject" ;
        }

        if (inputs_.abort) {
            if (str.length() > 0)
                str += "," ;
            str += "abort" ;
        }

        if (inputs_.execute) {
            if (str.length() > 0)
                str += "," ;
            str += "execute" ;
        }

        if (inputs_.coral_l1) {
            if (str.length() > 0)
                str += "," ;
            str += "coral_l1" ;
        }

        if (inputs_.coral_l2) {
            if (str.length() > 0)
                str += "," ;
            str += "coral_l2" ;
        }

        if (inputs_.coral_l3) {
            if (str.length() > 0)
                str += "," ;
            str += "coral_l3" ;
        }

        if (inputs_.coral_l4) {
            if (str.length() > 0)
                str += "," ;
            str += "coral_l4" ;
        }

        if (inputs_.coral_collect) {
            if (str.length() > 0)
                str += "," ;
            str += "coral_collect" ;
        }

        if (inputs_.coral_place) {
            if (str.length() > 0)
                str += "," ;
            str += "coral_place" ;
        }

        if (inputs_.algae_ground) {
            if (str.length() > 0)
                str += "," ;
            str += "algae_ground" ;
        }

        if (inputs_.algae_score) {
            if (str.length() > 0)
                str += "," ;
            str += "algae_score" ;
        }

        if (inputs_.algae_collect_l2) {
            if (str.length() > 0)
                str += "," ;
            str += "algae_collect_l2" ;
        }

        if (inputs_.algae_collect_l3) {
            if (str.length() > 0)
                str += "," ;
            str += "algae_collect_l3" ;
        }

        if (inputs_.climb_deploy) {
            if (str.length() > 0)
                str += "," ;
            str += "climb_deploy" ;
        }

        if (inputs_.climb_execute) {
            if (str.length() > 0)
                str += "," ;
            str += "climb_execute" ;
        }

        if (inputs_.climb_lock) {
            if (str.length() > 0)
                str += "," ;
            str += "climb_lock" ;
        }

        if (inputs_.coral_side) {
            if (str.length() > 0)
                str += "," ;
            str += "coral_side_left" ;
        } else {
            if (str.length() > 0)
                str += "," ;
            str += "coral_side_right" ;
        }

        if (gamepad_.getHID().getAButton()) {
            if (str.length() > 0)
                str += "," ;
            str += "a" ;
        }

        if (gamepad_.getHID().getBButton()) {
            if (str.length() > 0)
                str += "," ;
            str += "b" ;
        }

        if (gamepad_.getHID().getXButton()) {
            if (str.length() > 0)
                str += "," ;
            str += "x" ;
        }

        if (gamepad_.getHID().getYButton()) {
            if (str.length() > 0)
                str += "," ;
            str += "y" ;
        }

        if (gamepad_.getHID().getStartButton()) {
            if (str.length() > 0)
                str += "," ;
            str += "start" ;
        }

        if (gamepad_.getHID().getBackButton()) {
            if (str.length() > 0)
                str += "," ;
            str += "back" ;
        }

        if (gamepad_.getHID().getLeftStickButton()) {
            if (str.length() > 0)
                str += "," ;
            str += "ls" ;
        }

        if (gamepad_.getHID().getRightStickButton()) {
            if (str.length() > 0)
                str += "," ;
            str += "rs" ;
        }

        if (gamepad_.getHID().getLeftBumperButton()) {
            if (str.length() > 0)
                str += "," ;
            str += "lb" ;
        }

        if (gamepad_.getHID().getRightBumperButton()) {
            if (str.length() > 0)
                str += "," ;
            str += "rb" ;
        }
        

        return str ;
    }
}
