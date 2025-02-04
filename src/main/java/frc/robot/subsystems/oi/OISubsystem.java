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

    //
    // The left or right side of the reef for a given
    // reef face.
    //
    public enum CoralSide {
        Left,
        Right
    }

    //
    // This names the actions a robot can perform.  This is for a current action and a
    // next action.  Abort is on included in this as abort is immediate and interrupts any
    // of these that are underway.
    //
    public enum RobotAction {
        PlaceCoral,
        CollectCoral,
        CollectAlgaeGround,
        CollectAlgaeReefL2,
        CollectAlgaeReefL3,
        PlaceAlgae,
        ClimbDeploy,
        ClimbExecute,
        Eject,
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

    // The trigger for the abort button
    private Trigger abort_trigger_ ;

    // The trigger for the eject button, this interrupts anything that is going on and
    // is immediate.  It also sets the eject as a command being executed.
    private Trigger eject_trigger_ ;

    // The command associated with the current robot action
    private Command current_robot_action_command_ ;

    // The supplier for the robot action command based on the OI state
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

        // Initialize the LEDs
        for (OILed led : OILed.values()) {
            ios_.setLED(led.value, LEDState.Off) ;
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

    public int coralLevel() {
        return coral_level_ ;
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
            case Eject:
                setLEDState(OILed.Eject, st) ;
                break ;

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

            case ClimbDeploy:
                setLEDState(OILed.ClimbDeploy, st) ;
                break ;

            case ClimbExecute:
                setLEDState(OILed.ClimbExecute, st) ;
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
        }
        else {
            setLEDState(OILed.CoralLeft, LEDState.Off) ;
            setLEDState(OILed.CoralRight, LEDState.On) ;
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

    private void commandProcessing() {
        //
        // Process the robot action buttons
        //
        if (inputs_.coral_collect) {
            next_action_ = RobotAction.CollectCoral ;
            setRobotActionLEDState(next_action_, null);
        }
        else if (inputs_.coral_place) {
            next_action_ = RobotAction.PlaceCoral ;
            setRobotActionLEDState(next_action_, null);
        }
        else if (inputs_.algae_ground) {
            next_action_ = RobotAction.CollectAlgaeGround ;
            setRobotActionLEDState(next_action_, null);
        }
        else if (inputs_.algae_score) {
            next_action_ = RobotAction.PlaceAlgae ;
            setRobotActionLEDState(next_action_, null);
        }
        else if (inputs_.algae_collect_l2) {
            next_action_ = RobotAction.CollectAlgaeReefL2 ;
            setRobotActionLEDState(next_action_, null);
        }
        else if (inputs_.algae_collect_l3) {
            next_action_ = RobotAction.CollectAlgaeReefL3 ;
            setRobotActionLEDState(next_action_, null);
        }

        if (next_action_ != null) {
            setRobotActionLEDState(next_action_, LEDState.On);
        }

        if (current_robot_action_command_ != null && current_robot_action_command_.isFinished()) {
            //
            // The currently executing command has finished, we need to clear the LED
            // and mark the command as complete but setting it to null
            //
            setRobotActionLEDState(current_action_, LEDState.Off);
            current_robot_action_command_ = null ;
        }

        if (next_action_ != null && current_robot_action_command_ == null) {
            //
            // We are ready to execute a new command, we schedule the command
            //
            current_action_ = next_action_ ;
            next_action_ = null ;
            current_robot_action_command_ = robot_action_command_supplier_.get(current_action_, coral_level_, getCoralSide()) ;
            if (current_robot_action_command_ != null) {
                current_robot_action_command_.schedule();
            }
        }

        Logger.recordOutput("oi/current_action", (current_action_ != null) ? current_action_.toString() : "none") ; 
        Logger.recordOutput("oi/next_action", next_action_ != null ? next_action_.toString() : "none") ;
    }

    public RobotAction getCurrentAction() {
        return current_action_ ;
    }

    public RobotAction getNextAction() {
        return next_action_ ;
    }

    public CoralSide getCoralSide() {
        return inputs_.coral_side ? CoralSide.Left : CoralSide.Right ;
    }

    public Trigger abort() {
        return abort_trigger_ ;
    }

    public Trigger eject() {
        return eject_trigger_ ;
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
