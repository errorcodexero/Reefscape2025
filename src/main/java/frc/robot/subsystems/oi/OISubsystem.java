package frc.robot.subsystems.oi;

import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.oi.OIIO.OIIosInputs;

public class OISubsystem extends SubsystemBase {
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

    public enum LEDState {
        On,
        Off,
        Slow,
        Fast
    }

    public enum CoralSide {
        Left,
        Right
    }

    public enum RobotAction {
        PlaceCoral,
        CollectCoral,
        CollectAlgaeGround,
        CollectAlgaeReefL2,
        CollectAlgaeReefL3,
        PlaceAlgae,
        ClimbDeploy,
        ClimbExecute
    }

    private OIIO ios_ ;
    private OIIosInputs inputs_ ;

    private RobotAction current_action_ ;
    private RobotAction next_action_ ;
    
    private CommandXboxController gamepad_ ;
    private boolean rumbling_ ;
    private double end_time_ ;
    private int coral_level_ ;

    private Trigger eject_trigger_ ;
    private Trigger abort_trigger_ ;
    private Trigger execute_trigger_ ;
    private Trigger climb_deploy_trigger_ ;
    private Trigger climb_execute_trigger_ ;

    private Command current_robot_action_command_ ;
    private Supplier<Command> robot_action_command_supplier_ ;

    public OISubsystem(OIIO ios, CommandXboxController ctrl, Supplier<Command> robotActionCommandSupplier) {
        this.ios_ = ios ;
        this.inputs_ = new OIIosInputsAutoLogged() ;
        gamepad_ = ctrl ;
        robot_action_command_supplier_ = robotActionCommandSupplier ;

        current_action_ = null ;
        next_action_ = null ;
        current_robot_action_command_ = null ;

        clearAllActionLEDs() ;

        // Create the action triggers
        eject_trigger_ = new Trigger(() -> inputs_.eject) ;
        abort_trigger_ = new Trigger(() -> inputs_.abort) ;
        execute_trigger_ = new Trigger(() -> inputs_.execute) ;
        climb_deploy_trigger_ = new Trigger(() -> inputs_.climb_deploy && !inputs_.climb_lock) ;
        climb_execute_trigger_ = new Trigger(() -> inputs_.climb_execute && !inputs_.climb_lock) ;

        // Initialize the LEDs
        for (OILed led : OILed.values()) {
            ios_.setLED(led.value, LEDState.Off) ;
        }
    }

    public void badDriveBase() {
    }

    public void badVision() {
    }

    public void badGrabber() {
    }

    public void badManipulator() {
    }

    public void badClimber() {
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
        if (rumbling_ && Timer.getFPGATimestamp() > end_time_) {
            gamepad_.setRumble(RumbleType.kBothRumble, 0);
            rumbling_ = false ;
        }

        ios_.updateInputs(inputs_) ;

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

        setRobotActionLEDState(next_action_, LEDState.On);

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
            current_robot_action_command_ = robot_action_command_supplier_.get() ;
            if (current_robot_action_command_ != null) {
                current_robot_action_command_.schedule();
            }
        }
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

    public Trigger eject() {
        return eject_trigger_ ;
    }

    public Trigger abort() {
        return abort_trigger_ ;
    }

    public Trigger execute() {
        return execute_trigger_ ;
    }

    public Trigger climbDeploy() {
        return climb_deploy_trigger_ ;
    }

    public Trigger climbExecute() {
        return climb_execute_trigger_ ;
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

        return str ;
    }
}

