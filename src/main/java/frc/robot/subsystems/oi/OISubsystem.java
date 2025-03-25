package frc.robot.subsystems.oi;

import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;
import org.xerosw.hid.XeroGamepad;
import org.xerosw.util.XeroTimer;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ReefLevel;
import frc.robot.subsystems.brain.RobotAction;
import frc.robot.subsystems.oi.OIConstants.LEDState;
import frc.robot.subsystems.oi.OIConstants.OILed;

public class OISubsystem extends SubsystemBase { 

    // The IO layer for the OI
    private OIIO ios_ ;

    // The inputs from the OI IO layer during the last robot loop
    private OIIosInputsAutoLogged inputs_ ;

     
    // The gamepad controller attached for driving the robot
    private XeroGamepad gamepad_ ;

    // If true, we are rumbling the xbox controller
    private boolean rumbling_ ;

    // The time when the rumbling will end
    private double end_time_ ;

    // Triggers
    private Trigger abort_trigger_ ;
    private Trigger eject_trigger_ ;
    private Trigger climb_lock_trigger_ ;
    private Trigger climb_exec_trigger_ ;
    private Trigger climb_deploy_trigger_ ;
    private Trigger coral_place_trigger_ ;
    private Trigger coral_collect_trigger_ ;
    private Trigger algae_ground_trigger_ ;
    private Trigger algae_reef_trigger_ ;
    private Trigger algae_score_trigger_ ;
    private Trigger execute_trigger_ ;
    private Trigger rotate_arm_trigger_ ;
    private Trigger raise_arm_trigger_ ;
    private Trigger algae_net_trigger_ ;    
    
    private Trigger l1_ ;
    private Trigger l2_ ;
    private Trigger l3_ ;
    private Trigger l4_ ;
    private Trigger coral_left_right_ ;

    private boolean flashing_ ;
    private XeroTimer flashing_timer_ ;

    public OISubsystem(OIIO ios, XeroGamepad ctrl) {
        this.ios_ = ios ;
        this.inputs_ = new OIIosInputsAutoLogged() ;
        gamepad_ = ctrl ;
        rumbling_ = false ;

        flashing_timer_ = new XeroTimer(Seconds.of(1.0)) ;
        flashing_ = false ;

        // Create the action triggers
        abort_trigger_ = new Trigger(() -> inputs_.abort) ;
        eject_trigger_ = new Trigger(() -> inputs_.eject) ;
        climb_lock_trigger_ = new Trigger(() -> !inputs_.climb_lock) ;
        climb_exec_trigger_ = new Trigger(()-> inputs_.climb_execute) ;
        climb_deploy_trigger_ = new Trigger(()-> inputs_.climb_deploy) ;
        coral_place_trigger_  = new Trigger(()-> inputs_.coral_place) ;
        coral_collect_trigger_ = new Trigger(()-> inputs_.coral_collect) ;
        algae_ground_trigger_ = new Trigger(()-> inputs_.algae_ground) ;
        algae_reef_trigger_ = new Trigger(()-> inputs_.algae_reef) ;
        algae_score_trigger_ = new Trigger(()-> inputs_.algae_score) ;
        execute_trigger_ = new Trigger(()-> inputs_.execute) ;
        rotate_arm_trigger_ = new Trigger(()-> inputs_.rotate_arm) ;
        raise_arm_trigger_ = new Trigger(()-> inputs_.raise_arm) ;
        algae_net_trigger_ = new Trigger(()-> inputs_.algae_net) ;

        l1_ = new Trigger(()-> inputs_.coral_l1) ;
        l2_ = new Trigger(()-> inputs_.coral_l2) ;
        l3_ = new Trigger(()-> inputs_.coral_l3) ;
        l4_ = new Trigger(()-> inputs_.coral_l4) ;
        coral_left_right_ = new Trigger(()-> inputs_.coral_side) ;

        // Initialize the LEDs
        for (OILed led : OILed.values()) {
            ios_.setLED(led.value, LEDState.Off) ;
        }
    }

    public boolean sideSwitch() {
        return inputs_.coral_side ;
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

    public Trigger climbDeploy() {
        return climb_deploy_trigger_ ;
    }

    public Trigger coralPlace() {
        return coral_place_trigger_ ;
    }

    public Trigger coralCollect() {
        return coral_collect_trigger_ ;
    }

    public Trigger rotateArm() {
        return rotate_arm_trigger_ ;
    }

    public Trigger algaeNet() {
        return algae_net_trigger_ ;
    }

    public Trigger raiseArm() {
        return raise_arm_trigger_ ;
    }

    public Trigger algaeGround() {
        return algae_ground_trigger_ ;
    }

    public Trigger algaeReef() {
        return algae_reef_trigger_ ;
    }

    public Trigger algaeScore() {
        return algae_score_trigger_ ;
    }

    public Trigger execute() {
        return execute_trigger_ ;
    }

    public Trigger l1() {
        return l1_ ;
    }

    public Trigger l2() {
        return l2_ ;
    }

    public Trigger l3() {
        return l3_ ;
    }

    public Trigger l4() {
        return l4_ ;
    }

    public Trigger coralLeftRight() {
        return coral_left_right_ ;
    }

    public void rumble(Time duration) {
        end_time_ = Timer.getTimestamp() + duration.in(Seconds) ;
        rumbling_ = true ;
        gamepad_.setRumble(RumbleType.kBothRumble, 1.0);
    }

    public void flashDisplay() {
        ios_.flashLEDs();
        flashing_ = true ;
        flashing_timer_.start() ;
    }

    public void setLEDState(OILed led, LEDState st) {
        ios_.setLED(led.value, st) ;
    }    

    public void clearAllActionLEDs() {
        setLEDState(OILed.PlaceCoral, LEDState.Off) ;
        setLEDState(OILed.CollectCoral, LEDState.Off) ;
        setLEDState(OILed.CollectAlgaeReefEject, LEDState.Off) ;
        setLEDState(OILed.CollectAlgaeReefKeep, LEDState.Off) ;
        setLEDState(OILed.ScoreAlgae, LEDState.Off) ;
        setLEDState(OILed.Execute, LEDState.Off) ;
    }

    public void setRobotActionLEDState(RobotAction a, LEDState st) {
        switch(a) {
            case PlaceCoral:
                setLEDState(OILed.PlaceCoral, st) ;
                break ;

            case CollectCoral:
                setLEDState(OILed.CollectCoral, st) ;
                break ;

            case CollectAlgaeReefEject:
                setLEDState(OILed.CollectAlgaeReefEject, st) ;
                break ;

            case CollectAlgaeReefKeep:
                setLEDState(OILed.CollectAlgaeReefKeep, st) ;
                break ;

            case ScoreAlgae:
                setLEDState(OILed.ScoreAlgae, st) ;
                break ;
        }
    }

    int n = 0 ;
    boolean last = false ;

    @Override
    public void periodic() {
        ios_.updateInputs(inputs_) ;
        Logger.processInputs("OI", inputs_) ;

        if (flashing_ && flashing_timer_.isExpired()) {
            ios_.restoreLEDState() ;
            flashing_ = false ;
        }

        if (rumbling_ && Timer.getTimestamp() > end_time_) {
            gamepad_.setRumble(RumbleType.kBothRumble, 0);
            rumbling_ = false ;
        }

        Logger.recordOutput("oi/buttons", getPressedString()) ; 
    }

    public void setSideLED(CoralSide side) {
        if (side == CoralSide.Left) {
            setLEDState(OILed.CoralLeft, LEDState.On) ;
            setLEDState(OILed.CoralRight, LEDState.Off) ;
        }
        else {
            setLEDState(OILed.CoralLeft, LEDState.Off) ;
            setLEDState(OILed.CoralRight, LEDState.On) ;
        }
    }

    public void setLevelLED(ReefLevel level) {
        setLEDState(OILed.CoralL1, LEDState.Off) ;
        setLEDState(OILed.CoralL2, LEDState.Off) ;
        setLEDState(OILed.CoralL3, LEDState.Off) ;
        setLEDState(OILed.CoralL4, LEDState.Off) ;        

        switch(level) {
            case L1:
                setLEDState(OILed.CoralL1, LEDState.On) ;
                break ;

            case L2:
                setLEDState(OILed.CoralL2, LEDState.On) ;
                break ;

            case L3:
                setLEDState(OILed.CoralL3, LEDState.On) ;
                break ;

            case L4:
                setLEDState(OILed.CoralL4, LEDState.On) ;
                break ;

            default:
                // Should never happen
                setLEDState(OILed.CoralL1, LEDState.Slow) ;
                setLEDState(OILed.CoralL2, LEDState.Slow) ;
                setLEDState(OILed.CoralL3, LEDState.Slow) ;
                setLEDState(OILed.CoralL4, LEDState.Slow) ;   
                break ;
        }
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

        if (inputs_.algae_reef) {
            if (str.length() > 0)
                str += "," ;
            str += "algae_reef" ;
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

        if (inputs_.rotate_arm) {
            if (str.length() > 0)
                str += "," ;
            str += "rotate_arm" ;
        }

        if (gamepad_.a().getAsBoolean()) {
            if (str.length() > 0)
                str += "," ;
            str += "a" ;
        }

        if (gamepad_.b().getAsBoolean()) {
            if (str.length() > 0)
                str += "," ;
            str += "b" ;
        }

        if (gamepad_.x().getAsBoolean()) {
            if (str.length() > 0)
                str += "," ;
            str += "x" ;
        }

        if (gamepad_.y().getAsBoolean()) {
            if (str.length() > 0)
                str += "," ;
            str += "y" ;
        }

        if (gamepad_.start().getAsBoolean()) {
            if (str.length() > 0)
                str += "," ;
            str += "start" ;
        }

        if (gamepad_.back().getAsBoolean()) {
            if (str.length() > 0)
                str += "," ;
            str += "back" ;
        }

        if (gamepad_.leftStick().getAsBoolean()) {
            if (str.length() > 0)
                str += "," ;
            str += "ls" ;
        }

        if (gamepad_.rightStick().getAsBoolean()) {
            if (str.length() > 0)
                str += "," ;
            str += "rs" ;
        }

        if (gamepad_.leftBumper().getAsBoolean()) {
            if (str.length() > 0)
                str += "," ;
            str += "lb" ;
        }

        if (gamepad_.rightBumper().getAsBoolean()) {
            if (str.length() > 0)
                str += "," ;
            str += "rb" ;
        }

        return str ;
    }
}
