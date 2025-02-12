package frc.robot.subsystems.oi;

import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;
import org.xerosw.hid.XeroGamepad;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.brain.RobotAction;

public class OISubsystem extends SubsystemBase {
    
    //
    // The LEDs on the driver station
    //
    public enum OILed {

        CoralL1(1),                         //
        CoralL2(2),                         //
        CoralL3(3),                         //
        CoralL4(4),                         //
        ScoreAlgae(5),                      //
        CollectAlgaeGround(6),              //
        Spare1(7) ,                         // 
        HoldingAlgaeLow(8),                 //
        HoldingAlgaeHigh(9),                // 
        HoldingCoral(10),                   //
        Execute(11),                        //
        CoralLeft(12),                      //
        CoralRight(13),                     //
        CollectCoral(14),                   //
        PlaceCoral(15),                     //
        CollectAlgaeReef(16);               //

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
    private Trigger coral_place_trigger_ ;
    private Trigger coral_collect_trigger_ ;
    private Trigger algae_ground_trigger_ ;
    private Trigger algae_reef_ ;
    private Trigger algae_score_ ;
    private Trigger execute_trigger_ ;
    
    private Trigger l1_ ;
    private Trigger l2_ ;
    private Trigger l3_ ;
    private Trigger l4_ ;
    private Trigger coral_left_right_ ;

    public OISubsystem(OIIO ios, XeroGamepad ctrl) {
        this.ios_ = ios ;
        this.inputs_ = new OIIosInputsAutoLogged() ;
        gamepad_ = ctrl ;

        // Create the action triggers
        abort_trigger_ = new Trigger(() -> inputs_.abort) ;
        eject_trigger_ = new Trigger(() -> inputs_.eject) ;
        climb_lock_trigger_ = new Trigger(() -> inputs_.climb_lock) ;
        climb_exec_trigger_ = new Trigger(()-> inputs_.climb_execute) ;
        coral_place_trigger_  = new Trigger(()-> inputs_.coral_place) ;
        coral_collect_trigger_ = new Trigger(()-> inputs_.coral_collect) ;
        algae_ground_trigger_ = new Trigger(()-> inputs_.algae_ground) ;
        algae_reef_ = new Trigger(()-> inputs_.algae_reef) ;
        algae_score_ = new Trigger(()-> inputs_.algae_score) ;
        execute_trigger_ = new Trigger(()-> inputs_.execute) ;

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

    public Trigger coralPlace() {
        return coral_place_trigger_ ;
    }

    public Trigger coralCollect() {
        return coral_collect_trigger_ ;
    }

    public Trigger algaeGround() {
        return algae_ground_trigger_ ;
    }

    public Trigger algaeReef() {
        return algae_reef_ ;
    }

    public Trigger algaeScore() {
        return algae_score_ ;
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
        end_time_ = Timer.getFPGATimestamp() + duration.in(Seconds) ;
        rumbling_ = true ;
        gamepad_.setRumble(RumbleType.kBothRumble, 1.0);
    }

    public void setLEDState(OILed led, LEDState st) {
        ios_.setLED(led.value, st) ;
    }    

    public void clearAllActionLEDs() {
        setLEDState(OILed.PlaceCoral, LEDState.Off) ;
        setLEDState(OILed.CollectCoral, LEDState.Off) ;
        setLEDState(OILed.CollectAlgaeGround, LEDState.Off) ;
        setLEDState(OILed.CollectAlgaeReef, LEDState.Off) ;
        setLEDState(OILed.ScoreAlgae, LEDState.Off) ;
    }

    public void setRobotActionLEDState(RobotAction a, LEDState st) {
        switch(a) {
            case PlaceCoral:
                setLEDState(OILed.PlaceCoral, st) ;
                break ;

            case CollectCoral:
                setLEDState(OILed.CollectCoral, st) ;
                break ;

            case CollectAlgaeGround:
                setLEDState(OILed.CollectAlgaeGround, st) ;
                break ;

            case CollectAlgaeReef:
                setLEDState(OILed.CollectAlgaeReef, st) ;
                break ;

            case ScoreAlgae:
                setLEDState(OILed.ScoreAlgae, st) ;
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

    public void setLevelLED(int level) {
        setLEDState(OILed.CoralL1, LEDState.Off) ;
        setLEDState(OILed.CoralL2, LEDState.Off) ;
        setLEDState(OILed.CoralL3, LEDState.Off) ;
        setLEDState(OILed.CoralL4, LEDState.Off) ;        
        if (level == 1)
            setLEDState(OILed.CoralL1, LEDState.On) ;

        if (level == 2)
            setLEDState(OILed.CoralL2, LEDState.On) ;

        if (level == 3)
            setLEDState(OILed.CoralL3, LEDState.On) ;

        if (level == 4)
            setLEDState(OILed.CoralL4, LEDState.On) ;
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
