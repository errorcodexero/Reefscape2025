package frc.robot.subsystems.oi;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.subsystems.oi.OISubsystem.LEDState;

public class OIIOHID implements OIIO {
    private static final int kMaxLeds = 32 ;
    private static final int kFastLoopCount = 10 ;
    private static final int kSlowLoopCount = 25 ;
        
    private GenericHID hid_ ;    

    private boolean led_onoff_[];
    private OISubsystem.LEDState led_state_[] ;

    private boolean fast_on_off_ ;
    private int fast_on_off_loops_ ;
    private boolean slow_on_off_ ;
    private int slow_on_off_loops_ ;   

    public OIIOHID(int port) {
         hid_ = new GenericHID(port) ;

        led_onoff_ = new boolean[kMaxLeds] ;
        led_state_ = new OISubsystem.LEDState[kMaxLeds] ;
        for(int i = 0 ; i < kMaxLeds ; i++) {
            led_state_[i] = LEDState.Off ;
        }

        fast_on_off_ = false ;
        fast_on_off_loops_ = 0 ;
        slow_on_off_ = false ;
        slow_on_off_loops_ = 0 ;           
    }

    @Override
    public void updateInputs(OIIosInputs inputs) {
        if (hid_ != null) {
            inputs.eject = hid_.getRawButton(OIConstants.Buttons.kEject) ;
            inputs.abort = hid_.getRawButton(OIConstants.Buttons.kAbort) ;
            inputs.execute = hid_.getRawButton(OIConstants.Buttons.kExecute) ;
            inputs.coral_l1 = hid_.getRawButton(OIConstants.Buttons.kCoralL1) ;
            inputs.coral_l2 = hid_.getRawButton(OIConstants.Buttons.kCoralL2) ;
            inputs.coral_l3 = hid_.getRawButton(OIConstants.Buttons.kCoralL3) ;
            inputs.coral_l4 = hid_.getRawButton(OIConstants.Buttons.kCoralL4) ;
            inputs.coral_collect = hid_.getRawButton(OIConstants.Buttons.kCoralCollect) ;
            inputs.coral_place = hid_.getRawButton(OIConstants.Buttons.kCoralPlace) ;
            inputs.algae_ground = hid_.getRawButton(OIConstants.Buttons.kAlgaeGround) ;
            inputs.algae_score = hid_.getRawButton(OIConstants.Buttons.kAlgaeScore) ;
            inputs.algae_collect_l2 = hid_.getRawButton(OIConstants.Buttons.kAlgaeCollectL2) ;
            inputs.algae_collect_l3 = hid_.getRawButton(OIConstants.Buttons.kAlgaeCollectL3) ;
            inputs.climb_deploy = hid_.getRawButton(OIConstants.Buttons.kClimbDeploy) ;
            inputs.climb_execute = hid_.getRawButton(OIConstants.Buttons.kClimbExecute) ;
            inputs.climb_lock = hid_.getRawButton(OIConstants.Buttons.kClimbLock) ;
            inputs.coral_side = hid_.getRawButton(OIConstants.Buttons.kCoralSide) ;
        }

        updateLEDs() ;
    }

    @Override
    public void setLED(int index, LEDState st) {
        led_state_[index] = st ;        
    }

    public void updateLEDs() {
        fast_on_off_loops_++ ;
        if (fast_on_off_loops_ == kFastLoopCount) {
            fast_on_off_ = !fast_on_off_ ;
            fast_on_off_loops_ = 0 ;
        }

        slow_on_off_loops_++ ;
        if (slow_on_off_loops_ == kSlowLoopCount) {
            slow_on_off_ = !slow_on_off_ ;
            slow_on_off_loops_ = 0 ;
        }

        for(int i = 1 ; i <= kMaxLeds ; i++) {
            boolean desired = false ;

            switch(led_state_[i - 1]) {
                case On:
                    desired = true ;
                    break ;
                case Off:
                    desired = false ;
                    break ;
                case Slow:
                    desired = slow_on_off_ ;
                    break ;
                case Fast:
                    desired = fast_on_off_ ;
                    break ;
            }

            if (desired != led_onoff_[i - 1]) {
                hid_.setOutput(i, desired);
                led_onoff_[i - 1] = desired ;
            }
        }
    }
}
