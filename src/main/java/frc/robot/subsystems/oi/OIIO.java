package frc.robot.subsystems.oi;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.oi.OIConstants.LEDState;

public interface OIIO {
    @AutoLog
    public static class OIIosInputs {
        public boolean eject = false ;
        public boolean abort = false ;
        public boolean execute = false ;
        public boolean coral_l1 = false ;
        public boolean coral_l2 = false ;
        public boolean coral_l3 = false ;
        public boolean coral_l4 = false ;                        
        public boolean coral_collect = false ;
        public boolean coral_place = false ;
        public boolean rotate_arm = false ;
        public boolean algae_ground = false ;
        public boolean algae_score = false ;
        public boolean algae_reef = false ;
        public boolean climb_deploy = false ;
        public boolean climb_execute = false ;
        public boolean climb_lock = false ;
        public boolean coral_side = false ;
        public boolean raise_arm = false ;
    }

    public default void updateInputs(OIIosInputs inputs) {
    }

    public default void setLED(int index, LEDState on) {
    }    

    public default void flashLEDs() {
    }

    public default void restoreLEDState() {
    }
}
