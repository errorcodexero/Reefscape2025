package frc.robot.subsystems.oi;

public class OIConstants {
    public static final int kOIPort = 2 ;
    public static final int kGamepadPort = 0 ;

    public static class Buttons {
        public static final int kEject = 19 ;                 // Action
        public static final int kAbort = 18 ;                 // Action
        public static final int kExecute = 11 ;              // Action
        public static final int kCoralL1 = 1 ;              // Status
        public static final int kCoralL2 = 2 ;              // Status
        public static final int kCoralL3 = 3 ;              // Status
        public static final int kCoralL4 = 4 ;              // Status
        public static final int kCoralCollect = 14 ;         // Action
        public static final int kCoralPlace = 15 ;           // Action

        public static final int kAlgaeScore = 8 ;          // Action
        public static final int kAlgaeReef = 9 ;           // Action
        public static final int kClimbDeploy =  5;         // Action
        public static final int kClimbExecute = 10 ;        // Action
        public static final int kClimbLock =  16 ;           // Status
        public static final int kCoralSide = 17 ;           // Status

        public static final int kAlgaeOnReef = 7 ;          // Toggle, if lit, algae is on reef

        public static final int kSpare0 = 13 ;              // Spare
        public static final int kSpare1 = 12 ;              // Spare
        public static final int kSpare2 = 6 ;              // Spare
    }

    public enum OILed {
        CoralL1(1),                         //
        CoralL2(2),                         //
        CoralL3(3),                         //
        CoralL4(4),                         //
        ScoreAlgae(5),                      //
        CollectAlgaeGround(6),              //
        AlgaeOnReef(7) ,                    //
        ReadyToClimb(8),                    //
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
}
