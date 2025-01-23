package frc.robot.subsystems.leds;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BlinkyLights extends SubsystemBase {

    private static final LightPattern idlePattern_ = LightPattern.FLAME;
    
    private final DigitalOutput[] outputs_;
    private final int numBits_;
    
    // The currently attempted pattern from the rest of the code (null is default fallback)
    @AutoLogOutput
    private Optional<LightPattern> attemptedPattern_ = Optional.empty();
    
    // The most recently sent pattern to the arduino.
    @AutoLogOutput
    private LightPattern currentPattern_ = null;

    private int numRobotLoops_;
    
    public static enum LightPattern {
        LOADING(0),
        FLAME(1),
        ERROR(2);

        private int id;

        private LightPattern(int id) {
            this.id = id;
        }

        public int getId() {
            return this.id;
        }
    }

    public BlinkyLights(int... channels) {
        // Initialize Arrays
        numBits_ = channels.length;
        outputs_ = new DigitalOutput[numBits_];

        // Initialze Outputs
        for (int i = 0; i < numBits_; i++) {
            outputs_[i] = new DigitalOutput(channels[i]);
        }

        sendPattern(LightPattern.LOADING);
    }

    @Override
    public void periodic() {
        if (++numRobotLoops_ < 10) { // Wait for ten robot loops to exit the loading sequence.
            return;
        }

        if (DriverStation.isEStopped()) {
            sendPattern(LightPattern.ERROR);
        } else {
            if (currentPattern_ != attemptedPattern_.orElse(null))
            sendPattern(attemptedPattern_.orElse(idlePattern_));
        }
    }
    
    /**
     * This will attempt to put a pattern onto the led lights.
     * I say "attempt" because it will do nothing until something of more importance subsides. (e.g. an ESTOP)
     * @param pattern
     */
    public void attemptPattern(LightPattern attemptedPattern) {
        attemptedPattern_ = Optional.of(attemptedPattern);
    }

    /**
     * This creates a command that attempts a pattern.
     */
    public Command attemptPatternCmd(LightPattern attemptedPattern) {
        return runOnce(() -> attemptPattern(attemptedPattern));
    }

    /**
     * This gives control of the led pattern back to this subsystem.
     */
    public void defaultPattern() {
        attemptedPattern_ = Optional.empty();
    }

    /**
     * Creates a command to give control of the led pattern back to this subsystem.
     * @param attemptedPattern
     * @return
     */
    public Command defaultPatternCmd() {
        return runOnce(this::defaultPattern);
    }

    private void sendPattern(LightPattern pattern) {
        for (int i = 0; i < numBits_; i++) {
            outputs_[i].set((pattern.getId() & (1 << i)) != 0);
        }
        
        // Update Current Pattern
        currentPattern_ = pattern;
    }

}
