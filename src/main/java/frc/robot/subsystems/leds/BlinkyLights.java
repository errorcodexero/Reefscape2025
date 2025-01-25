package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class BlinkyLights {

    private final DigitalOutput[] outputs_;
    private final int numBits_;

    public static enum LightPattern
    {
        FREEZE(0),
        FLAME(1),
        CAGE(2),
        GAMEPIECE(3),
        COLLECT(4);

        private int id;

        private LightPattern(int id)
        {
            this.id = id;
        }

        public int getId()
        {
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

        setPattern(LightPattern.FREEZE);
    }

    public void setPattern(LightPattern pattern) {
        for (int i = 0; i < numBits_; i++) {
            outputs_[i].set((pattern.getId() & (1 << i)) != 0);
        }
    }

    public void setPattern(int bits) {
        for (int i = 0; i < numBits_; i++) {
            outputs_[i].set((bits & (1 << i)) != 0);
        }
    }
}
