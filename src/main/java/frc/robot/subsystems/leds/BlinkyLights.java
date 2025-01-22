package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class BlinkyLights {

    private final DigitalOutput[] outputs_;
    private final int numBits_;

    public static enum LightPattern {
        IDLE((byte) 0),
        ERROR((byte) 1),
        CLIMB((byte) 2);

        private byte id;

        private LightPattern(byte id) {
            this.id = id;
        }

        public byte getId() {
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

        setPattern(LightPattern.IDLE);

    }

    public void setPattern(LightPattern pattern) {
        Logger.recordOutput("Leds/CurrentPattern", pattern);

        for (int i = 0; i < numBits_; i++) {
            outputs_[i].set((pattern.getId() & (1 << i)) != 0);
        }
    }

    public Command setPatternCmd(LightPattern pattern) {
        return Commands.runOnce(() -> {
            setPattern(pattern);
        });
    }
}
