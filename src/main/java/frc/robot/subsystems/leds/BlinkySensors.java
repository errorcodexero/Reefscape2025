package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.DigitalInput;

public class BlinkySensors {

    private final DigitalInput[] inputs_;
    private final int numBits_;

    public BlinkySensors(int... channels) {
        // Initialize Arrays
        numBits_ = channels.length;
        inputs_ = new DigitalInput[numBits_];

        // Initialze Inputs
        for (int i = 0; i < numBits_; i++) {
            inputs_[i] = new DigitalInput(channels[i]);
        }
    }

    public int getPattern() {
        int id = 0;
        for (int i = 0; i < numBits_; i++) {
            if (! inputs_[i].get()) {
                id |= (1 << i);
            }
        }
        return id;
    }
}
