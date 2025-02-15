package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.Command;

public class EnableDisableCmd extends Command {
    private final AprilTagVision visionSubsystem;
    private final boolean enable;

    public EnableDisableCmd(AprilTagVision visionSubsystem, boolean enable) {
        this.visionSubsystem = visionSubsystem;
        this.enable = enable;
        addRequirements(visionSubsystem);
    }

    @Override
    public void initialize() {
        if (enable) {
            visionSubsystem.enable();
        } else {
            visionSubsystem.disable();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
