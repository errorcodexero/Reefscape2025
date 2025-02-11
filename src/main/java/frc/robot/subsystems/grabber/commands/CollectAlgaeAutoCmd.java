package frc.robot.subsystems.grabber.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Height;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class CollectAlgaeAutoCmd extends Command {

    private final GrabberSubsystem grabber_;
    private final ManipulatorSubsystem manipulator_;

    public CollectAlgaeAutoCmd(GrabberSubsystem grabber, ManipulatorSubsystem manipulator, Height height) {
        addRequirements(grabber, manipulator);

        grabber_ = grabber;
        manipulator_ = manipulator;
    }

    @Override
    public void initialize() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
