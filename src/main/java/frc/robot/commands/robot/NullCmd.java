package frc.robot.commands.robot;

import org.xerosw.util.XeroSequenceCmd;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class NullCmd extends XeroSequenceCmd {
    public NullCmd() {
        super();
    }

    @Override
    public void initSequence(SequentialCommandGroup seq) {
    }
}
