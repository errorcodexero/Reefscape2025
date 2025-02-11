package frc.robot.commands.robot;

import org.xerosw.util.XeroSequence;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CollectAlgaeReefPt1Cmd extends SequentialCommandGroup {
    static private List<CollectAlgaeReefPt1Cmd> commands ;

    static private void commandFinished(Command cmd) {
        if (commands.contains(cmd)) {
            CollectAlgaeReefPt1Cmd seq = (CollectAlgaeReefPt1Cmd) cmd;
            if (seq != null) {
                seq.setComplete() ;
                commands.remove(cmd);
            }
        }
    }

    static {
        commands = new ArrayList<CollectAlgaeReefPt1Cmd>();
        CommandScheduler.getInstance().onCommandFinish(CollectAlgaeReefPt1Cmd::commandFinished);
    }

    private boolean complete_ ;

    public CollectAlgaeReefPt1Cmd() {
        super();
        complete_ = false ;
        commands.add(this);
    }

    public boolean isComplete() {
        return complete_ ;
    }

    private void setComplete() {
        complete_ = true ;
    }
}
