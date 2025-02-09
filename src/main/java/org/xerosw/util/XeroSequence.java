package org.xerosw.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class XeroSequence extends SequentialCommandGroup {
    static private List<XeroSequence> commands ;

    static private void commandFinished(Command cmd) {
        if (commands.contains(cmd)) {
            XeroSequence seq = (XeroSequence) cmd;
            if (seq != null) {
                seq.setComplete() ;
                commands.remove(cmd);
            }
        }
    }

    static {
        commands = new ArrayList<XeroSequence>();
        CommandScheduler.getInstance().onCommandFinish(XeroSequence::commandFinished);
    }

    private boolean complete_ ;

    public XeroSequence() {
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

