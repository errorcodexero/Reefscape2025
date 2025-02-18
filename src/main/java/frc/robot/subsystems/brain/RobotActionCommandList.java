package frc.robot.subsystems.brain;

import java.util.List;
import java.util.function.BooleanSupplier;

import org.xerosw.util.XeroSequenceCmd;

public class RobotActionCommandList {
    public RobotAction action ;
    public List<XeroSequenceCmd> commands ;
    public List<BooleanSupplier> conditions ;

    public RobotActionCommandList(RobotAction action, List<XeroSequenceCmd> commands, List<BooleanSupplier> conditions) {
        this.action = action ;
        this.commands = commands ;
        this.conditions = conditions ;
    }

    public int size() {
        return this.commands.size() ;
    }

    public XeroSequenceCmd getCommand(int index) {
        return this.commands.get(index) ;
    }

    public BooleanSupplier getCondition(int index) {
        return this.conditions.get(index) ;
    }
}
