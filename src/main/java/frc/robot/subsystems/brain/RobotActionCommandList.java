package frc.robot.subsystems.brain;

import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class RobotActionCommandList {
    public RobotAction action ;
    public List<Command> commands ;
    public List<BooleanSupplier> conditions ;

    public RobotActionCommandList(RobotAction action, List<Command> commands, List<BooleanSupplier> conditions) {
        this.action = action ;
        this.commands = commands ;
        this.conditions = conditions ;
    }

    public int size() {
        return this.commands.size() ;
    }

    public Command getCommand(int index) {
        return this.commands.get(index) ;
    }

    public BooleanSupplier getCondition(int index) {
        return this.conditions.get(index) ;
    }
}
