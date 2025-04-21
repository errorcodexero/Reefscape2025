package frc.robot.commands.misc;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class StateCmd extends Command {
    static Robot theRobot ;

    public static void setRobot(Robot robot) {
        theRobot = robot;
    }

    private String key_ ;
    private String value_ ;

    public StateCmd(String key, String value) {
        key_ = key ;
        value_ = value ;
    }

    @Override
    public void initialize() {
        theRobot.setNamedState(key_, value_);
    }

    @Override
    public boolean isFinished() {
        return true ;
    }
}
