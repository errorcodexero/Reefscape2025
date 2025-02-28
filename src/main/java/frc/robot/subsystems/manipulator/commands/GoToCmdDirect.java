package frc.robot.subsystems.manipulator.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class GoToCmdDirect extends Command {
    private ManipulatorSubsystem m_ ;
    private Distance targetElevPos_ ;
    private Angle targetArmPos_ ;

    public GoToCmdDirect(ManipulatorSubsystem m, Distance targetElevPos, Angle targetArmPos) {
        m_ = m ;
        targetElevPos_ = targetElevPos ;
        targetArmPos_ = targetArmPos ;
    }

    @Override
    public void initialize() {
        m_.setElevatorTarget(targetElevPos_);
        m_.setArmTarget(targetArmPos_);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return m_.isArmAtTarget() && m_.isElevAtTarget() ;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            m_.setArmTarget(m_.getArmPosition()) ;
            m_.setElevatorTarget(m_.getElevatorPosition()) ;
        }
    }
}
