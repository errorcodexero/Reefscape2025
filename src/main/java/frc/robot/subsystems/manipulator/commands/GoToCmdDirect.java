package frc.robot.subsystems.manipulator.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class GoToCmdDirect extends Command {
    private ManipulatorSubsystem m_ ;

    private Distance targetElevPos_ ;
    private Distance elev_pos_tolerance_ ;
    private LinearVelocity elev_vel_tolerance_ ;

    private Angle targetArmPos_ ;
    private Angle arm_pos_tolerance_ ;
    private AngularVelocity arm_vel_tolerance_ ;

    public GoToCmdDirect(ManipulatorSubsystem m, Distance targetElevPos, Angle targetArmPos) {
        m_ = m ;
        targetElevPos_ = targetElevPos ;
        targetArmPos_ = targetArmPos ;
    }

    public GoToCmdDirect(ManipulatorSubsystem m, 
                            Distance targetElevPos, Distance elevPosTol, LinearVelocity elevVelTol,
                            Angle targetArmPos, Angle armPosTol, AngularVelocity armVelTol) {
        m_ = m ;

        targetElevPos_ = targetElevPos ;
        elev_pos_tolerance_ = elevPosTol ;
        elev_vel_tolerance_ = elevVelTol ;

        targetArmPos_ = targetArmPos ;
        arm_pos_tolerance_ = armPosTol ;
        arm_vel_tolerance_ = armVelTol ;
    }

    @Override
    public void initialize() {
        m_.setElevatorTarget(targetElevPos_, elev_pos_tolerance_, elev_vel_tolerance_) ;
        m_.setArmTarget(targetArmPos_, arm_pos_tolerance_, arm_vel_tolerance_) ;
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
