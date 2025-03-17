package frc.robot.subsystems.manipulator.commands;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class GoToWhenClose extends Command {
    private ManipulatorSubsystem m_ ;

    private Distance targetElevPos_ ;
    private Distance elev_pos_tolerance_ ;
    private LinearVelocity elev_vel_tolerance_ ;

    private Angle targetArmPos_ ;
    private Angle arm_pos_tolerance_ ;
    private AngularVelocity arm_vel_tolerance_ ;


    private Drive db_ ;
    private Pose2d target_ ;
    private Distance range_ ;
    private boolean started_ ;

    public GoToWhenClose(Drive db, ManipulatorSubsystem m, 
                            Distance targetElevPos, Distance elevPosTol, LinearVelocity elevVelTol,
                            Angle targetArmPos, Angle armPosTol, AngularVelocity armVelTol,
                            Pose2d target, Distance d) {
        m_ = m ;

        targetElevPos_ = targetElevPos ;
        elev_pos_tolerance_ = elevPosTol ;
        elev_vel_tolerance_ = elevVelTol ;

        targetArmPos_ = targetArmPos ;
        arm_pos_tolerance_ = armPosTol ;
        arm_vel_tolerance_ = armVelTol ;

        db_ = db ;
        target_ = target ;
        range_ = d ;
    }

    @Override
    public void initialize() {
        started_ = false ;
    }
    
    @Override
    public void execute() {
        Distance actual = Meters.of(db_.getPose().getTranslation().getDistance(target_.getTranslation())) ;
        Logger.recordOutput("place/dist", actual) ;
        Logger.recordOutput("place/started", started_) ;
        Logger.recordOutput("place/range", range_) ;
        if (!started_ && actual.lt(range_)) {
            m_.setElevatorTarget(targetElevPos_, elev_pos_tolerance_, elev_vel_tolerance_) ;
            m_.setArmTarget(targetArmPos_, arm_pos_tolerance_, arm_vel_tolerance_) ;
            started_ = true ;
        }
    }

    @Override
    public boolean isFinished() {
        return started_ && m_.isArmAtTarget() && m_.isElevAtTarget() ;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            m_.setArmTarget(m_.getArmPosition()) ;
            m_.setElevatorTarget(m_.getElevatorPosition()) ;
        }
    }
}
