package frc.robot.commands.robot.algaenet;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import org.xerosw.math.XeroMath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.drive.RotateRobotCmd;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.util.ReefUtil;

public class AlgaeNetWhileMovingCmd extends Command {
    static LinearVelocity kMaxVel = MetersPerSecond.of(2.0) ;
    static LinearAcceleration kMaxAcc = MetersPerSecondPerSecond.of(2.0) ;
    static Distance kShootDistance = Centimeters.of(60) ;

    private enum State {
        Idle,
        Rotating,
        Driving,
        DrivingAndShooting,
        Stopping,
        Done
    }

    private BrainSubsystem b_ ;
    private Drive db_ ;
    private ManipulatorSubsystem m_ ;
    private GrabberSubsystem g_ ;

    private Pose2d target_ ;
    private State state_ = State.Idle ;

    private Command rotate_cmd_ ;
    private Command drive_cmd_ ;
    private Command shoot_cmd_ ;
    private Command stop_cmd_ ;

    public AlgaeNetWhileMovingCmd(BrainSubsystem b, Drive db, ManipulatorSubsystem m, GrabberSubsystem g) {
        b_ = b ;
        db_ = db ;
        m_ = m ;
        g_ = g ;

        state_ = State.Idle ;
        shoot_cmd_ = new AlgaeNetCmd(b, m, g);
        stop_cmd_ = db.stopCmd() ;
    }

    @Override
    public void initialize() {
        target_ = ReefUtil.getBargeScorePose(db_.getPose(), Constants.BargeConstants.distanceFromBargeTagWhileMoving) ;



        //
        // Make sure we are within the correct distance and angle to the target.  We need to be at least 2 meters away from 
        // the target loation under the barge so there is time to be at at steady velocity.  We don't want to be more than 4 meters
        // away because the we want to be sure the path is clear and remains clear.  We want the rotation to be within 30 degrees
        // so we are at a reasonable angle to the target when we shoot.
        //
        double dist = target_.getTranslation().getDistance(db_.getPose().getTranslation()) ;
        if (dist < 2.0 || dist > 4.0) {
            state_ = State.Done ;
            return ;
        }

        double angle = XeroMath.normalizeAngleDegrees(target_.getRotation().getDegrees() - db_.getPose().getRotation().getDegrees()) ;
        if (Math.abs(angle) > 30.0) {
            state_ = State.Done ;
            return ;
        }

        state_ = State.Rotating ;
        rotate_cmd_ = new RotateRobotCmd(db_, target_.getRotation()) ;
        rotate_cmd_.initialize() ;

        state_ = State.Driving ;

    }

    @Override
    public void execute() {
        Distance dist = Meters.of(target_.getTranslation().getDistance(db_.getPose().getTranslation())) ;
        switch(state_) {
            case Rotating:
                rotate_cmd_.execute() ;
                if (rotate_cmd_.isFinished()) {
                    state_ = State.Driving ;
                    drive_cmd_ = DriveCommands.simplePathCommand(db_, target_, kMaxVel, kMaxAcc) ;
                    drive_cmd_.initialize() ;
                }
                break ;

            case Driving:
                drive_cmd_.execute() ;

                if (dist.lte(kShootDistance)) {
                    state_ = State.DrivingAndShooting ;
                    shoot_cmd_.initialize() ;
                }
                break ;

            case DrivingAndShooting:
                drive_cmd_.execute() ;
                shoot_cmd_.execute() ;
                if (shoot_cmd_.isFinished()) {
                    state_ = State.Stopping ;
                    stop_cmd_.initialize();
                }
                break ;

            case Stopping:
                stop_cmd_.execute() ;
                if (stop_cmd_.isFinished()) {
                    state_ = State.Done ;
                }
                break ;

            case Idle:
            case Done:
                break ;
        }
    }

    @Override
    public boolean isFinished() {
        return state_ == State.Done ;
    }
}
