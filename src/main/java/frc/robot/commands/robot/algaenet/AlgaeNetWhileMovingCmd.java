package frc.robot.commands.robot.algaenet;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import org.littletonrobotics.junction.Logger;
import org.xerosw.math.XeroMath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.drive.RotateRobotCmd;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.manipulator.commands.GoToCmd;
import frc.robot.util.ReefUtil;

public class AlgaeNetWhileMovingCmd extends Command {
    static LinearVelocity kMaxVel = MetersPerSecond.of(2.0) ;
    static LinearAcceleration kMaxAcc = MetersPerSecondPerSecond.of(2.0) ;
    static Distance kShootDistance = Centimeters.of(130) ;

    private enum State {
        Idle,
        Rotating,
        Driving,
        DrivingAndShooting,
        Stopping,
        Done
    }

    private Drive db_ ;

    private Pose2d target_ ;
    private State state_ = State.Idle ;

    private Command rotate_cmd_ ;
    private Command drive_cmd_ ;
    private Command shoot_cmd_ ;
    private Command stop_cmd_ ;
    private Command down_cmd_ ;

    public AlgaeNetWhileMovingCmd(BrainSubsystem b, Drive db, ManipulatorSubsystem m, GrabberSubsystem g) {
        db_ = db ;

        state_ = State.Idle ;
        shoot_cmd_ = new AlgaeNetCmd(b, m, g);
        stop_cmd_ = db.stopCmd() ;
        down_cmd_ = new GoToCmd(m, ManipulatorConstants.Elevator.Positions.kStow, ManipulatorConstants.Arm.Positions.kStow) ;
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
        if (dist < 1.5 || dist > 4.0) {
            state_ = State.Done ;
            return ;
        }

        double angle = XeroMath.normalizeAngleDegrees(target_.getRotation().getDegrees() - db_.getPose().getRotation().getDegrees()) ;
        if (Math.abs(angle) > 30.0) {
            state_ = State.Done ;
            return ;
        }

        RobotContainer.getInstance().gamepad().setLocked(true);
        state_ = State.Rotating ;
        rotate_cmd_ = new RotateRobotCmd(db_, target_.getRotation()) ;
        rotate_cmd_.initialize() ;
    }

    @Override
    public void execute() {
        Logger.recordOutput("algae/state", state_.toString()) ;

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
                    RobotContainer.getInstance().gamepad().setLocked(false);
                    state_ = State.Stopping ;
                    stop_cmd_.initialize();
                    down_cmd_.initialize() ;
                }
                break ;

            case Stopping:
                stop_cmd_.execute() ;
                down_cmd_.execute() ;
                if (stop_cmd_.isFinished() && down_cmd_.isFinished()) {
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
