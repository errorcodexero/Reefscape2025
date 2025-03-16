package frc.robot.commands.robot.algaenet;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.drive.DriveCommands;
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
        double dist = target_.getTranslation().getDistance(db_.getPose().getTranslation()) ;
        if (dist < 2.0 || dist > 4.0) {
            state_ = State.Done ;
            return ;
        }

        state_ = State.Driving ;
        drive_cmd_ = DriveCommands.simplePathCommand(db_, target_, kMaxVel, kMaxAcc) ;
    }

    @Override
    public void execute() {
        drive_cmd_.execute() ;

        switch(state_) {
            case Driving:
                {
                    Distance dist = Meters.of(target_.getTranslation().getDistance(db_.getPose().getTranslation())) ;
                    if (dist.lte(kShootDistance)) {
                        state_ = State.DrivingAndShooting ;
                        shoot_cmd_.initialize() ;
                    }
                }
                break ;

            case DrivingAndShooting:
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
