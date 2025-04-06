package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class RotateRobotCmd extends Command {
    static final double kP = 0.1 ;
    static final double kI = 0.0 ;
    static final double kD = 0.0 ;

    private Drive db_ ;
    private Rotation2d target_ ;
    private boolean done_ ;
    private PIDController pid_ ;
    
    public RotateRobotCmd(Drive db, Angle angle) {
        db_ = db;
        addRequirements(db_);

        target_ = Rotation2d.fromDegrees(angle.in(Degrees)) ;
    }

    public RotateRobotCmd(Drive db, Rotation2d angle) {
        db_ = db;
        addRequirements(db_);

        target_ = angle ;
    }

    @Override
    public void initialize() {
        done_ = false ;
        pid_ = new PIDController(kP, kI, kD);
        pid_.enableContinuousInput(-180.0, 180.0);
        pid_.setTolerance(3);

        pid_.setSetpoint(target_.getDegrees());
    }

    @Override
    public void execute() {
        double output = pid_.calculate(db_.getPose().getRotation().getDegrees());
        db_.runVelocity(new ChassisSpeeds(0.0, 0.0, output)) ;
        done_ = pid_.atSetpoint() ;
    }

    @Override
    public void end(boolean interrupted) {
        db_.stop();
    }

    @Override
    public boolean isFinished() {
        return done_ ;
    }
}
