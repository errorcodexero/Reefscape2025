package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;
import org.xerosw.SampleAverager;

import edu.wpi.first.wpilibj2.command.Command;

public class CalibrateCmd extends Command {
    private ManipulatorSubsystem m_ ;
    private boolean elev_calibrated_ ;
    private SampleAverager avg_ ;
    private boolean wait_reset_ ;

    public CalibrateCmd(ManipulatorSubsystem manipulatorSubsystem) {
        addRequirements(manipulatorSubsystem);
        m_ = manipulatorSubsystem;
        avg_ = new SampleAverager(16) ;
        elev_calibrated_ = false ;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (!m_.isElevatorCalibrated() || m_.getElevatorPosition().lte(Centimeters.of(4.0))) {
            m_.enableSoftLimits(false);
            m_.setElevatorVoltage(ManipulatorConstants.Elevator.kCalibrateVoltage);
            elev_calibrated_ = false ;
            wait_reset_ = false ;
            avg_.reset() ;
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        boolean running = false ;
        if (!elev_calibrated_) {
            running = true ;
            if (wait_reset_) {
                m_.setElevatorVoltage(Volts.of(0.0)) ;
                elev_calibrated_ = true ;
            }
            else {
                avg_.addSample(m_.getElevatorPosition().in(Centimeters)) ;
                if (avg_.isComplete() && avg_.maxDeviationFromAverage() < 0.1) {
                        m_.resetElevator();
                        wait_reset_ = true ;
                }
            }
        }

        Logger.recordOutput("Manipulator/calibraterunning", running) ;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false ;
    }
}
