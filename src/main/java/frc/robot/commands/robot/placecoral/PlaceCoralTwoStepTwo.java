package frc.robot.commands.robot.placecoral;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.GamepadEnabled;
import frc.robot.commands.misc.RumbleGamepadCmd;
import frc.robot.commands.robot.ReportStateCmd;
import frc.robot.commands.robot.SetHoldingCmd;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.DepositCoralCmd;
import frc.robot.subsystems.manipulator.GoToCmd;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.oi.CoralSide;
import frc.robot.util.ReefUtil;
import frc.robot.util.ReefUtil.ReefFace;

public class PlaceCoralTwoStepTwo extends Command {

    private static Voltage nominal = Volts.of(12.0) ;

    private SequentialCommandGroup sequence_ ;
    private BrainSubsystem b_ ;
    private Drive db_ ;
    private ManipulatorSubsystem m_ ;
    private GrabberSubsystem g_ ;

    public PlaceCoralTwoStepTwo(BrainSubsystem brain, Drive db, ManipulatorSubsystem m, GrabberSubsystem g) {
        setName("PlaceCoralAfterCmd") ;
        db_ = db ;
        b_ = brain ;
        m_ = m ;
        g_ = g ;
        sequence_ = new SequentialCommandGroup() ;
    }

    @Override
    public void initialize() {
        Optional<Alliance> a = DriverStation.getAlliance() ;
        if (!a.isPresent())
            return ;

        Optional<ReefFace> target = ReefUtil.getTargetedReefFace(db_.getPose()) ;
        if (!target.isPresent()) {
            return ;
        }

        ReefFace t = target.get() ;
        Pose2d place = (b_.coralSide() == CoralSide.Left) ? t.getLeftScoringPose() : t.getRightScoringPose() ;
        
        PathConstraints driveto_constraints = new PathConstraints(PlaceCoralConstants.DriveToMaxVelocity, 
                                                                PlaceCoralConstants.DriveToMaxAcceleration, 
                                                                PlaceCoralConstants.DriveToMaxAngularVelocity, 
                                                                PlaceCoralConstants.DriveToMaxAngularAcceleration, nominal, false) ;

        sequence_.addCommands(

            // Turn off gamepad so driver cannot drive
            new ReportStateCmd(getName(), "gp-disabled"),
            new GamepadEnabled(false),

            // Drive to the scoring pose
            new ReportStateCmd(getName(), "driveto"),
            AutoBuilder.pathfindToPose(place, driveto_constraints),

            // Go to the elevator height
            new ReportStateCmd(getName(), "goto-elev"),
            new GoToCmd(m_, PlaceCoralConstants.Place.ElevatorHeight[b_.coralLevel()], ManipulatorConstants.Positions.kStowedAngle),

            // Go to the arm angle
            new ReportStateCmd(getName(), "goto-arm"),
            new GoToCmd(m_, PlaceCoralConstants.Place.ElevatorHeight[b_.coralLevel()], PlaceCoralConstants.Place.ArmAngle[b_.coralLevel()]),

            // Place the coral on the reef
            new ReportStateCmd(getName(), "deposit-coral"),
            new DepositCoralCmd(g_),

            // Signal we are no longer holding coral
            new SetHoldingCmd(b_, RobotContainer.GamePiece.NONE),
            new ReportStateCmd(getName(), "backup"),

            // Bring the arm back into the robot
            new ReportStateCmd(getName(), "goto-elev"),
            new GoToCmd(m_, PlaceCoralConstants.Place.ElevatorHeight[b_.coralLevel()], ManipulatorConstants.Positions.kStowedAngle),
            
            // Lower the elevator
            new ReportStateCmd(getName(), "goto-arm"),
            new GoToCmd(m_, ManipulatorConstants.Positions.kStowedHeight, ManipulatorConstants.Positions.kStowedAngle),

            // Turn the gamepad back on
            new ReportStateCmd(getName(), "gamepad-enabled"),
            new GamepadEnabled(true),

            // Signal we are done
            new ReportStateCmd(getName(), "rumble"),
            new RumbleGamepadCmd(Milliseconds.of(500))) ;

        sequence_.schedule();
    }

    @Override
    public boolean isFinished() {
        return sequence_.isFinished() ;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            sequence_.cancel() ;
        }
    }
}
