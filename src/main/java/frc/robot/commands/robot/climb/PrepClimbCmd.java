package frc.robot.commands.robot.climb;

import java.util.Optional;

import org.xerosw.util.XeroSequenceCmd;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.climber.ClimberPositionCmd;
import frc.robot.subsystems.climber.ClimberState;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.funnel.FunnelSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.manipulator.commands.GoToCmd;
import frc.robot.subsystems.funnel.DeployFunnelCmd ;

public class PrepClimbCmd extends XeroSequenceCmd {
    private ClimberSubsystem climber_ ;
    private FunnelSubsystem funnel_ ;
    private ManipulatorSubsystem m_ ;
    private Rotation2d angle_ ;

    public PrepClimbCmd(Drive drive, ClimberSubsystem climber, FunnelSubsystem funnel, ManipulatorSubsystem manipulator) {
        super("PrepClimbCmd");

        climber_ = climber ;
        funnel_ = funnel ;
        m_ = manipulator ;
    }

    private Rotation2d getDesiredAngle() {
        if (angle_ == null) {
            Optional<Alliance> alliance = DriverStation.getAlliance() ;
            if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                angle_ = Rotation2d.fromDegrees(90.0) ;
            }
            else {
                angle_ = Rotation2d.fromDegrees(-90.0) ;
            }
        }
        return angle_ ;
    }

    @Override
    public void initSequence(SequentialCommandGroup sequence) {
        sequence.addCommands(Commands.parallel(
            DriveCommands.joystickDriveAtAngle(this::getDesiredAngle),
            new DeployFunnelCmd(funnel_, DeployFunnelCmd.Position.Climb),
            new GoToCmd(m_, ManipulatorConstants.Elevator.Positions.kStow, ManipulatorConstants.Arm.Positions.kClimb),
            new ClimberPositionCmd(climber_, ClimberState.PrepareToClimb)
        ));
    }
}
