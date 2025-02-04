package frc.robot.commands.gps;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorGotoCmd;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.util.ReefUtil;
import frc.robot.util.ReefUtil.ReefFace;

public class PlaceCoralCmdBefore extends PlaceCoralCmd {

    public PlaceCoralCmdBefore(Drive db, ManipulatorSubsystem m, GrabberSubsystem g, int level, boolean left) {
        setName("PlaceCoralCmd") ;

        Optional<Alliance> a = DriverStation.getAlliance() ;
        if (a.isPresent()) {
            Optional<ReefFace> target = ReefUtil.getTargetedReefFace(db.getPose()) ;

            if (target.isPresent()) {
                addCommands(new ManipulatorGotoCmd(m, ElevatorPlaceHeight[level], ArmPlaceAngle[level])) ;
            }
        }
    }
}
