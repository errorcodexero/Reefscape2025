package frc.robot.commands.auto;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoModeBaseCmd extends SequentialCommandGroup {
    private Pose2d pose_ ;

    public AutoModeBaseCmd() {
        pose_ = new Pose2d() ;   
    }

    public AutoModeBaseCmd(PathPlannerPath p) {
        if (p.getStartingHolonomicPose().isPresent()) {
            pose_ = p.getStartingHolonomicPose().get() ;
        }
    }

    public AutoModeBaseCmd(Pose2d p) {
        pose_ = p ;
    }

    public Pose2d getStartingPose() {
        return pose_ ;
    }
}
