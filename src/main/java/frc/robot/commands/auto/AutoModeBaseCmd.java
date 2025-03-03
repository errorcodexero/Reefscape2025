package frc.robot.commands.auto;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoModeBaseCmd extends SequentialCommandGroup {
    private Pose2d pose_ ;
    private String name_ ;

    public AutoModeBaseCmd(String name) {
        pose_ = new Pose2d() ;   
        name_ = name ;
    }

    public AutoModeBaseCmd(String name, PathPlannerPath p) {
        if (p.getStartingHolonomicPose().isPresent()) {
            pose_ = p.getStartingHolonomicPose().get() ;
            name_ = name ;
        }
    }

    public AutoModeBaseCmd(String name, Pose2d p) {
        pose_ = p ;
        name_ = name ;
    }

    public String getName() {
        return name_ ;
    }

    public Pose2d getStartingPose() {
        return pose_ ;
    }
}
