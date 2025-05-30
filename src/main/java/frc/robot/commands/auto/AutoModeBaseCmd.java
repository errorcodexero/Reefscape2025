package frc.robot.commands.auto;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveCommands;

public class AutoModeBaseCmd extends SequentialCommandGroup {
    private Pose2d pose_ ;
    private String name_ ;

    public AutoModeBaseCmd(String name) {
        this(name, new Pose2d());
    }

    /**
     * Creates an auto mode with a path, BLUE centric.
     */
    public AutoModeBaseCmd(String name, PathPlannerPath p) {
        this(name, p.getStartingHolonomicPose().orElseThrow()); 
    }

    /**
     * Creates an auto mode with no path, just a pose. This pose is BLUE centric!
     */
    public AutoModeBaseCmd(String name, Pose2d p) {
        pose_ = DriveCommands.rotateIfRed(p);
        name_ = name ;
    }

    public String getName() {
        return name_ ;
    }

    public Pose2d getStartingPose() {
        return pose_ ;
    }
}
