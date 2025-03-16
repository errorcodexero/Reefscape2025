package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ReefConstants;

public class ReefUtil {
    private static Map<ReefFace, ReefFaceInfo> faceInfoMap_ = new HashMap<>();

    private static Pose2d blueBargeTagPose = FieldConstants.layout.getTagPose(14).orElseThrow().toPose2d();
    private static Pose2d redBargeTagPose = FieldConstants.layout.getTagPose(5).orElseThrow().toPose2d();
    
    public static void initialize() {
        for (ReefFace face : ReefFace.values()) {
            faceInfoMap_.put(face, new ReefFaceInfo(face));
        }
    }

    /**
     * Gets the reef face the robot should target with a given robot pose.
     * @param robotPose The pose of the robot.
     * @return An optional of a ReefFace object. Empty if there is no face of the reef that is deemed suitable.
     */    
    public static Optional<ReefFaceInfo> getTargetedReefFace(Pose2d robotPose) {
        Optional<ReefFaceInfo> ret = Optional.empty();

        ReefFaceInfo nearestFace = getNearestReefFace(robotPose);
        Pose2d nearestWall = nearestFace.getTagPose();

        Rotation2d rotationToFace = new Rotation2d(
            nearestWall.relativeTo(robotPose).getTranslation().getAngle().getMeasure().abs(Radians)
        );

        if (rotationToFace.getMeasure().lte(ReefConstants.maximumAngleToFace) && // Angle is within limit
                    getDistanceFromFace(robotPose, nearestFace).lte(ReefConstants.maximumDistanceToFace)) 
        {
            ret = Optional.of(nearestFace); 
        }

        return ret ;
    }

    /**
     * Gets the nearest reef face to the robot or any pose provided.
     * Note: This is not meant for deciding which face of the reef to go to. This is simply getting the nearest face.
     * To get the face that should be targeted use {@link #getTargetedReefFace(Pose2d) getTargetedReefFace()}.
     * @param robotPose
     * @return The reef face closest to the provided pose.
     */
    public static ReefFaceInfo getNearestReefFace(Pose2d robotPose) {

        ReefFaceInfo nearest = null ;
        Distance distance = Meters.of(1e10) ;

        for (ReefFace face : faceInfoMap_.keySet()) {
            ReefFaceInfo info = faceInfoMap_.get(face);
            Distance tagdist = getDistanceFromFace(robotPose, info);

            if (tagdist.lt(distance)) {
                //
                // This tag is closer
                //
                nearest = info ;
                distance = tagdist ;
            }
        }

        return nearest;
    }

    public static Pose2d getBargeScorePose(Pose2d robotPose, Distance d) {
        boolean isOnBlue = robotPose.getX() < FieldConstants.layout.getFieldLength() / 2;
        Pose2d tagPose = isOnBlue ? blueBargeTagPose : redBargeTagPose;

        Pose2d baseScorePose = tagPose.transformBy(new Transform2d(d, Meters.zero(), Rotation2d.k180deg)) ;
        
        return new Pose2d(baseScorePose.getX(), robotPose.getY(), baseScorePose.getRotation());
    }

    /**
     * Gets the distance from a reef face from a given pose.
     * @param robot The pose to get the distance from.
     * @param face The face to get the distance to.
     * @return
     */
    public static Distance getDistanceFromFace(Pose2d robot, ReefFaceInfo face) {
        return Meters.of(robot.getTranslation().getDistance(face.getTagPose().getTranslation()));
    }
}
