package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.ArrayList;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.ReefConstants;

public class ReefUtil {
    
    public static enum ReefFace {
        RED_FRONTLEFT(6),
        RED_FRONT(7),
        RED_FRONTRIGHT(8),
        RED_BACKRIGHT(9),
        RED_BACK(10),
        RED_BACKLEFT(11),
        BLUE_FRONTLEFT(19),
        BLUE_FRONT(18),
        BLUE_FRONTRIGHT(17),
        BLUE_BACKRIGHT(22),
        BLUE_BACK(21),
        BLUE_BACKLEFT(20);

        private final int tagID_;

        private final Pose2d wallPose_;
        private final Pose2d algaeScoringPose_;
        private final Pose2d leftScoringPose_;
        private final Pose2d rightScoringPose_;

        private ReefFace(int aprilTagID) {
            tagID_ = aprilTagID;
            wallPose_ = FieldConstants.layout.getTagPose(aprilTagID).orElseThrow().toPose2d();

            algaeScoringPose_ = wallPose_.transformBy(new Transform2d(
                new Translation2d(
                    ReefConstants.distanceFromTagAlgae,
                    Meters.zero()
                ),
                new Rotation2d(Degrees.of(180))
            ));

            leftScoringPose_ = wallPose_.transformBy(new Transform2d(
                new Translation2d(
                    ReefConstants.distanceFromTagCoral,
                    ReefConstants.leftRightOffset.unaryMinus()
                ),
                new Rotation2d(Degrees.of(180))
            ));

            rightScoringPose_ = wallPose_.transformBy(new Transform2d(
                new Translation2d(
                    ReefConstants.distanceFromTagCoral,
                    ReefConstants.leftRightOffset
                ),
                new Rotation2d(Degrees.of(180))
            ));
        }

        public int getTagID() {
            return tagID_;
        }

        public Pose2d getWallPose() {
            return wallPose_;
        }

        public Pose2d getAlgaeScoringPose() {
            return algaeScoringPose_;
        }

        public Pose2d getLeftScoringPose() {
            return leftScoringPose_;
        }

        public Pose2d getRightScoringPose() {
            return rightScoringPose_;
        }
    }

    // Log Debug Data If In Replay or Simulator
    static {
        if (Constants.getMode() != Mode.REAL) {
            ReefFace[] faces = ReefFace.values();

            ArrayList<Pose2d> poses = new ArrayList<>();

            for (ReefFace face : faces) {
                String path = "ReefFaces/" + face.toString() + "/";

                Logger.recordOutput(path + "TagId", face.getTagID());
                Logger.recordOutput(path + "WallPose", face.getWallPose());
                Logger.recordOutput(path + "ScoringPoseAlgae", face.getAlgaeScoringPose());
                Logger.recordOutput(path + "ScoringPoseLeft", face.getLeftScoringPose());
                Logger.recordOutput(path + "ScoringPoseRight", face.getRightScoringPose());

                poses.add(face.getAlgaeScoringPose());
                poses.add(face.getLeftScoringPose());
                poses.add(face.getRightScoringPose());
            }

            Logger.recordOutput("ReefFaces/AllBotPoses", poses.toArray(new Pose2d[0]));
        }
    }

    /**
     * Gets the reef face the robot should target with a given robot pose.
     * @param robotPose The pose of the robot.
     * @return An optional of a ReefFace object. Empty if there is no face of the reef that is deemed suitable.
     */
    public static Optional<ReefFace> getTargetedReefFace(Pose2d robotPose) {
        ReefFace nearestFace = getNearestReefFace(robotPose);
        Pose2d nearestWall = nearestFace.getWallPose();

        Rotation2d rotationToFace = new Rotation2d(
            nearestWall.relativeTo(robotPose).getTranslation().getAngle().getMeasure().abs(Radians)
        );

        if (
            rotationToFace.getMeasure().lte(ReefConstants.maximumAngleToFace) && // Angle is within limit
            getDistanceFromFace(robotPose, nearestFace) <= ReefConstants.maximumDistanceToFace.in(Meters) // Distance is within limit
        ) {
            return Optional.of(nearestFace); 
        } else {
            return Optional.empty();
        }
    } 

    /**
     * Gets the nearest reef face to the robot or any pose provided.
     * Note: This is not meant for deciding which face of the reef to go to. This is simply getting the nearest face.
     * To get the face that should be targeted use {@link #getTargetedReefFace(Pose2d) getTargetedReefFace()}.
     * @param robotPose
     * @return The reef face closest to the provided pose.
     */
    public static ReefFace getNearestReefFace(Pose2d robotPose) {
        ReefFace[] faces = ReefFace.values();

        ReefFace nearest = faces[0];

        for (ReefFace face : faces) {
            double distanceMeters = getDistanceFromFace(robotPose, face);

            if (distanceMeters < getDistanceFromFace(robotPose, nearest)) {
                nearest = face;
            }
        }

        return nearest;
    }

    /**
     * Gets the distance from a reef face from a given pose.
     * @param robot The pose to get the distance from.
     * @param face The face to get the distance to.
     * @return
     */
    public static double getDistanceFromFace(Pose2d robot, ReefFace face) {
        return robot.getTranslation().getDistance(face.getWallPose().getTranslation());
    }

}
