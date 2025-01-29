package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.ArrayList;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.FieldConstants;

public class ReefUtil {

    public static class ReefConstants {
        /**
         * The maximum angle from the robot to the nearest face of the reef for it to be considered targeting that face.
         */
        public static final Angle maximumAngleToFace = Degrees.of(40);

        /**
         * The maximum distance from the robot to the nearest face of the reef for it to be considered targeting that face.
         */
        public static final Distance maximumDistanceToFace = Meters.of(3);

        /**
         * The distance from the center of the robot to the tag while placing coral.
         */
        public static final Distance distanceFromTagCoral = Inches.of(20);

        /**
         * The distance from the center of the robot to the tag while collecting algae.
         */
        public static final Distance distanceFromTagAlgae = Inches.of(20);

        /**
         * The offset from the center of the tag to where we want the arm to be positioned.
         */
        public static final Distance leftRightOffset = Inches.of(5);

        /**
         * The distance from the center of the robot to the arm.
         */
        public static final Distance robotToArm = Inches.zero();
    }
    
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
                new Rotation2d()
            ));

            leftScoringPose_ = wallPose_.transformBy(new Transform2d(
                new Translation2d(
                    ReefConstants.distanceFromTagCoral,
                    ReefConstants.leftRightOffset.unaryMinus()
                ),
                new Rotation2d()
            ));

            rightScoringPose_ = wallPose_.transformBy(new Transform2d(
                new Translation2d(
                    ReefConstants.distanceFromTagCoral,
                    ReefConstants.leftRightOffset
                ),
                new Rotation2d()
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

    public static void logPoses() {
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

    private static double getDistanceFromFace(Pose2d robot, ReefFace face) {
        return robot.getTranslation().getDistance(face.getWallPose().getTranslation());
    }

}
