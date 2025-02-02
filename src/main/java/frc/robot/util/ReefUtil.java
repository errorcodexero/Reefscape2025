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
        RED_AB(7),
        RED_CD(8),
        RED_EF(9),
        RED_GH(10),
        RED_IJ(11),
        RED_KL(6),
        BLUE_AB(18),
        BLUE_CD(17),
        BLUE_EF(22),
        BLUE_GH(21),
        BLUE_IJ(20),
        BLUE_KL(19);

        // Alias Names
        public static final ReefFace RED_FRONT = RED_AB;
        public static final ReefFace RED_FRONTRIGHT = RED_CD;
        public static final ReefFace RED_BACKRIGHT = RED_EF;
        public static final ReefFace RED_BACK = RED_GH;
        public static final ReefFace RED_BACKLEFT = RED_IJ;
        public static final ReefFace RED_FRONTLEFT = RED_KL;

        public static final ReefFace BLUE_FRONT = BLUE_AB;
        public static final ReefFace BLUE_FRONTRIGHT = BLUE_CD;
        public static final ReefFace BLUE_BACKRIGHT = BLUE_EF;
        public static final ReefFace BLUE_BACK = BLUE_GH;
        public static final ReefFace BLUE_BACKLEFT = BLUE_IJ;
        public static final ReefFace BLUE_FRONTLEFT = BLUE_KL;

        private final int tagID_;

        private final Pose2d tagPose_;
        private final Pose2d algaeScoringPose_;
        private final Pose2d leftScoringPose_;
        private final Pose2d rightScoringPose_;
        private final Pose2d algaeBackupPose_;
        private final Pose2d leftBackupPose_;
        private final Pose2d rightBackupPose_;

        private ReefFace(int aprilTagID) {
            tagID_ = aprilTagID;
            tagPose_ = FieldConstants.layout.getTagPose(aprilTagID).orElseThrow().toPose2d().transformBy(new Transform2d(
                new Translation2d(),
                new Rotation2d(Degrees.of(180))
            ));

            algaeScoringPose_ = tagPose_.transformBy(new Transform2d(
                new Translation2d(
                    ReefConstants.distanceFromTagAlgae.unaryMinus(),
                    ReefConstants.robotToArm.unaryMinus()
                ),
                new Rotation2d()
            ));

            algaeBackupPose_ = algaeScoringPose_.transformBy(new Transform2d(
                new Translation2d(
                    ReefConstants.backupDistanceAlgae.unaryMinus(),
                    Meters.zero()
                ),
                new Rotation2d()
              
            leftScoringPose_ = tagPose_.transformBy(new Transform2d(
                new Translation2d(
                    ReefConstants.distanceFromTagCoral.unaryMinus(),
                    ReefConstants.leftRightOffset.minus(ReefConstants.robotToArm)
                ),
                new Rotation2d()
            ));

            leftBackupPose_ = leftScoringPose_.transformBy(new Transform2d(
                new Translation2d(
                    ReefConstants.backupDistanceCoral.unaryMinus(),
                    Meters.zero()
                ),
                new Rotation2d()
            ));

            rightScoringPose_ = tagPose_.transformBy(new Transform2d(
                new Translation2d(
                    ReefConstants.distanceFromTagCoral.unaryMinus(),
                    ReefConstants.leftRightOffset.unaryMinus().minus(ReefConstants.robotToArm)
                ),
                new Rotation2d()
            ));

            rightBackupPose_ = rightScoringPose_.transformBy(new Transform2d(
                new Translation2d(
                    ReefConstants.backupDistanceCoral.unaryMinus(),
                    Meters.zero()
                ),
                new Rotation2d()
            ));

            rightBackupPose_ = rightScoringPose_.transformBy(new Transform2d(
                new Translation2d(
                    ReefConstants.backupDistanceCoral.unaryMinus(),
                    Meters.zero()
                ),
                new Rotation2d()
            ));
        }

        public int getTagID() {
            return tagID_;
        }

        public Pose2d getTagPose() {
            return tagPose_;
        }

        public Pose2d getAlgaeScoringPose() {
            return algaeScoringPose_;
        }

        public Pose2d getAlgaeBackupPose() {
            return algaeBackupPose_;
        }

        public Pose2d getLeftScoringPose() {
            return leftScoringPose_;
        }

        public Pose2d getLeftBackupPose() {
            return leftBackupPose_;
        }

        public Pose2d getRightScoringPose() {
            return rightScoringPose_;
        }

        public Pose2d getRightBackupPose() {
            return rightBackupPose_;
        }
    }

    // Log Debug Data If In Replay or Simulator
    static {
        if (Constants.getMode() != Mode.REAL) {
            ReefFace[] faces = ReefFace.values();

            ArrayList<Pose2d> algaeScoringPoses = new ArrayList<>();
            ArrayList<Pose2d> algaeBackupPoses = new ArrayList<>();
            ArrayList<Pose2d> leftScoringPoses = new ArrayList<>();
            ArrayList<Pose2d> leftBackupPoses = new ArrayList<>();
            ArrayList<Pose2d> rightScoringPoses = new ArrayList<>();
            ArrayList<Pose2d> rightBackupPoses = new ArrayList<>();

            for (ReefFace face : faces) {
                String path = "ReefFaces/" + face.toString() + "/";

                Logger.recordOutput(path + "TagId", face.getTagID());
                Logger.recordOutput(path + "TagPose", face.getTagPose());
                Logger.recordOutput(path + "ScoringPoseAlgae", face.getAlgaeScoringPose());
                Logger.recordOutput(path + "BackupPoseAlgae", face.getAlgaeBackupPose());
                Logger.recordOutput(path + "ScoringPoseLeft", face.getLeftScoringPose());
                Logger.recordOutput(path + "BackupPoseLeft", face.getLeftBackupPose());
                Logger.recordOutput(path + "ScoringPoseRight", face.getRightScoringPose());
                Logger.recordOutput(path + "BackupPoseRight", face.getRightBackupPose());
                
                algaeScoringPoses.add(face.getAlgaeScoringPose());
                algaeBackupPoses.add(face.getAlgaeBackupPose());
                leftScoringPoses.add(face.getLeftScoringPose());
                leftBackupPoses.add(face.getLeftBackupPose());
                rightScoringPoses.add(face.getRightScoringPose());
                rightBackupPoses.add(face.getRightBackupPose());
            }

            Logger.recordOutput("ReefFaces/Summary/AlgaeScoringPoses", algaeScoringPoses.toArray(new Pose2d[0]));
            Logger.recordOutput("ReefFaces/Summary/AlgaeBackupPoses", algaeBackupPoses.toArray(new Pose2d[0]));
            Logger.recordOutput("ReefFaces/Summary/LeftScoringPoses", leftScoringPoses.toArray(new Pose2d[0]));
            Logger.recordOutput("ReefFaces/Summary/LeftBackupPoses", leftBackupPoses.toArray(new Pose2d[0]));
            Logger.recordOutput("ReefFaces/Summary/RightScoringPoses", rightScoringPoses.toArray(new Pose2d[0]));
            Logger.recordOutput("ReefFaces/Summary/RightBackupPoses", rightBackupPoses.toArray(new Pose2d[0]));
        }
    }

    /**
     * Gets the reef face the robot should target with a given robot pose.
     * @param robotPose The pose of the robot.
     * @return An optional of a ReefFace object. Empty if there is no face of the reef that is deemed suitable.
     */
    public static Optional<ReefFace> getTargetedReefFace(Pose2d robotPose) {
        ReefFace nearestFace = getNearestReefFace(robotPose);
        Pose2d nearestWall = nearestFace.getTagPose();

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
        return robot.getTranslation().getDistance(face.getTagPose().getTranslation());
    }

}