package frc.robot.util;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ReefscapeMath {

    public enum PlaceSide {
        Left,
        Right
    } ;

    private static double AprilTagToPlacePosInches = 12.0 ;
    private static double BackupDistance = 18.0 ;

    public static Translation2d BlueReefCenter = new Translation2d(176.75, 109.13) ;
    public static ReefPlaceInfo[] BlueFaces = new ReefPlaceInfo[12] ;

    public static Translation2d RedReefCenter = new Translation2d(514.125, 109.13) ;
    public static ReefPlaceInfo[] RedFaces = new ReefPlaceInfo[12] ;

    private static final List<Integer> RedTags = Arrays.asList(7, 8, 9, 10, 11, 6) ;
    private static final List<Integer> BlueTags = Arrays.asList(18, 17, 22, 21, 20, 19) ;

    public static final AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape) ;

    public static class ReefPlaceInfo {
        public final char which ;
        public final Alliance alliance ;
        public final int tag ;
        public final Pose2d tag_pose ;
        public final Pose2d place_pose ;
        public final Pose2d backup_pose ;

        public ReefPlaceInfo(Alliance a, char w, int t, Pose2d tp, Pose2d p, Pose2d b) {
            alliance = a ;
            which = w ;
            tag = t ;
            tag_pose = tp ;
            place_pose = p ;
            backup_pose = b ;
        }
    }

    public static ReefPlaceInfo findReefInfo(Alliance a, Pose2d dbpose, PlaceSide s) {
        if (BlueFaces[0] == null) {
            generateReefPlacements();
        }

        ReefPlaceInfo ret = null ;
        ReefPlaceInfo [] faces = (a == Alliance.Red) ? ReefscapeMath.RedFaces : ReefscapeMath.BlueFaces ;
        for(int i = 0 ; i < faces.length ; i++) {

        }

        return ret ;
    }

    private static void generateReefPlacements() {
        Pose2d p, b, tp ;
        int tag ;

        char face = 'A' ;
        for(int i = 0 ; i < 6 ; i++) {
            tag = ReefscapeMath.RedTags.get(i) ;
            tp = ReefscapeMath.field.getTagPose(tag).get().toPose2d() ;
            p = ReefscapeMath.getPlacePosition(tp, PlaceSide.Left) ;
            b = ReefscapeMath.getBackupPosition(tp, PlaceSide.Left) ;
            RedFaces[i * 2] = new ReefPlaceInfo(Alliance.Red, face, tag, tp, p, b) ;

            tag = ReefscapeMath.BlueTags.get(i) ;
            tp = ReefscapeMath.field.getTagPose(tag).get().toPose2d() ;
            p = ReefscapeMath.getPlacePosition(tp, PlaceSide.Left) ;
            b = ReefscapeMath.getBackupPosition(tp, PlaceSide.Left) ;
            BlueFaces[i * 2] = new ReefPlaceInfo(Alliance.Blue, face, tag, tp, p, b) ;
            face++ ;
            
            tag = ReefscapeMath.RedTags.get(i) ;
            tp = ReefscapeMath.field.getTagPose(tag).get().toPose2d() ;
            p = ReefscapeMath.getPlacePosition(tp, PlaceSide.Right) ;
            b = ReefscapeMath.getBackupPosition(tp, PlaceSide.Right) ;
            tp = ReefscapeMath.field.getTagPose(tag).get().toPose2d() ;
            RedFaces[i * 2 + 1] = new ReefPlaceInfo(Alliance.Red, face, tag, tp, p, b) ;

            tag = ReefscapeMath.BlueTags.get(i) ;
            tp = ReefscapeMath.field.getTagPose(tag).get().toPose2d() ;
            p = ReefscapeMath.getPlacePosition(tp, PlaceSide.Right) ;
            b = ReefscapeMath.getBackupPosition(tp, PlaceSide.Right) ;
            tp = ReefscapeMath.field.getTagPose(tag).get().toPose2d() ;
            BlueFaces[i * 2 + 1] = new ReefPlaceInfo(Alliance.Blue, face, tag, tp, p, b) ;
            face++ ;
        }
    }

    private static Translation2d projectPointAlongHeading(Translation2d p, Rotation2d angle, double projection) {
        double dx = Math.cos(angle.getRadians()) * projection;
        double dy = Math.sin(angle.getRadians()) * projection;
        return new Translation2d(p.getX() + dx, p.getY() + dy);
    }

    private static Pose2d getPlacePosition(Pose2d tag2d, PlaceSide side) {
        Rotation2d by = (side == PlaceSide.Left) ? Rotation2d.fromDegrees(90.0) : Rotation2d.fromDegrees(-90.0) ;
        Rotation2d rot = tag2d.getRotation().rotateBy(by) ;
        Translation2d pos = ReefscapeMath.projectPointAlongHeading(tag2d.getTranslation(), rot, ReefscapeMath.AprilTagToPlacePosInches) ;
        Rotation2d dest = tag2d.getRotation().rotateBy(Rotation2d.k180deg) ;
        return new Pose2d(pos, dest) ;
    }

    private static Pose2d getBackupPosition(Pose2d tag2d, PlaceSide side) {
        Pose2d temp = ReefscapeMath.getPlacePosition(tag2d, side) ;
        Rotation2d rot = temp.getRotation().rotateBy(Rotation2d.k180deg) ;
        Translation2d bpos = projectPointAlongHeading(temp.getTranslation(), rot, ReefscapeMath.BackupDistance) ;
        return new Pose2d(bpos, temp.getRotation()) ;
    }
}
