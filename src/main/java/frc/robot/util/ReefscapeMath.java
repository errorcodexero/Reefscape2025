package frc.robot.util;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ReefscapeMath {
    public static class ReefPlaceInfo {
        private final String legal = "ABCDEFGHIJKL" ;
        private final List<Integer> valid = Arrays.asList(17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11) ;

        public final Character which ;
        public final Alliance alliance ;
        public final int tag ;
        public final Pose2d place_pose ;
        public final Pose2d backup_pose ;

        public ReefPlaceInfo(Alliance a, char w, int t, Pose2d p, Pose2d b) {
            if (legal.indexOf(w) == -1) {
                throw new RuntimeException("invalid which (w) value for ReefPlaceInfo") ;
            }

            if (!valid.contains(t)) {
                throw new RuntimeException("invalid april tag (t) value for ReefPlaceInfo") ;
            }

            alliance = a ;
            which = w ;
            tag = t ;
            place_pose = p ;
            backup_pose = b ;
        }
    }

    public static ReefPlaceInfo findReefInfo() {
        return new ReefPlaceInfo(Alliance.Red, 'A', 11, new Pose2d(), new Pose2d()) ;
    }
}
