package com.team254.frc2023.field;

import java.util.HashMap;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class Field {

    public static class Red {
        public static final HashMap<Integer, AprilTag> kAprilTagMap = new HashMap<>();
        // From field manual: 46.355 cm
        // From CAD: 46.2534 cm
        // From step layout marking: 46.2788 cm
        // Old value: 46.272  cm
        private static final Translation2d kTag1ToCenterAlign = new Translation2d(0.77, 0.0);
        private static final Translation2d kTag1ToRightAlign = new Translation2d(0.77, -0.55);
        private static final Translation2d kTag1ToLeftAlign = new Translation2d(0.77, 0.55);

        private static final Translation2d kTag2ToCenterAlign = new Translation2d(0.77, 0.0);
        private static final Translation2d kTag2ToRightAlign = new Translation2d(0.77, -0.55);
        private static final Translation2d kTag2ToLeftAlign = new Translation2d(0.77, 0.55);

        private static final Translation2d kTag3ToCenterAlign = new Translation2d(0.77, 0.0);
        private static final Translation2d kTag3ToRightAlign = new Translation2d(0.77, -0.55);
        private static final Translation2d kTag3ToLeftAlign = new Translation2d(0.77, 0.55);

        private static final Translation2d kTag5ToCenterAlign = new Translation2d(0.77, 0.0);
        private static final Translation2d kTag5ToRightAlign = new Translation2d(0.77, -0.55);
        private static final Translation2d kTag5ToLeftAlign = new Translation2d(0.77, 0.55);
        public static final AprilTag kAprilTag1 = new AprilTag(
                1,
                0.462534,
                new Pose2d(1.027, 6.94659, Rotation2d.fromDegrees(0)),
                true,
                kTag1ToCenterAlign,
                kTag1ToLeftAlign,
                kTag1ToRightAlign
        );
        public static final AprilTag kAprilTag2 = new AprilTag(
                2,
                0.462534,
                new Pose2d(1.027, 5.27019, Rotation2d.fromDegrees(0)),
                true,
                kTag2ToCenterAlign,
                kTag2ToLeftAlign,
                kTag2ToRightAlign);
        public static final AprilTag kAprilTag3 = new AprilTag(
                3,
                0.462534,
                new Pose2d(1.027, 3.59379, Rotation2d.fromDegrees(0)),
                true,
                kTag3ToCenterAlign,
                kTag3ToLeftAlign,
                kTag3ToRightAlign);
        public static final AprilTag kAprilTag5 = new AprilTag(
                5,
                0.695452,
                new Pose2d(16.17832, 1.26839, Rotation2d.fromDegrees(0)),
                false,
                kTag5ToCenterAlign,
                kTag5ToLeftAlign,
                kTag5ToRightAlign);


        static {
            kAprilTagMap.put(1, kAprilTag1);
            kAprilTagMap.put(2, kAprilTag2);
            kAprilTagMap.put(3, kAprilTag3);
            kAprilTagMap.put(5, kAprilTag5);
        }


    }

    public static class Blue {
        public static final HashMap<Integer, AprilTag> kAprilTagMap = new HashMap<>();
        // From field manual: 46.355 cm
        // From CAD: 46.2534 cm
        // From step layout marking: 46.2788 cm
        // Old value: 46.272  cm
        private static final Translation2d kTag8ToCenterAlign = new Translation2d(0.77, 0.0);
        private static final Translation2d kTag8ToRightAlign = new Translation2d(0.77, -0.55);
        private static final Translation2d kTag8ToLeftAlign = new Translation2d(0.77, 0.55);

        private static final Translation2d kTag7ToCenterAlign = new Translation2d(0.77, 0.0);
        private static final Translation2d kTag7ToRightAlign = new Translation2d(0.77, -0.55);
        private static final Translation2d kTag7ToLeftAlign = new Translation2d(0.77, 0.55);

        private static final Translation2d kTag6ToCenterAlign = new Translation2d(0.77, 0.0);
        private static final Translation2d kTag6ToRightAlign = new Translation2d(0.77, -0.55);
        private static final Translation2d kTag6ToLeftAlign = new Translation2d(0.77, 0.55);

        private static final Translation2d kTag4ToCenterAlign = new Translation2d(0.77, 0.0);
        private static final Translation2d kTag4ToRightAlign = new Translation2d(0.77, -0.55);
        private static final Translation2d kTag4ToLeftAlign = new Translation2d(0.77, 0.55);
        public static final AprilTag kAprilTag8 = new AprilTag(
                8,
                0.462534,
                new Pose2d(1.027, 1.07341, Rotation2d.fromDegrees(0)),
                true,
                kTag8ToCenterAlign,
                kTag8ToLeftAlign,
                kTag8ToRightAlign
        );
        public static final AprilTag kAprilTag7 = new AprilTag(
                7,
                0.462534,
                new Pose2d(1.027, 2.74981, Rotation2d.fromDegrees(0)),
                true,
                kTag7ToCenterAlign,
                kTag7ToLeftAlign,
                kTag7ToRightAlign);
        public static final AprilTag kAprilTag6 = new AprilTag(
                6,
                0.462534,
                new Pose2d(1.027, 4.4221, Rotation2d.fromDegrees(0)),
                true,
                kTag6ToCenterAlign,
                kTag6ToLeftAlign,
                kTag6ToRightAlign);
        public static final AprilTag kAprilTag4 = new AprilTag(
                4,
                0.695452,
                new Pose2d(16.17832, 6.75161, Rotation2d.fromDegrees(0)),
                false,
                kTag4ToCenterAlign,
                kTag4ToLeftAlign,
                kTag4ToRightAlign);


        static {
            kAprilTagMap.put(4, kAprilTag4);
            kAprilTagMap.put(6, kAprilTag6);
            kAprilTagMap.put(7, kAprilTag7);
            kAprilTagMap.put(8, kAprilTag8);
        }
    }
}

