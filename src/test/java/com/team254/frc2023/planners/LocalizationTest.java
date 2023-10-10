package com.team254.frc2023.planners;

import com.team254.frc2023.Constants;
import com.team254.frc2023.RobotState;
import com.team254.lib.geometry.*;
import com.team254.lib.swerve.ChassisSpeeds;
import com.team254.lib.trajectory.*;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingUtil;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class LocalizationTest {

    @Test
    public void testTrajectory() {
        List<Pose2d> waypoints = new ArrayList<>();
        List<Rotation2d> headings = new ArrayList<>();
        waypoints.add(new Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(0)));
        headings.add(Rotation2d.fromDegrees(180));
        waypoints.add(new Pose2d(100, 4, Rotation2d.fromDegrees(0)));
        headings.add(Rotation2d.fromDegrees(180));
        waypoints.add(new Pose2d(196, 13, Rotation2d.fromDegrees(0)));
        headings.add(Rotation2d.fromDegrees(0));

        double start_vel = 0.0;
        double end_vel = 0.0;
        double max_vel = 100;
        double max_accel = 100;

        Trajectory<Pose2dWithMotion> traj = new Trajectory<>();
        Assertions.assertTrue(traj.isEmpty());
        Assertions.assertEquals(0.0, traj.getIndexView().first_interpolant(), 0.2);
        Assertions.assertEquals(0.0, traj.getIndexView().last_interpolant(), 0.2);
        Assertions.assertEquals(0, traj.length());

        // Set states at construction time.
        traj = TrajectoryUtil.trajectoryFromWaypointsAndHeadings(waypoints, headings, 2, 0.25, 0.1);
        Assertions.assertFalse(traj.isEmpty());
        Assertions.assertEquals(0.0, traj.getIndexView().first_interpolant(), 0.2);
//        Assertions.assertEquals(3.0, traj.getIndexView().last_interpolant(), 0.2);
//        Assertions.assertEquals(3, traj.length());

//        for(int i = 0; i < traj.length(); i++) {
//            System.out.println(traj.getPoint(i).toString());
//            System.out.println(traj.getPoint(i).state().toString());
//            System.out.println(traj.getPoint(i).index());
//        }


        Trajectory<TimedState<Pose2dWithMotion>> timed_trajectory = TimingUtil.timeParameterizeTrajectory
                (false, new DistanceView<>(traj), 2, Arrays.asList(), start_vel, end_vel, max_vel, max_accel);

        System.out.println("\n\n\n\n\n\n\n");

//        for(int i = 0; i < timed_trajectory.length(); i++) {
//            System.out.println(timed_trajectory.getPoint(i).toString());
//            System.out.println(timed_trajectory.getPoint(i).state().state().toString());
//            System.out.println(timed_trajectory.getPoint(i).index());
//        }

        DriveMotionPlanner planner = new DriveMotionPlanner(Constants.kKinematics, Constants.kSmoothKinematicLimits);
        TrajectoryIterator<TimedState<Pose2dWithMotion>> traj_iterator =
                new TrajectoryIterator<>(new TimedView<>(timed_trajectory));
        planner.setTrajectory(traj_iterator);

        Pose2d pose = new Pose2d(timed_trajectory.getPoint(0).state().state().getPose());
        Twist2d velocity = Twist2d.identity();
        Pose2d initialOffset = Pose2d.fromTranslation(new Translation2d(2, 0));
        Pose2d finalOffset = Pose2d.fromTranslation(new Translation2d(3, 0));
        System.out.println(finalOffset.inverse().transformBy(initialOffset));
        double time = 0.0;
        double mDt = 0.005;
        while (!planner.isDone()) {
                var pose_with_localization = pose.transformBy(finalOffset.inverse().transformBy(initialOffset).inverse());
            ChassisSpeeds speeds = planner.update(time, pose_with_localization, velocity);
            Twist2d twist = new Twist2d(speeds.vxMetersPerSecond * mDt, speeds.vyMetersPerSecond * mDt,
                    speeds.omegaRadiansPerSecond * mDt);
            velocity = speeds.toTwist2d();
            pose = pose.transformBy(Pose2d.exp(twist));
//            System.out.println("\n\n\n\n-----t="+time);
//            System.out.println(speeds);
//            System.out.println(pose);
//            System.out.println("Pathsetpoint:" + planner.getSetpoint());
            time += mDt;
        }
        Assertions.assertEquals(195, pose.getTranslation().x(), 0.2);
        Assertions.assertEquals(13, pose.getTranslation().y(), 0.2);
        Assertions.assertEquals(0, pose.getRotation().getDegrees(), 0.4);
    }

    public Pose2d getLocalizationCorrection() {
        return Pose2d.fromTranslation(new Translation2d(3, 0)).inverse().transformBy(Pose2d.fromTranslation(new Translation2d(2, 0)));
    }
}
