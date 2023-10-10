package com.team254.frc2023.planners;

import com.team254.frc2023.Constants;
import com.team254.frc2023.paths.TrajectoryGenerator;
import com.team254.lib.geometry.*;
import com.team254.lib.swerve.ChassisSpeeds;
import com.team254.lib.swerve.SwerveModuleState;
import com.team254.lib.swerve.SwerveSetpoint;
import com.team254.lib.swerve.SwerveSetpointGenerator;
import com.team254.lib.trajectory.*;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingUtil;
import com.team254.lib.util.Units;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class DriveMotionPlannerTest {

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

        double time = 0.0;
        double mDt = 0.005;
        while (!planner.isDone()) {
            ChassisSpeeds speeds = planner.update(time, pose, velocity);
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
        Assertions.assertEquals(196, pose.getTranslation().x(), 0.1);
        Assertions.assertEquals(13, pose.getTranslation().y(), 0.1);
        Assertions.assertEquals(0, pose.getRotation().getDegrees(), 0.4);
    }

    @Test
    public void testAllTrajectories() {
        var kinematics = Constants.kKinematics;
        var limits = Constants.kSmoothKinematicLimits;
        DriveMotionPlanner planner = new DriveMotionPlanner(kinematics, limits);
        TrajectoryGenerator generator = new TrajectoryGenerator(planner);
        generator.generateTrajectories();
        var trajectories = generator.getTrajectorySet().getAllTrajectories();

        SwerveSetpointGenerator setpoint_generator = new SwerveSetpointGenerator(kinematics);

        for (var traj : trajectories) {
//            System.out.println("\n" + traj.toString());
            planner.reset();
            TrajectoryIterator<TimedState<Pose2dWithMotion>> traj_iterator =
                new TrajectoryIterator<>(new TimedView<>(traj));
            planner.setTrajectory(traj_iterator);
            final Pose2d kInjectedError = new Pose2d(0.3, -0.1, Rotation2d.fromDegrees(9.0));
            final Twist2d kInjectedVelocityError = new Twist2d(0.1, 0.3, 0.0);
            final double kInjectionTime = 20.0;
            Pose2d pose = new Pose2d(traj.getPoint(0).state().state().getPose());
            Twist2d velocity = Twist2d.identity();
            SwerveSetpoint setpoint = null;
            double time = 0.0;
            double mDt = 0.005;
            boolean error_injected = false;
            while (!planner.isDone()) {
                System.out.println("\n\n-----t="+time);
                if (!error_injected && time >= kInjectionTime) {
                    pose = pose.transformBy(kInjectedError);
                    velocity = new Twist2d(velocity.dx + kInjectedVelocityError.dx, velocity.dy + kInjectedVelocityError.dy, velocity.dtheta + kInjectedVelocityError.dtheta);
                    error_injected = true;
                }
                ChassisSpeeds speeds = planner.update(time, pose, velocity);
                if (true ) {//setpoint == null) {
                    // Initialilze from first chassis speeds.
                    setpoint = new SwerveSetpoint(new ChassisSpeeds(), new SwerveModuleState[kinematics.getNumModules()]);
                    var states = kinematics.toSwerveModuleStates(speeds);
                    setpoint.mChassisSpeeds = speeds;
                    setpoint.mModuleStates = states;
                }
                // setpoint = setpoint_generator.generateSetpoint(limits, setpoint, speeds, mDt);
                // Don't use a twist here (assume Drive compensates for that)
                Pose2d delta = new Pose2d(new Translation2d(
                        setpoint.mChassisSpeeds.vxMetersPerSecond * mDt,
                        setpoint.mChassisSpeeds.vyMetersPerSecond * mDt),
                        Rotation2d.fromRadians(setpoint.mChassisSpeeds.omegaRadiansPerSecond * mDt));
                pose = pose.transformBy(delta);
                velocity = setpoint.mChassisSpeeds.toTwist2d();

//                System.out.println("\n\n-----t="+time);
//                System.out.println(speeds);
//                System.out.println(pose);
//                System.out.println("Setpoint:" + planner.getSetpoint());
                // Inches and degrees
                var error = pose.inverse().transformBy(planner.getSetpoint().state().getPose());
                // System.out.println("Setpoint: " + planner.getSetpoint());
                // System.out.println("Error: " + error);
                Assertions.assertEquals(0.0, error.getTranslation().x(), 0.0508);
                Assertions.assertEquals(0.0, error.getTranslation().y(), 0.0508);
                Assertions.assertEquals(0.0, error.getRotation().getDegrees(), 5.0);

                if (planner.isDone()) {
                    planner.getErrorTracker().printSummary();
                }

                time += mDt;
            }
        }

    }
}
