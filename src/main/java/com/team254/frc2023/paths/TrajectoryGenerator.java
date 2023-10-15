package com.team254.frc2023.paths;

import com.team254.frc2023.Constants;
import com.team254.frc2023.planners.DriveMotionPlanner;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithMotion;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;
import com.team254.lib.trajectory.timing.VelocityLimitRegionConstraint;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;

public class TrajectoryGenerator {
    private static final double kMaxAccel = 2.54;
    private static final double kMaxVoltage = 9.0;

    private final DriveMotionPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet = null;

    public TrajectoryGenerator(DriveMotionPlanner motion_planner) {
        mMotionPlanner = motion_planner;
    }

    public void generateTrajectories() {
        if (mTrajectorySet == null) {
            System.out.println("Generating trajectories...");
            mTrajectorySet = new TrajectorySet();
            System.out.println("Finished trajectory generation");
        }
    }

    public void forceRegenerateTrajectories() {
        System.out.println("Generating trajectories...");
        mTrajectorySet = new TrajectorySet();
        System.out.println("Finished trajectory generation");
    }

    public TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
    }

    public Trajectory<TimedState<Pose2dWithMotion>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<Rotation2d> headings,
            final List<TimingConstraint<Pose2dWithMotion>> constraints,
            double max_vel,  // m/s
            double max_accel,  // m/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, headings, constraints, max_vel, max_accel, max_voltage);
    }

    public Trajectory<TimedState<Pose2dWithMotion>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<Rotation2d> headings,
            final List<TimingConstraint<Pose2dWithMotion>> constraints,
            double start_vel,  // m/s
            double end_vel,  // m/s
            double max_vel,  // m/s
            double max_accel,  // m/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, headings, constraints, start_vel, end_vel, max_vel, max_accel, max_voltage);
    }

    public class TrajectorySet {
        public final Trajectory<TimedState<Pose2dWithMotion>> testTrajectory;
        public final Trajectory<TimedState<Pose2dWithMotion>> testTrajectory2;
        public final Trajectory<TimedState<Pose2dWithMotion>> coopLinkStartToCoopFirstPickup;
        public final Trajectory<TimedState<Pose2dWithMotion>> coopFirstPickupToCoopSecondScore;
        public final Trajectory<TimedState<Pose2dWithMotion>> coopFirstScoreToDock;
        public final Trajectory<TimedState<Pose2dWithMotion>> coopSecondScoreToDock;
        public final Trajectory<TimedState<Pose2dWithMotion>> redNCPCubeScoringPositionToFarSideDockWithPickup;
        public final Trajectory<TimedState<Pose2dWithMotion>> redNCPCubeScoringPositionToThirdCube;
        public final Trajectory<TimedState<Pose2dWithMotion>> redCPLeftScoringPositionToCPFirstPickup;
        public final Trajectory<TimedState<Pose2dWithMotion>> CPFirstPickupToFarSideDock;
        public final Trajectory<TimedState<Pose2dWithMotion>> ncpScoringToCubePickup;
        public final Trajectory<TimedState<Pose2dWithMotion>> ncpThirdScoreToBackoff;

        public final Trajectory<TimedState<Pose2dWithMotion>> cpFirstScoreToOutsidePickup;
        public final Trajectory<TimedState<Pose2dWithMotion>> cpFirstScoreToInsidePickup;
        public final Trajectory<TimedState<Pose2dWithMotion>> cpOutsidePickupToSecondAlign;
        public final Trajectory<TimedState<Pose2dWithMotion>> cpInsidePickupToSecondAlign;
        public final Trajectory<TimedState<Pose2dWithMotion>> cpSecondScoreToInsidePickup;
        public final Trajectory<TimedState<Pose2dWithMotion>> cpSecondScoreToOutsidePickup;
        public final Trajectory<TimedState<Pose2dWithMotion>> cpInsidePickupToDock;
        public final Trajectory<TimedState<Pose2dWithMotion>> cpOutsidePickupToDock;
        public final Trajectory<TimedState<Pose2dWithMotion>> cpInsidePickupToThirdScore;
        public final Trajectory<TimedState<Pose2dWithMotion>> cpOutsidePickupToThirdScore;
        public final Trajectory<TimedState<Pose2dWithMotion>> cpThirdScoreToBackoff;


        public final VelocityLimitRegionConstraint<Pose2dWithMotion> center_outbound_charging_station_constraint;
        public final VelocityLimitRegionConstraint<Pose2dWithMotion> center_inbound_charging_station_constraint;


        private List<Trajectory<TimedState<Pose2dWithMotion>>> allTrajectories;

        public List<Trajectory<TimedState<Pose2dWithMotion>>> getAllTrajectories() {
            return allTrajectories;
        }

        private TrajectorySet() {


            center_outbound_charging_station_constraint = new VelocityLimitRegionConstraint<>(
                    new Translation2d(1.0, -1.27),
                    new Translation2d(3.0, 1.27),
                    1.25
            );

            center_inbound_charging_station_constraint = new VelocityLimitRegionConstraint<>(
                    new Translation2d(0.0, -1.27),
                    new Translation2d(4.5, 1.27),
                    1.25
            );

            allTrajectories = new ArrayList<>();
            testTrajectory = getTestTrajectory();
            allTrajectories.add(testTrajectory);
            testTrajectory2 = getTestTrajectory2();
            allTrajectories.add(testTrajectory2);
            coopLinkStartToCoopFirstPickup = getCoopLinkStartToCoopFirstPickup();
            allTrajectories.add(coopLinkStartToCoopFirstPickup);
            coopFirstPickupToCoopSecondScore = getCoopFirstPickupToCoopSecondScore();
            allTrajectories.add(coopFirstPickupToCoopSecondScore);
            coopFirstScoreToDock = getCoopFirstScoreToDock();
            allTrajectories.add(coopFirstScoreToDock);
            coopSecondScoreToDock = getCoopSecondScoreToDock();
            allTrajectories.add(coopSecondScoreToDock);
            redNCPCubeScoringPositionToFarSideDockWithPickup = getRedNCPCubeScoringPositionToFarSideDockWithPickup();
            allTrajectories.add(redNCPCubeScoringPositionToFarSideDockWithPickup);
            redNCPCubeScoringPositionToThirdCube = getRedNCPCubeScoringPositionToThirdCube();
            allTrajectories.add(redNCPCubeScoringPositionToThirdCube);
            redCPLeftScoringPositionToCPFirstPickup = getRedCPLeftScoringPositionToCPFirstPickup();
            allTrajectories.add(redCPLeftScoringPositionToCPFirstPickup);
            CPFirstPickupToFarSideDock = getCPFirstPickupToFarSideDock();
            allTrajectories.add(CPFirstPickupToFarSideDock);
            ncpScoringToCubePickup = getNCPScoringtoCubePickup();
            allTrajectories.add(ncpScoringToCubePickup);
            cpFirstScoreToOutsidePickup = getCPFirstScoreToOutsidePickup();
            allTrajectories.add(cpFirstScoreToOutsidePickup);
            cpFirstScoreToInsidePickup = getCPFirstScoreToInsidePickup();
            allTrajectories.add(cpFirstScoreToInsidePickup);
            cpOutsidePickupToSecondAlign = getCPOutsidePickupToSecondAlign();
            allTrajectories.add(cpOutsidePickupToSecondAlign);
            cpInsidePickupToSecondAlign = getCPInsidePickupToSecondAlign();
            allTrajectories.add(cpInsidePickupToSecondAlign);
            cpSecondScoreToInsidePickup = getCPSecondScoreToInsidePickup();
            allTrajectories.add(cpSecondScoreToInsidePickup);
            cpSecondScoreToOutsidePickup = getCPSecondScoreToOutsidePickup();
            allTrajectories.add(cpSecondScoreToOutsidePickup);
            cpInsidePickupToDock = getCPInsidePickupToDock();
            allTrajectories.add(cpInsidePickupToDock);
            cpOutsidePickupToDock = getCPOutsidePickupToDock();
            allTrajectories.add(cpOutsidePickupToDock);
            cpInsidePickupToThirdScore = getCPInsidePickupToThirdScore();
            allTrajectories.add(cpInsidePickupToThirdScore);
            cpOutsidePickupToThirdScore = getCPOutsidePickupToThirdScore();
            allTrajectories.add(cpOutsidePickupToThirdScore);
            cpThirdScoreToBackoff = getCPThirdScoreToBackoff();
            allTrajectories.add(cpThirdScoreToBackoff);
            ncpThirdScoreToBackoff = getNcpThirdScoreToBackoff();
            allTrajectories.add(ncpThirdScoreToBackoff);
        }

        private void convertToM(List<Pose2d> waypoints, List<Rotation2d> headings) {
            for (int i = 0; i < waypoints.size(); ++i) {
                System.out.println("waypoints.add(new Pose2d(" + Units.inchesToMeters(waypoints.get(i).getTranslation().x())
                        + ", " + Units.inchesToMeters(waypoints.get(i).getTranslation().y())
                        + ", Rotation2d.fromDegrees(" + waypoints.get(i).getRotation().getDegrees()
                        + ")));");
                System.out.println("headings.add(Rotation2d.fromDegrees("
                + headings.get(i).getDegrees()+"));");
            }
            System.out.println("\n\n");

        }

        private Trajectory<TimedState<Pose2dWithMotion>> getTestTrajectory() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(1.0, 0.0, Rotation2d.fromDegrees(0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(2.5, 0.0, Rotation2d.fromDegrees(0)));
            headings.add(Rotation2d.fromDegrees(0));
            return generate(waypoints, headings, List.of(), false, 0.8, 1.0);
        }

        private Trajectory<TimedState<Pose2dWithMotion>> getTestTrajectory2() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(2.5, 0.0, Rotation2d.fromDegrees(180)));
            headings.add(Rotation2d.fromDegrees(0));
            waypoints.add(new Pose2d(1.0, 0.0, Rotation2d.fromDegrees(180)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180)));
            headings.add(Rotation2d.fromDegrees(180.0));
            return generate(waypoints, headings, List.of(), false, 0.3, 1.0);
        }

        private Trajectory<TimedState<Pose2dWithMotion>> getCPFirstScoreToOutsidePickup() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(0.9652, -0.125, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(2.85, -0.125, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(5.1832, -0.4, Rotation2d.fromDegrees(0)));
            headings.add(Rotation2d.fromDegrees(-0.01));
            waypoints.add(new Pose2d(5.2832, -0.45, Rotation2d.fromDegrees(0)));
            headings.add(Rotation2d.fromDegrees(-0.01));
            return generate(waypoints, headings, List.of(), false, 0.75, 1.57);
        }

        private Trajectory<TimedState<Pose2dWithMotion>> getCPFirstScoreToInsidePickup() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(1.016, -0.362, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(-180.0));

            waypoints.add(new Pose2d(2.8447999999999998, -0.362, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(-180.0));

            waypoints.add(new Pose2d(5.2, -1.5, Rotation2d.fromDegrees(-20.158531783)));
            headings.add(Rotation2d.fromDegrees(-60));

            waypoints.add(new Pose2d(5.5, -1.8, Rotation2d.fromDegrees(-25.158531783)));
            headings.add(Rotation2d.fromDegrees(-60));
            return generate(waypoints, headings, List.of(), false, 0.7, 1.57);
        }

        private Trajectory<TimedState<Pose2dWithMotion>> getCPOutsidePickupToSecondAlign() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(5.2832, -0.45, Rotation2d.fromDegrees(180)));
            headings.add(Rotation2d.fromDegrees(-0.01));
            waypoints.add(new Pose2d(2.845, -0.5, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(-179));
            waypoints.add(new Pose2d(1.2176, -0.55, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(-180));
            waypoints.add(new Pose2d(0.5016, -0.55, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(-180));
            waypoints.add(new Pose2d(0.1, -0.62, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(-180.0));
            return generate(waypoints, headings, List.of(), false, 0.7, 1.57);
        }

        private Trajectory<TimedState<Pose2dWithMotion>> getCPInsidePickupToSecondAlign() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(5.5, -1.8, Rotation2d.fromDegrees(180 - 25.158531783)));
            headings.add(Rotation2d.fromDegrees(-60));

            waypoints.add(new Pose2d(2.81, -0.762, Rotation2d.fromDegrees(180)));
            headings.add(Rotation2d.fromDegrees(180));

            waypoints.add(new Pose2d(1.9176, -0.762, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(180));
            
            waypoints.add(new Pose2d(0.8176, -0.762, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(180));

            waypoints.add(new Pose2d(0.1, -0.8, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(180));
            return generate(waypoints, headings, List.of(), false, 0.75, 1.57);
        }

        private Trajectory<TimedState<Pose2dWithMotion>> getCPSecondScoreToInsidePickup() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();

            waypoints.add(new Pose2d(0.1, -0.6, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(-180.0));

            waypoints.add(new Pose2d(1.016, -0.25, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(-180.0));

            waypoints.add(new Pose2d(2.8447999999999998, -0.3, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(-180.0));

            waypoints.add(new Pose2d(5.1, -1.6, Rotation2d.fromDegrees(-20.158531783)));
            headings.add(Rotation2d.fromDegrees(-60));

            waypoints.add(new Pose2d(5.5, -1.9, Rotation2d.fromDegrees(-25.158531783)));
            headings.add(Rotation2d.fromDegrees(-60));

            return generate(waypoints, headings, List.of(), false, 0.75, 1.57);
        }

        private Trajectory<TimedState<Pose2dWithMotion>> getCPSecondScoreToOutsidePickup() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();

            waypoints.add(new Pose2d(0.1, -0.8, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(-180.0));

            waypoints.add(new Pose2d(0.9652, -0.25, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(2.8447999999999998, -0.25, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(5.1832, -0.508, Rotation2d.fromDegrees(0)));
            headings.add(Rotation2d.fromDegrees(-0.01));
            waypoints.add(new Pose2d(5.2832, -0.508, Rotation2d.fromDegrees(0)));
            headings.add(Rotation2d.fromDegrees(-0.01));

            return generate(waypoints, headings, List.of(), false, 0.75, 1.57);
        }

        private Trajectory<TimedState<Pose2dWithMotion>> getCPInsidePickupToDock() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();

            waypoints.add(new Pose2d(5.5, -1.8, Rotation2d.fromDegrees(-135)));
            headings.add(Rotation2d.fromDegrees(-60));

            waypoints.add(new Pose2d(4.50, -2.25, Rotation2d.fromDegrees(-135)));
            headings.add(Rotation2d.fromDegrees(0.0));

            waypoints.add(new Pose2d(3.0, -2.25, Rotation2d.fromDegrees(180)));
            headings.add(Rotation2d.fromDegrees(0.0));

            return generate(waypoints, headings, List.of(), false, 0.7, 1.75);
        }

        private Trajectory<TimedState<Pose2dWithMotion>> getCPOutsidePickupToDock() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();

            waypoints.add(new Pose2d(5.2832, -0.508, Rotation2d.fromDegrees(-135)));
            headings.add(Rotation2d.fromDegrees(-0.01));

            waypoints.add(new Pose2d(4.50, -2.25, Rotation2d.fromDegrees(-135)));
            headings.add(Rotation2d.fromDegrees(0.0));

            waypoints.add(new Pose2d(3.0, -2.25, Rotation2d.fromDegrees(180)));
            headings.add(Rotation2d.fromDegrees(0.0));

            return generate(waypoints, headings, List.of(), false, 0.8, 1.75);
        }

        private Trajectory<TimedState<Pose2dWithMotion>> getCPInsidePickupToThirdScore() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();

            waypoints.add(new Pose2d(5.5, -1.9, Rotation2d.fromDegrees(180 - 25.158531783)));
            headings.add(Rotation2d.fromDegrees(-60));

            waypoints.add(new Pose2d(3.01, -0.75, Rotation2d.fromDegrees(180)));
            headings.add(Rotation2d.fromDegrees(180));

            waypoints.add(new Pose2d(1.9176, -0.75, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(180));

            waypoints.add(new Pose2d(0.8176, -0.75, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(180));

            waypoints.add(new Pose2d(0.1, -0.72, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(180));

            return generate(waypoints, headings, List.of(), false, 0.75, 1.57);
        }

        private Trajectory<TimedState<Pose2dWithMotion>> getCPOutsidePickupToThirdScore() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();

            waypoints.add(new Pose2d(5.2832, -0.508, Rotation2d.fromDegrees(180)));
            headings.add(Rotation2d.fromDegrees(-0.01));
            waypoints.add(new Pose2d(2.8447999999999998, -0.55, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(-179));
            waypoints.add(new Pose2d(1.2176, -0.55, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(-180));
            waypoints.add(new Pose2d(0.5016, -0.8, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(-180));
            waypoints.add(new Pose2d(0.2, -0.8, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(-180.0));

            return generate(waypoints, headings, List.of(), false, 0.75, 1.7);
        }

        private Trajectory<TimedState<Pose2dWithMotion>> getCPThirdScoreToBackoff() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();

            waypoints.add(new Pose2d(0.1, -0.72, Rotation2d.fromDegrees(45.0)));
            headings.add(Rotation2d.fromDegrees(180));
            waypoints.add(new Pose2d(1.0, -0.50, Rotation2d.fromDegrees(45.0)));
            headings.add(Rotation2d.fromDegrees(180));
            waypoints.add(new Pose2d(2.59, -0.5, Rotation2d.fromDegrees(45.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(5.18, -0.6, Rotation2d.fromDegrees(45.0)));
            headings.add(Rotation2d.fromDegrees(180.0));

            return generate(waypoints, headings, List.of(), false, 1.0, 2.0);
        }

        private Trajectory<TimedState<Pose2dWithMotion>> generate(List<Pose2d> waypoints, List<Rotation2d> headings, List<TimingConstraint<Pose2dWithMotion>> constraints, boolean reversed, double percentSpeed, double percentAccel) {
            handleAllianceFlip(waypoints, headings);

            return generateTrajectory(reversed, waypoints, headings, constraints,
                    percentSpeed * Constants.kMaxVelocityMetersPerSecond, percentAccel * kMaxAccel, kMaxVoltage);
        }


        private void handleAllianceFlip(List<Pose2d> waypoints, List<Rotation2d> headings) {
            SmartDashboard.putString("Alliance", DriverStation.getAlliance().toString());
            if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
                for (int i = 0; i < waypoints.size(); i++) {
                    waypoints.set(i, new Pose2d(new Translation2d(waypoints.get(i).getTranslation().x(), -waypoints.get(i).getTranslation().y()), waypoints.get(i).getRotation().inverse()));
                }
                for (int i = 0; i < headings.size(); i++) {
                    headings.set(i, headings.get(i).inverse());
                }
            }
        }

        private Trajectory<TimedState<Pose2dWithMotion>> getCoopLinkStartToCoopFirstPickup() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(0.75, 0.0, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(90.0));
            waypoints.add(new Pose2d(3.0, 0.0, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(90.0));
            waypoints.add(new Pose2d(6.0, 0.20, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(0.0));
            return generate(waypoints, headings, List.of(center_outbound_charging_station_constraint), false, 0.8, 1.375);
        }

        private Trajectory<TimedState<Pose2dWithMotion>> getCoopFirstPickupToCoopSecondScore() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(6.0, 0.20, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(0.0));
            waypoints.add(new Pose2d(4.5, -0.40, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(0.75, -0.40, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(0.2, -0.40, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            return generate(waypoints, headings, List.of(center_inbound_charging_station_constraint), false, 0.8, 1.375);
        }

        public Trajectory<TimedState<Pose2dWithMotion>> getNcpThirdScoreToBackoff() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();

            waypoints.add(new Pose2d(0.05, 0.55, Rotation2d.fromDegrees(-10.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(2.50, 0.05, Rotation2d.fromDegrees(-10.0)));
            headings.add(Rotation2d.fromDegrees(90.0));
            waypoints.add(new Pose2d(5.87, -0.47, Rotation2d.fromDegrees(-10.0)));
            headings.add(Rotation2d.fromDegrees(0.0));

            return generate(waypoints, headings, List.of(), false, 1.0, 2.0);
        }

        private Trajectory<TimedState<Pose2dWithMotion>> getCoopFirstScoreToDock() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(0.2, 0.00, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(2.0, 0.00, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            return generate(waypoints, headings, List.of(center_outbound_charging_station_constraint), false, 0.8, 1.0);
        }

        private Trajectory<TimedState<Pose2dWithMotion>> getCoopSecondScoreToDock() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(0.2, -0.40, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(2.0, -0.40, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            return generate(waypoints, headings, List.of(center_outbound_charging_station_constraint), false, 0.8, 1.0);
        }

        private Trajectory<TimedState<Pose2dWithMotion>> getRedNoTurnNCPRightScoringPositionToNCPFirstPickup() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(0)));
            headings.add(Rotation2d.fromDegrees(180));
            waypoints.add(new Pose2d(92.5, 10, Rotation2d.fromDegrees(0)));
            headings.add(Rotation2d.fromDegrees(90));
            waypoints.add(new Pose2d(185, 25, Rotation2d.fromDegrees(0)));
            headings.add(Rotation2d.fromDegrees(0));
            System.out.println(new Object() {}
                    .getClass()
                    .getEnclosingMethod()
                    .getName());
            convertToM(waypoints, headings);
            return generate(waypoints, headings, List.of(), false, 0.5, 1.0);
        }

        private Trajectory<TimedState<Pose2dWithMotion>> getNCPScoringtoCubePickup() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(1.143, 0.127, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(3.7592, -0.4588, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(134.99999999999994));
            waypoints.add(new Pose2d(5.534799999999999, -0.43179999999999996, Rotation2d.fromDegrees(90.0)));
            headings.add(Rotation2d.fromDegrees(134.99999999999994));
            waypoints.add(new Pose2d(5.3038, 0.5111999999999999, Rotation2d.fromDegrees(135)));
            headings.add(Rotation2d.fromDegrees(134.99999999999994));
            waypoints.add(new Pose2d(3.302, 0.308, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(1.2191999999999998, 0.408, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(0.05, 0.55, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            return generate(waypoints, headings, List.of(), false, 0.7, 1.0);
        }

        private Trajectory<TimedState<Pose2dWithMotion>> getRedNCPCubeScoringPositionToFarSideDockWithPickup() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(0.05, 0.55, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(1.143, 0.327, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(3.7592, 0.327, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(134.99999999999994));
            waypoints.add(new Pose2d(5.5, 0.8, Rotation2d.fromDegrees(90.0)));
            headings.add(Rotation2d.fromDegrees(134.99999999999994));
            waypoints.add(new Pose2d(5.5, 1.6, Rotation2d.fromDegrees(135)));
            headings.add(Rotation2d.fromDegrees(134.99999999999994));
            waypoints.add(new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(2.5, 2.0, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            return generate(waypoints, headings, List.of(), false, 0.7, 1.0);
        }

        private Trajectory<TimedState<Pose2dWithMotion>> getRedNCPCubeScoringPositionToThirdCube() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(0.05, 0.55, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(1.143, 0.327, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(3.7592, 0.327, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(134.99999999999994));
            waypoints.add(new Pose2d(5.5, 0.8, Rotation2d.fromDegrees(90.0)));
            headings.add(Rotation2d.fromDegrees(134.99999999999994));
            waypoints.add(new Pose2d(5.5, 1.6, Rotation2d.fromDegrees(135)));
            headings.add(Rotation2d.fromDegrees(134.99999999999994));
            waypoints.add(new Pose2d(3.302, 0.408, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(1.2191999999999998, 0.408, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(0.05, 0.55, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            return generate(waypoints, headings, List.of(), false, 0.7, 1.0);
        }

        private Trajectory<TimedState<Pose2dWithMotion>> getRedCPLeftScoringPositionToCPFirstPickup() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(2.3495, -0.1016, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(90.0));
            waypoints.add(new Pose2d(5.334, -0.254, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(0.0));
            return generate(waypoints, headings, List.of(), false, 0.5, 1.0);
        }

        private Trajectory<TimedState<Pose2dWithMotion>> getCPFirstPickupToFarSideDock() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(5.334, -0.254, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(0.0));
            waypoints.add(new Pose2d(4.191, -2.032, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(0.0));
            return generate(waypoints, headings, List.of(), false, 0.5, 1.0);
        }
    }
}