package com.team254.frc2023.planners;

import com.team254.frc2023.Constants;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.motion.IMotionProfileGoal;
import com.team254.lib.motion.MotionProfileGoal;
import com.team254.lib.motion.MotionState;
import com.team254.lib.motion.ProfileFollower;
import com.team254.lib.swerve.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

import java.util.OptionalDouble;

public class AutoAlignMotionPlanner {

    private ProfileFollower mXController = new ProfileFollower(2.5, 0.0, 0.0, 1.0, 0.0, 0.0);
    private ProfileFollower mYController = new ProfileFollower(2.5, 0.0, 0.0, 1.0, 0.0, 0.0);
    private ProfileFollower mThetaController = new ProfileFollower(1.5, 0.0, 0.0, 1.0, 0.0, 0.0);

    boolean mAutoAlignComplete = false;

    private Pose2d mFieldToTargetPoint;
    private OptionalDouble mStartTime;

    public void reset() {
        mStartTime = OptionalDouble.of(Timer.getFPGATimestamp());
        mXController.resetProfile();
        mXController.resetSetpoint();
        mYController.resetProfile();
        mYController.resetSetpoint();
        mThetaController.resetProfile();
        mThetaController.resetSetpoint();
        mAutoAlignComplete = false;
    }

    public synchronized void setTargetPoint(Pose2d targetPoint) {
        mFieldToTargetPoint = targetPoint;

    }

    public synchronized ChassisSpeeds updateAutoAlign(double timestamp, Pose2d current_odom_to_vehicle, Pose2d current_field_to_odom, Twist2d current_vel) {
        var odom_to_target_point = current_field_to_odom.inverse().transformBy(mFieldToTargetPoint);

        mXController.setGoalAndConstraints(
            new MotionProfileGoal(odom_to_target_point.getTranslation().x(), 0, IMotionProfileGoal.CompletionBehavior.VIOLATE_MAX_ACCEL, 0.08, 0.05),
            Constants.kPositionMotionProfileConstraints);
        mYController.setGoalAndConstraints(
            new MotionProfileGoal(odom_to_target_point.getTranslation().y(), 0, IMotionProfileGoal.CompletionBehavior.OVERSHOOT, 0.02, 0.05),
            Constants.kPositionMotionProfileConstraints);
        mThetaController.setGoalAndConstraints(
            new MotionProfileGoal(odom_to_target_point.getRotation().getRadians(), 0, IMotionProfileGoal.CompletionBehavior.OVERSHOOT, 0.03, 0.05),
            Constants.kHeadingMotionProfileConstraints);

        double currentRotation = current_field_to_odom.getRotation().rotateBy(current_odom_to_vehicle.getRotation()).getRadians();

        Translation2d current_vel_robot_frame = new Translation2d(current_vel.dx, current_vel.dy);
        Translation2d current_vel_odom_frame = current_vel_robot_frame.rotateBy(current_odom_to_vehicle.getRotation());

        if (odom_to_target_point.getRotation().getRadians() - currentRotation > Math.PI) {
            currentRotation += 2 * Math.PI;
        } else if (odom_to_target_point.getRotation().getRadians() - currentRotation < -Math.PI) {
            currentRotation -= 2 * Math.PI;
        }

        double xOutput = mXController.update(
               new MotionState(timestamp, current_odom_to_vehicle.getTranslation().x(), current_vel_odom_frame.x(), 0.0),
                timestamp + Constants.kLooperDt);
        double yOutput = mYController.update(
               new MotionState(timestamp, current_odom_to_vehicle.getTranslation().y(), current_vel_odom_frame.y(), 0.0),
                timestamp + Constants.kLooperDt);
        double thetaOutput = mThetaController.update(
                new MotionState(timestamp, currentRotation, current_vel.dtheta, 0.0),
                timestamp + Constants.kLooperDt);

        ChassisSpeeds setpoint;

        boolean thetaWithinDeadband = mThetaController.onTarget();
        boolean xWithinDeadband = mXController.onTarget();
        boolean yWithinDeadband = mYController.onTarget();

        setpoint = ChassisSpeeds.fromFieldRelativeSpeeds(
                xWithinDeadband ? 0.0 : xOutput,
                yWithinDeadband ? 0.0 : yOutput,
                thetaWithinDeadband ? 0.0 : thetaOutput,
                current_field_to_odom.getRotation().rotateBy(current_odom_to_vehicle.getRotation()));
        mAutoAlignComplete = thetaWithinDeadband && xWithinDeadband && yWithinDeadband;

        if (mStartTime.isPresent() && mAutoAlignComplete) {
            System.out.println("Auto align took: " + (Timer.getFPGATimestamp() - mStartTime.getAsDouble()));
            mStartTime = OptionalDouble.empty();
        }

        return setpoint;
    }

    public synchronized boolean getAutoAlignComplete() {
        return mAutoAlignComplete;
    }
}
