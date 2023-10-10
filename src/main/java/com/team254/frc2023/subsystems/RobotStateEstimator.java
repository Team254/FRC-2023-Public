package com.team254.frc2023.subsystems;

import com.team254.frc2023.Constants;
import com.team254.frc2023.RobotState;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import com.team254.lib.motion.MotionState;
import com.team254.lib.swerve.SwerveDriveOdometry;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.Util;

import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.Timer;

import com.team254.lib.drivers.Subsystem;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Twist2d;

public class RobotStateEstimator extends Subsystem {
    static RobotStateEstimator mInstance = null;
    private Drive mDrive = null;
    private Elevator mElevator;
    private Laterator mLaterator;
    private final SwerveDriveOdometry mOdometry = new SwerveDriveOdometry(Constants.kKinematics, Pose2d.identity());

    public static class OdometryLogValues {
        public double timestamp;
        public double frontLeftPosition;
        public double frontRightPosition;
        public double backLeftPosition;
        public double backRightPosition;

        public double frontLeftVelocity;
        public double frontRightVelocity;
        public double backLeftVelocity;
        public double backRightVelocity;

        public double frontLeftAngle;
        public double frontRightAngle;
        public double backLeftAngle;
        public double backRightAngle;

        public double gyroAngle;

        public double frontLeftAngleSetpoint;
        public double frontRightAngleSetpoint;
        public double backLeftAngleSetpoint;
        public double backRightAngleSetpoint;

        public double frontLeftError;
        public double frontRightError;
        public double backLeftError;
        public double backRightError;

        public double timestamp1;
        public double poseX;
        public double autonXSetpoint;
        public double xError;
        public double timestamp2;
        public double poseY;
        public double autonYSetpoint;
        public double yError;
        public double timestamp3;
        public double poseTheta;
        public double autonHeadingSetpoint;
        public double headingError;

        public double timestamp4;
        public double xSetpointPos;
        public double xSetpointVel;
        public double timestamp5;
        public double ySetpointPos;
        public double ySetpointVel;
        public double timestamp6;
        public double thetaSetpointPos;
        public double thetaSetpointVel;

        public double elevator_pos;
        public double elevator_vel;
        public double laterator_pos;
        public double laterator_vel;
        public double elevator_setpoint;
        public double laterator_setpoint;

        public String controlState;


        public OdometryLogValues(double timestamp, double frontLeftPosition, double frontRightPosition, double backLeftPosition, double backRightPosition,
                                 double frontLeftVelocity, double frontRightVelocity, double backLeftVelocity, double backRightVelocity,
                                 double frontLeftAngle, double frontRightAngle, double backLeftAngle, double backRightAngle,
                                 double flCLosedLoopErr,double frCLosedLoopErr,double blCLosedLoopErr,double brCLosedLoopErr,
                                 double gyroAngle, double poseX, double poseY, double poseTheta,
                                 double frontLeftAngleSetpoint, double frontRightAngleSetpoint, double backLeftAngleSetpoint, double backRightAngleSetpoint,
                                 double xSetpoint, double ySetpoint, double headingSetpoint, double xError, double yError, double headingError, double xSetpointPos,
                                 double xSetpointVel, double ySetpointPos, double ySetpointVel, double thetaSetpointPos, double thetaSetpointVel,
                                 double elevator_pos, double elevator_vel, double laterator_pos, double laterator_vel, double elevator_setpoint, double laterator_setpoint, String controlState) {
            this.timestamp = timestamp;
            this.timestamp1 = timestamp;
            this.timestamp2 = timestamp;
            this.timestamp3 = timestamp;
            this.timestamp4 = timestamp;
            this.timestamp5 = timestamp;
            this.timestamp6 = timestamp;
            this.frontLeftPosition = frontLeftPosition;
            this.frontRightPosition = frontRightPosition;
            this.backLeftPosition = backLeftPosition;
            this.backRightPosition = backRightPosition;
            this.frontLeftVelocity = frontLeftVelocity;
            this.frontRightVelocity = frontRightVelocity;
            this.backLeftVelocity = backLeftVelocity;
            this.backRightVelocity = backRightVelocity;
            this.frontLeftAngle = frontLeftAngle;
            this.frontRightAngle = frontRightAngle;
            this.backLeftAngle = backLeftAngle;
            this.backRightAngle = backRightAngle;
            this.gyroAngle = gyroAngle;
            this.poseX = poseX;
            this.poseY = poseY;
            this.poseTheta = poseTheta;
            this.frontLeftAngleSetpoint = frontLeftAngleSetpoint;
            this.frontRightAngleSetpoint = frontRightAngleSetpoint;
            this.backLeftAngleSetpoint = backLeftAngleSetpoint;
            this.backRightAngleSetpoint = backRightAngleSetpoint;
            this.autonXSetpoint = xSetpoint;
            this.autonYSetpoint = ySetpoint;
            this.autonHeadingSetpoint = headingSetpoint;
            this.xError = xError;
            this.yError = yError;
            this.headingError = headingError;
            this.frontLeftError = flCLosedLoopErr;
            this.frontRightError = frCLosedLoopErr;
            this.backLeftError = blCLosedLoopErr;
            this.backRightError = brCLosedLoopErr;
            this.xSetpointPos = xSetpointPos;
            this.xSetpointVel = xSetpointVel;
            this.ySetpointPos = ySetpointPos;
            this.ySetpointVel = ySetpointVel;
            this.thetaSetpointPos = thetaSetpointPos;
            this.thetaSetpointVel = thetaSetpointVel;
            this.elevator_pos = elevator_pos;
            this.elevator_vel = elevator_vel;
            this.laterator_pos = laterator_pos;
            this.laterator_vel = laterator_vel;
            this.elevator_setpoint = elevator_setpoint;
            this.laterator_setpoint = laterator_setpoint;
            this.controlState = controlState;
        }
    }

    private ReflectingCSVWriter<OdometryLogValues> mCSVWriter = null;

    public static RobotStateEstimator getInstance() {
        if (mInstance == null) {
            mInstance = new RobotStateEstimator();
        }

        return mInstance;
    }

    private RobotStateEstimator() {
        mDrive = Drive.getInstance();
        mElevator = Elevator.getInstance();
        mLaterator = Laterator.getInstance();
    }

    @Override
    public void registerEnabledLoops(ILooper looper) {
        looper.register(new EnabledLoop());
    }

    private class EnabledLoop implements Loop {
        @Override
        public synchronized void onStart(double timestamp) {
        }

        @Override
        public synchronized void onLoop(double timestamp) {
            synchronized(RobotStateEstimator.this) {
                mOdometry.updateWithWheelConstraints(timestamp, mDrive.getFieldRelativeGyroscopeRotation(), mDrive.getModuleStates());
                Twist2d measured_velocity = mOdometry.getVelocity().toTwist2d();
                Twist2d predicted_velocity = mDrive.getSetpoint().mChassisSpeeds.toTwist2d();
                RobotState.getInstance().addOdomObservations(timestamp, mOdometry.getPoseMeters(),
                        measured_velocity, predicted_velocity);

                if (mCSVWriter != null) {
                    try {
                        logCSV();
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            stopLogging();
        }
    }

    @Override
    public void stop() {}

    public void resetOdometry(Pose2d initialPose) {
        synchronized(RobotStateEstimator.this) {
            mOdometry.resetPosition(initialPose);
            RobotState.getInstance().reset(Timer.getFPGATimestamp(), initialPose);
        }
    }

    public void resetGyro(Rotation2d gyro) {
        synchronized(RobotStateEstimator.this) {
            mOdometry.resetPosition(new Pose2d(mOdometry.getPoseMeters().getTranslation(), gyro));
        }
    }

    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/AUTO-LOGS.csv", OdometryLogValues.class);
        }
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }

    private synchronized void logCSV() {
        double currentTime = WPIUtilJNI.now() * 1.0e-6;

        var modules_meas = mDrive.getModuleStates();
        var modules_des = mDrive.getDesiredModuleStates();


        double frontLeftPosition = modules_meas[Drive.kFrontLeftModuleIdx].distanceMeters;
        double frontRightPosition = modules_meas[Drive.kFrontRightModuleIdx].distanceMeters;
        double backLeftPosition = modules_meas[Drive.kBackLeftModuleIdx].distanceMeters;
        double backRightPosition = modules_meas[Drive.kBackRightModuleIdx].distanceMeters;

        double frontLeftVelocity = modules_meas[Drive.kFrontLeftModuleIdx].speedMetersPerSecond;
        double frontRightVelocity = modules_meas[Drive.kFrontRightModuleIdx].speedMetersPerSecond;
        double backLeftVelocity = modules_meas[Drive.kBackLeftModuleIdx].speedMetersPerSecond;
        double backRightVelocity = modules_meas[Drive.kBackRightModuleIdx].speedMetersPerSecond;

        double frontLeftAngle = modules_meas[Drive.kFrontLeftModuleIdx].angle.getRadians();
        double frontRightAngle = modules_meas[Drive.kFrontRightModuleIdx].angle.getRadians();
        double backLeftAngle = modules_meas[Drive.kBackLeftModuleIdx].angle.getRadians();
        double backRightAngle = modules_meas[Drive.kBackRightModuleIdx].angle.getRadians();

        double[] closed_loop_err = mDrive.getSteerClosedLoopErrors();
        double flErr = closed_loop_err[Drive.kFrontLeftModuleIdx];
        double frErr = closed_loop_err[Drive.kFrontRightModuleIdx];
        double brErr = closed_loop_err[Drive.kBackRightModuleIdx];
        double blErr = closed_loop_err[Drive.kBackLeftModuleIdx];

        boolean stationary = mOdometry.getVelocity().toTwist2d().epsilonEquals(Twist2d.identity(), Util.kEpsilon);

        Pose2d currentPose = mOdometry.getPoseMeters();

        var desiredStates = mDrive.getSetpoint().mModuleStates;

        Pose2d autonSetpoint = mDrive.getAutonSetpoint();
        Pose2d autonError = mDrive.getAutonError();
        MotionState xSetpoint = mDrive.getXSetpoint();
        MotionState ySetpoint = mDrive.getYSetpoint();
        MotionState thetaSetpoint = mDrive.getThetaSetpoint();

        double elevator_pos = mElevator.getPosition();
        double elevator_vel = mElevator.getVelocity();
        double laterator_pos = mLaterator.getPosition();
        double laterator_vel = mLaterator.getVelocity();
        double elevator_setpoint = mElevator.getSetpoint();
        double laterator_setpoint = mLaterator.getSetpoint();

        String controlState = mElevator.getControlState();

        OdometryLogValues logMsg = new OdometryLogValues(
                currentTime,
                frontLeftPosition,
                frontRightPosition,
                backLeftPosition,
                backRightPosition,
                frontLeftVelocity,
                frontRightVelocity,
                backLeftVelocity,
                backRightVelocity,
                frontLeftAngle,
                frontRightAngle,
                backLeftAngle,
                backRightAngle,
                flErr,
                frErr,
                blErr,
                brErr,
                mDrive.getFieldRelativeGyroscopeRotation().getDegrees(),
                currentPose.getTranslation().x(),
                currentPose.getTranslation().y(),
                currentPose.getRotation().getDegrees(),
                stationary ? frontLeftAngle : desiredStates[Drive.kFrontLeftModuleIdx].angle.getRadians(),
                stationary ? frontRightAngle : desiredStates[Drive.kFrontRightModuleIdx].angle.getRadians(),
                stationary ? backLeftAngle : desiredStates[Drive.kBackLeftModuleIdx].angle.getRadians(),
                stationary ? backRightAngle : desiredStates[Drive.kBackRightModuleIdx].angle.getRadians(),
                autonSetpoint.getTranslation().x(),
                autonSetpoint.getTranslation().y(),
                autonSetpoint.getRotation().getDegrees(),
                autonError.getTranslation().x(),
                autonError.getTranslation().y(),
                autonError.getRotation().getDegrees(),
                xSetpoint.pos(),
                xSetpoint.vel(),
                ySetpoint.pos(),
                ySetpoint.vel(),
                thetaSetpoint.pos(),
                thetaSetpoint.vel(),
                elevator_pos,
                elevator_vel,
                laterator_pos,
                laterator_vel,
                elevator_setpoint,
                laterator_setpoint,
                controlState
        );

        mCSVWriter.add(logMsg);
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public synchronized void outputTelemetry(boolean disabled) {
        if (disabled) {
            RobotState.getInstance().outputToSmartDashboard();
        }
        RobotState.getInstance().displayField();
    }
}