package com.team254.frc2023;

import com.team254.frc2023.limelight.VisionPoseAcceptor;
import com.team254.frc2023.subsystems.Limelight.VisionUpdate;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.kalman.UnscentedKalmanFilter;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;
import com.team254.lib.util.MovingAverageTwist2d;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;

import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Map;
import java.util.Optional;

public class RobotState {
    private static RobotState mInstance;
    private Optional<VisionUpdate> mLatestVisionUpdate;
    private UnscentedKalmanFilter<N2, N2, N2> mKalmanFilter;
    private VisionPoseAcceptor mPoseAcceptor;
    private Pose2d mDisplayVisionPose;
    private Pose2d mSetpointPose;

    public Field2d mField2d;

    private boolean mHasBeenEnabled = false;

    public static RobotState getInstance() {
        if (mInstance == null) {
            mInstance = new RobotState();
        }

        return mInstance;
    }

    private static final int kObservationBufferSize = 50;

    /*
     * RobotState keeps track of the poses of various coordinate frames throughout
     * the match. A coordinate frame is simply a point and direction in space that
     * defines an (x,y) coordinate system. Transforms (or poses) keep track of the
     * spatial relationship between different frames.
     *
     * Robot frames of interest (from parent to child):
     *
     * 1. Field frame: origin is the right near corner of the playing field.
     *
     * 2. Odom frame: origin is where the robot is turned on.
     *
     * 3. Vehicle frame: origin is the center of the robot wheelbase, facing
     * forwards
     *
     * 4. Camera frame: origin is the center of the Limelight relative to the
     * turret.
     *
     * 5. Target frame: origin is the center of the vision target, facing outwards
     * along the normal.
     *
     * As a kinematic chain with 5 frames, there are 4 transforms of interest:
     *
     * 1. Field-to-odom: This is derived from a fused field-to-vehicle transform incorporating both vision and odometry.
     *
     * 2. Odom-to-vehicle: Tracked by integrating encoder and gyro odometry.
     *
     * 3. Vehicle-to-camera: This is a constant (per camera).
     *
     * 4. Camera-to-target: Measured by the vision system.
     */
    private Optional<Translation2d> initial_field_to_odom_ = Optional.empty();
    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> odom_to_vehicle_;
    private InterpolatingTreeMap<InterpolatingDouble, Translation2d> field_to_odom_;

    private Twist2d vehicle_velocity_predicted_;
    private Twist2d vehicle_velocity_measured_;
    private MovingAverageTwist2d vehicle_velocity_measured_filtered_;

    private RobotState() {
        reset(0.0, Pose2d.identity());
    }


    public synchronized void reset(double start_time, Pose2d initial_odom_to_vehicle) {
        odom_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        odom_to_vehicle_.put(new InterpolatingDouble(start_time), initial_odom_to_vehicle);
        field_to_odom_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        field_to_odom_.put(new InterpolatingDouble(start_time), getInitialFieldToOdom().getTranslation());
        vehicle_velocity_predicted_ = Twist2d.identity();
        vehicle_velocity_measured_ = Twist2d.identity();
        vehicle_velocity_measured_filtered_ = new MovingAverageTwist2d(25);
        mLatestVisionUpdate = Optional.empty();
        mDisplayVisionPose = Pose2d.identity();
        mSetpointPose = Pose2d.identity();
        mPoseAcceptor = new VisionPoseAcceptor();

        mField2d = new Field2d();
        mField2d.setRobotPose(Constants.kWidthField2d, Constants.kHeightField2d, new edu.wpi.first.math.geometry.Rotation2d(0));
        //mField2d.getObject("vision").setPose(Constants.kWidthField2d, Constants.kHeightField2d, new edu.wpi.first.math.geometry.Rotation2d(0));
        //mField2d.getObject("fused").setPose(Constants.kWidthField2d, Constants.kHeightField2d, new edu.wpi.first.math.geometry.Rotation2d(0));
    }

    public synchronized void reset() {
        reset(Timer.getFPGATimestamp(), Pose2d.identity());
    }

    public synchronized void resetKalmanFilters() {
        mKalmanFilter =
        new UnscentedKalmanFilter<>(
            Nat.N2(),
            Nat.N2(),
            (x, u) -> VecBuilder.fill(0.0, 0.0),
            (x, u) -> x,
            Constants.kStateStdDevs,
            Constants.kLocalMeasurementStdDevs, Constants.kLooperDt);

    }

    public synchronized boolean getHasBeenEnabled() {
        return mHasBeenEnabled;
    }

    public synchronized void setHasBeenEnabled(boolean hasBeenEnabled) {
        mHasBeenEnabled = hasBeenEnabled;
    }

    /**
     * Returns the robot's position on the field at a certain time. Linearly interpolates between stored robot positions
     * to fill in the gaps.
     */
    public synchronized Pose2d getOdomToVehicle(double timestamp) {
        return odom_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestOdomToVehicle() {
        return odom_to_vehicle_.lastEntry();
    }

    public synchronized Pose2d getPredictedOdomToVehicle(double lookahead_time) {
        return getLatestOdomToVehicle().getValue()
                .transformBy(Pose2d.exp(vehicle_velocity_predicted_.scaled(lookahead_time)));
    }

    public synchronized void addOdomToVehicleObservation(double timestamp, Pose2d observation) {
        odom_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addOdomObservations(double timestamp, Pose2d odom_to_robot, Twist2d measured_velocity, Twist2d predicted_velocity) {
        try {
            mKalmanFilter.predict(VecBuilder.fill(0.0, 0.0), Constants.kLooperDt);
        } catch (Exception e) {
            DriverStation.reportError("QR Decomposition failed: ", e.getStackTrace());
        }
        addOdomToVehicleObservation(timestamp, odom_to_robot);

        vehicle_velocity_measured_ = measured_velocity;
        vehicle_velocity_measured_filtered_.add(vehicle_velocity_measured_);
        vehicle_velocity_predicted_ = predicted_velocity;
    }

    public synchronized Twist2d getPredictedVelocity() {
        return vehicle_velocity_predicted_;
    }

    public synchronized Twist2d getMeasuredVelocity() {
        return vehicle_velocity_measured_;
    }

    public synchronized Twist2d getSmoothedVelocity() {
        return vehicle_velocity_measured_filtered_.getAverage();
    }

    /**
     * Adds a Vision Update
     * @param visionUpdate
     */
    public synchronized void addVisionUpdate(VisionUpdate visionUpdate) {
        mLatestVisionUpdate = Optional.ofNullable(visionUpdate);
        if (!mLatestVisionUpdate.isEmpty()) {
            //Get the Timestamp of the Vision Reading
            double visionTimestamp = mLatestVisionUpdate.get().getTimestamp();
            
            Pose2d odomToVehicle = getOdomToVehicle(visionTimestamp);

            //Rotating Camera by Yaw Offset
            Pose2d cameraToTag = Pose2d.fromTranslation(mLatestVisionUpdate.get().getCameraToTag().rotateBy(Constants.kLimelightConstants.getYawOffset()));

            //Getting Vehicle to Tag in Field Frame
            Pose2d vehicleToTag = Pose2d.fromTranslation(Constants.kRobotToCamera.transformBy(cameraToTag).getTranslation().rotateBy(odomToVehicle.getRotation()));

            //Getting Field to Vehicle via Vehicle to Tag
            Pose2d visionFieldToVehicle = mLatestVisionUpdate.get().getFieldToTag().transformBy(vehicleToTag.inverse());

            if (!mPoseAcceptor.shouldAcceptVision(mLatestVisionUpdate.get().getTimestamp(), visionFieldToVehicle, vehicleToTag, vehicle_velocity_measured_)) {
                return;
            }

            boolean disabledAndNeverEnabled = DriverStation.isDisabled() && !mHasBeenEnabled;
            if (initial_field_to_odom_.isEmpty() || disabledAndNeverEnabled) {
                var odom_to_vehicle_translation = disabledAndNeverEnabled ? Translation2d.identity() : getOdomToVehicle(visionTimestamp).getTranslation();
                field_to_odom_.put(new InterpolatingDouble(visionTimestamp), visionFieldToVehicle.getTranslation().translateBy(odom_to_vehicle_translation.inverse()));
                initial_field_to_odom_ = Optional.of(field_to_odom_.lastEntry().getValue());
                mKalmanFilter.setXhat(0, field_to_odom_.lastEntry().getValue().x());
                mKalmanFilter.setXhat(1, field_to_odom_.lastEntry().getValue().y());
                mDisplayVisionPose = visionFieldToVehicle;

            } else if (DriverStation.isEnabled()) { 
                var field_to_odom = visionFieldToVehicle.getTranslation().translateBy(odomToVehicle.getTranslation().inverse());
                if(DriverStation.isAutonomous()) {
                    final double kMaxDistanceToAccept = 2.0;
                    if (field_to_odom.inverse().translateBy(field_to_odom_.lastEntry().getValue()).norm() > kMaxDistanceToAccept) {
                        System.out.println("Invalid vision update!");
                        return;
                    }
                }

                mDisplayVisionPose = visionFieldToVehicle;
                try {
                    mKalmanFilter.correct(VecBuilder.fill(0.0, 0.0), VecBuilder.fill(field_to_odom.getTranslation().x(), field_to_odom.getTranslation().y()));
                    field_to_odom_.put(new InterpolatingDouble(visionTimestamp), Pose2d.fromTranslation(new Translation2d(mKalmanFilter.getXhat(0), mKalmanFilter.getXhat(1))).getTranslation());
                } catch (Exception e) {
                    DriverStation.reportError("QR Decomposition failed: ", e.getStackTrace());
                }
            } else {
                mDisplayVisionPose = null;
            }
        }
    }

    public synchronized Pose2d getDisplayVisionPose() {
        if (mDisplayVisionPose == null) {
            return Pose2d.fromTranslation(new Translation2d(Constants.kOutofFrameValue, Constants.kOutofFrameValue)); //Out of frame
        }
        return mDisplayVisionPose;
    }

    /**
     * Return Initial Vision Offset for Pure Odometry Visualization Purposes
     * @return
     */
    public synchronized Pose2d getInitialFieldToOdom() {
        if (initial_field_to_odom_.isEmpty()) return Pose2d.identity();
        return Pose2d.fromTranslation(initial_field_to_odom_.get());
    }

    public synchronized Translation2d getFieldToOdom(double timestamp) {
        if (initial_field_to_odom_.isEmpty()) return Translation2d.identity();
        return initial_field_to_odom_.get().inverse().translateBy(field_to_odom_.getInterpolated(new InterpolatingDouble(timestamp)));
    }

    public synchronized Translation2d getAbsoluteFieldToOdom(double timestamp) {
        return field_to_odom_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Translation2d getLatestFieldToOdom() {
        return getFieldToOdom(field_to_odom_.lastKey().value);
    }

    /**
     * Get Current Field to Vehicle using Filter Idea 1 (Offset Space) => Add the Offset outputted by the Filter to Current Odom
     * @param timestamp
     * @return
     */
    public synchronized Pose2d getFieldToVehicle(double timestamp) {
        Pose2d odomToVehicle = getOdomToVehicle(timestamp);

        Translation2d fieldToOdom = getFieldToOdom(timestamp);
        return new Pose2d(fieldToOdom.translateBy(odomToVehicle.getTranslation()), odomToVehicle.getRotation());

    }

    public synchronized Pose2d getFieldToVehicleAbsolute(double timestamp) {
        var field_to_odom = initial_field_to_odom_.orElse(Translation2d.identity());
        return Pose2d.fromTranslation(field_to_odom).transformBy(getFieldToVehicle(timestamp));
    }

    /**d
     *
     * @return
     */
    public synchronized Pose2d getLatestFieldToVehicle() {
        Pose2d odomToVehicle = getLatestOdomToVehicle().getValue();
        return new Pose2d(getLatestFieldToOdom().getTranslation().add(odomToVehicle.getTranslation()), odomToVehicle.getRotation());
    }


    public synchronized Optional<VisionUpdate> getLatestVisionUpdate() {
        return mLatestVisionUpdate;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("Robot Velocity", getMeasuredVelocity().toString());

        SmartDashboard.putString("Odom To Robot", getOdomToVehicle(Timer.getFPGATimestamp()).toString());

        SmartDashboard.putString("Field To Robot", getFieldToVehicle(Timer.getFPGATimestamp()).toString());

        SmartDashboard.putNumber("Field To Robot X", getFieldToVehicle(Timer.getFPGATimestamp()).getTranslation().x());
        double fieldX = getFieldToVehicle(Timer.getFPGATimestamp()).getTranslation().x();
        double odomX = getOdomToVehicle(Timer.getFPGATimestamp()).getTranslation().x() + getInitialFieldToOdom().getTranslation().x();

        double [] arr = new double[]{fieldX, odomX};
        SmartDashboard.putNumberArray("Poses", arr);
        SmartDashboard.putNumber("Field To Robot Y", getFieldToVehicle(Timer.getFPGATimestamp()).getTranslation().y());
        SmartDashboard.putNumber("Field To Robot Theta", getFieldToVehicle(Timer.getFPGATimestamp()).getRotation().getDegrees());

        SmartDashboard.putNumber("Odom X", getOdomToVehicle(Timer.getFPGATimestamp()).getTranslation().x());
        SmartDashboard.putNumber("Odom Y", getOdomToVehicle(Timer.getFPGATimestamp()).getTranslation().y());
        SmartDashboard.putNumber("Odom Theta", getOdomToVehicle(Timer.getFPGATimestamp()).getRotation().getDegrees());

        SmartDashboard.putString("Field to Odom Offset", getLatestFieldToOdom().toString());
        SmartDashboard.putString("initial offset", initial_field_to_odom_.toString());
   }

   public synchronized void displayField() {
        double timestamp = Timer.getFPGATimestamp();
        var displayVisionPose = getDisplayVisionPose();
        var fusedPose = getFieldToVehicleAbsolute(timestamp);
        var setpointPose = mSetpointPose;
        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            mField2d.getObject("vision").setPose(displayVisionPose.getTranslation().x(), displayVisionPose.getTranslation().y(), new edu.wpi.first.math.geometry.Rotation2d(displayVisionPose.getRotation().getRadians()));
            mField2d.getObject("fused").setPose(fusedPose.getTranslation().x(), fusedPose.getTranslation().y(), new edu.wpi.first.math.geometry.Rotation2d(fusedPose.getRotation().getRadians()));
            mField2d.getObject("setpoint").setPose(setpointPose.getTranslation().x(), setpointPose.getTranslation().y(), new edu.wpi.first.math.geometry.Rotation2d(setpointPose.getRotation().getRadians()));
        } else {
            mField2d.getObject("vision").setPose(Constants.kWidthField2d - displayVisionPose.getTranslation().x(), Constants.kHeightField2d - displayVisionPose.getTranslation().y(), new edu.wpi.first.math.geometry.Rotation2d(displayVisionPose.getRotation().getRadians() + Math.PI));
            mField2d.getObject("fused").setPose(Constants.kWidthField2d - fusedPose.getTranslation().x(), Constants.kHeightField2d - fusedPose.getTranslation().y(), new edu.wpi.first.math.geometry.Rotation2d(fusedPose.getRotation().getRadians() + Math.PI));
            mField2d.getObject("setpoint").setPose(Constants.kWidthField2d - setpointPose.getTranslation().x(), Constants.kHeightField2d - setpointPose.getTranslation().y(), new edu.wpi.first.math.geometry.Rotation2d(setpointPose.getRotation().getRadians() + Math.PI));
        }
        SmartDashboard.putData("field", mField2d);
    }

    public Pose2d getFieldToGoal() {
        return new Pose2d();
    }

    public void setDisplaySetpointPose(Pose2d setpoint) {
        mSetpointPose = setpoint;
    }
}