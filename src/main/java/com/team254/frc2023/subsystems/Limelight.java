package com.team254.frc2023.subsystems;


import com.team254.frc2023.Constants;
import com.team254.frc2023.RobotState;
import com.team254.frc2023.field.AprilTag;
import com.team254.frc2023.field.Field;
import com.team254.frc2023.limelight.undistort.UndistortMap;
import com.team254.lib.drivers.Subsystem;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import com.team254.lib.vision.TargetInfo;
import com.team254.lib.util.Units;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable.TableEventListener;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;

import java.util.*;

import static org.opencv.core.CvType.CV_64FC1;

/**
 * Subsystem for interacting with the Limelight 3
 */
public class Limelight extends Subsystem {
    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    private final NetworkTable mNetworkTable;
    private static HashMap<Integer, AprilTag> mTagMap = Field.Red.kAprilTagMap;
    private boolean mOutputsHaveChanged = true;
    private int mListenerId = -1;

    private final PeriodicIO mPeriodicIO = new PeriodicIO();
    private static Limelight mInstance;

    private Mat mCameraMatrix = new Mat(3, 3, CV_64FC1);
    private Mat mDistortionCoeffients = new Mat(1, 5, CV_64FC1);

    private boolean mDisableProcessing = false;

    public static Limelight getInstance() {
        if (mInstance == null) {
            mInstance = new Limelight();
        }
        return mInstance;
    }

    private Limelight() {
        mNetworkTable = NetworkTableInstance.getDefault().getTable("limelight");
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                mCameraMatrix.put(i, j, Constants.kLimelightConstants.getUndistortConstants().getCameraMatrix()[i][j]);
            }
        }
        for (int i = 0; i < 5; i++) {
            mDistortionCoeffients.put(0, i, Constants.kLimelightConstants.getUndistortConstants().getCameraDistortion()[i]);
        }
    }


    /**
     * Represents a Class to Store a Vision Update
     * Allows us to maintain the timestamp of a vision capture along with the corresponding 2d Translation from Camera to Goal
     */
    public static class VisionUpdate {
        private double timestamp;
        private Translation2d cameraToTarget;
        private int tagId;
        private Pose2d fieldToTag;

        public VisionUpdate(double timestamp, Translation2d cameraToTarget, int tagId) {
            this.timestamp = timestamp;
            this.cameraToTarget = cameraToTarget;
            this.fieldToTag = mTagMap.get(tagId).getFieldToTag();
        }

        public double getTimestamp() {
            return timestamp;
        }

        public Translation2d getCameraToTag()  {
            return cameraToTarget;
        }
        public Pose2d getFieldToTag() {
            return fieldToTag;
        }

        public int getTagId() {
            return tagId;
        }
    }

    private class Listener implements TableEventListener {
        @Override
        public void accept(NetworkTable table, String key, NetworkTableEvent event) {
            if (key.equals("json")) {
                if (!mDisableProcessing) {
                    readInputsAndAddVisionUpdate();
                }
            }
        }
    }

    public void setBlueTagMap() {
        mTagMap = Field.Blue.kAprilTagMap;
    }


    public static class PeriodicIO {

        // INPUTS
        public double latency;
        public Number[] corners;
        public int givenLedMode;
        public int givenPipeline;
        public boolean seesTarget;
        public int tagId;
        public double imageCaptureLatency;

        // OUTPUTS
        public int ledMode = 0; // 0 - use pipeline mode, 1 - off, 2 - blink, 3 - on
        public int camMode = 0; // 0 - vision processing, 1 - driver camera
        public int pipeline = 0; // 0 - 9
        public double stream = 0; // sets stream layout if another webcam is attached
        public double snapshot = 0; // 0 - stop snapshots, 1 - 2 Hz
    }

    /**
     * Reads from Network Tables and sends an update to Robot State
     * Called on an interrupt
     */
    private void readInputsAndAddVisionUpdate() {
        final double timestamp = Timer.getFPGATimestamp();
        mPeriodicIO.imageCaptureLatency = mNetworkTable.getEntry("cl").getDouble(Constants.kImageCaptureLatency);
        mPeriodicIO.latency = mNetworkTable.getEntry("tl").getDouble(0) / 1000.0 + mPeriodicIO.imageCaptureLatency / 1000.0
                + Constants.kLimelightTransmissionTimeLatency;
        mPeriodicIO.givenPipeline = (int) mNetworkTable.getEntry("getpipe").getDouble(0);
        mPeriodicIO.seesTarget = mNetworkTable.getEntry("tv").getDouble(0) == 1.0;
        mPeriodicIO.tagId = (int) mNetworkTable.getEntry("tid").getNumber(-1).doubleValue();
        mPeriodicIO.givenLedMode = (int) mNetworkTable.getEntry("ledMode").getDouble(1.0);
        mPeriodicIO.corners = mNetworkTable.getEntry("tcornxy").getNumberArray(new Number[] { 0, 0, 0, 0, 0 });
        Translation2d cameraToTarget = getCameraToTargetTranslation();
        int tagId = mPeriodicIO.tagId;

        if (mPeriodicIO.seesTarget) {
            if (mTagMap.keySet().contains(tagId) && cameraToTarget != null) {
                RobotState.getInstance().addVisionUpdate(
                        new VisionUpdate(timestamp - mPeriodicIO.latency, cameraToTarget, tagId));
            } else {
                RobotState.getInstance().addVisionUpdate(null);
            }
        }
    }

    @Override
    public void readPeriodicInputs() {
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mPeriodicIO.givenLedMode != mPeriodicIO.ledMode ||
                mPeriodicIO.givenPipeline != mPeriodicIO.pipeline) {
            System.out.println("Table has changed from expected, retrigger!!");
            mOutputsHaveChanged = true;
        }
        if (mOutputsHaveChanged) {
            mNetworkTable.getEntry("ledMode").setNumber(mPeriodicIO.ledMode);
            mNetworkTable.getEntry("camMode").setNumber(mPeriodicIO.camMode);
            mNetworkTable.getEntry("pipeline").setNumber(mPeriodicIO.pipeline);
            mNetworkTable.getEntry("stream").setNumber(mPeriodicIO.stream);
            mNetworkTable.getEntry("snapshot").setNumber(mPeriodicIO.snapshot);

            mOutputsHaveChanged = false;
        }
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                setLed(LedMode.PIPELINE);
                start();
            }

            @Override
            public void onLoop(double timestamp) {
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    /**
     * Starts the Listener
     */
    public synchronized void start() {
        if (mListenerId < 0) {
            mListenerId = mNetworkTable.addListener("json", EnumSet.of(Kind.kValueAll), new Listener());
        }
    }


    @Override
    public void stop() { }

    @Override
    public boolean checkSystem() {
        return false;
    }

    public enum LedMode {
        PIPELINE, OFF, BLINK, ON
    }

    public synchronized void setLed(LedMode mode) {
        if (mode.ordinal() != mPeriodicIO.ledMode) {
            mPeriodicIO.ledMode = mode.ordinal();
            mOutputsHaveChanged = true;
        }
    }

    /**
     * Set Pipeline Mode
     * @param mode mode number
     */
    public synchronized void setPipelineNumber(int mode) {
        if (mode != mPeriodicIO.pipeline) {
            mPeriodicIO.pipeline = mode;

            System.out.println(mPeriodicIO.pipeline + ", " + mode);
            mOutputsHaveChanged = true;
        }
    }

    /**
     * Returns the Latency of the Limelight
     * @return latency
     */
    public double getLatency() {
        return mPeriodicIO.latency;
    }

    /**
     * Returns Pipeline Number
     * @return pipeline
     */
    public synchronized int getPipeline() {
        return mPeriodicIO.pipeline;
    }

    /**
     * Returns whether LL sees target
     * @return if LL sees target
     */
    public synchronized boolean hasTarget() {
        return mPeriodicIO.seesTarget;
    }


    /**
     * Returns to the 2D Translation from the Camera to the Center of the April Tag
     * @return
     */
    public synchronized Translation2d getCameraToTargetTranslation() {
        //Get all Corners Normalized
        List<TargetInfo> targetPoints  = getTarget();
        if (targetPoints == null || targetPoints.size() < 4) {
            return null;
        }
        //Project Each Corner into XYZ Space
        Translation2d cameraToTagTranslation = Translation2d.identity();
        List<Translation2d> cornerTranslations = new ArrayList<>(targetPoints.size());
        for (int i = 0; i < targetPoints.size(); i++) {
            Translation2d cameraToCorner;

            //Add 3 Inches to the Height of Top Corners
            if (i < 2) {
                cameraToCorner = getCameraToPointTranslation(targetPoints.get(i), true);
            } else {
                cameraToCorner = getCameraToPointTranslation(targetPoints.get(i), false);
            }
            if (cameraToCorner == null) {
                return null;
            }
            cornerTranslations.add(cameraToCorner);
            cameraToTagTranslation = cameraToTagTranslation.translateBy(cameraToCorner);
        }

        //Divide by 4 to get the average Camera to Goal Translation
        cameraToTagTranslation = cameraToTagTranslation.scale(0.25);

        return cameraToTagTranslation;

    }

    /**
     * Pinhole Camera Calculations
     * @param target Normalized Coordinates
     * @param isTopCorner whether the point we're receiving is a Top Corner
     * @return the 2D Translation from Camera to Goal
     */
    public synchronized Translation2d getCameraToPointTranslation(TargetInfo target, boolean isTopCorner) {
        // Compensate for camera pitch
        Translation2d xz_plane_translation = new Translation2d(target.getX(), target.getZ()).rotateBy(Rotation2d.fromDegrees(Constants.kLimelightConstants.getHorizontalPlaneToLens().getDegrees()));
        double x = xz_plane_translation.x();
        double y = target.getY();
        double z = xz_plane_translation.y();

        double offset = isTopCorner ? Units.inches_to_meters(3) : - Units.inches_to_meters(3);
        // find intersection with the goal
        double differential_height = mTagMap.get(target.getTagId()).getHeight() - Constants.kLensHeight + offset;
        if ((z > 0.0) == (differential_height > 0.0)) {
            double scaling = differential_height / z;
            double distance = Math.hypot(x, y) * scaling;
            Rotation2d angle = new Rotation2d(x, y, true);
            return new Translation2d(distance * angle.cos(), distance * angle.sin());
        }
        return null;
    }


    private static final Comparator<Translation2d> ySort = Comparator.comparingDouble(Translation2d::y);

    /**
     * Get the Normalized Corners
     * @return
     */
    public List<TargetInfo> getTarget() {
        // Get corners
        List<Translation2d> corners = getCorners(mPeriodicIO.corners);

        if (corners.size() < 4 || !mTagMap.containsKey(mPeriodicIO.tagId)) {
            return null;
        }

        // Sort by y, list will have "highest in image" corner first
        corners.sort(ySort);
        ArrayList<TargetInfo> targetInfos = new ArrayList<>();

        for (Translation2d corner : corners) {
            targetInfos.add(getRawTargetInfo(new Translation2d(corner.x(), corner.y()), getTagId()));

        }


        return targetInfos;
    }

    /**
     * Returns Normalized Undistorted View Plane Coordinate
     * @param desiredTargetPixel Raw Pixel Value
     * @param tagId Tag ID
     * @return Normalized Target Info ready for Pinhole Calculations
     */
    public synchronized TargetInfo getRawTargetInfo(Translation2d desiredTargetPixel, int tagId) {
        if (desiredTargetPixel == null) {
            return null;
        } else {
            double[] undistortedNormalizedPixelValues;
            UndistortMap undistortMap = Constants.kLimelightConstants.getUndistortMap();
            if (undistortMap == null) {
                try {
                    undistortedNormalizedPixelValues = undistortFromOpenCV(new double[]{desiredTargetPixel.x() / Constants.kResolutionWidth, desiredTargetPixel.y() / Constants.kResolutionHeight});
                } catch (Exception e) {
                    DriverStation.reportError("Undistorting Point Throwing Error!", false);
                    return null;
                }
            } else {
                undistortedNormalizedPixelValues = undistortMap.pixelToUndistortedNormalized((int) desiredTargetPixel.x(), (int) desiredTargetPixel.y());
            }

            double y_pixels = undistortedNormalizedPixelValues[0];
            double z_pixels = undistortedNormalizedPixelValues[1];


            //Negate OpenCV Undistorted Pixel Values to Match Robot Frame of Reference
            //OpenCV: Positive Downward and Right
            //Robot: Positive Upward and Left
            double nY = -(y_pixels - mCameraMatrix.get(0, 2)[0]);// -(y_pixels * 2.0 - 1.0);
            double nZ = -(z_pixels - mCameraMatrix.get(1, 2)[0]);// -(z_pixels * 2.0 - 1.0);

            double y = nY / mCameraMatrix.get(0, 0)[0];
            double z = nZ / mCameraMatrix.get(1, 1)[0];

            return new TargetInfo(y, z, tagId);
        }
    }


    /**
     * Undoes radial and tangential distortion using opencv
     */
    public synchronized double[] undistortFromOpenCV(double[] point) throws Exception {
        Point coord = new Point();
        coord.x = point[0];
        coord.y = point[1];

        MatOfPoint2f coordMat = new MatOfPoint2f(coord);

        if (coordMat.empty() || mCameraMatrix.empty() || mDistortionCoeffients.empty()) throw new Exception("Matrix Required For Undistortion Is Empty!");

        Point dstCoord = new Point();
        MatOfPoint2f dst = new MatOfPoint2f(dstCoord);
        Calib3d.undistortImagePoints(coordMat, dst, mCameraMatrix, mDistortionCoeffients);

        if (dst.empty() || dst.rows() < 1 || dst.cols() < 1) throw new Exception("Undistorted Point Matrix is Empty or undersized!");

        return dst.get(0, 0);
    }

    /**
     * Returns the Tag Id
     * @return
     */
    public int getTagId() {
        return mPeriodicIO.tagId;
    }


    /**
     * Returns the Lens Height
     * @return
     */
    public double getLensHeight() {
        return Constants.kLensHeight;
    }


    /**
     * Returns the Horizontl Plane to Lens (Pitch)
     * @return
     */
    public Rotation2d getHorizontalPlaneToLens() {
        return Constants.kLimelightConstants.getHorizontalPlaneToLens();
    }


    /**
     * Stores each Corner received by LL as a Translation2d for further processing
     * @param tcornxy array from LL with Pixel Coordinate
     * @return List of Corners
     */
    private static List<Translation2d> getCorners(Number[] tcornxy) {
        // Check if there is a non even number of corners
        if (tcornxy.length % 2 != 0) {
            return List.of();
        }

        ArrayList<Translation2d> corners = new ArrayList<>(tcornxy.length / 2);
        for (int i = 0; i < tcornxy.length; i += 2) {
            corners.add(new Translation2d(tcornxy[i].doubleValue(), tcornxy[i + 1].doubleValue()));
        }

        return corners;
    }

    public synchronized void setDisableProcessing(boolean disableLimelight) {
        mDisableProcessing = disableLimelight;
    }

    public synchronized boolean getIsDisabled() {
        return mDisableProcessing;
    }

    @Override
    public synchronized void outputTelemetry(boolean disabled) {
        if (disabled) {
            SmartDashboard.putBoolean("Has Target", mPeriodicIO.seesTarget);
            SmartDashboard.putNumber("Pipeline Latency (ms)", mPeriodicIO.latency);
            SmartDashboard.putNumber("LED Mode", mPeriodicIO.ledMode);
        }
    }

    @Override
    public void rewriteDeviceConfiguration() {
        // todo, should we check and rewrite LL config?
    }

    @Override
    public boolean checkDeviceConfiguration() {
        // todo, should we check and rewrite LL config?
        return true;
    }

}