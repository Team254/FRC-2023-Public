package com.team254.frc2023;

import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.team254.frc2023.field.Field;
import com.team254.frc2023.limelight.LimelightConstants;
import com.team254.frc2023.limelight.LimelightConstantsFactory;
import com.team254.lib.drivers.CanDeviceId;
import com.team254.lib.drivers.ServoMotorSubsystem;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.motion.MotionProfileConstraints;
import com.team254.lib.swerve.SwerveKinematicLimits;
import com.team254.lib.swerve.SwerveDriveKinematics;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

public class Constants {
    public static final double kLooperDt = 0.01;
    public static final double kEpsilon = 1E-3;

    // CAN
    public static final int kCANTimeoutMs = 10; // use for important on the fly updates
    public static final int kLongCANTimeoutMs = 100; // use for constructors
    public static final String kRioCANBusName = "rio";
    public static final String kCANivoreCANBusName = "canivore";
    public static final double kCancoderBootAllowanceSeconds = 10.0;

    // Swerve Config
    public static final String kPracticeBotMACAddress = "00:80:2F:33:D1:5F";
    public static final boolean kPracticeBot = hasMacAddress(kPracticeBotMACAddress);
    public static final boolean kUseVelocityDrive = true;

    // Controls
    public static final boolean kForceDriveGamepad = false;
    public static final int kDriveGamepadPort = 0;
    public static final int kMainThrottleJoystickPort = 0;
    public static final int kMainTurnJoystickPort = 1;
    public static final int kOperatorControllerPort = 2;
    public static final double kDriveJoystickThreshold = 0.075;
    public static final double kJoystickThreshold = 0.1;

    public static final int kPigeonIMUId = 20;
    public static final double kPigeonZScaleFactor = kPracticeBot ? (99.34-89.83785)/10.0 : (18.5 / 30.0); // Degrees per rotation. Measured 4/11

    // Config checking
    public static final double kRecheckDeviceIntervalSeconds = 10.0;

    // Drive constants
    public static final double kDriveReduction = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);
    public static final double kSteerReduction = (14.0 / 50.0) * (10.0 / 60.0); //For Practice
    public static final double kDriveWheelDiameter = 0.10033 * 81.0 / 84.213; /// meters, TODO measure
    public static final double kDriveTrackwidthMeters = 0.52705; // DONE Measure and set trackwidth
    public static final double kDriveWheelbaseMeters = 0.52705; // DONE Measure and set wheelbase

    public static final double kMaxDriveVoltage = 12.0;

    // Measure the drivetrain's maximum velocity or calculate the theoretical.
    //  The formula for calculating the theoretical maximum velocity is:
    //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi

    public static final double kMaxVelocityMetersPerSecond = 5.05; //Calibrated 3/12 on Comp Bot
    public static final double kMaxAccelerationMetersPerSecondSquared = 4.4;


    // Robot constants
    public static final double kMaxDriveAcceleration = 1867 * 0.8;   // m/s^2 tuned 2/18 practice bot
    public static final double kTrackScrubFactor = 1;

    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
    public static final double kMaxAngularVelocityRadiansPerSecond = 11.386413;

    public static final double kScaleTranslationInputs = 0.5;
    public static final double kScaleRotationInputs = 0.2;

    public static final SwerveKinematicLimits kUncappedKinematicLimits = new SwerveKinematicLimits();
    static {
        kUncappedKinematicLimits.kMaxDriveVelocity = Constants.kMaxVelocityMetersPerSecond;
        kUncappedKinematicLimits.kMaxDriveAcceleration = Double.MAX_VALUE;
        kUncappedKinematicLimits.kMaxSteeringVelocity = Double.MAX_VALUE;
    }

    public static final SwerveKinematicLimits kAzimuthOnlyKinematicLimits = new SwerveKinematicLimits();
    static {
        kAzimuthOnlyKinematicLimits.kMaxDriveVelocity = Constants.kMaxVelocityMetersPerSecond;
        kAzimuthOnlyKinematicLimits.kMaxDriveAcceleration = Double.MAX_VALUE;
        kAzimuthOnlyKinematicLimits.kMaxSteeringVelocity = Units.degreesToRadians(1500.0);
    }

    public static final SwerveKinematicLimits kTeleopKinematicLimits = new SwerveKinematicLimits();
    static {
        kTeleopKinematicLimits.kMaxDriveVelocity = Constants.kMaxVelocityMetersPerSecond;
        kTeleopKinematicLimits.kMaxDriveAcceleration = kTeleopKinematicLimits.kMaxDriveVelocity / 0.1;
        kTeleopKinematicLimits.kMaxSteeringVelocity = Units.degreesToRadians(1500.0);
    }

    public static final SwerveKinematicLimits kFastKinematicLimits = new SwerveKinematicLimits();
    static {
        kFastKinematicLimits.kMaxDriveVelocity = Constants.kMaxVelocityMetersPerSecond;
        kFastKinematicLimits.kMaxDriveAcceleration = kFastKinematicLimits.kMaxDriveVelocity / 0.2;
        kFastKinematicLimits.kMaxSteeringVelocity = Units.degreesToRadians(1000.0);
    }

    public static final SwerveKinematicLimits kSmoothKinematicLimits = new SwerveKinematicLimits();
    static {
        kSmoothKinematicLimits.kMaxDriveVelocity = Constants.kMaxVelocityMetersPerSecond * .9;
        kSmoothKinematicLimits.kMaxDriveAcceleration = Constants.kMaxAccelerationMetersPerSecondSquared;
        kSmoothKinematicLimits.kMaxSteeringVelocity = Units.degreesToRadians(750.0);
    }

    public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(Constants.kDriveTrackwidthMeters / 2.0, Constants.kDriveWheelbaseMeters / 2.0),
            // Front right
            new Translation2d(Constants.kDriveTrackwidthMeters / 2.0, -Constants.kDriveWheelbaseMeters / 2.0),
            // Back left
            new Translation2d(-Constants.kDriveTrackwidthMeters / 2.0, Constants.kDriveWheelbaseMeters / 2.0),
            // Back right
            new Translation2d(-Constants.kDriveTrackwidthMeters / 2.0, -Constants.kDriveWheelbaseMeters / 2.0)
    );


    //CANCoder or MagEncoder?


    public static final CanDeviceId kBackLeftDriveTalonId = new CanDeviceId(5, kCANivoreCANBusName);
    public static final CanDeviceId kBackLeftAziTalonId = new CanDeviceId(6, kCANivoreCANBusName);

    public static final CanDeviceId kBackRightDriveTalonId = new CanDeviceId(7, kCANivoreCANBusName);
    public static final CanDeviceId kBackRightAziTalonId = new CanDeviceId(8, kCANivoreCANBusName);

    public static final CanDeviceId kFrontRightDriveTalonId = new CanDeviceId(1, kCANivoreCANBusName);
    public static final CanDeviceId kFrontRightAziTalonId = new CanDeviceId(2, kCANivoreCANBusName);

    public static final CanDeviceId kFrontLeftDriveTalonId = new CanDeviceId(3, kCANivoreCANBusName);
    public static final CanDeviceId kFrontLeftAziTalonId = new CanDeviceId(4, kCANivoreCANBusName);

   /**
    * Azimuth Encoder Configurations
    */
    public static final boolean  kUseMagEncoders = false;

    //USE if Swerve Uses CANCoders
    public static final CanDeviceId kBackLeftEncoderPortId = new CanDeviceId(11, kCANivoreCANBusName);
    public static final CanDeviceId kBackRightEncoderPortId = new CanDeviceId(12, kCANivoreCANBusName);
    public static final CanDeviceId kFrontRightEncoderPortId = new CanDeviceId(9, kCANivoreCANBusName);
    public static final CanDeviceId kFrontLeftEncoderPortId = new CanDeviceId(10, kCANivoreCANBusName);


    //USE if Swerve Uses MAG Encoders (DIO)
    public static final int kDIOBackLeftEncoderPortId = 22;
    public static final int kDIOBackRightEncoderPortId = 23;
    public static final int kDIOFrontRightEncoderPortId = 9;
    public static final int kDIOFrontLeftEncoderPortId = 8;

    // Encoder Offsets - Bevel to the Left
    public static final Rotation2d kBackLeftAziEncoderOffset = kPracticeBot ? Rotation2d.fromDegrees(-85.6) : Rotation2d.fromDegrees(-184.83); //-94.83
    public static final Rotation2d kBackRightAziEncoderOffset = kPracticeBot ? Rotation2d.fromDegrees(-102.39) : Rotation2d.fromDegrees(90.43); //0.43
    public static final Rotation2d kFrontRightAziEncoderOffset = kPracticeBot ? Rotation2d.fromDegrees(103.44) : Rotation2d.fromDegrees(73.57); //-16.43
    public static final Rotation2d kFrontLeftAziEncoderOffset = kPracticeBot ? Rotation2d.fromDegrees(17.03) : Rotation2d.fromDegrees(-127.53); //142.47

    // TODO test converted values
    public static final double kMk4AziMMKp = 6.000; //0.505
    public static final double kMk4AziMMKi = 0;
    public static final double kMk4AziMMKd = 0;//0.0004; //0.2 / 500
    public static final double kMk4AziMMKs = 0.8;
    public static final double kMk4AziKv = 0.1224;

    public static final double kMk4AziMMCruiseVelocity = 98.0;
    public static final double kMk4AziMMAccel = 1000.0;


    public static final double kMk4AziPositionKp = 1.0005; //0.505
    public static final double kMk4AziPositionKi = 0;
    public static final double kMk4AziPositionKd = 0.0004; //0.2 / 500

    public static final double kMk4DriveVelocityKp = 0.02 * 12;
    public static final double kMk4DriveVelocityKi = 0.0;
    public static final double kMk4DriveVelocityKd = 0.000002 * 12; //0.01
    public static final double kMk4DriveVelocityKv = 1 / 101.98 * 12;// (kMaxVelocityMetersPerSecond / (Math.PI * Constants.kDriveWheelDiameter * Constants.kDriveReduction));
    public static final double kMk4DriveVelocityKs = 0.8;

    // TODO check angular acceleration value
    public static final double kMaxAngularSpeedRadiansPerSecond = kMaxVelocityMetersPerSecond /
            Math.hypot(kDriveTrackwidthMeters / 2.0, kDriveWheelbaseMeters / 2.0);
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = kMaxAccelerationMetersPerSecondSquared /
            Math.hypot(kDriveTrackwidthMeters / 2.0, kDriveWheelbaseMeters / 2.0);
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);
    public static final TrapezoidProfile.Constraints kPositionControllerConstraints =
            new TrapezoidProfile.Constraints(kMaxVelocityMetersPerSecond, kMaxVelocityMetersPerSecond);

    public static final MotionProfileConstraints kPositionMotionProfileConstraints = new MotionProfileConstraints(
            0.8 * Constants.kMaxVelocityMetersPerSecond,
            0.8 * -Constants.kMaxVelocityMetersPerSecond,
            0.6 * Constants.kMaxAccelerationMetersPerSecondSquared);
    public static final MotionProfileConstraints kHeadingMotionProfileConstraints = new MotionProfileConstraints(
            0.5 * Constants.kMaxAngularSpeedRadiansPerSecond,
            0.5 * -Constants.kMaxAngularSpeedRadiansPerSecond,
            1.0 * Constants.kMaxAngularAccelerationRadiansPerSecondSquared);

    // Pneumatics
    public static final int kAnalogSensorChannel = 0;


    // Swerve Heading Controller
    public static final double kSwerveHeadingControllerErrorTolerance = 1.5; // degree error

    //TODO tune heading controller snap PID
    public static final double kSnapSwerveHeadingKp = 0.05;
    public static final double kSnapSwerveHeadingKi = 0.0;
    public static final double kSnapSwerveHeadingKd = 0.0075;

    //TODO tune heading controller maintain PID
    public static final double kMaintainSwerveHeadingKpHighVelocity = 0.0225;
    public static final double kMaintainSwerveHeadingKiHighVelocity = 0.0;
    public static final double kMaintainSwerveHeadingKdHighVelocity = 0.003;

    public static final double kMaintainSwerveHeadingKpLowVelocity = 0.02;  // 0.01;
    public static final double kMaintainSwerveHeadingKiLowVelocity = 0.0;
    public static final double kMaintainSwerveHeadingKdLowVelocity = 0.0;

    // Swerve heading controller gains
    public static final double kHeadingControllerKp = 2.54;
    public static final double kHeadingControllerKi = 0.0;
    public static final double kHeadingControllerKd = 0.0;
    public static final double kHeadingControllerKffv = 1.0;
    public static final double kHeadingControllerKffa = 0.0;
    public static final double kHeadingControllerKs = 0.0;

    //TODO tune radius controller snap PID
    public static final double kSnapRadiusKp = 2.0;
    public static final double kSnapRadiusKi = 0.0;
    public static final double kSnapRadiusKd = 0.0;

    //TODO tune radius controller maintain PID
    public static final double kMaintainRadiusKp = 1.5;
    public static final double kMaintainRadiusKi = 0.0;
    public static final double kMaintainRadiusKd = 0.0;


    /**
     * Check if this system has a certain mac address in any network device.
     * @param mac_address Mac address to check.
     * @return true if some device with this mac address exists on this system.
     */
    public static boolean hasMacAddress(final String mac_address) {
        try {
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            while (nwInterface.hasMoreElements()) {
                NetworkInterface nis = nwInterface.nextElement();
                if (nis == null) {
                    continue;
                }
                StringBuilder device_mac_sb = new StringBuilder();
                System.out.println("hasMacAddress: NIS: " + nis.getDisplayName());
                byte[] mac = nis.getHardwareAddress();
                if (mac != null) {
                    for (int i = 0; i < mac.length; i++) {
                        device_mac_sb.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? ":" : ""));
                    }
                    String device_mac = device_mac_sb.toString();
                    System.out.println("hasMacAddress: NIS " + nis.getDisplayName() + " device_mac: " + device_mac);
                    if (mac_address.equals(device_mac)) {
                        System.out.println("hasMacAddress: ** Mac address match! " + device_mac);
                        return true;
                    }
                } else {
                    System.out.println("hasMacAddress: Address doesn't exist or is not accessible");
                }
            }

        } catch (SocketException e) {
            e.printStackTrace();
        }
        return false;
    }

    // TODO fix constants
    public static final ServoMotorSubsystem.ServoMotorSubsystemConstants kElevatorConstants = new ServoMotorSubsystem.ServoMotorSubsystemConstants();
    static {
        kElevatorConstants.kName = "Elevator";

        kElevatorConstants.kMasterConstants.id = new CanDeviceId(15, Constants.kCANivoreCANBusName);
        kElevatorConstants.kMasterConstants.counterClockwisePositive = false;
        kElevatorConstants.kMasterConstants.invert_sensor_phase = false;

        kElevatorConstants.kHomePosition = 28.61; // Stowed position (inches);
        kElevatorConstants.kRotationsPerUnitDistance = (Math.PI * 1.504) / ((30.0 / 14.0) * (26.0 / 12.0)) / (42.0 / 40.401);
        kElevatorConstants.kSoftLimitDeadband = 0.15; // (inches)

        kElevatorConstants.kPositionKp = 0.15*12;
        kElevatorConstants.kPositionKd = 0.0;
        kElevatorConstants.kPositionKi = 0.0;
        //kElevatorConstants.kPositionKf = 0.075;

        kElevatorConstants.kKf = 0.0125*12;
        kElevatorConstants.kKa = 0.0;
        kElevatorConstants.kKs = 0.0;
        kElevatorConstants.kPositionDeadband = 1;

        kElevatorConstants.kArbitraryFeedforward = 0.085*12;
        kElevatorConstants.kVelocityFeedforward = 0.0105*12;
        kElevatorConstants.kCruiseVelocity = 100.0;//86.8;
        kElevatorConstants.kDeadband = 1;
        kElevatorConstants.kAcceleration = 600.0;
        kElevatorConstants.kJerk = 1000000;

        kElevatorConstants.kEnableStatorCurrentLimit = false;
        kElevatorConstants.kEnableSupplyCurrentLimit = true;
        kElevatorConstants.kSupplyCurrentLimit = 70;

        kElevatorConstants.kMinUnitsLimit = 0.0;
        kElevatorConstants.kMaxUnitsLimit = 40.1;
    }

    // TODO fix constants
    public static final ServoMotorSubsystem.ServoMotorSubsystemConstants kLateratorConstants = new ServoMotorSubsystem.ServoMotorSubsystemConstants();
    static {
        kLateratorConstants.kName = "Laterator";

        kLateratorConstants.kMasterConstants.id = new CanDeviceId(16, Constants.kCANivoreCANBusName);
        kLateratorConstants.kMasterConstants.counterClockwisePositive = true;
        kLateratorConstants.kMasterConstants.invert_sensor_phase = false;
        kLateratorConstants.kNeutralMode = NeutralModeValue.Coast;

        // Unit == Degrees
        kLateratorConstants.kHomePosition = 0.0; // Stowed Position (inches)
        kLateratorConstants.kRotationsPerUnitDistance = 0.3699;
        kLateratorConstants.kSoftLimitDeadband = 0.05;

        kLateratorConstants.kPositionKp = 0.8*12;
        kLateratorConstants.kPositionKd = 0.0;
        kLateratorConstants.kPositionKi = 0.0;
        //kLateratorConstants.kPositionKf = 0.0;

        kLateratorConstants.kKf = 0.0; //Tuned by Sending 0.4 percent output to turret motor and measuring closed loop velocity
        kLateratorConstants.kKa = 0.0;
        kLateratorConstants.kKs = 0.0;
        kLateratorConstants.kPositionDeadband = 1;

        kLateratorConstants.kArbitraryFeedforward = 0.05*12;
        kLateratorConstants.kVelocityFeedforward = 0.014*12;
        kLateratorConstants.kCruiseVelocity = 73.0;
        kLateratorConstants.kDeadband = 1;
        kLateratorConstants.kAcceleration = 150.0;
        kLateratorConstants.kJerk = 1000000;

        kLateratorConstants.kEnableStatorCurrentLimit = false;
        kLateratorConstants.kEnableSupplyCurrentLimit = true;
        kLateratorConstants.kSupplyCurrentLimit = 90;

        kLateratorConstants.kMinUnitsLimit = 0;
        kLateratorConstants.kMaxUnitsLimit = 50.5;//49.875;
    }

    public static final ServoMotorSubsystem.ServoMotorSubsystemConstants kGroundIntakeDeployConstants = new ServoMotorSubsystem.ServoMotorSubsystemConstants();
    static {
        kGroundIntakeDeployConstants.kName = "Ground Intake Deploy";

        kGroundIntakeDeployConstants.kMasterConstants.id = new CanDeviceId(19, Constants.kCANivoreCANBusName);
        kGroundIntakeDeployConstants.kMasterConstants.counterClockwisePositive = true;
        kGroundIntakeDeployConstants.kMasterConstants.invert_sensor_phase = false;

        kGroundIntakeDeployConstants.kNeutralMode = NeutralModeValue.Coast;

        // Unit == Degrees
        kGroundIntakeDeployConstants.kHomePosition = 0; // Stowed Position (inches from full retraction)
        kGroundIntakeDeployConstants.kRotationsPerUnitDistance = (40.0/12.0) / 3.14; // 12 to 40 reduction to 1"dp pinion gear on rack
        kGroundIntakeDeployConstants.kSoftLimitDeadband = 0.05;

        kGroundIntakeDeployConstants.kPositionKp = 4.0;
        kGroundIntakeDeployConstants.kPositionKd = 0.0;
        kGroundIntakeDeployConstants.kPositionKi = 0.0;
        //kGroundIntakeDeployConstants.kPositionKf = 0.0;

        kGroundIntakeDeployConstants.kKf = 0.0;
        kGroundIntakeDeployConstants.kKa = 0.0;
        kGroundIntakeDeployConstants.kKs = 0.0;
        kGroundIntakeDeployConstants.kPositionDeadband = 1;

        kGroundIntakeDeployConstants.kArbitraryFeedforward = 0.0;
        kGroundIntakeDeployConstants.kVelocityFeedforward =  0.01 * 12;
        kGroundIntakeDeployConstants.kCruiseVelocity = 100;
        kGroundIntakeDeployConstants.kDeadband = 1; // This is wrong but not used
        kGroundIntakeDeployConstants.kAcceleration = 800;

        kGroundIntakeDeployConstants.kEnableStatorCurrentLimit = false;
        kGroundIntakeDeployConstants.kEnableSupplyCurrentLimit = true;
        kGroundIntakeDeployConstants.kSupplyCurrentLimit = 60;

        kGroundIntakeDeployConstants.kMinUnitsLimit = 0;
        kGroundIntakeDeployConstants.kMaxUnitsLimit = 15.5+1.5;
    }

    public static final ServoMotorSubsystem.ServoMotorSubsystemConstants kForksConstants = new ServoMotorSubsystem.ServoMotorSubsystemConstants();
    static {
        kForksConstants.kName = "Forks";

        kForksConstants.kMasterConstants.id = new CanDeviceId(22, Constants.kCANivoreCANBusName);
        kForksConstants.kMasterConstants.counterClockwisePositive = false;
        kForksConstants.kMasterConstants.invert_sensor_phase = false;

        kForksConstants.kNeutralMode = NeutralModeValue.Brake;

        kForksConstants.kHomePosition = 1.3; // Stowed Position (inches)
        kForksConstants.kRotationsPerUnitDistance = 10.6154;
        kForksConstants.kSoftLimitDeadband = 0.05;

        kForksConstants.kPositionKp = 1.5;
        kForksConstants.kPositionKd = 0.0;
        kForksConstants.kPositionKi = 0.0;

        kForksConstants.kEnableSupplyCurrentLimit = true;
        kForksConstants.kSupplyCurrentLimit = 100;

        kForksConstants.kMinUnitsLimit = 0;
        kForksConstants.kMaxUnitsLimit = 8.8;
    }

    public static final double kForksClimbingTimeout = 5.0;

    public static final double kGroundIntakeDeployExtendPosition = 15.25+1.0;
    public static final double kGroundIntakePushinPosition = 10.0;
    public static final double kGroundIntakeDeployStowPosition = 1.25;
    public static final double kGroundIntakeDeploySpringySupplyCurrentLimit = 2.0;
    public static final double kGroundIntakeDeploySpringyStatorCurrentLimit = 3.0;
    public static final double kGroundIntakeDeploySpringyAllowableError = 1.0;

    public static final int kIntakeId = 18;

    public static final int kGroundIntakeRollerId = 17;

    // TODO find below constants
    public static final double kLateratorAngle = Math.toRadians(10.0); // radians from horizontal
    public static final double kHeightClearBumperPosition = 10.0 * 2.54 / 100;  // when below this, extension required to clear bumper
    public static final double kExtensionClearBumperPosition = 12.0 * 2.54 / 100; // inches from superstructure home to bumper end
    public static final double kHeightClearMidZone = 30.0 * 2.54 / 100; // inches from ground to mid scoring zone
    public static final double kExtensionLimitMidZone = 5.0 * 2.54 / 100; // inches of extension underneath mid scoring zone;

    public static final double kElevatorSlowAcceleration = 300.0;
    public static final double kElevatorSlowJerk = 200.0;
    public static final double kElevatorMediumAcceleration = 600.0;
    public static final double kElevatorMediumJerk = 100000.0; // Infinity
    public static final double kElevatorFastAcceleration = 600.0;
    public static final double kElevatorFastJerk = 100000.0; // Infinity
    public static final double kLateratorSlowAcceleration = 117.5;
    public static final double kLateratorSlowJerk = 117.5*6;
    public static final double kLateratorMediumAuto = 135.0;
    public static final double kLateratorMediumAcceleration = 175.0;
    public static final double kLateratorMediumJerk = 117.5*6;
    public static final double kLateratorFastAcceleration = 250.0;
    public static final double kLateratorFastJerk = 1000000.0;// Infinity

    public static final double kLiberalLateratorAllowableError = 24; // Inches
    public static final double kLiberalElevatorAllowableError = 36; // Inches
    public static final double kModerateLateratorAllowableError = 12; // Inches
    public static final double kModerateElevatorAllowableError = 12; // Inches
    public static final double kConservativeLateratorAllowableError = 1; // Inches
    public static final double kConservativeElevatorAllowableError = 1; // Inches

    public static final double kFlingingElevatorAllowableError = 3; // Inches
    public static final double kFlingingLateratorAllowableError = 20; // Inches
    public static final double kExpectedVsMeasuredElevatorAllowableError = 3; // Inches
    public static final double kExpectedVsMeasuredLateratorAllowableError = 3; // Inches

    public static final double kSpringyLateratorStatorCurrentLimit = 20;
    public static final double kSpringyLateratorSupplyCurrentLimit = 5;

    public static final double kAutoStowWaitTime = 0.0; // Seconds

    //Pure Pursuit Constants
    public static final double kPathLookaheadTime = 0.25; // From 1323 (2019)
    public static final double kPathMinLookaheadDistance = 12.0; //From 1323 (2019)
    public static final double kAdaptivePathMinLookaheadDistance = 6.0;
    public static final double kAdaptivePathMaxLookaheadDistance = 24.0;
    public static final double kAdaptiveErrorLookaheadCoefficient = 4.0;

     //April Tag Poses
     public static final Field kField = new Field();


     //Kalman Filters
     public static final Matrix<N2, N1> kStateStdDevs = VecBuilder.fill(Math.pow(0.04, 1), Math.pow(0.04, 1));
     public static final Matrix<N2, N1> kLocalMeasurementStdDevs = VecBuilder.fill(Math.pow(0.01, 1), Math.pow(0.01, 1));
       public static final double kTagPoseRejectionFilter = 2.25; //Reject Vision Correction if Current Pose Farther than 3m from April Tag
     public static final double kNumInitialMeasurements = 15;//100; //Number of Measurements to Let Into Filter Without Rejection to Initialize X_Hat



    //Limelight
    public static final double kLimelightTransmissionTimeLatency = 0.0 / 1000.0; // seconds
    public static final double kImageCaptureLatency = 11.0; // milliseconds

    //Field2d
    public static final double kCenterWidthField2d = 16.54/2; //16.54
    public static final double kCenterHeightField2d = 8.02/2; //8.02
    public static final double kWidthField2d = 16.54;
    public static final double kHeightField2d = 8.02;
    public static final double kRotOffset = Math.PI;
    public static final double kOutofFrameValue = 20;

    //Limelight
    public static final double kHorizontalFOV = 63.3; //degrees
    public static final double kVerticalFOV = 49.7;

    public static final double kResolutionWidth = 1280;
    public static final double kResolutionHeight = 960;

    public static final double kResolutionWidthHalf = kResolutionWidth / 2;
    public static final double kResolutionHeightHalf = kResolutionHeight / 2;
    // From floor to estimated focal point, using curvature of Lens
    // Measured from CAD: 2/25: 133296 cm
    public static final double kLensHeight = 1.33296;
    // Measured from CAD: 2/25: 27.621 cm
    public static final Pose2d kRobotToCamera = new Pose2d(new Translation2d(-0.27621 , 0), Rotation2d.fromDegrees(0));

    public static final int kDefaultPipeline = 0;
    public static final int kYawCalibrationPipeline = 1;
    // Measured from CAD: 40 degrees
    //public static final Rotation2d kHorizontalPlaneToLens = kPracticeBot ? Rotation2d.fromDegrees(-40) : Rotation2d.fromDegrees(-39);
    //public static final Rotation2d kLimelightYawOffset = kPracticeBot ?  Rotation2d.fromDegrees(-3.0) : Rotation2d.fromDegrees(-2.75);

    public static final String kPracticeLLId = "A";

    public static final String kCompLLId = "B";
    public static final LimelightConstants kLimelightConstants = kPracticeBot ? LimelightConstantsFactory.getConstantsForId(kPracticeLLId) : LimelightConstantsFactory.getConstantsForId(kCompLLId);

    //Auto-Align
    public static final double kAutoAlignAllowableDistance = 2.0; //Meters


    //Heading Controller Drive
    public static final double kFeedforwardScaleFactor = 2.0;

    //LED
    public static final int kCANdleId = 30;
    public static final int kMaxLEDCount = kPracticeBot ? 26 : 42;

}
