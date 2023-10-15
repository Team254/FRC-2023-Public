package com.team254.lib.drivers;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.*;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.team254.lib.geometry.Rotation2d;
import com.team254.frc2023.Constants;
import com.team254.lib.swerve.SwerveModuleState;

import edu.wpi.first.wpilibj.Timer;

public abstract class SwerveModule extends Subsystem {
    private final TalonFX mSteeringMotor;
    private final TalonFX mDriveMotor;
    TalonFXConfiguration mDriveConfig;
    TalonFXConfiguration mSteerConfig;
    private final Rotation2d mEncoderZero;
    private Rotation2d mTalonOffset;

    private final StatusSignalValue<Double> mDriveMotorVelocitySignalValue;
    private final StatusSignalValue<Double> mDriveMotorPositionSignalValue;
    private final StatusSignalValue<Double> mSteeringMotorPositionSignalValue;
    private final StatusSignalValue<Double> mDriveMotorClosedLoopErrorSignalValue;
    private final StatusSignalValue<Double> mSteeringMotorClosedLoopErrorSignalValue;
    private final StatusSignalValue<Double> mSteeringMotorVelocitySignalValue;

    private final StatusSignalValue<Double> mSteeringTemperatureSignalValue;

    private final StatusSignalValue<Double> mDriveTemperatureSignalValue;

    private final double kDriveMotorCoefficient = Math.PI * Constants.kDriveWheelDiameter * Constants.kDriveReduction;

    private final double kSteerPositionCoefficient = 2.0 * Math.PI * Constants.kSteerReduction;

    public SwerveModule(CanDeviceId driveId, CanDeviceId steeringId, Rotation2d encoderZero) {
        mDriveMotor = TalonFXFactory.createDefaultTalon(driveId);
        mDriveMotorVelocitySignalValue = mDriveMotor.getRotorVelocity();
        mDriveMotorPositionSignalValue = mDriveMotor.getRotorPosition();
        mDriveMotorClosedLoopErrorSignalValue = mDriveMotor.getClosedLoopError();
        mSteeringMotor = TalonFXFactory.createDefaultTalon(steeringId);
        mSteeringMotorPositionSignalValue = mSteeringMotor.getRotorPosition();
        mSteeringMotorVelocitySignalValue = mSteeringMotor.getRotorVelocity();
        mSteeringMotorClosedLoopErrorSignalValue = mSteeringMotor.getClosedLoopError();

        mSteeringTemperatureSignalValue = mSteeringMotor.getDeviceTemp();
        mDriveTemperatureSignalValue = mDriveMotor.getDeviceTemp();

        mEncoderZero = encoderZero;

        double start = Timer.getFPGATimestamp();
        while (Timer.getFPGATimestamp() - start < 10.0) {
            try {
                configureTalons();
                break;
            } catch (RuntimeException e) {
                Timer.delay(0.5);
                if (Timer.getFPGATimestamp() - start >= 10.0) {
                    System.out.println("TALON CONFIGURATION TIMED OUT: PORTS " + driveId.getDeviceNumber() + " AND " + steeringId.getDeviceNumber());
                }
            }
        }
    }


    public void configureTalons() throws RuntimeException {
//        TODO voltage compensation
//        TODO: investigate - might not be necessary due to https://pro.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/migration-guide.html?highlight=saturation
//        TalonUtil.checkErrorWithThrow(
//                mDriveMotor.configVoltageCompSaturation(Constants.kMaxDriveVoltage, Constants.kLongCANTimeoutMs),
//                "Failed to set voltage compensation");
//
//        TalonUtil.checkErrorWithThrow(
//                mDriveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(false, 100.0, 120.0, 0.0), Constants.kLongCANTimeoutMs),
//                "Failed to set supply current limit");
//        TalonUtil.checkErrorWithThrow(
//                mDriveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 100.0, 120.0, 0.0), Constants.kLongCANTimeoutMs),
//                "Failed to set stator current limit");
//
//        TalonUtil.checkErrorWithThrow(
//                mDriveMotor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_5Ms, Constants.kLongCANTimeoutMs),
//                "Failed to set velocity measurement period");
//        TalonUtil.checkErrorWithThrow(
//                mDriveMotor.configVelocityMeasurementWindow(32, Constants.kLongCANTimeoutMs),
//                "Failed to set velocity measurement window");

        mDriveConfig = new TalonFXConfiguration();
        PhoenixProUtil.checkErrorAndRetry(() -> {return mDriveMotor.getConfigurator().refresh(mDriveConfig);});
        mDriveConfig.CurrentLimits.SupplyCurrentLimit = 120;
        mDriveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        mDriveConfig.CurrentLimits.StatorCurrentLimit = 120;
        mDriveConfig.CurrentLimits.StatorCurrentLimitEnable = false;

//        mDriveMotor.enableVoltageCompensation(true);
//        mDriveMotor.setNeutralMode(NeutralMode.Brake);
        mDriveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
//        mDriveMotor.setInverted(TalonFXInvertType.Clockwise);
        mDriveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

//        TODO set sensor phase?
//        mDriveMotor.setSensorPhase(true);
//        mDriveMotor.setSelectedSensorPosition(0.0);
//        mDriveMotor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_5Ms, 10);
//        mDriveMotor.configVelocityMeasurementWindow(32, 10);

        // Reduce CAN status frame rates
    //    TalonUtil.checkErrorWithThrow(
    //            mDriveMotor.setStatusFramePeriod(
    //                    StatusFrameEnhanced.Status_1_General,
    //                    250,
    //                    Constants.kLongCANTimeoutMs
    //            ),
    //            "Failed to configure Falcon status frame period"
    //    );
        //TODO confirm this is equivalent to status frame 1
        PhoenixProUtil.checkErrorAndRetry(() -> {return mDriveMotor.getBridgeOuput().setUpdateFrequency(100, 0.05);});
        PhoenixProUtil.checkErrorAndRetry(() -> {return mDriveMotor.getFault_Hardware().setUpdateFrequency(4, 0.05);});

        // Reduce CAN status frame rates
//        TalonUtil.checkErrorWithThrow(
//                mDriveMotor.setStatusFramePeriod(
//                        StatusFrameEnhanced.Status_2_Feedback0,
//                        5,
//                        Constants.kLongCANTimeoutMs
//                ),
//                "Failed to configure Falcon status frame period"
//        );

        PhoenixProUtil.checkErrorAndRetry(() -> {return mDriveMotorVelocitySignalValue.setUpdateFrequency(200, 0.05);});
        PhoenixProUtil.checkErrorAndRetry(() -> {return mDriveMotorPositionSignalValue.setUpdateFrequency(200, 0.05);});
        PhoenixProUtil.checkErrorAndRetry(() -> {return mDriveMotorClosedLoopErrorSignalValue.setUpdateFrequency(200, 0.05);});
        PhoenixProUtil.checkErrorAndRetry(() -> {return mDriveMotor.setRotorPosition(0.0, 0.05);});


        // PID
        // TODO(get rid of dependency on Constants)
//        TalonUtil.checkErrorWithThrow(
//                mDriveMotor.config_kI(0, Constants.kMk4DriveVelocityKi, Constants.kLongCANTimeoutMs),
//                "Failed to set kI");
//        TalonUtil.checkErrorWithThrow(
//                mDriveMotor.config_kP(0, Constants.kMk4DriveVelocityKp, Constants.kLongCANTimeoutMs),
//                "Failed to set kP");
//        TalonUtil.checkErrorWithThrow(
//                mDriveMotor.config_kD(0, Constants.kMk4DriveVelocityKd, Constants.kLongCANTimeoutMs),
//                "Failed to set kD");
//        TalonUtil.checkErrorWithThrow(
//                mDriveMotor.config_kF(0, Constants.kMk4DriveVelocityKf, Constants.kLongCANTimeoutMs),
//                "Failed to set kF");

        mDriveConfig.Slot0.kI = Constants.kMk4DriveVelocityKi;
        mDriveConfig.Slot0.kP = Constants.kMk4DriveVelocityKp;
        mDriveConfig.Slot0.kD = Constants.kMk4DriveVelocityKd;
        mDriveConfig.Slot0.kV = Constants.kMk4DriveVelocityKv;
        mDriveConfig.Slot0.kS = Constants.kMk4DriveVelocityKs;

        TalonUtil.applyAndCheckConfiguration(mDriveMotor, mDriveConfig);

        // Steering
//        TODO voltage compensation
//        TalonUtil.checkErrorWithThrow(
//                mSteeringMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs),
//                "Failed to set encoder");
//        TalonUtil.checkErrorWithThrow(
//                mSteeringMotor.configVoltageCompSaturation(Constants.kMaxDriveVoltage, Constants.kLongCANTimeoutMs),
//                "Failed to set voltage compensation");
//        TalonUtil.checkErrorWithThrow(
//                mSteeringMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(false, 100.0, 120.0, 0.0), Constants.kLongCANTimeoutMs),
//                "Failed to set supply current limit");
//        TalonUtil.checkErrorWithThrow(
//                mSteeringMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 100.0, 120.0, 0.0), Constants.kLongCANTimeoutMs),
//                "Failed to set stator current limit");

        mSteerConfig = new TalonFXConfiguration();
        mSteeringMotor.getConfigurator().refresh(mSteerConfig);
        mSteerConfig.CurrentLimits.SupplyCurrentLimit = 120;
        mSteerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        mSteerConfig.CurrentLimits.StatorCurrentLimit = 120;
        mSteerConfig.CurrentLimits.StatorCurrentLimitEnable = false;

//        mSteeringMotor.enableVoltageCompensation(true);
//        mSteeringMotor.setNeutralMode(NeutralMode.Coast);
        mSteerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        mSteerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
//        mSteeringMotor.setInverted(TalonFXInvertType.CounterClockwise);
//        TODO set sensor phase?
//        mSteeringMotor.setSensorPhase(false);

        // Reduce CAN status frame rates
//        TalonUtil.checkErrorWithThrow(
//                mSteeringMotor.setStatusFramePeriod(
//                        StatusFrameEnhanced.Status_1_General,
//                        5,
//                        Constants.kLongCANTimeoutMs
//                ),
//                "Failed to configure Falcon status frame period"
//        );


        PhoenixProUtil.checkErrorAndRetry(() -> { return mSteeringMotor.getBridgeOuput().setUpdateFrequency(200, 0.05); });
        PhoenixProUtil.checkErrorAndRetry(() -> {return mSteeringMotor.getFault_Hardware().setUpdateFrequency(4, 0.05);});


        // Reduce CAN status frame rates
//        TalonUtil.checkErrorWithThrow(
//                mSteeringMotor.setStatusFramePeriod(
//                        StatusFrameEnhanced.Status_2_Feedback0,
//                        5,
//                        Constants.kLongCANTimeoutMs
//                ),
//                "Failed to configure Falcon status frame period"
//        );

        PhoenixProUtil.checkErrorAndRetry(() -> { return mSteeringMotorPositionSignalValue.setUpdateFrequency(200, 0.05); });
        PhoenixProUtil.checkErrorAndRetry(() -> { return mSteeringMotorVelocitySignalValue.setUpdateFrequency(200, 0.05); });
        PhoenixProUtil.checkErrorAndRetry(() -> { return mSteeringMotorClosedLoopErrorSignalValue.setUpdateFrequency(200, 0.05); });


        // PID
        // TODO(get rid of dependency on Constants)
//        TalonUtil.checkErrorWithThrow(
//                mSteeringMotor.config_kP(0, Constants.kMk4AziKp, Constants.kLongCANTimeoutMs),
//                "Failed to set kP");
//        TalonUtil.checkErrorWithThrow(
//                mSteeringMotor.config_kI(0, Constants.kMk4AziKi, Constants.kLongCANTimeoutMs),
//                "Failed to set kI");
//        TalonUtil.checkErrorWithThrow(
//                mSteeringMotor.config_kD(0, Constants.kMk4AziKd, Constants.kLongCANTimeoutMs),
//                "Failed to set kD");
//        TalonUtil.checkErrorWithThrow(
//                mSteeringMotor.config_kF(0, 0.0, Constants.kLongCANTimeoutMs),
//                "Failed to set kF");
        mSteerConfig.Slot0.kI = Constants.kMk4AziMMKi;
        mSteerConfig.Slot0.kP = Constants.kMk4AziMMKp;
        mSteerConfig.Slot0.kD = Constants.kMk4AziMMKd;
        mSteerConfig.Slot0.kS = Constants.kMk4AziMMKs;
        mSteerConfig.Slot0.kV = Constants.kMk4AziKv;

        mSteerConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.kMk4AziMMCruiseVelocity;
        mSteerConfig.MotionMagic.MotionMagicAcceleration = Constants.kMk4AziMMAccel;

        mSteerConfig.Slot1.kP = Constants.kMk4AziPositionKp;
        mSteerConfig.Slot1.kD = Constants.kMk4AziPositionKd;
        mSteerConfig.Slot1.kI = Constants.kMk4AziPositionKi;

        TalonUtil.applyAndCheckConfiguration(mSteeringMotor, mSteerConfig);
    }

    public Rotation2d getTalonOffset() {
        return mTalonOffset;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(mSteeringMotorPositionSignalValue.getValue() * kSteerPositionCoefficient).rotateBy(mTalonOffset);
    }

    public void setSteerCoastMode() {
        mSteerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        TalonUtil.applyAndCheckConfiguration(mSteeringMotor, mSteerConfig);
    }

    public void setSteerBrakeMode() {
        mSteerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        TalonUtil.applyAndCheckConfiguration(mSteeringMotor, mSteerConfig);
    }

    public void setSteerCoastModeUnchecked() {
        mSteerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        mSteeringMotor.getConfigurator().apply(mSteerConfig);
    }

    public void setSteerBrakeModeUnchecked() {
        mSteerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        mSteeringMotor.getConfigurator().apply(mSteerConfig);
    }

    public void setDriveBreakModeUnchecked() {
        mDriveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        mDriveMotor.getConfigurator().apply(mDriveConfig.MotorOutput);
    }

    public void setDriveCoastModeUnchecked() {
        mDriveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        mDriveMotor.getConfigurator().apply(mDriveConfig.MotorOutput);
    }

    public void setDriveCoastMode() {
        mDriveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        TalonUtil.applyAndCheckConfiguration(mDriveMotor, mDriveConfig);
    }

    public void setDriveBrakeMode() {
        mDriveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        TalonUtil.applyAndCheckConfiguration(mDriveMotor, mDriveConfig);
    }

    public void rezeroSteeringMotor() {
        mTalonOffset = Rotation2d.fromRadians(mSteeringMotorPositionSignalValue.asSupplier().get() * kSteerPositionCoefficient)
                .rotateBy(getAdjustedSteerEncoderAngle().inverse());
    }

    /** Needs to be Overridden when using a different encoder */
    public Rotation2d getSteerEncoderAngle() {
        return Rotation2d.identity();
    }

    public Rotation2d getAdjustedSteerEncoderAngle() {
        return getSteerEncoderAngle().rotateBy(mEncoderZero.inverse());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getDriveDistance(), getSteerAngle());
    }

    public SwerveModuleState getPreviewedState() {
        // TODO plumb this through
        return new SwerveModuleState(
            getDriveVelocity(),
            kDriveMotorCoefficient * StatusSignalValue.getLatencyCompensatedValue(mDriveMotorPositionSignalValue, mDriveMotorVelocitySignalValue),
            Rotation2d.fromRadians((StatusSignalValue.getLatencyCompensatedValue(mSteeringMotorPositionSignalValue, mSteeringMotorVelocitySignalValue) * kSteerPositionCoefficient) - mTalonOffset.getRadians()) );
    }

    public double getDriveClosedLoopError() {
        return mDriveMotorClosedLoopErrorSignalValue.asSupplier().get() * kDriveMotorCoefficient;
    }

    public double getSteerClosedLoopError() { return mSteeringMotorClosedLoopErrorSignalValue.asSupplier().get() * kSteerPositionCoefficient;}

    public double getDriveDistance() { return mDriveMotorPositionSignalValue.getValue() * kDriveMotorCoefficient; }

    public double getDriveVelocity() { return mDriveMotorVelocitySignalValue.getValue() * kDriveMotorCoefficient; }

    public Rotation2d getSteerAngle() {
        return Rotation2d.fromRadians(getUnclampedSteerAngleRadians());
    }

    public double getUnclampedSteerAngleRadians() {
        return (mSteeringMotorPositionSignalValue.getValue() * kSteerPositionCoefficient) - mTalonOffset.getRadians();
    }

    public double getDriveTemperature() {
        return mDriveTemperatureSignalValue.asSupplier().get();
    }

    public double getAziTemperature() {
        return mSteeringTemperatureSignalValue.asSupplier().get();
    }


    public void setWithVoltageShortestPath(double drivePercentage, Rotation2d steerAngle, boolean motionMagic) {
        final boolean flip = setSteerAngleShortestPath(steerAngle, motionMagic);
        mDriveMotor.setControl(new VoltageOut((flip ? -drivePercentage : drivePercentage) * 12.0));
    }

    public void setWithVelocityShortestPath(double driveVelocity, Rotation2d steerAngle, boolean motionMagic) {
        final boolean flip = setSteerAngleShortestPath(steerAngle, motionMagic);
        mDriveMotor.setControl(new VelocityVoltage((flip ? -driveVelocity : driveVelocity) / kDriveMotorCoefficient).withSlot(0));
    }

    public void setMaxOutput() {
        mDriveMotor.setControl(new VoltageOut(12.0));
    }

    public double getDriveSupplyCurrent() {
        return mDriveMotor.getSupplyCurrent().asSupplier().get();
    }

    public double getSteerSupplyCurrent(){
        return mSteeringMotor.getSupplyCurrent().asSupplier().get();
    }

    // Returns true if the drive velocity should be inverted.
    private boolean setSteerAngleShortestPath(Rotation2d steerAngle, boolean motionMagic) {
        boolean flip = false;
        final double unclampedPosition = getUnclampedSteerAngleRadians();
        final Rotation2d clampedPosition = Rotation2d.fromRadians(unclampedPosition);
        final Rotation2d relativeRotation = steerAngle.rotateBy(clampedPosition.inverse());
        double relativeRadians = relativeRotation.getRadians();
        final double kPiOver2 = Math.PI / 2.0;
        if (relativeRadians > kPiOver2) {
            // Flipping drive direction would be the shorter path.
            flip = true;
            relativeRadians -= Math.PI;
        } else if (relativeRadians < -kPiOver2) {
            // Flipping drive direction would be the shorter path.
            flip = true;
            relativeRadians += Math.PI;
        }
        setSteerAngleUnclamped(unclampedPosition + relativeRadians, motionMagic);

        return flip;
    }

    public void setSteerAngleUnclamped(double steerAngleRadians, boolean motionMagic) {
        if (motionMagic) {
            mSteeringMotor.setControl(new MotionMagicVoltage((steerAngleRadians + mTalonOffset.getRadians()) / kSteerPositionCoefficient).withSlot(0));
        } else {
            mSteeringMotor.setControl(new PositionDutyCycle((steerAngleRadians + mTalonOffset.getRadians()) / kSteerPositionCoefficient).withSlot(1));
        }
    }

    @Override
    public void readPeriodicInputs() {
//        mDriveMotor.feed();
//        mSteeringMotor.feed();
    }

    @Override
    public void stop() {
        mDriveMotor.stopMotor();
        mSteeringMotor.stopMotor();
    }

    public StatusSignalValue<Double> getDrivePositionSignalValue() {
        return mDriveMotorPositionSignalValue;
    }

    public StatusSignalValue<Double> getDriveMotorClosedLoopErrorSignalValue() {
        return mDriveMotorClosedLoopErrorSignalValue;
    }

    public StatusSignalValue<Double> getDriveVelocitySignalValue() {
        return mDriveMotorVelocitySignalValue;
    }

    public StatusSignalValue<Double> getSteerPositionSignalValue() {
        return mSteeringMotorPositionSignalValue;
    }

    public StatusSignalValue<Double> getSteerVelocitySignalValue() {
        return mSteeringMotorVelocitySignalValue;
    }

    public StatusSignalValue<Double> getSteerErrorSignalValue() {
        return mSteeringMotorClosedLoopErrorSignalValue;
    }

    @Override
    public boolean checkSystem() {
        // Not implemented
        return false;
    }

    @Override
    public void outputTelemetry(boolean disabled) {
        // Not implemented
    }

    @Override
    public void rewriteDeviceConfiguration() {
        TalonUtil.applyAndCheckConfiguration(mDriveMotor, mDriveConfig);
        TalonUtil.applyAndCheckConfiguration(mSteeringMotor, mSteerConfig);
    }

    @Override
    public boolean checkDeviceConfiguration() {
        return TalonUtil.readAndVerifyConfiguration(mDriveMotor, mDriveConfig)
                && TalonUtil.readAndVerifyConfiguration(mSteeringMotor, mSteerConfig);
    }
}
