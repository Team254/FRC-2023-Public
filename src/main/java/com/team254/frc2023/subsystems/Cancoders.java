package com.team254.frc2023.subsystems;


import com.ctre.phoenixpro.StatusCode;
import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.CANcoderConfigurator;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.signals.SensorDirectionValue;
import com.team254.frc2023.Constants;
import com.team254.lib.drivers.CanDeviceId;
import edu.wpi.first.wpilibj.Timer;

import java.util.Optional;

// Conatiner to hold the Cancoders so we can initialize them
// earlier than everything else and DI them to the swerve modules
public class Cancoders {
    private final CANcoder mFrontLeft;
    private final CANcoder mFrontRight;
    private final CANcoder mBackLeft;
    private final CANcoder mBackRight;

    private final CanTsObserver mFrontRightObserver;
    private final CanTsObserver mFrontLeftObserver;
    private final CanTsObserver mBackLeftObserver;
    private final CanTsObserver mBackRightObserver;

    private static final double kBootUpErrorAllowanceTime = 10.0;

    private static class CanTsObserver {
        private final CANcoder cancoder;
        private Optional<Double> lastTs = Optional.empty();
        private int validUpdates = 0;
        private static final int kRequiredValidTimestamps = 10;

        public CanTsObserver(CANcoder cancoder) {
            this.cancoder = cancoder;
        }

        public boolean hasUpdate() {
            StatusSignalValue<Double> absolutePositionSignal = cancoder.getAbsolutePosition(); // Need to call this to update ts
            double ts = absolutePositionSignal.getTimestamp().getTime();
            if (lastTs.isEmpty()) {
                lastTs = Optional.of(ts);
            }
            if(ts > lastTs.get()) {
                validUpdates++;
                lastTs = Optional.of(ts);
            }
            return validUpdates > kRequiredValidTimestamps;
        }

    }

    private static Cancoders sInstance;
    public static Cancoders getInstance() {
        if (sInstance == null) {
            sInstance = new Cancoders();
        }
        return sInstance;
    }

    private CANcoder build(CanDeviceId canDeviceId) {
        CANcoder thisCancoder = new CANcoder(canDeviceId.getDeviceNumber(), canDeviceId.getBus());
        CANcoderConfigurator configurator = thisCancoder.getConfigurator();
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();

        canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        canCoderConfig.MagnetSensor.MagnetOffset = 0.0;
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; 

        double startTime = Timer.getFPGATimestamp();
        boolean timedOut = false;
        boolean goodInit = false;
        int attempt = 1;
        while (!goodInit && !timedOut) {
            System.out.println("Initing CANCoder " + canDeviceId.getDeviceNumber() + " / attempt: " + attempt);
            StatusCode settingsCode = configurator.apply(canCoderConfig);
            StatusCode sensorCode = thisCancoder.getAbsolutePosition().setUpdateFrequency(20);

            goodInit = settingsCode == StatusCode.OK && sensorCode == StatusCode.OK;

            timedOut = (Timer.getFPGATimestamp()) - startTime >= kBootUpErrorAllowanceTime;
            attempt++;
        }

        return thisCancoder;
    }

    private Cancoders() {
        mFrontLeft = build(Constants.kFrontLeftEncoderPortId);
        mFrontLeftObserver = new CanTsObserver(mFrontLeft);

        mFrontRight = build(Constants.kFrontRightEncoderPortId);
        mFrontRightObserver = new CanTsObserver(mFrontRight);

        mBackLeft = build(Constants.kBackLeftEncoderPortId);
        mBackLeftObserver = new CanTsObserver(mBackLeft);

        mBackRight = build(Constants.kBackRightEncoderPortId);
        mBackRightObserver = new CanTsObserver(mBackRight);
    }

    public boolean allHaveBeenInitialized() {
        return mFrontLeftObserver.hasUpdate() && mFrontRightObserver.hasUpdate() && mBackLeftObserver.hasUpdate() && mBackRightObserver.hasUpdate();
    }

    public CANcoder getFrontLeft() {
        return mFrontLeft;
    }

    public CANcoder getFrontRight() {
        return mFrontRight;
    }

    public CANcoder getBackLeft() {
        return mBackLeft;
    }

    public CANcoder getBackRight() {
        return mBackRight;
    }
}
