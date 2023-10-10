package com.team254.frc2023.led;

import com.team254.frc2023.Constants;
import com.team254.lib.util.Util;

public interface TimedLEDState {
    void getCurrentLEDState(LEDStateContainer desiredState, double timestamp);

    class BlinkingLEDState implements TimedLEDState {
        public static BlinkingLEDState kTest = new BlinkingLEDState(LEDState.kOff, LEDState.kBlue, 1);
        public static BlinkingLEDState kConfigureFail = new BlinkingLEDState(LEDState.kOff, LEDState.kRed, 0.1);
        public static BlinkingLEDState kAutoAlign = new BlinkingLEDState(LEDState.kOff, LEDState.kAutoAlign, 0.2);
        public static BlinkingLEDState kScore = new BlinkingLEDState(LEDState.kOff, LEDState.kScore, 0.2);
        public static BlinkingLEDState kConeIntakeWaiting = new BlinkingLEDState(LEDState.kOff, LEDState.kIntakingCone, 0.25);
        public static BlinkingLEDState kCubeIntakeWaiting = new BlinkingLEDState(LEDState.kOff, LEDState.kIntakingCube, 0.25);

        public static BlinkingLEDState kVisionMissing = new BlinkingLEDState(LEDState.kOff, LEDState.kYellow, 0.1);
        public static BlinkingLEDState kVisionPresent = new BlinkingLEDState(LEDState.kOff, LEDState.kGreen, 0.1);


        LEDState mStateOne = new LEDState(0, 0, 0);
        LEDState mStateTwo = new LEDState(0, 0, 0);
        private boolean mAsymmetricDuration = false;
        private double mDuration;
        private double mDurationTwo;

        public BlinkingLEDState(LEDState stateOne, LEDState stateTwo, double duration) {
            mStateOne.copyFrom(stateOne);
            mStateTwo.copyFrom(stateTwo);
            mDuration = duration;
        }

        public BlinkingLEDState(LEDState stateOne, LEDState stateTwo, double durationOne, double durationTwo) {
            mStateOne.copyFrom(stateOne);
            mStateTwo.copyFrom(stateTwo);
            mDuration = durationOne;
            mDurationTwo = durationTwo;
            mAsymmetricDuration = true;
        }

        @Override
        public void getCurrentLEDState(LEDStateContainer desiredState, double timestamp) {
            if(mAsymmetricDuration) {
                if (timestamp % (mDuration + mDurationTwo) > mDuration) {
                    desiredState.copyFrom(mStateTwo);
                } else {
                    desiredState.copyFrom(mStateOne);
                }
            } else {
                if ((int) (timestamp / mDuration) % 2 == 0) {
                    desiredState.copyFrom(mStateOne);
                } else {
                    desiredState.copyFrom(mStateTwo);
                }
            }
        }
    }

    class PercentFullLEDState implements TimedLEDState {
        AddressableLEDState mState;

        public PercentFullLEDState(double percentFull, LEDState fullColor) {
            LEDState[] pixels = new LEDState[Constants.kMaxLEDCount / 2];
            for (int i = 0; i < pixels.length; i++) {
                if (i < pixels.length * Util.limit(percentFull, 0.0, 1.0)) {
                    pixels[i] = fullColor;
                }
            }
            mState = new AddressableLEDState(pixels).mirrored();
        }

        @Override
        public void getCurrentLEDState(LEDStateContainer desiredState, double timestamp) {
            desiredState.copyFrom(mState);
        }
    }

    class StaticLEDState implements TimedLEDState {
        public static StaticLEDState kStaticOff = new StaticLEDState(LEDState.kOff);
        public static StaticLEDState kStaticRobotZeroedWithGoodBattery = new StaticLEDState(LEDState.kRobotZeroedWithGoodBattery);
        public static StaticLEDState kStaticBatteryLow = new StaticLEDState(LEDState.kBatteryLow);
        public static StaticLEDState kTest = new StaticLEDState(LEDState.kPurple);
        public static StaticLEDState kHasCube = new StaticLEDState(LEDState.kIntakingCube);
        public static StaticLEDState kHasCone = new StaticLEDState(LEDState.kIntakingCone);
        public static StaticLEDState kStow = new StaticLEDState(LEDState.kStow);
        public static StaticLEDState kStaticNotHomed = new StaticLEDState(LEDState.kYellow);
        public static StaticLEDState kAtAlignment = new StaticLEDState(LEDState.kAutoAlign);
        public static StaticLEDState kVisionDisabled = new StaticLEDState(LEDState.kRed);

        LEDState mStaticState = new LEDState(0, 0, 0);

        public StaticLEDState(LEDState staticState) {
            mStaticState.copyFrom(staticState);
        }

        @Override
        public void getCurrentLEDState(LEDStateContainer desiredState, double timestamp) {
            desiredState.copyFrom(mStaticState);
        }
    }
}
