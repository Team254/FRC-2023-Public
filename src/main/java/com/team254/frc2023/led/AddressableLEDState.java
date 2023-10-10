package com.team254.frc2023.led;

import com.ctre.phoenix.led.CANdle;
import com.team254.frc2023.Constants;

import java.util.Arrays;

public class AddressableLEDState implements ILEDDisplayable {
    private final int kCANdlePixels = 8;

    protected LEDState[] mPixels;

    public AddressableLEDState(LEDState pattern[]) {
        set(pattern);
    }

    public AddressableLEDState(LEDState color) {
        reset(color);
    }

    public AddressableLEDState() {
        reset(LEDState.kOff);
    }

    public AddressableLEDState(AddressableLEDState other) {
        reset(LEDState.kOff);
        this.copyFrom(other);
    }

    public void set(LEDState pattern[]) {
        if (mPixels == null) {
            reset(LEDState.kOff);
        }
        System.arraycopy(pattern, 0, mPixels, kCANdlePixels, Math.min(pattern.length, Constants.kMaxLEDCount));
    }

    public void reset(LEDState color) {
        if (mPixels == null) {
            mPixels = new LEDState[Constants.kMaxLEDCount + kCANdlePixels];
        }
        Arrays.fill(mPixels, color);
    }

    public AddressableLEDState mirrored() {
        AddressableLEDState s = new AddressableLEDState(this);
        for (int i = kCANdlePixels; i < kCANdlePixels + (mPixels.length - kCANdlePixels) / 2; i++) {
            s.mPixels[s.mPixels.length - i - 1 + kCANdlePixels] = s.mPixels[i];
        }
        return s;
    }

    @Override
    public void writePixels(CANdle candle) {
        if (mPixels == null || mPixels.length == 0) {
            // do not write empty data
            return;
        }
        // ctre api writes pixels in runs, so to optimally write pixels, we calculate run lengths
        LEDState run = mPixels[0];
        int runStart = 0;
        for (int i = 0; i < mPixels.length; i++) {
            if (mPixels[i] == null) mPixels[i] = LEDState.kOff;
            if (!run.equals(mPixels[i])) {
                candle.setLEDs(run.red, run.green, run.blue, 255, runStart, (i-runStart));
                runStart = i;
                run = mPixels[i];
            }
        }
        candle.setLEDs(run.red, run.green, run.blue, 255, runStart, (mPixels.length-runStart));
    }

    public void copyFrom(AddressableLEDState other) {
        System.arraycopy(other.mPixels, 0, mPixels, 0, Math.min(this.mPixels.length, other.mPixels.length));
    }

    @Override
    public String toString() {
        return Arrays.toString(this.mPixels);
    }
}
