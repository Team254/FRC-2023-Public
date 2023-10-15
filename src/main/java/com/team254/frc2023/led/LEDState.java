package com.team254.frc2023.led;

import com.ctre.phoenix.led.CANdle;

public class LEDState implements ILEDDisplayable {
    public static final LEDState kOff = new LEDState(0, 0, 0); //No Color

    public static final LEDState kRobotZeroedWithGoodBattery = new LEDState(0, 255, 0); //Green

    public static final LEDState kBatteryLow = new LEDState(255, 0, 0); //Red

    public static final LEDState kRed = new LEDState(255, 0 , 0);
    public static final LEDState kBlue = new LEDState(0, 0 , 255);
    public static final LEDState kGreen = new LEDState(0, 255, 0);
    public static final LEDState kYellow = new LEDState(255, 215, 0);
    public static final LEDState kPurple = new LEDState(255, 0, 255);

    public static final LEDState kAutoAlign = new LEDState(0, 0, 255);
    public static final LEDState kIntakingCube = new LEDState(125, 0, 255);
    public static final LEDState kIntakingCone = new LEDState(255, 215, 0);

    public static final LEDState kScore = new LEDState(0, 255, 0);
    public static final LEDState kStow = new LEDState(0, 0, 255);

    public LEDState() {
        blue = 0;
        green = 0;
        red = 0;
    }

    public LEDState(int r, int g, int b) {
        blue = b;
        green = g;
        red = r;
    }

    public void copyFrom(LEDState other) {
        this.blue = other.blue;
        this.green = other.green;
        this.red = other.red;
    }

    public int blue;
    public int green;
    public int red;

    @Override
    public void writePixels(CANdle candle) {
        candle.setLEDs(this.red, this.green, this.blue);
    }

    @Override
    public boolean equals(Object other) {
        if (other == null) {
            return false;
        }

        if (other.getClass() != this.getClass()) {
            return false;
        }

        LEDState s = (LEDState) other;
        return this.blue == s.blue && this.red == s.red && this.green == s.green;
    }

    @Override
    public String toString() {
        return "#" + Integer.toHexString(red) + Integer.toHexString(green) + Integer.toHexString(blue);
    }
}
