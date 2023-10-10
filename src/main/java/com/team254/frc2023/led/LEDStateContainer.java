package com.team254.frc2023.led;

import com.ctre.phoenix.led.CANdle;

public class LEDStateContainer implements ILEDDisplayable {
    LEDState staticState;
    AddressableLEDState addressableState;
    boolean inAddressableMode;

    public LEDStateContainer() {
        staticState = new LEDState();
        addressableState = new AddressableLEDState();
        inAddressableMode = false;
    }

    public void copyFrom(ILEDDisplayable other) {
        if (other instanceof LEDState) {
            staticState.copyFrom((LEDState)other);
            inAddressableMode = false;
        } else if (other instanceof AddressableLEDState) {
            addressableState.copyFrom((AddressableLEDState) other);
            inAddressableMode = true;
        } else if (other instanceof LEDStateContainer) {
            inAddressableMode = ((LEDStateContainer)other).inAddressableMode;
            staticState.copyFrom(((LEDStateContainer)other).staticState);
            addressableState.copyFrom(((LEDStateContainer)other).addressableState);
        }
    }

    @Override
    public void writePixels(CANdle candle) {
        if (inAddressableMode) {
            this.addressableState.writePixels(candle);
        } else {
            this.staticState.writePixels(candle);
        }
    }
}
