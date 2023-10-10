package com.team254.frc2023.led;

import com.ctre.phoenix.led.CANdle;

public interface ILEDDisplayable {
    void writePixels(CANdle candle);
}
