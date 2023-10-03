package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WindaConstants;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.OIConstants;


public class WindochwytakLedSubsystem extends SubsystemBase {

    AddressableLED windochwytakLed = new AddressableLED(9);
    AddressableLEDBuffer windochwytakLed_buffer = new AddressableLEDBuffer(55);
    AddressableLEDBuffer driverLedMask = new AddressableLEDBuffer(55);
    int activeLedLenght = 5;

    public WindochwytakLedSubsystem() {
        windochwytakLed.setLength(windochwytakLed_buffer.getLength());
        windochwytakLed.setData(windochwytakLed_buffer);
        windochwytakLed.start();
        for(int i = 0; i < driverLedMask.getLength()-activeLedLenght; i++) {
            driverLedMask.setHSV(i, (driverLedMask.getLength()-activeLedLenght-i)*(100/(driverLedMask.getLength()-activeLedLenght)), 255, 255);
        }
        new Thread(() -> {}).start();
    }

    public void SetLed(int r, int g, int b) {
        for(int i = 0; i < windochwytakLed_buffer.getLength()-activeLedLenght; i++)
            windochwytakLed_buffer.setRGB(i, r, g, b);
        windochwytakLed.setData(windochwytakLed_buffer);
    }

    public void SetLed(int r, int g, int b, int begin, int end) {
        int size = windochwytakLed_buffer.getLength()-activeLedLenght;
        if(begin >= size || end >= size || begin < 0 || end < 0) {
            return;
        }
        for(int i = begin; i <= end; i++) {
            windochwytakLed_buffer.setRGB(i, r, g, b);
        }
        windochwytakLed.setData(windochwytakLed_buffer);
    }

    public void SetActiveLed(int r, int g, int b) {
        int size = windochwytakLed_buffer.getLength();
        for(int i = size-activeLedLenght+1; i < size; i++) {
            windochwytakLed_buffer.setRGB(i, r, g, b);
        }
        windochwytakLed_buffer.setRGB(size-activeLedLenght, 0, 0, 0);
        windochwytakLed.setData(windochwytakLed_buffer);
    }

    public void DriverMode(double power) {
        for(int i = 0; i < windochwytakLed_buffer.getLength()-activeLedLenght; i++) {
            windochwytakLed_buffer.setRGB(i, 0, 0, 0);
        }
        for(int i = 0; i < power*(driverLedMask.getLength()-activeLedLenght); i++) {
            windochwytakLed_buffer.setLED(i, driverLedMask.getLED(i));
        }
        windochwytakLed.setData(windochwytakLed_buffer);
    }

    @Override
    public void periodic() {
    }


}
