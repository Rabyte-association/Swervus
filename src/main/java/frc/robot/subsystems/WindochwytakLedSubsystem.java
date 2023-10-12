package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WindaConstants;

import java.util.Random;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.OIConstants;


public class WindochwytakLedSubsystem extends SubsystemBase {

    private final AddressableLED windochwytakLed = new AddressableLED(8);

    public final WindochwytakLedModule left = new WindochwytakLedModule(50);
    public final WindochwytakLedModule right = new WindochwytakLedModule(54);

    public WindochwytakLedSubsystem() {
        windochwytakLed.setLength(left.getBuffer().getLength() + right.getBuffer().getLength());
        windochwytakLed.start();
        new Thread(() -> {}).start();
    } // 8

    public void setBrightness(double brightness) {
        left.brightness = brightness;
        right.brightness = brightness;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("rand", left.brightness);
        AddressableLEDBuffer leftBuffer = left.getBuffer();
        AddressableLEDBuffer rightBuffer = right.getBuffer();
        AddressableLEDBuffer final_buffer = new AddressableLEDBuffer(leftBuffer.getLength() + rightBuffer.getLength());
        for(int i = 0; i < leftBuffer.getLength(); i++) {
            final_buffer.setLED(i, leftBuffer.getLED(i));
        }
        for(int i = leftBuffer.getLength(); i < rightBuffer.getLength()+leftBuffer.getLength(); i++) {
            final_buffer.setLED(i, rightBuffer.getLED((rightBuffer.getLength()-1)-(i-leftBuffer.getLength())));
        }
        windochwytakLed.setData(final_buffer);
    }


}
