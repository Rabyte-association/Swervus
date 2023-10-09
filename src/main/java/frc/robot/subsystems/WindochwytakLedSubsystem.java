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
    AddressableLEDBuffer windochwytakLed_buffer = new AddressableLEDBuffer(54);
    AddressableLEDBuffer driverLedMask = new AddressableLEDBuffer(54);
    int RSL_LedLenght = 5;
    public double brightness = 1;


    public WindochwytakLedSubsystem() {
        windochwytakLed.setLength(windochwytakLed_buffer.getLength());
        windochwytakLed.setData(windochwytakLed_buffer);
        windochwytakLed.start();
        for(int i = 0; i < driverLedMask.getLength()-RSL_LedLenght; i++) {
            driverLedMask.setHSV(i, (driverLedMask.getLength()-RSL_LedLenght-i)*(100/(driverLedMask.getLength()-RSL_LedLenght)), 255, 255);
        }
        new Thread(() -> {}).start();
    }

    public void SetLed(int r, int g, int b) {
        for(int i = 0; i < windochwytakLed_buffer.getLength()-RSL_LedLenght; i++)
            windochwytakLed_buffer.setRGB(i, (int)(r*brightness), (int)(g*brightness), (int)(b*brightness));
        windochwytakLed.setData(windochwytakLed_buffer);
    }

    public void SetLed(int r, int g, int b, int begin, int end) {
        int size = windochwytakLed_buffer.getLength()-RSL_LedLenght;
        if(begin >= size || end >= size || begin < 0 || end < 0) {
            return;
        }
        for(int i = begin; i <= end; i++) {
            windochwytakLed_buffer.setRGB(i, (int)(r*brightness), (int)(g*brightness), (int)(b*brightness));
        }
        windochwytakLed.setData(windochwytakLed_buffer);
    }

    public void SetLed_RSL(int r, int g, int b) { // rsl
        int size = windochwytakLed_buffer.getLength();
        for(int i = size-RSL_LedLenght+1; i < size; i++) {
            windochwytakLed_buffer.setRGB(i, r, g, b);
        }
        windochwytakLed_buffer.setRGB(size-RSL_LedLenght, 0, 0, 0);
        windochwytakLed.setData(windochwytakLed_buffer);
    }

    public void DriverMode(double power) {
        for(int i = 0; i < windochwytakLed_buffer.getLength()-RSL_LedLenght; i++) {
            windochwytakLed_buffer.setRGB(i, 0, 0, 0);
        }
        for(int i = 0; i < power*(driverLedMask.getLength()-RSL_LedLenght); i++) {
            windochwytakLed_buffer.setRGB(i, 
            (int)(driverLedMask.getLED(i).red),
            (int)(driverLedMask.getLED(i).green), 
            (int)(driverLedMask.getLED(i).blue));
        }
        windochwytakLed.setData(windochwytakLed_buffer);
    }

    private int winda_center[] = {10, 25, 43};
    private int winda_steps = 5;
    public void Winda(boolean isInverted) {
        for(int i = 0; i < windochwytakLed_buffer.getLength()-RSL_LedLenght; i++) { windochwytakLed_buffer.setRGB(i, 0,0,0); }
        for(int j = 0; j < winda_center.length; j++) {
            for(int i = 0; i < windochwytakLed_buffer.getLength()-RSL_LedLenght; i++) {
                int absolute = 0;
                if(winda_center[j] - i > 0) absolute = winda_center[j] - i;
                if(winda_center[j] - i < 0) absolute = i - winda_center[j];
                
                if(absolute <= winda_steps) {
                    double current_brightness = (double)((double)(winda_steps - absolute) / (double)winda_steps);
                    //if(current_brightness != 1) current_brightness *= 0.1;
                    windochwytakLed_buffer.setRGB(i, (int)((double)255*(double)current_brightness*current_brightness), (int)((double)80*(double)current_brightness*current_brightness), 0);
                    SmartDashboard.putNumber("jaaapierdoleee", windochwytakLed_buffer.getLED(i).red);
                }
                
            }
            winda_center[j] = winda_center[j] + 1*(isInverted?-1:1);
            if(winda_center[j] >= windochwytakLed_buffer.getLength()-RSL_LedLenght) winda_center[j] = 0;
            if(winda_center[j] < 0) winda_center[j] = windochwytakLed_buffer.getLength() - RSL_LedLenght;
        }
        windochwytakLed.setData(windochwytakLed_buffer);                
    }

    public void PrzodTyl(boolean isInverted) {

    }

    @Override
    public void periodic() {
    }


}
