package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import java.io.Console;

import com.reduxrobotics.sensors.canandcoder.CANandcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class WindochwytakLedModule {

    private AddressableLEDBuffer windochwytakLed_buffer;
    private AddressableLEDBuffer driverLedMask;
    int RSL_LedLenght = 5;
    public double brightness = 1;

    public WindochwytakLedModule(int size) {
        windochwytakLed_buffer = new AddressableLEDBuffer(size);
        driverLedMask = new AddressableLEDBuffer(size);

        for(int i = 0; i < driverLedMask.getLength()-RSL_LedLenght; i++) {
            driverLedMask.setHSV(i, (driverLedMask.getLength()-RSL_LedLenght-i)*(100/(driverLedMask.getLength()-RSL_LedLenght)), 255, 255);
        }
    }

    public AddressableLEDBuffer SetLed(int r, int g, int b) {
        for(int i = 0; i < windochwytakLed_buffer.getLength()-RSL_LedLenght; i++)
            windochwytakLed_buffer.setRGB(i, (int)(r*brightness), (int)(g*brightness), (int)(b*brightness));
        return windochwytakLed_buffer;
    }

    public AddressableLEDBuffer SetLed(int r, int g, int b, int begin, int end) {
        int size = windochwytakLed_buffer.getLength()-RSL_LedLenght;
        if(begin >= size || end >= size || begin < 0 || end < 0) {
            return windochwytakLed_buffer;
        }
        for(int i = begin; i <= end; i++) {
            windochwytakLed_buffer.setRGB(i, (int)(r*brightness), (int)(g*brightness), (int)(b*brightness));
        }
        return windochwytakLed_buffer;
    }

    public AddressableLEDBuffer SetLed_RSL(int r, int g, int b) { // rsl
        int size = windochwytakLed_buffer.getLength();
        for(int i = size-RSL_LedLenght+1; i < size; i++) {
            windochwytakLed_buffer.setRGB(i, r, g, b);
        }
        windochwytakLed_buffer.setRGB(size-RSL_LedLenght, 0, 0, 0);
        return windochwytakLed_buffer;
    }

    public AddressableLEDBuffer DriverMode(double power) {
        for(int i = 0; i < windochwytakLed_buffer.getLength()-RSL_LedLenght; i++) {
            windochwytakLed_buffer.setRGB(i, 0, 0, 0);
        }
        for(int i = 0; i < power*(driverLedMask.getLength()-RSL_LedLenght); i++) {
            windochwytakLed_buffer.setRGB(i, 
            (int)(driverLedMask.getLED(i).red),
            (int)(driverLedMask.getLED(i).green), 
            (int)(driverLedMask.getLED(i).blue));
        }
        return windochwytakLed_buffer;
    }

    private int winda_center[] = {2, 20, 38};
    private int winda_steps = 5;
    public AddressableLEDBuffer Winda(boolean isInverted) {
        for(int i = 0; i < windochwytakLed_buffer.getLength()-RSL_LedLenght; i++) { windochwytakLed_buffer.setRGB(i, 0,0,0); }
        for(int j = 0; j < winda_center.length; j++) {
            for(int i = 0; i < windochwytakLed_buffer.getLength()-RSL_LedLenght; i++) {
                int absolute = 0;
                if(winda_center[j] - i > 0) absolute = winda_center[j] - i;
                if(winda_center[j] - i < 0) absolute = i - winda_center[j];
                
                if(absolute <= winda_steps) {
                    double current_brightness = (double)((double)(winda_steps - absolute) / (double)winda_steps);
                    windochwytakLed_buffer.setRGB(i, (int)((double)255*(double)current_brightness*current_brightness), (int)((double)80*(double)current_brightness*current_brightness), 0);
                    SmartDashboard.putNumber("jaaapierdoleee", windochwytakLed_buffer.getLED(i).red);
                }
                
            }
            winda_center[j] = winda_center[j] + 1*(isInverted?-1:1);
            if(winda_center[j] >= windochwytakLed_buffer.getLength()-RSL_LedLenght) winda_center[j] = 0;
            if(winda_center[j] < 0) winda_center[j] = windochwytakLed_buffer.getLength() - RSL_LedLenght;
        }
        return windochwytakLed_buffer;                
    }

    public AddressableLEDBuffer getBuffer() {
        return windochwytakLed_buffer;
    }

}
