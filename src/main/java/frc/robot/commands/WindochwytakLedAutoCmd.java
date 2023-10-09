package frc.robot.commands;

import java.util.function.Supplier;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.WindochwytakLedSubsystem;

public class WindochwytakLedAutoCmd extends CommandBase {

    private final WindochwytakLedSubsystem windochwytakLedSubsystem;
    private boolean ledMasterButton = false;
    private boolean ledSwitchButton = false;
    private boolean ledStrobeButton = false;
    private boolean ledDriverButton = false;
    private boolean ledFastStrobeButton = false;
    private boolean windaButton = false;
    private boolean autoButton = false;
    private final Supplier<Joystick> ledDriverJoystick;
    private final Supplier<Joystick> ledWindochwytakJoystick;
    //private boolean driverSwitch = false;
    //private boolean driverSwitchDebounce = false;
    private int strobeSpeed = 10;
    private int fastStrobeSpeed = 2;
    private int blinkerSpeed = 20;

    

    private int RSL_LedSpeed = 20;
    private int delta = 0;

    public WindochwytakLedAutoCmd(WindochwytakLedSubsystem windochwytakLedSubsystem,
            Supplier<Joystick> ledDriverJoystick, Supplier<Joystick> ledWindochwytakJoystick) {

        this.windochwytakLedSubsystem = windochwytakLedSubsystem;
        this.ledDriverJoystick = ledDriverJoystick;
        this.ledWindochwytakJoystick = ledWindochwytakJoystick;

        addRequirements(windochwytakLedSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if(delta%RSL_LedSpeed < RSL_LedSpeed/2) // rsl led blinking
            windochwytakLedSubsystem.SetLed_RSL(255, 0, 0);
        else
            windochwytakLedSubsystem.SetLed_RSL(0, 0, 0);
        if(delta > 100) delta = 0; // delta
        delta++;

        // -----------------------------
        int mode = 0;
        
        if(true) mode = 1; // driver mode
        if(ledDriverJoystick.get().getPOV() == 0 && ledDriverJoystick.get().getPOV() == 180) mode = 5; //winda button
        if(ledStrobeButton) mode = 10;
        if(ledFastStrobeButton) mode = 11;

        
        switch(mode) { // led priority
        case 0: // off
            windochwytakLedSubsystem.SetLed(0, 0, 0);
            break;
        case 1: // driver led
            double joystick = Math.abs(ledDriverJoystick.get().getRawAxis(OIConstants.kDriverXAxis));
            windochwytakLedSubsystem.DriverMode(joystick);
            break;
        case 5: // winda
            if(delta%2 == 0) {
                windochwytakLedSubsystem.Winda(ledDriverButton);
            }
            break;
        case 9: // blinker
            if(delta % blinkerSpeed < blinkerSpeed/2) { windochwytakLedSubsystem.SetLed(255, 128, 0); } 
            else { windochwytakLedSubsystem.SetLed(0, 0, 0); }
            break;
        case 10: // polizei
            if(delta % strobeSpeed < strobeSpeed/2) { windochwytakLedSubsystem.SetLed(255, 0, 0); } 
            else { windochwytakLedSubsystem.SetLed(0, 0, 255); }
            break;
        case 11: // strobe
            if(delta % fastStrobeSpeed < fastStrobeSpeed/2) { windochwytakLedSubsystem.SetLed(255, 255, 255); } 
            else { windochwytakLedSubsystem.SetLed(0, 0, 0); }
            break;
        default:
            windochwytakLedSubsystem.SetLed(255, 0, 255);
            break;
        }

        //if(ledDriverFunction == true && driverSwitchDebounce == false) {
        //    driverSwitch = !driverSwitch;
        //    driverSwitchDebounce = true;
        //} else if(ledDriverFunction == false) {
        //    driverSwitchDebounce = false;
        //}
    }

    @Override
    public void end(boolean interrupted) {
        windochwytakLedSubsystem.SetLed_RSL(0, 255, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
