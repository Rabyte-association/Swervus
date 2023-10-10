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
import frc.robot.subsystems.WindochwytakLedModule;

public class WindochwytakLedCmd extends CommandBase {

    private final WindochwytakLedSubsystem windochwytakLedSubsystem;

    private final Supplier<Joystick> ledJoystick;
    private boolean ledMasterButton = false;
    private boolean ledSwitchButton = false;
    private boolean ledStrobeButton = false;
    private boolean ledDriverButton = false;
    private boolean ledFastStrobeButton = false;
    private boolean windaButton = false;
    private boolean windaButtonInverted = false;
    private boolean autoButton = false;
    private final Supplier<Joystick> ledDriverJoystick;
    private final Supplier<Joystick> windochwytakJoystick;
    //private boolean driverSwitch = false;
    //private boolean driverSwitchDebounce = false;
    private int strobeSpeed = 10;
    private int fastStrobeSpeed = 2;
    private int blinkerSpeed = 20;

    

    private int RSL_LedSpeed = 20;
    private int delta = 0;

    public WindochwytakLedCmd(WindochwytakLedSubsystem windochwytakLedSubsystem,
            Supplier<Joystick> ledJoystick,
            Supplier<Joystick> ledDriverJoystick,
            Supplier<Joystick> windochwytakJoystick) {

        this.windochwytakLedSubsystem = windochwytakLedSubsystem;
        this.ledJoystick = ledJoystick;
        this.ledDriverJoystick = ledDriverJoystick;
        this.windochwytakJoystick = windochwytakJoystick;

        addRequirements(windochwytakLedSubsystem);
    }

    @Override
    public void initialize() {
    }

    private void RSL(WindochwytakLedSubsystem windochwytakLedSubsystem, int delta) {
        if(delta%RSL_LedSpeed < RSL_LedSpeed/2) {// rsl led blinking
            windochwytakLedSubsystem.left.SetLed_RSL(255, 0, 0);
            windochwytakLedSubsystem.right.SetLed_RSL(255, 0, 0); }
        else {
            windochwytakLedSubsystem.left.SetLed_RSL(0, 0, 0);
            windochwytakLedSubsystem.right.SetLed_RSL(0, 0, 0);
        }
    }

    @Override
    public void execute() {
        ledMasterButton =  ledJoystick.get().getRawButton(OIConstants.kLedMaster);
        ledDriverButton = ledJoystick.get().getRawButton(OIConstants.kLedDriverSwitch);
        ledStrobeButton = ledJoystick.get().getRawButton(OIConstants.kRedBlue);
        ledFastStrobeButton = ledJoystick.get().getRawButton(OIConstants.kStrobeButton);
        windaButton = ledJoystick.get().getRawButton(9);
        windaButtonInverted = ledJoystick.get().getRawButton(10);
        autoButton = ledJoystick.get().getRawButton(8);

        double brightnessAxis = -ledJoystick.get().getRawAxis(OIConstants.kLedBrighnessAxis); // brightness control
        if(brightnessAxis > 0.5 && windochwytakLedSubsystem.left.brightness < 1) { windochwytakLedSubsystem.setBrightness(windochwytakLedSubsystem.left.brightness+=0.01); } 
        else if ( brightnessAxis < -0.5 && windochwytakLedSubsystem.left.brightness > 0) { windochwytakLedSubsystem.setBrightness(windochwytakLedSubsystem.left.brightness-=0.01); }

        RSL(windochwytakLedSubsystem, delta);
        
        if(delta > 100) delta = 0; // delta
        delta++;

        if(!ledMasterButton) // led master switch
            return;

        // -----------------------------
        int mode = 0;
        
        if(!autoButton) {
            if(ledDriverButton) {SetMode(windochwytakLedSubsystem.left, 1, delta); SetMode(windochwytakLedSubsystem.right, 1, delta);}
            if(windaButton) {SetMode(windochwytakLedSubsystem.left, 5, delta); SetMode(windochwytakLedSubsystem.right, 5, delta);}
            if(windaButtonInverted) {SetMode(windochwytakLedSubsystem.left, 6, delta); SetMode(windochwytakLedSubsystem.right, 6, delta);}
            if(ledStrobeButton) {SetMode(windochwytakLedSubsystem.left, 10, delta); SetMode(windochwytakLedSubsystem.right, 10, delta);}
            if(ledFastStrobeButton) {SetMode(windochwytakLedSubsystem.left, 11, delta); SetMode(windochwytakLedSubsystem.right, 11, delta);}
        }
        else {
            mode = 1;
            if(ledDriverJoystick.get().getPOV() == 0) {SetMode(windochwytakLedSubsystem.left, 5, delta); SetMode(windochwytakLedSubsystem.right, 5, delta);}
            if(ledDriverJoystick.get().getPOV() == 180) {SetMode(windochwytakLedSubsystem.left, 6, delta); SetMode(windochwytakLedSubsystem.right, 6, delta);}
        }

        //if(ledDriverFunction == true && driverSwitchDebounce == false) {
        //    driverSwitch = !driverSwitch;
        //    driverSwitchDebounce = true;
        //} else if(ledDriverFunction == false) {
        //    driverSwitchDebounce = false;
        //}
    }

    private void SetMode(WindochwytakLedModule windochwytakLedModule, int mode, int delta) {
        switch(mode) { // led priority
        case 0: // off
            windochwytakLedModule.SetLed(0, 0, 0);
            break;
        case 1: // driver led
            double joystick = Math.abs(ledDriverJoystick.get().getRawAxis(OIConstants.kDriverYAxis));
            windochwytakLedModule.DriverMode(joystick);
            break;
        case 5: // winda
            if(delta%2 == 0) {
                windochwytakLedModule.Winda(false);
            }
            break;
        case 6:
            if(delta%2==0) {
                windochwytakLedModule.Winda(true);
            }
            break;
        case 9: // blinker
            if(delta % blinkerSpeed < blinkerSpeed/2) { windochwytakLedModule.SetLed(255, 128, 0); } 
            else { windochwytakLedModule.SetLed(0, 0, 0); }
            break;
        case 10: // polizei
            if(delta % strobeSpeed < strobeSpeed/2) { windochwytakLedModule.SetLed(255, 0, 0); } 
            else { windochwytakLedModule.SetLed(0, 0, 255); }
            break;
        case 11: // strobe
            if(delta % fastStrobeSpeed < fastStrobeSpeed/2) { windochwytakLedModule.SetLed(255, 255, 255); } 
            else { windochwytakLedModule.SetLed(0, 0, 0); }
            break;
        default:
            windochwytakLedModule.SetLed(255, 0, 255);
            break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        windochwytakLedSubsystem.right.SetLed_RSL(0, 255, 0);
        windochwytakLedSubsystem.left.SetLed_RSL(0, 255, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
