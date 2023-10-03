package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.WindochwytakLedSubsystem;

public class WindochwytakLedCmd extends CommandBase {

    private final WindochwytakLedSubsystem windochwytakLedSubsystem;
    private final Supplier<Joystick> ledJoystick;
    private boolean ledMasterButton = false;
    private boolean ledSwitchButton = false;
    private boolean ledStrobeButton = false;
    private boolean ledDriverButton = false;
    private final Supplier<Double> ledDriverJoystick;
    //private boolean driverSwitch = false;
    //private boolean driverSwitchDebounce = false;
    private int strobeSpeed = 10;
    private int activeLedSpeed = 20;
    private int delta = 0;

    public WindochwytakLedCmd(WindochwytakLedSubsystem windochwytakLedSubsystem,
            Supplier<Joystick> ledJoystick,
            Supplier<Double> ledDriverJoystick) {

        this.windochwytakLedSubsystem = windochwytakLedSubsystem;
        this.ledJoystick = ledJoystick;
        this.ledDriverJoystick = ledDriverJoystick;

        addRequirements(windochwytakLedSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        ledMasterButton =  ledJoystick.get().getRawButton(OIConstants.kLedMaster);
        ledDriverButton = ledJoystick.get().getRawButton(OIConstants.kLedDriverSwitch);
        ledStrobeButton = ledJoystick.get().getRawButton(OIConstants.kRedBlue);
        
        if(delta%activeLedSpeed < activeLedSpeed/2) // active led blinking
            windochwytakLedSubsystem.SetActiveLed(255, 0, 0);
        else
            windochwytakLedSubsystem.SetActiveLed(0, 0, 0);
        if(delta > 100) delta = 0; // delta
        delta++;

        if(!ledMasterButton) // led master switch
            return;

        // -----------------------------

        if(ledSwitchButton)
            windochwytakLedSubsystem.SetLed(0, 0, 255);
        else if(!ledStrobeButton && !ledSwitchButton && !ledDriverButton)
            windochwytakLedSubsystem.SetLed(255, 0, 255);
        if(ledStrobeButton) {
            if(delta % strobeSpeed < strobeSpeed/2) {
                windochwytakLedSubsystem.SetLed(255, 0, 0);
            } else {
                windochwytakLedSubsystem.SetLed(0, 0, 255);
            }
        }

        //if(ledDriverFunction == true && driverSwitchDebounce == false) {
        //    driverSwitch = !driverSwitch;
        //    driverSwitchDebounce = true;
        //} else if(ledDriverFunction == false) {
        //    driverSwitchDebounce = false;
        //}

        if(ledDriverButton) {
            double joystick = Math.abs(ledDriverJoystick.get());
            windochwytakLedSubsystem.DriverMode(joystick);
        }

        
    }

    @Override
    public void end(boolean interrupted) {
        windochwytakLedSubsystem.SetActiveLed(0, 255, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
