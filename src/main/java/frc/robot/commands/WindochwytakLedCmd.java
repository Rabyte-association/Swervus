package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.WindochwytakLedSubsystem;

public class WindochwytakLedCmd extends CommandBase {

    private final WindochwytakLedSubsystem windochwytakLedSubsystem;
    private final Supplier<Boolean> ledSwitchFunction;
    private final Supplier<Boolean> ledStrobeFunction;
    private final Supplier<Boolean> ledDriverFunction;
    private final Supplier<Double> ledDriverJoystick;
    private boolean driverSwitch = false;
    private boolean driverSwitchDebounce = false;
    private int strobeSpeed = 10;
    private int activeLedSpeed = 20;
    private int delta = 0;

    public WindochwytakLedCmd(WindochwytakLedSubsystem windochwytakLedSubsystem,
            Supplier<Boolean> ledSwitchFunction,
            Supplier<Boolean> ledStrobeFunction,
            Supplier<Boolean> ledDriverFunction,
            Supplier<Double> ledDriverJoystick) {

        this.windochwytakLedSubsystem = windochwytakLedSubsystem;
        this.ledSwitchFunction = ledSwitchFunction;
        this.ledStrobeFunction = ledStrobeFunction;
        this.ledDriverFunction = ledDriverFunction;
        this.ledDriverJoystick = ledDriverJoystick;

        addRequirements(windochwytakLedSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if(ledSwitchFunction.get())
            windochwytakLedSubsystem.SetLed(0, 0, 255);
        else if(!ledStrobeFunction.get() && !ledSwitchFunction.get() && !driverSwitch)
            windochwytakLedSubsystem.SetLed(255, 0, 255);
        if(ledStrobeFunction.get()) {
            if(delta % strobeSpeed < strobeSpeed/2) {
                windochwytakLedSubsystem.SetLed(255, 0, 0);
            } else {
                windochwytakLedSubsystem.SetLed(0, 0, 255);
            }
        }

        if(ledDriverFunction.get() == true && driverSwitchDebounce == false) {
            driverSwitch = !driverSwitch;
            driverSwitchDebounce = true;
        } else if(ledDriverFunction.get() == false) {
            driverSwitchDebounce = false;
        }
        SmartDashboard.putBoolean("driver", driverSwitch);

        if(driverSwitch) {
            double joystick = Math.abs(ledDriverJoystick.get());
            windochwytakLedSubsystem.DriverMode(joystick);
        }

        if(delta%activeLedSpeed < activeLedSpeed/2) // active led blinking
            windochwytakLedSubsystem.SetActiveLed(255, 0, 0);
        else
            windochwytakLedSubsystem.SetActiveLed(0, 0, 0);
        if(delta > 100) delta = 0; // delta
        delta++;
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
