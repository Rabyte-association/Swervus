package frc.robot.commands;

import java.util.function.Supplier;

import org.ietf.jgss.Oid;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.WindochwytakSubsystem;

public class WindochwytakCmd extends CommandBase {

    private final WindochwytakSubsystem windochwytakSubsystem;
    private final Supplier<Joystick> windochwytakJoystick, driverJoystick;
    private double wyciagarkaFunction;
    private double przodtylFunction;
    private double lewyPrzodFunction, prawyPrzodFunction;
    private boolean lewyTylFunction, prawyTylFunction;

    public WindochwytakCmd(WindochwytakSubsystem windochwytakSubsystem,
            Supplier<Joystick> driverJoystick,
            Supplier<Joystick> windochwytakJoystick) {

        this.windochwytakSubsystem = windochwytakSubsystem;
        this.windochwytakJoystick = windochwytakJoystick;
        this.driverJoystick = driverJoystick;

        addRequirements(windochwytakSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        wyciagarkaFunction = windochwytakJoystick.get().getRawAxis(OIConstants.kWyciagarkaAxis);
        przodtylFunction = windochwytakJoystick.get().getRawAxis(OIConstants.kPrzodTylAxis);
        lewyPrzodFunction = windochwytakJoystick.get().getRawAxis(OIConstants.kLewyPrzod);
        lewyTylFunction = windochwytakJoystick.get().getRawButton(OIConstants.kLewyTyl);
        prawyPrzodFunction = windochwytakJoystick.get().getRawAxis(OIConstants.kPrawyPrzod);
        prawyTylFunction = windochwytakJoystick.get().getRawButton(OIConstants.kPrawyTyl);

        wyciagarkaFunction = ApplyDeadband(wyciagarkaFunction);
        przodtylFunction = ApplyDeadband(przodtylFunction);
        lewyPrzodFunction = ApplyDeadband(lewyPrzodFunction);
        prawyPrzodFunction = ApplyDeadband(prawyPrzodFunction);

        double lewy, prawy;
        lewy = lewyPrzodFunction - (lewyTylFunction ? 1 : 0);
        prawy = prawyPrzodFunction - (prawyTylFunction ? 1 : 0);

        if(wyciagarkaFunction == 0) wyciagarkaFunction = driverJoystick.get().getPOV() == 0 ? -1 : 0;
        if(wyciagarkaFunction == 0) wyciagarkaFunction = driverJoystick.get().getPOV() == 180 ? 1 : 0;
        if(przodtylFunction == 0) przodtylFunction = driverJoystick.get().getPOV() == 90 ? 1 : 0;
        if(przodtylFunction == 0) przodtylFunction = driverJoystick.get().getPOV() == 270 ? -1 : 0;
        if(lewy == 0) lewyPrzodFunction = driverJoystick.get().getRawAxis(OIConstants.kLewyPrzod);
        if(lewy == 0) lewyTylFunction = driverJoystick.get().getRawButton(OIConstants.kLewyTyl);
        if(prawy == 0) prawyPrzodFunction = driverJoystick.get().getRawAxis(OIConstants.kPrawyPrzod);
        if(prawy == 0) prawyTylFunction = driverJoystick.get().getRawButton(OIConstants.kPrawyTyl);

        lewy = lewyPrzodFunction - (lewyTylFunction ? 1 : 0);
        prawy = prawyPrzodFunction - (prawyTylFunction ? 1 : 0);

        lewyPrzodFunction = ApplyDeadband(lewyPrzodFunction);
        prawyPrzodFunction = ApplyDeadband(prawyPrzodFunction);

        // 6. Output each module states to motors
        windochwytakSubsystem.moveWindochwytak(wyciagarkaFunction, przodtylFunction, lewy, prawy);
    }

    @Override
    public void end(boolean interrupted) {
        windochwytakSubsystem.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public double ApplyDeadband(double x) {
        double value = x;
        if(Math.abs(value) <= OIConstants.kDeadbandWindochwytak)
            value = 0;
        return value;
    }
}
