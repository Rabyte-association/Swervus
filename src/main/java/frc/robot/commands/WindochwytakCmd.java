package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.WindochwytakSubsystem;

public class WindochwytakCmd extends CommandBase {

    private final WindochwytakSubsystem windochwytakSubsystem;
    private final Supplier<Double> wyciagarkaFunction;
    private final Supplier<Double> przodtylFunction;
    private final Supplier<Double> lewyPrzodFunction, prawyPrzodFunction;
    private final Supplier<Boolean> lewyTylFunction, prawyTylFunction;

    public WindochwytakCmd(WindochwytakSubsystem windochwytakSubsystem,
            Supplier<Double> wyciagarkaFunction, Supplier<Double> przodtylFunction, 
            Supplier<Double> lewyPrzod, Supplier<Double> prawyPrzod, 
            Supplier<Boolean> lewyTyl, Supplier<Boolean> prawyTyl) {

        this.windochwytakSubsystem = windochwytakSubsystem;
        this.wyciagarkaFunction = wyciagarkaFunction;
        this.przodtylFunction = przodtylFunction;
        this.lewyPrzodFunction = lewyPrzod;
        this.prawyPrzodFunction = prawyPrzod;
        this.lewyTylFunction = lewyTyl;
        this.prawyTylFunction = prawyTyl;

        addRequirements(windochwytakSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double wyciagarka = wyciagarkaFunction.get();
        if(Math.abs(wyciagarka) <= OIConstants.kDeadbandWindochwytak)
            wyciagarka = 0;
        double przodtyl = przodtylFunction.get();
        if(Math.abs(przodtyl) <= OIConstants.kDeadbandWindochwytak)
            przodtyl = 0;

        double lewyprzod = lewyPrzodFunction.get();
        if(Math.abs(lewyprzod) <= OIConstants.kDeadbandWindochwytak)
            lewyprzod = 0;
        double prawyprzod = prawyPrzodFunction.get();
        if(Math.abs(prawyprzod) <= OIConstants.kDeadbandWindochwytak)
            prawyprzod = 0;
        boolean lewytyl = lewyTylFunction.get();
        boolean prawytyl = prawyTylFunction.get();

        double lewy, prawy;
        lewy = lewyprzod - (lewytyl ? 1 : 0);
        prawy = prawyprzod - (prawytyl ? 1 : 0);

        // 6. Output each module states to motors
        windochwytakSubsystem.moveWindochwytak(wyciagarka, przodtyl, lewy, prawy);
    }

    @Override
    public void end(boolean interrupted) {
        windochwytakSubsystem.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
