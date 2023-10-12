package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WindaConstants;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.OIConstants;

public class WindochwytakSubsystem extends SubsystemBase {

    private WPI_TalonSRX chwytakLewy = new WPI_TalonSRX(WindaConstants.kChwytakLewyPort);
    private WPI_TalonSRX chwytakPrawy = new WPI_TalonSRX(WindaConstants.kChwytakPrawyPort);
    private WPI_TalonSRX wyciagarka = new WPI_TalonSRX(WindaConstants.kWciagarkaPort);
    private WPI_TalonSRX przodtyl = new WPI_TalonSRX(WindaConstants.kPrzodTylPort);

    public WindochwytakSubsystem() {
        wyciagarka.setInverted(true);
        przodtyl.setInverted(true);
        chwytakLewy.configContinuousCurrentLimit(1);
        chwytakPrawy.configContinuousCurrentLimit(1);
        przodtyl.configContinuousCurrentLimit(5);
        wyciagarka.configContinuousCurrentLimit(10);
        new Thread(() -> {}).start();
    }

    public void stopMotors() {
        chwytakLewy.set(0);
        chwytakPrawy.set(0);
        wyciagarka.set(0);
        przodtyl.set(0);
    }

    public void moveWindochwytak(double wyciagarkaValue, double przodtylValue, double lewyValue, double prawyValue) {
        wyciagarka.set(wyciagarkaValue);
        przodtyl.set(przodtylValue);
        chwytakLewy.set(-lewyValue);
        chwytakPrawy.set(-prawyValue);
    }

    @Override
    public void periodic() {
    }


}
