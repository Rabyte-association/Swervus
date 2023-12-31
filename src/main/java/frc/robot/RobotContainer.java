package frc.robot;

import java.util.List;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.WindochwytakLedSubsystem;
import frc.robot.subsystems.WindochwytakSubsystem;
import frc.robot.commands.WindochwytakCmd;


import frc.robot.commands.WindochwytakLedCmd;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final WindochwytakSubsystem windochwytakSubsystem = new WindochwytakSubsystem();
    private final WindochwytakLedSubsystem windochwytakLedSubsystem = new WindochwytakLedSubsystem();

    private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
    private final Joystick windochwytakJoystick = new Joystick(OIConstants.kWindochwytakControllerPort);

    public RobotContainer() {

        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));
        windochwytakSubsystem.setDefaultCommand(new WindochwytakCmd(
                windochwytakSubsystem,
                () -> driverJoytick,
                () -> windochwytakJoystick
        ));
        if(true)
        windochwytakLedSubsystem.setDefaultCommand(new WindochwytakLedCmd(
                windochwytakLedSubsystem,
                () -> driverJoytick,
                () -> windochwytakJoystick
        ));
        windochwytakLedSubsystem.left.SetLed_RSL(0, 255, 0);
        windochwytakLedSubsystem.right.SetLed_RSL(0, 255, 0);
        
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        new JoystickButton(driverJoytick, 8).whenPressed(() -> swerveSubsystem.resetEncoders());
        new JoystickButton(driverJoytick,(9)).whenPressed(() -> swerveSubsystem.zeroHeading());
        new JoystickButton(driverJoytick,(3)).whenPressed(() -> swerveSubsystem.breakingOFF());
        new JoystickButton(driverJoytick,(4)).whenPressed(() -> swerveSubsystem.breakingON());
    }


    public Command getAutonomousCommand() {
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                1.5,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory
        /*Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(1, 1),
                        new Translation2d(2, 0),
                        new Translation2d(3,1)),
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                trajectoryConfig);*/
        SmartDashboard.putNumber("metryrx", swerveSubsystem.getPose().getX());
        SmartDashboard.putNumber("metryry", swerveSubsystem.getPose().getY());


        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0,0 );
        PIDController yController = new PIDController(AutoConstants.kPYController, 0,0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                Robot.trajectory,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                //swerveSubsystem::desiredRotation,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);
        
        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(Robot.trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
    }
}