// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import javax.sound.sampled.LineEvent;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
// import com.ctre.phoenix6.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
// import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.driveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.DriveModes;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LinearServo;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Pivort;
import frc.robot.subsystems.Vision.Vision;
import com.ctre.phoenix6.controls.Follower;

import com.ctre.phoenix6.Orchestra.*;
import com.ctre.phoenix6.configs.Slot0Configs;

public class RobotContainer {
    private double MaxSpeed = driveConstants.MaxSpeed;
    // private double MaxAngularRate = driveConstants.MaxAngularRate;

    /* Setting up bindings for necessary control of the swerve drive platform */
    // private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    //         .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    // private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public static final Telemetry logger = new Telemetry(driveConstants.MaxSpeed);

    public static CommandXboxController joystick = new CommandXboxController(0);
    public static XboxController xbox1 = new XboxController(0);
    public static XboxController xbox2 = new XboxController(1);

    
    public static final CANBus Driveloop = new CANBus("DriveLoop", "./logs/example.hoot");
    
    public static final CANBus Rio = new CANBus("Rio", "./logs/example.hoot");

    public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final static PhotonCamera camera = new PhotonCamera("PC_Camera");

    public static final Vision vision = new Vision();
    public static final Vision turretVision = new Vision();

    
    public static boolean hasFieldOriented = false;


    // // Ballz Launcher
    public static Pivort pivort = new Pivort();
    public static PivortCommand pivortCommand = new PivortCommand();

    //From back of robot.
    public static TalonFX shootMotorLeft = new TalonFX(15); //Leader
    public static TalonFX shootMotorMiddle = new TalonFX(14);
    public static TalonFX shootMotorRight = new TalonFX(13);
    public static TalonFX feedMotorLeft = new TalonFX(18);
    
    public static Slot0Configs shootMotorConfigs = new Slot0Configs()
        .withKP(0.0095)
        .withKI(0.0)
        .withKD(0.0)
        .withKV(.06);

    // public static TalonFX feedMotorMiddle = new TalonFX(17);
    public static TalonFX ExtraShootMotor = new TalonFX(16);
    public static TalonFX beltMotor = new TalonFX(19);
    
    public static Intake intake = new Intake();
    public static IntakeCommand intakeCommand = new IntakeCommand();
    public static TalonFX intakeMotor = new TalonFX(21);
    public static TalonFX intakeExtendMotor = new TalonFX(23);

    
    // public static LinearServo linearServo = new LinearServo(8, 100, 20);
    // public static LinearServo linearServo2 = new LinearServo(9, 100, 20);
    // public static LinearServoCommand linearServoCommand = new LinearServoCommand();

    public static Hood hood = new Hood();
    public static HoodCommand hoodCommand = new HoodCommand();
    public static TalonFX hoodMotor = new TalonFX(22);

    public static Shooter Shooter = new Shooter();
    public static ShootCommand shootCommand = new ShootCommand();

    // public static CANcoder shootCaNcoder = new CANcoder(22);

    public static boolean isRedAlliance = DriverStation.getAlliance().get().equals(Alliance.Red);

    Orchestra m_orchestra = new Orchestra();


    
    /* Path follower */
    private final SendableChooser<Command> autoChooser;
    public boolean isLowBattery = false;

    public RobotContainer() {
        shootMotorLeft.getConfigurator().apply(shootMotorConfigs);
        shootMotorMiddle.getConfigurator().apply(shootMotorConfigs);
        shootMotorRight.getConfigurator().apply(shootMotorConfigs);
        ExtraShootMotor.getConfigurator().apply(shootMotorConfigs);
        RobotContainer.intakeExtendMotor.setPosition(0);
        RobotContainer.hoodMotor.setPosition(0);
        SmartDashboard.putBoolean("Is low battery", !isLowBattery);
        shootMotorLeft.getVelocity().setUpdateFrequency(50);
        // shootMotorMiddle.setControl(new Follower(shootMotorLeft.getDeviceID(), MotorAlignmentValue.Aligned));
        // shootMotorRight.setControl(new Follower(shootMotorLeft.getDeviceID(), MotorAlignmentValue.Aligned));
        // ExtraShootMotor.setControl(new Follower(shootMotorLeft.getDeviceID(), MotorAlignmentValue.Opposed));
        
        // m_orchestra.addInstrument(shootMotorLeft);
        // m_orchestra.addInstrument(intakeMotor);
        // m_orchestra.addInstrument(shootMotorMiddle);
        // m_orchestra.addInstrument(shootMotorRight);
        // m_orchestra.addInstrument(ExtraShootMotor);
        // m_orchestra.addInstrument(feedMotorLeft);
        // m_orchestra.addInstrument(beltMotor);
        
        // var status = m_orchestra.loadMusic("funkytown.chrp");
        
        // m_orchestra.play();
        
        
        pivort.setDefaultCommand(pivortCommand);
        intake.setDefaultCommand(intakeCommand);
        Shooter.setDefaultCommand(shootCommand);
        hood.setDefaultCommand(hoodCommand);
        // linearServo.setDefaultCommand(linearServoCommand);
        // linearServo2.setDefaultCommand(linearServoCommand);
        
        NamedCommands.registerCommand("Intake", AutoCommands.test);
        NamedCommands.registerCommand("Line Up Shot", AutoCommands.lineUpShot);
        NamedCommands.registerCommand("Shoot", AutoCommands.Shoot);
        NamedCommands.registerCommand("Intake On", AutoCommands.IntakeOn);
        NamedCommands.registerCommand("Intake Off", AutoCommands.IntakeOff);
        NamedCommands.registerCommand("Extend Intake", AutoCommands.ExtendIntake);

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    public static void disableDefaultCommand() {
        drivetrain.setDefaultCommand(null);
    }
    public static void enableDefaultCommand() {
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                DriveModes.driveField
                    .withVelocityX(0) // Drive forward with negative Y (forward)
                    .withVelocityY(0) // Drive left with negative X (left)
                    .withRotationalRate(-Math.pow(Deadzone(joystick.getRightX()), driveConstants.Linearity) * driveConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
    }


    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                DriveModes.driveField
                    .withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(pivort.findRotateSpeed(-Deadzone(joystick.getRightX()) * driveConstants.MaxAngularRate)) // Drive counterclockwise with negative X (left)
            )
        );


        joystick.a().whileTrue(drivetrain.applyRequest(() -> 
            DriveModes.driveRobot
                .withVelocityX(-Sine(RobotContainer.joystick.getLeftX(), RobotContainer.joystick.getLeftY()) * driveConstants.MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-Cosine(RobotContainer.joystick.getLeftX(), RobotContainer.joystick.getLeftY()) * driveConstants.MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-Math.pow(Deadzone(RobotContainer.joystick.getRightX()), driveConstants.Linearity) * driveConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

        joystick.x().whileTrue(drivetrain.applyRequest(() -> DriveModes.brake));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.povDown().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
        
    
    }

    /** Function that returns a given speed, as long as it is above the deadzone set in Constants. */
    public static double Deadzone(double speed) {
        if (Math.abs(speed) > driveConstants.ControllerDeadzone) return speed;
        return 0;
    }

    /** Function that returns a given speed, as long as it is above the given deadzone. */
    public static double Deadzone(double speed, double minValue) {
        if (Math.abs(speed) > minValue) return speed;
        return 0;
    }

    ///// Controller Y value curving \\\\\
    public static double Cosine(double x, double y) {
        return Math.pow(Deadzone(Math.sqrt((x*x)+(y*y))), driveConstants.Linearity) * Math.cos(Math.atan2(y,x));
    }
    public static double Cosine(double x, double y, double exp) {
        return Math.pow(Math.sqrt((x*x)+(y*y)), exp) * Math.cos(Math.atan2(y,x));
    }

    ///// Controller X value curving \\\\\
    public static double Sine(double x, double y) {
        return Math.pow(Deadzone(Math.sqrt((x*x)+(y*y))), driveConstants.Linearity) * Math.sin(Math.atan2(y,x));
    }
    public static double Sine(double x, double y, double exp) {
        return Math.pow(Math.sqrt((x*x)+(y*y)), exp) * Math.sin(Math.atan2(y,x));
    }


    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
