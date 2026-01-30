// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
// import com.ctre.phoenix6.motorcontrol.can.WPI_VictorSPX;

import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.XboxController;
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
import frc.robot.subsystems.DriveModes;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LinearServo;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Pivort;
import frc.robot.subsystems.Vision.Vision;

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

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final static PhotonCamera camera = new PhotonCamera("PC_Camera");

    public static final Vision vision = new Vision();
    public static final Vision turretVision = new Vision();


    // // Ballz Launcher
    public static Pivort pivort = new Pivort();
    public static PivortCommand pivortCommand = new PivortCommand();
    // public static SparkMax rotateMotor = new SparkMax(22, MotorType.kBrushless);
    public static TalonFXS rotateMotor = new TalonFXS(14, Rio);
    public static TalonFX shootMotor = new TalonFX(15, Driveloop);
    
    public static Intake intake = new Intake();
    public static IntakeCommand intakeCommand = new IntakeCommand();

    public static Shooter Shooter = new Shooter();
    public static ShootCommand shootCommand = new ShootCommand();

    public static LinearServo linearServo = new LinearServo(9, 100, 20);
    public static LinearServo linearServo2 = new LinearServo(8, 100, 20);
    public static LinearServoCommand linearServoCommand = new LinearServoCommand();
    /* Path follower */
    private final SendableChooser<Command> autoChooser;


    public RobotContainer() {
        pivort.setDefaultCommand(pivortCommand);
        intake.setDefaultCommand(intakeCommand);
        Shooter.setDefaultCommand(shootCommand);
        linearServo.setDefaultCommand(linearServoCommand);
        linearServo2.setDefaultCommand(linearServoCommand);
        
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
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
                    .withRotationalRate(-joystick.getRightX() * driveConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> DriveModes.brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->      //Can be enabled to make more precise movements
        //     forwardStraight.withVelocityX(0.5).withVelocityY(0))
        // );
        // joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        // );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
