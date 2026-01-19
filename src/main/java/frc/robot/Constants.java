// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.generated.TunerConstants;


/** Add your docs here. */
public class Constants {

    public static final class driveConstants {

        public final static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public final static double MaxAngularRate = RotationsPerSecond.of(4.5).in(RadiansPerSecond); // 4.5 rotations per second max angular velocity

        /** Deadzone value to control when to start using joystick input values */
        public final static double RotationalDeadband = MaxAngularRate * 0.1; // Add a 10% deadband

        // public final static double MaxSpeed = 5; //Max targeted speed in M/S (15 NORMALLY)
        // public final static double MaxAngularRate = 1.5 * Math.PI * 3; //Max targeted rotations / second -- 3/4ths of a rotation for now
        public final static double MaxAcceleration = 2; //Max acceleration in M/s/s;
        public final static double MaxAngularAcceleration = 4 * Math.PI;
        public final static double SpeedDeadbandPercentage = 0; //Deadband or Deadzone of requested speed, as a percentage of the maximum speed;

        public final static double Linearity = 3; // How steep the response curve is (typically cubic, meaning 50% on stick = 12.5% speed)

        public final static double ControllerDeadzone = 0.2;

        public final static PIDController chassisPID = new PIDController(01, 0, 0);
        public final static double ChassisPidTolerence = Math.PI/180;

        public final static double ChassisModulePosX = 13.375; //Middle of robot to module left/right position
        public final static double ChassisModulePosY = 10.375; //Middle of robot to module front/back position

    }

    public static final class visionConstants {

        public final static double AngleDistrust = 10; // How much the angle should impact vision trust
        public final static Transform3d cameraOffset = new Transform3d(0, 0, 0, new Rotation3d(0,0,0));
    
    }

    public static final class pivotConstants {
        public final static double ShooterDeadzone = .5;
        public final static double PivotPIDkp = .008; //TODO Tune these
        public final static double PivotPIDki = 0;
        public final static double PivotPIDkd = 0;
        public final static double PivotMotorRatio = .2;
        public final static double PivotTableRatio = 30/(88.66*3);
        public final static double ShootSpeed = 1.15;
        public final static double DistanceToShootSpeedMultiplier = .2;
        public final static double AngleOfShooter = 65; //Degrees
        public final static double HeightOfShooter = 1.33;//Feet
        public final static double MetersToFeet = 3.28;
        public final static double VelocityToMotorPower = 0;
        public final static double DiameterOfWheel = .1148; //Meters
        public final static double MaxRPMPivot = 7700; //Motor type dependent
        public final static double RPMToRadPS = ((2*Math.PI) / 60);
        public final static double MaxPivotAngularRate = MaxRPMPivot * PivotMotorRatio * PivotTableRatio * RPMToRadPS;
        public final static double MotorTransferEfficency = .55;
    }

    public static final class fieldConstants {
        public final static double HeightOfHub = 2.5; //Feet
    }
}
