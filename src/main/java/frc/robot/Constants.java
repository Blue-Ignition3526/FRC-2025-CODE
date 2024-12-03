// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import lib.BlueShift.constants.CTRECANDevice;
import lib.BlueShift.constants.PIDFConstants;
import lib.BlueShift.constants.SwerveModuleOptions;
import lib.BlueShift.utils.SwerveChassis;



/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
   public static final class SwerveDrive {
        // * Gyro
        public static final CTRECANDevice kGyroDevice = new CTRECANDevice(34, "*");

        // * Controller
        public static final double kJoystickDeadband = 0.1;

        // * Heading Controller
        public static final Measure<Angle> kHeadingTolerance = Degrees.of(3);
        public static final TrapezoidProfile.Constraints kHeadingConstraints = new TrapezoidProfile.Constraints(100, 10);
        public static final ProfiledPIDController kHeadingController = new ProfiledPIDController(0.1, 0, 0, kHeadingConstraints);

        // * Translation Controller
        public static final Measure<Distance> kTranslationTolerance = Inches.of(1);
        public static final TrapezoidProfile.Constraints kTranslationConstraints = new TrapezoidProfile.Constraints(100, 10);
        public static final ProfiledPIDController kTranslationController = new ProfiledPIDController(0.1, 0, 0, kTranslationConstraints);

        // * Physical model of the robot
        public static final class PhysicalModel {
            // * MAX DISPLACEMENT SPEED (and acceleration)
            public static Measure<Velocity<Distance>> kMaxSpeed = MetersPerSecond.of(4);
            public static final Measure<Velocity<Velocity<Distance>>> kMaxAcceleration = MetersPerSecondPerSecond.of(3);

            // * MAX ROTATIONAL SPEED (and acceleration)
            public static final Measure<Velocity<Angle>> kMaxAngularSpeed = RotationsPerSecond.of(1);
            public static final Measure<Velocity<Velocity<Angle>>> kMaxAngularAcceleration = RotationsPerSecond.per(Second).of(Math.pow(kMaxAngularSpeed.in(RotationsPerSecond), 2));

            // * Drive wheel diameter
            public static final Measure<Distance> kWheelDiameter = Inches.of(4);

            // * Gear ratios
            public static final double kDriveMotorGearRatio = 1.0 / 6.75;
            public static final double kTurningMotorGearRatio = 1.0 / 21.428571428571427;

            // * Conversion factors (Drive Motor) DO NOT CHANGE
            public static final double kDriveEncoder_RotationToMeter = kDriveMotorGearRatio * (kWheelDiameter.in(Meters) / 2) * 2 * Math.PI;
            public static final double kDriveEncoder_RPMToMeterPerSecond = kDriveEncoder_RotationToMeter / 60.0;

            // * Conversion factors (Turning Motor) DO NOT CHANGE
            public static final double kTurningEncoder_RotationToRadian = kTurningMotorGearRatio * 2.0 * Math.PI;
            public static final double kTurningEncoder_RPMToRadianPerSecond = kTurningEncoder_RotationToRadian / 60.0;

            // * Robot Without bumpers measures
            public static final Measure<Distance> kTrackWidth = Inches.of(26);
            public static final Measure<Distance> kWheelBase = Inches.of(30.5);
    
            // * Create a kinematics instance with the positions of the swerve modules
            public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(SwerveChassis.sizeToModulePositions(kTrackWidth.in(Meters), kWheelBase.in(Meters)));
        }

        // * Swerve modules configuration
        public static final class SwerveModules {
            // * PID
            public static final PIDFConstants kTurningPIDConstants = new PIDFConstants(0.5);

            // * Swerve modules options
            public static final SwerveModuleOptions kFrontLeftOptions = new SwerveModuleOptions()
                .setAbsoluteEncoderInverted(false)
                .setAbsoluteEncoderCANDevice(new CTRECANDevice(11, "*"))
                .setDriveMotorID(22)
                .setTurningMotorID(21)
                .setTurningMotorInverted(true)
                .setName("Front Left");

            public static final SwerveModuleOptions kFrontRightOptions = new SwerveModuleOptions()
                .setAbsoluteEncoderInverted(false)
                .setAbsoluteEncoderCANDevice(new CTRECANDevice(12, "*"))
                .setDriveMotorID(24)
                .setTurningMotorID(23)
                .setTurningMotorInverted(true)
                .setName("Front Right");

            public static final SwerveModuleOptions kBackLeftOptions = new SwerveModuleOptions()
                .setAbsoluteEncoderInverted(false)
                .setAbsoluteEncoderCANDevice(new CTRECANDevice(13, "*"))
                .setDriveMotorID(26)
                .setTurningMotorID(25)
                .setTurningMotorInverted(true)
                .setName("Back Left");


            public static final SwerveModuleOptions kBackRightOptions = new SwerveModuleOptions()
                .setAbsoluteEncoderInverted(false)
                .setAbsoluteEncoderCANDevice(new CTRECANDevice(14, "*"))
                .setDriveMotorID(28)
                .setTurningMotorID(27)
                .setTurningMotorInverted(true)
                .setName("Back Right");
        }
    }
}
