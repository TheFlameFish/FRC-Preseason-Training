// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.can.CANDeviceID;
import frc.lib.config.odometry.OdometryStdDevs;
import frc.lib.config.robot.PhysicalConfiguration;
import frc.lib.config.robot.Pigeon2GyroConfiguration;
import frc.lib.config.subsystems.drive.DrivetrainConfiguration;
import frc.lib.config.subsystems.drive.simulation.DrivetrainSimConfiguration;
import frc.robot.config.robots.demobot.DemobotSwerveModuleConfiguration;
import frc.robot.subsystems.drive.DrivetrainIOSim;
import frc.robot.subsystems.drive.DrivetrainSubsystem;

/** Creates instances of each subsystem specific to the current state of the robot */
public class SubsystemFactory {

    //------------------------------------
    // TODO: (Do this one last) Adjust some of the values below to see how it affects the simulation :)
    //------------------------------------

    public static DrivetrainSubsystem createDrivetrainSubsystem(){
        // ---- Drivetrain configuration ---
        // Typically this would happen in a designated robot config file set but for this we will just create them here
        String kDriveSubsystemName = "Drive";
        String kCanivoreBusName = "CANivore";

        // ---- Robot Physical Attributes ----
        PhysicalConfiguration kPhysicalConfiguration = 
            new PhysicalConfiguration()
                .withRobotWeightPounds(150)
                .withWheelBaseLengthM(Units.inchesToMeters(22.75))
                .withWheelTrackWidthM(Units.inchesToMeters(22.75))
                .withBumperLengthM(Units.inchesToMeters(35.625))
                .withBumperWidthM(Units.inchesToMeters(35.625))
                .withWheelCoefficientOfFriction(1.2);

        // ---- Gyro Attributes ----
        Pigeon2GyroConfiguration kGyroConfiguration = 
        new Pigeon2GyroConfiguration()
            .withGyroYawError(0.0)
            .withGyroMountRotation(
                new Rotation3d(
                    Math.toRadians(0),      // Roll
                    Math.toRadians(0),      // Pitch
                    Math.toRadians(0)       // Yaw
                )
            )
            .withCANDevice(
                new CANDeviceID(
                    1, 
                    "DriveTrainGyro",
                    kDriveSubsystemName,
                    CANDeviceID.CANDeviceType.PIGEON2,
                    kCanivoreBusName
                )
            );

        // ---- Drivetrain Attributes ----

        double kChassisTranslationSpeedThreshold = 0.05; // Anything less than this chassis speed in meters per second will be set to 0
        double kChassisRotationalSpeedThreshold = 0.05; // Anything less than this chassis speed in radians per second will be set to 0
        double kMaxDriveSpeed = 3.6; // Max speed of the robot in meters per second
        double kMaxAngularRate = 8.2; // Radians per second

        // Example configurations for some robot
        DemobotSwerveModuleConfiguration kSwerveModuleConfigurations = new DemobotSwerveModuleConfiguration(kDriveSubsystemName, kCanivoreBusName);

        // Odometry standard deviations for enabled and disabled mode
        OdometryStdDevs kDisabledModeStdDevs = new OdometryStdDevs(1, 1, 1);
        OdometryStdDevs kEnabledModeStdDevs = new OdometryStdDevs(0.3, 0.3, 0.2);

        DrivetrainConfiguration kDrivetrainConfiguration = 
            new DrivetrainConfiguration()
                    .withName(kDriveSubsystemName)
                    .withMaxDriveSpeed(kMaxDriveSpeed)
                    .withMaxAngularRate(kMaxAngularRate)
                    .withChassisSpeedDeadband(
                        kChassisTranslationSpeedThreshold, 
                        kChassisRotationalSpeedThreshold
                    )
                    .withGyroDevice(kGyroConfiguration.kCANDevice)
                    .withDrivetrainConstants(
                        new SwerveDrivetrainConstants()
                            .withCANBusName(kCanivoreBusName)
                            .withPigeon2Id(kGyroConfiguration.kCANDevice.getDeviceID())
                            .withPigeon2Configs(kGyroConfiguration.kConfiguration)
                    )
                    // .withDrivetrainConstants(NautilusSwerveConstantsComp.DrivetrainConstants) // Phoenix Tuner Supplied Constants
                    .withModuleConstants(
                        new SwerveModuleConstants<?, ?, ?>[]{
                            kSwerveModuleConfigurations.kFrontLeftModule.getModuleConstants(),
                            kSwerveModuleConfigurations.kFrontRightModule.getModuleConstants(),
                            kSwerveModuleConfigurations.kBackLeftModule.getModuleConstants(),
                            kSwerveModuleConfigurations.kBackRightModule.getModuleConstants()
                        }
                    )
                    // .withModuleConstants(NautilusSwerveConstantsComp.kSwerveModuleConstants) // Phoenix Tuner Supplied Constants
                    .withOdometryStdDevs(
                        kEnabledModeStdDevs,
                        kDisabledModeStdDevs
                    )
                    .withJoystickDeadband(
                        0.05, 
                        0.05
                    );
        
        // ---- Drivetrain Simulation Attributes ----
        // Simulation specific drivetrain configuration elements
        DrivetrainSimConfiguration kSimulatedDrivetrainConfiguration = 
            new DrivetrainSimConfiguration(0.005) // 5 ms thread loop time
                    .withName(kDriveSubsystemName)
                    .withPhysicalConfiguration(kPhysicalConfiguration)
                    .withModuleDriveMotorCount(1)
                    .withModuleSteerMotorCount(1);

        return new DrivetrainSubsystem(
                        kDrivetrainConfiguration,
                        new DrivetrainIOSim(
                            kSimulatedDrivetrainConfiguration,
                            kDrivetrainConfiguration,
                            kSwerveModuleConfigurations.kConfigurations
                        )
                )
                .withStartingPose(new Pose2d(2.5, 4, Rotation2d.fromDegrees(0)));



        // --------------------- STANDARD APPROACH ---------------------
        // ---- Standard Subsystem Factory Setup, Spawning simulated systems in simulation mode and real on actual bot and in replay
        // switch (RobotRuntimeConstants.kCurrentRuntimeMode) {
        //     // ---- Simulation instance of drivetrain ----
        //     case SIM:
        //         return new DrivetrainSubsystem(
        //                 RobotRuntimeConstants.kRobotConfiguration.getDrivetrainConfiguration(),
        //                 new DrivetrainIOSim(
        //                     RobotRuntimeConstants.kRobotConfiguration.getSimulatedDrivetrainConfiguration(),
        //                     RobotRuntimeConstants.kRobotConfiguration.getDrivetrainConfiguration(),
        //                     RobotRuntimeConstants.kRobotConfiguration.getSwerveConfigurations()
        //                 )
        //         )
        //         .withStartingPose(new Pose2d(2.5, 4, Rotation2d.fromDegrees(0)));

        //     // ---- Physical instance of drivetrain ----
        //     case REPLAY: // fall down to default
        //     case REAL:
        //     default:
        //         return new DrivetrainSubsystem(
        //             RobotRuntimeConstants.kRobotConfiguration.getDrivetrainConfiguration(),
        //             new DrivetrainIOHardware(
        //                     RobotRuntimeConstants.kRobotConfiguration.getDrivetrainConfiguration(),
        //                     RobotRuntimeConstants.kRobotConfiguration.getSwerveConfigurations()
        //                 )
        //         );
        // }

    }
}
