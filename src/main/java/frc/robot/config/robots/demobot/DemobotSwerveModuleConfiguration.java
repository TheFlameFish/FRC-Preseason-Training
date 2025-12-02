// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config.robots.demobot;

import java.util.List;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.lib.can.CANDeviceID;
import frc.lib.config.swerve.SwerveModuleConfiguration;
import frc.lib.config.swerve.TalonFXSwerveModuleConfiguration;
import frc.robot.config.robots.demobot.swerve_tunings.DemobotSwerveConstantsComp;

/** 
 * Example swerve module configuration class
 */
public class DemobotSwerveModuleConfiguration {
    // ---------------------- Front Left -----------------------
    public final SwerveModuleConfiguration<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> kFrontLeftModule;

    // ---------------------- Front Right ----------------------
    public final SwerveModuleConfiguration<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> kFrontRightModule;

    // ---------------------- Back Left ------------------------
    public final SwerveModuleConfiguration<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> kBackLeftModule;

    // ---------------------- Back Right -----------------------
    public final SwerveModuleConfiguration<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> kBackRightModule;

    public final List<SwerveModuleConfiguration<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>> kConfigurations;
    public DemobotSwerveModuleConfiguration(String driveSubsystemName, String driveCANBusName){

        // --- Front Left Module ---
        this.kFrontLeftModule = new TalonFXSwerveModuleConfiguration(
            DemobotSwerveConstantsComp.FrontLeft,
            "FrontLeftSwerveModule", 

            // Drive Motor
            new CANDeviceID(
                5, 
                "FrontRightSwerveDriveMotor", 
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX, 
                driveCANBusName
            ), 

            // Steer Motor
            new CANDeviceID(
            8, 
                "FrontLeftSwerveSteerMotor", 
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX, 
                driveCANBusName
            ), 

            // Steer Encoder
            new CANDeviceID(
            26, 
                "FrontLeftSwerveSteerEncoder", 
                driveSubsystemName,
                CANDeviceID.CANDeviceType.CANCODER, 
                driveCANBusName
            )
        );

        // --- Front Right Module ---
        this.kFrontRightModule = new TalonFXSwerveModuleConfiguration(
            DemobotSwerveConstantsComp.FrontRight,
            "FrontRightSwerveModule", 

            // Drive Motor
            new CANDeviceID(
                5, 
                "FrontRightSwerveDriveMotor", 
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX, 
                driveCANBusName
            ),

            // Steer Motor
            new CANDeviceID(
            6, 
                "FrontRightSwerveSteerMotor", 
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX, 
                driveCANBusName
            ),

            // Steer Encoder
            new CANDeviceID(
            24, 
                "FrontRightSwerveSteerEncoder", 
                driveSubsystemName,
                CANDeviceID.CANDeviceType.CANCODER, 
                driveCANBusName
            )
        );

        // --- Back Left Module ---
        this.kBackLeftModule = new TalonFXSwerveModuleConfiguration(
            DemobotSwerveConstantsComp.BackLeft,
            "BackLeftSwerveModule", 

            // Drive Motor
            new CANDeviceID(
                3, 
                "BackLeftSwerveDriveMotor", 
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX, 
                driveCANBusName
            ),
            
            // Steer Motor
            new CANDeviceID(
            4, 
                "BackLeftSwerveSteerMotor", 
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX, 
                driveCANBusName
            ),

            // Steer Encoder
            new CANDeviceID(
            25, 
                "BackLeftSwerveSteerEncoder", 
                driveSubsystemName,
                CANDeviceID.CANDeviceType.CANCODER, 
                driveCANBusName
            )
        );

        // --- Back Right Module ---
        this.kBackRightModule = new TalonFXSwerveModuleConfiguration(
            DemobotSwerveConstantsComp.BackRight,
            "BackRightSwerveModule", 

            // Drive Motor
            new CANDeviceID(
                9, 
                "BackRightSwerveDriveMotor", 
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX, 
                driveCANBusName
            ),

            // Steer Motor
            new CANDeviceID(
                2, 
                "BackRightSwerveSteerMotor",  
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX, 
                driveCANBusName
            ),

            // Steer Encoder
            new CANDeviceID(
                23, 
                "BackRightSwerveSteerEncoder",  
                driveSubsystemName,
                CANDeviceID.CANDeviceType.CANCODER, 
                driveCANBusName
            )
        );

        // List of configurations in FL, FR, BL, BR order
        this.kConfigurations = List.of(
            kFrontLeftModule,
            kFrontRightModule,
            kBackLeftModule,
            kBackRightModule
        );
    }
}
