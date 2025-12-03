// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.List;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import frc.lib.SimulatedRobotState;
import frc.lib.config.subsystems.drive.DrivetrainConfiguration;
import frc.lib.config.subsystems.drive.simulation.DrivetrainSimConfiguration;
import frc.lib.config.swerve.SwerveModuleConfiguration;
import frc.lib.subsystems.drive.DrivetrainInputs;
import frc.lib.subsystems.simulation.drive.MapleSimSwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

/**
 * The {@code DrivetrainIOSim} class extends {@link DrivetrainIOHardware} to provide simulation-specific
 * functionality for the swerve drive system. It integrates with WPILib's simulation framework for
 * testing and development.
 */
public class DrivetrainIOSim extends DrivetrainIOHardware {
    // What to prepend to logs from this subsystem
    private String logPrefix;

    // Simulation helpers
    private Notifier simulationThread = null;        

    // Simulation configuration properties of the drive train, things like sim update rate
    private final DrivetrainSimConfiguration simConfig; // 5 ms
    public MapleSimSwerveDrivetrain drivetrainSim = null;
    private SwerveModuleConstants<?, ?, ?>[] moduleConstants;
    private List<SwerveModuleConfiguration<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>> moduleConfigurations;

    // Update the swerve drive state for the simulation
    private Consumer<SwerveDriveState> simSwerveStateConsumer =
        state -> {
            if(drivetrainSim != null){
                state.Pose = drivetrainSim.mapleSimSwerveDrivetrain.getSimulatedDriveTrainPose();
            }
            SimulatedRobotState.get().addOdometryMeasurement(state.Pose);
            swerveTelemetryConsumer.accept(state);
        };


    public DrivetrainIOSim(
        DrivetrainSimConfiguration simConfig,
        DrivetrainConfiguration driveTrainConfiguration,
        List<SwerveModuleConfiguration<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>> swerveModuleConfiguration
    ){
        // regulation occurs in-place
        super(MapleSimSwerveDrivetrain.regulateModuleConstantForSimulation(driveTrainConfiguration), swerveModuleConfiguration);

        this.simConfig = simConfig;
        this.moduleConstants = driveTrainConfiguration.kModuleConstants;
        this.moduleConfigurations = swerveModuleConfiguration;

        registerTelemetry(simSwerveStateConsumer);
        startSimThread();
    }

    /**
     * Start the simulation thread for the maple sim drive train
     */
    public void startSimThread() {
        drivetrainSim =
                new MapleSimSwerveDrivetrain(
                        // Simulation Update Rate 5ms = 200hz
                        Units.Seconds.of(simConfig.kSimLoopPeriodMS),
                        Units.Pounds.of(simConfig.kPhysicalConfiguration.kRobotWeightPounds),
                        Units.Meters.of(simConfig.kPhysicalConfiguration.kBumperWidthMeters),    
                        Units.Meters.of(simConfig.kPhysicalConfiguration.kBumperLengthMeters),
                        // Number of drive motors on 1 swerve module        
                        DCMotor.getKrakenX60(simConfig.kModuleDriveMotorCount),
                        // Number of steer motors on 1 swerve module   
                        DCMotor.getKrakenX60(simConfig.kModuleSteerMotorCount),
                        // Wheel coef. of friction (its ability to resist movement)   
                        simConfig.kPhysicalConfiguration.kWheelCoefficientOfFriction,       
                        getModuleLocations(),      
                        getPigeon2(),
                        getModules(),
                        moduleConstants,
                        moduleConfigurations);                  


        simulationThread = new Notifier(drivetrainSim::update);
        simulationThread.setName("DrivetrainSimNotifier");
        simulationThread.startPeriodic(simConfig.kSimLoopPeriodMS / 1000);
    }

    /**
     * Handles resetting the odometry position, if we have a valid drivetrain sim its world pose will be reset
     * @param pose The pose of which we want 0,0 to now be
     */
    public void resetOdometry(Pose2d pose){
        if(drivetrainSim != null){
            drivetrainSim.mapleSimSwerveDrivetrain.setSimulationWorldPose(pose);
            Timer.delay(0.05); // Wait one loop
        }
        super.resetOdometry(pose);
    }

    @Override
    public void updateInputs(DrivetrainInputs inputs) {
        super.updateInputs(inputs);

        /* After updating inputs we want to find the latest pose and log it if its not null */
        Pose2d pose = SimulatedRobotState.get().getLatestFieldRobotPose();
        if(pose != null){
            Logger.recordOutput(this.logPrefix + "/Viz/SimPose", pose);
        }
    }

    /**
     * Get a reference to the underlying maple sim drive train
     * @return The MapleSimSwerveDrivetrain running the simulation
     */
    public MapleSimSwerveDrivetrain getMapleSimDrive() {
        return drivetrainSim;
    }

    /**
     * Set the logging prefix for this IO class
     */
    @Override
    public void setLoggingPrefix(String prefix) {
        this.logPrefix = prefix + "/IO/Sim";
    }

}
