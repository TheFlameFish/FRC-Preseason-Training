// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.lib.config.odometry.OdometryStdDevs;
import frc.lib.config.subsystems.drive.DrivetrainConfiguration;
import frc.lib.subsystems.drive.DrivetrainIO;
import frc.lib.subsystems.drive.DrivetrainInputs;
import frc.lib.subsystems.simulation.drive.MapleSimSwerveDrivetrain;
import frc.lib.subsystems.simulation.visualizations.DrivetrainVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {

  // Reference to the drivetrain and the inputs to the drivetrain
  protected DrivetrainIO io;
  protected DrivetrainInputs inputs = new DrivetrainInputs();

  // Configuration of this given drivetrain
  protected final DrivetrainConfiguration configuration;

  // Drive train visualizer
  protected final DrivetrainVisualization visualization;

  // What to prepend to logs from this subsystem
  private final String logPrefix;
  
  /** Creates a new DriveSubsystem. */
  public DrivetrainSubsystem(
    DrivetrainConfiguration configuration,
    DrivetrainIO drivetrain
  ) {
    this.logPrefix = "Subsystems/" + configuration.kConfigurationName;
    this.configuration = configuration;
    this.io = drivetrain;
    this.visualization = new DrivetrainVisualization(configuration.kMaxDriveSpeed, logPrefix);

    this.io.setLoggingPrefix(logPrefix);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    visualization.updateViz(inputs);
    io.logModules(inputs, logPrefix);
    Logger.processInputs(logPrefix, inputs);

    setStateStdDevs(
      DriverStation.isEnabled() 
        ? configuration.kEnabledOdometryStdDevs
        : configuration.kDisabledOdometryStdDevs
    );

    Logger.recordOutput(
      logPrefix + "/CurrentCommand", 
      (getCurrentCommand() != null) ? getCurrentCommand().getName() : "N/A");
  }

  public DrivetrainSubsystem withStartingPose(Pose2d pose){
    resetOdometry(pose);
    return this;
  }

  // ---- Odometry updates ----
  public void resetOdometry(Pose2d pose) {
    io.resetOdometry(pose);
  }

  // --- Drive train commanding ----

  /**
   * Start a continuous command to apply new swerve drive requests each loop
   * @param request Supplier of SwerveRequests that will be used to command the drivetrain
   * @return The command applying the request
   */
  public Command applyRequest(Supplier<SwerveRequest> request){
    return io.continuousRequestCommand(request, this)
      .withName("SwerveDriveRequest");
  }

  /**
   * Set a control request instantaneously
   * @param request The swerve drive request to pass to the drivetrain
   */
  public void setControl(SwerveRequest request) {
    io.setControl(request);
  }

  // ---- Input Deadbanding ----
  protected ChassisSpeeds applyDeadbands(ChassisSpeeds input){
    // Check translational speed
    if (Math.hypot(input.vxMetersPerSecond, input.vyMetersPerSecond) < configuration.kChassisTranslationSpeedThreshold) {
      input.vxMetersPerSecond = input.vyMetersPerSecond = 0.0;
    }

    // Check rotational speed
    if (Math.abs(input.omegaRadiansPerSecond) < configuration.kChassisRotationalSpeedThreshold) {
        input.omegaRadiansPerSecond = 0.0;
    }
    return input;
  }

  // ---- Odometry standard deviation adjustment ----
  protected void setStateStdDevs(OdometryStdDevs stdDevs){
    io.setOdometryStdDevs(stdDevs.xStdDev(), stdDevs.yStdDev(), stdDevs.rotStdDev());
  }

  // Attempt to get the sim drive train
  public MapleSimSwerveDrivetrain getSimDrivetrain(){
    if (io instanceof DrivetrainIOSim){
      return ((DrivetrainIOSim) io).getMapleSimDrive();
    }

    return null;
  }
}
