// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.config.odometry;

/** 
 * Class used to represent odometry standard deviations
 */
public record OdometryStdDevs(double xStdDev, double yStdDev, double rotStdDev) {}
