// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.config.motors.factories;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.lib.can.CANDeviceID;
import frc.lib.config.motors.ServoMotorConfiguration;
import frc.lib.config.motors.ServoMotorFollowerConfiguration;

/** 
 * Factory for creating Talon FX configs
 */
public class TalonFXConfigurationFactory {
    public static ServoMotorFollowerConfiguration.FollowerConfiguration<TalonFXConfiguration> generateFollowerTalonFXConfiguration(){
        return new ServoMotorFollowerConfiguration.FollowerConfiguration<>(
                    new ServoMotorConfiguration<TalonFXConfiguration>()
                    .withConfig(new TalonFXConfiguration())
                );
    }

    public static ServoMotorFollowerConfiguration.FollowerConfiguration<TalonFXConfiguration> generateFollowerTalonFXConfiguration(String name, CANDeviceID device){
        return new ServoMotorFollowerConfiguration.FollowerConfiguration<>(
                    new ServoMotorConfiguration<TalonFXConfiguration>()
                        .withConfig(new TalonFXConfiguration())
                        .withName(name)
                        .withCANDevice(device)
                );
    }
}
