// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.servohub.config.ServoHubParameter;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;

/** Add your docs here. */
public class ElevatorIOSparkMAX implements ElevatorIO {
    private final SparkMax leadMotor;
    private final SparkMax followerMotor;
    private final RelativeEncoder encoder;

    //Constructor
    public ElevatorIOSparkMAX() {
        //Initialize the SparkMAX motors for main and follower
        leadMotor = new SparkMax(11, MotorType.kBrushless);
        followerMotor = new SparkMax(12, MotorType.kBrushless);

        //Invert follower
        SparkMaxConfig config = new SparkMaxConfig();
        config.follow(11, true);
        
        followerMotor.configure(config, null, null);

        //Initialize the encoder for main
        encoder = leadMotor.getEncoder();

        

        

        
    }



}
