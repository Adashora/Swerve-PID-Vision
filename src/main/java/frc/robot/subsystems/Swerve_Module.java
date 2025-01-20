// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;

/** Add your docs here. */
public class Swerve_Module {
private final SparkMax driveMotor; //creating motors
private final SparkMax turnMotor;

private final SparkMaxConfig drive_config; //creating configs
private final SparkMaxConfig turn_config;

private final int module_order;
private final int module_number;

private final PIDController turn_controller; //creating PID controllers
private final PIDController drive_Controller;

private final LinearFilter drive_VelocityFilter = LinearFilter.singlePoleIIR(0,0);
private final LinearFilter desired_drive_Filter = LinearFilter.singlePoleIIR(0,0);

private final RelativeEncoder Drive_encoder; //creating encoders
private final RelativeEncoder Turn_encoder;

private final CANcoder best_turn;

private final Rotation2d Swerve_offset; //offset


Swerve_Module(int module_number, int module_order, int drive_ID, int turn_ID, Rotation2d Swerve_offset, int CanCoderID){//add more

     this.module_number = module_number;
     this.module_order = module_order;
     this.Swerve_offset = Swerve_offset;

     this.best_turn = new CANcoder(CanCoderID);
     this.driveMotor = new SparkMax(drive_ID, MotorType.kBrushless);

     Drive_encoder = driveMotor.getEncoder();

    this.drive_config      //configguring the motors
    .inverted(false) 
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(0)
    .voltageCompensation(0);
    
    this.drive_config.encoder
    .positionConversionFactor(0)
    .velocityConversionFactor(0);

    this.Drive_encoder.setPosition(0);

    this.driveMotor.configure(drive_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    this.drive_Controller = new PIDController(Constants.Drive_KP, Constants.Drive_KI, Constants.Drive_KD);




   
    this.turnMotor = new SparkMax(turn_ID, MotorType.kBrushless);

    Turn_encoder = turnMotor.getEncoder();

    this.turn_config
    .inverted(false)
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(0)
    .voltageCompensation(0);

    this.turn_config.encoder
    .positionConversionFactor(0)
    .velocityConversionFactor(0);

    this.Turn_encoder.setPosition(0);

    this.turnMotor.configure(turn_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    
   


    this.turn_controller = new PIDController(Constants.Turn_KP, Constants.Turn_KI, Constants.Turn_KD); //PID turn controller

}

    public double get_turn_motor_volt(){
        return this.turnMotor.getBusVoltage();

}
public SwerveModuleState getState(){

    return SwerveModuleState(this.Drive_encoder.getVelocity(), this.getCANCoder());
}

public SwerveModulePosition getposition (){

    return new SwerveModulePosition(this.Drive_encoder.getPosition(), new Rotation2d(Turn_encoder.getPosition()));
}

public Rotation2d getCANCoder() {
    return Rotation2d.fromDegrees(this.best_turn.getAbsolutePosition());
    //gets CANCoder position in degrees
}

public void resetToAbsolute (){

    System.out.print(getCANCoder().getDegrees());

    this.Turn_encoder.setPosition(Swerve_offset.getDegrees());
    //sets turning encoders to abosolute position
}








}



