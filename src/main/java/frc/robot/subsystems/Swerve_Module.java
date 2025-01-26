// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.sensors.CANCoder;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

/** Add your docs here. */
public class Swerve_Module {
private final SparkMax driveMotor; //creating motors
private final SparkMax turnMotor;

private final SparkMaxConfig drive_config = new SparkMaxConfig(); //creating configs
private final SparkMaxConfig turn_config = new SparkMaxConfig();

private final int module_order = 0; // Initialize with a default value
final int module_number;

private final PIDController turn_controller; //creating PID controllers
private final PIDController drive_Controller;

private double drive_velocity;
private double desired_drive_velocity;


private final LinearFilter drive_VelocityFilter = LinearFilter.singlePoleIIR(0,0);
private final LinearFilter desired_drive_Filter = LinearFilter.singlePoleIIR(0,0);

private final RelativeEncoder Drive_encoder; //creating encoders
private final RelativeEncoder Turn_encoder;

public final CANCoder best_turn;

private final Rotation2d Swerve_offset; //offset


Swerve_Module(int module_number, int drive_ID, int turn_ID, Rotation2d Swerve_offset, int CanCoderID){//add more

     this.module_number = module_number;
     // this.module_order = module_order; // Remove this line if module_order is not needed
     this.Swerve_offset = Swerve_offset;

     this.best_turn = new CANCoder(CanCoderID);
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

    return new SwerveModuleState(this.Drive_encoder.getVelocity(), this.getCANCoder());
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



public void set_desired_state(SwerveModuleState desired_state, boolean isAuto){
    boolean invertDriveMotor = setAngle(desired_state);
        if (isAuto) {
            new WaitCommand(2);
        }


        setSpeed(desired_state, isAuto, invertDriveMotor);
}



private void setSpeed(SwerveModuleState desired_state, boolean isAuto, boolean invertDriveMotor){

                                                        //calculates the speed of the motors and sets it
        if (isAuto ==  false) {
            double drive_motor_output = desired_state.speedMetersPerSecond/Constants.maxSpeed;
            driveMotor.set(invertDriveMotor ? -drive_motor_output : drive_motor_output);
        }
        else {

            drive_velocity = drive_VelocityFilter.calculate(Drive_encoder.getVelocity());
            desired_drive_velocity = desired_drive_Filter.calculate(desired_state.speedMetersPerSecond);

            double drive_motor_voltage = drive_Controller.calculate(drive_velocity, desired_drive_velocity);

            double drive_motor_voltage_out = desired_state.speedMetersPerSecond/Constants.maxAutoSpeed;
            driveMotor.setVoltage(invertDriveMotor ? -drive_motor_voltage_out : drive_motor_voltage_out);
            SmartDashboard.putNumber("Drive Motor Volt out", drive_motor_voltage_out);
        }



}





private boolean setAngle(SwerveModuleState desired_state) {
    // Implement the logic for setting the angle here
    // This is a placeholder implementation
    boolean invertDriveMotor = false;
    SwerveModuleState current_state = this.getState(); //gets state fo swerve module (direction speed)

    double current_degrees = (current_state.angle.getDegrees() - this.Swerve_offset.getDegrees()); //gets current angle in degrees
        //We want the current degrees between [-180, 180]. When looking at a circle
        // -190 is equivalent to 170 so the below ternaries convert numbers less than -180
        // or greater than 180 into [-180,180]                                                          Matthews Comments

        current_degrees = current_degrees > 180 ? current_degrees - 360 : current_degrees;
        current_degrees = current_degrees < -180 ? current_degrees + 360 : current_degrees;


        double desired_degrees = desired_state.angle.getDegrees(); //gets desired angle in degrees

        double difference = (desired_degrees - current_degrees + 180) % 360 - 180; //gets difference between desired and current angle
        difference = difference < -180 ? difference + 360 : difference; //converts difference to [-180, 180]
        difference = difference > 180 ? difference - 360 : difference;



        if (Math.abs(difference) > 90) {
            invertDriveMotor = true;
            if (difference < 0) {
                difference += 180;
            } else {
                difference -= 180;
            }
        }


        double turning_motor_value = Math.abs(difference) < 1 ? 0 : turn_controller.calculate(difference); //calculates turning motor value

        turning_motor_value = turning_motor_value > 1 ? 1 : turning_motor_value; //sets turning motor value to 1 if it is greater than 1
        turning_motor_value = turning_motor_value < -1 ? -1 : turning_motor_value; //sets turning motor value to -1 if it is less than -1
    
        turnMotor.set(turning_motor_value); //sets turning motor value
    
        return invertDriveMotor;

}






}



