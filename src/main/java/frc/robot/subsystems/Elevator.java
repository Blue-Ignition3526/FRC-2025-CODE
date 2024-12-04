// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.BlueShift.control.motor.LazyCANSparkMax;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  
  //motors
  public final LazyCANSparkMax m_rightElevatorMotor;
  public final LazyCANSparkMax m_leftElevatorMotor;

  //encoder
  private final RelativeEncoder m_encoder;

  //setpoint
  private double m_setpoint;

  ProfiledPIDController pid = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(1,10));



  public Elevator() {
    //motors
    //right
    this.m_rightElevatorMotor = new LazyCANSparkMax(0, MotorType.kBrushless);
    this.m_rightElevatorMotor.setIdleMode(IdleMode.kBrake);
    this.m_rightElevatorMotor.setSmartCurrentLimit(60);

    //left
    this.m_leftElevatorMotor = new LazyCANSparkMax(1, MotorType.kBrushless);
    this.m_leftElevatorMotor.setIdleMode(IdleMode.kBrake);
    this.m_leftElevatorMotor.follow(m_rightElevatorMotor, true);
    this.m_leftElevatorMotor.setSmartCurrentLimit(60);

    //encoder
    this.m_encoder = m_rightElevatorMotor.getEncoder();
    this.m_encoder.setPositionConversionFactor((7.588 * (2*Math.PI)) / 20); // radians to rotations and then rotations to inches / gear ratio
    this.m_encoder.setVelocityConversionFactor(((7.588 * (2*Math.PI)) / 20)/ 60 ); // position conversion factor / 60 
    
    // reset PID
    this.pid.reset(0);
  }

  public double getPosition(){ 
    return m_encoder.getPosition();
  }

  public double setPoint(double setpoint) {
    this.m_setpoint = setpoint;
    return setpoint;
}

  //  quiero checar si asi se hace una funcion que haga que la speed sea correcta
  // osea yo tengo para posicion actual y tengo para tener el setpoint

  //se que ahorita no tengo lo de pasar de rotaciones a movimiento lineal, en eso tengo duda

  public void setSpeed (){
    m_rightElevatorMotor.set(pid.calculate(getPosition(), setPoint(m_setpoint)));
  }

  public Command giveSetPoint(double setPoint){
    return run(() -> setPoint(setPoint)); 
   }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setSpeed();
  }
}
