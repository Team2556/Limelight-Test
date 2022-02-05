package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class Drive extends TimedRobot{
    //right motors
    Limelight limeLight = new Limelight();
    //WPI_TalonSRX rFMotor = new WPI_TalonSRX(1);
    private CANSparkMax rFMotor = new CANSparkMax(8, MotorType.kBrushless);
    //WPI_TalonSRX rRMotor = new WPI_TalonSRX(4);
    private CANSparkMax rRMotor = new CANSparkMax(4, MotorType.kBrushless);
    //WPI_TalonSRX rDrop1 = new WPI_TalonSRX(2);
    //WPI_TalonSRX rDrop2 = new WPI_TalonSRX(3);
  
    //left motors
    //WPI_TalonSRX lFMotor = new WPI_TalonSRX(5);
    private CANSparkMax lFMotor = new CANSparkMax(12, MotorType.kBrushless);
    //WPI_TalonSRX lRMotor = new WPI_TalonSRX(8);
    private CANSparkMax lRMotor = new CANSparkMax(1, MotorType.kBrushless);
    //WPI_TalonSRX lDrop1 = new WPI_TalonSRX(6);
    //WPI_TalonSRX lDrop2 = new WPI_TalonSRX(7);
 

    //accessories
    // Compressor compressor = new Compressor();
    // DoubleSolenoid pistons = new DoubleSolenoid(0, 1);
    XboxController Xbox1 = new XboxController(0);
    double oldError = 0;
    double properError = 0;
    
    public void drivebaseInit(){
        rFMotor.setInverted(true);
        rRMotor.setInverted(true);



    }
    //drivebase init
    MecanumDrive driveMecanum = new MecanumDrive(lFMotor, lRMotor, rFMotor, rRMotor);
    
    //  DifferentialDrive driveTank = new DifferentialDrive();
    // DifferentialDrive driveDrop = new DifferentialDrive(lSide2, rSide2);

    public void doubleDrive(){
        SmartDashboard.putString("a", "test");

    }

    public double PID(double error, double constant){
    double P = rFMotor.getPIDController().getP();
    double I = rFMotor.getPIDController().getI();
    double D = rFMotor.getPIDController().getD();

    double PID = P + I + D;
    double PIDE = PID * error * constant;
    return PIDE;
    }

    public void PIDDashboard()
    {
        SmartDashboard.putNumber("P", rFMotor.getPIDController().getP());
        SmartDashboard.putNumber("I", rFMotor.getPIDController().getI());
        SmartDashboard.putNumber("D", rFMotor.getPIDController().getD());
    }

    public void limelightDrive(double xP, double x){
        SmartDashboard.putNumber("Motor speed", rFMotor.getAppliedOutput());
        SmartDashboard.putNumber("xP", xP);
        driveMecanum.driveCartesian(0, 0, Xbox1.getLeftX());
        if(Xbox1.getAButton()){
            if(java.lang.Math.abs(x) >= 4)
            {
                driveMecanum.driveCartesian(0, 0, xP);
            }
            else
            {
                driveMecanum.driveCartesian(0, 0, 0);
            }
     }
    }
      
    
    public double flickerError(double error){


        if (error != 0)
        {
            oldError = error;
        }

        if (error == 0)
        {
            properError = oldError;
        }


        return properError;
    }

}