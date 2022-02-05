package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Limelight extends TimedRobot {
    //Drive drive = new Drive(); 


    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");
    
    //read values periodically


    

    // tv	Whether the limelight has any valid targets (0 or 1)
    // tx	Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
    // ty	Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
    // ta	Target Area (0% of image to 100% of image)

    double x;
    double y;
    double area;
    double v;
    
    public double LimelightxP(){
    
    x = tx.getDouble(0.0);
    
    double absx = x/java.lang.Math.abs(tx.getDouble(0.0));
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    v = tv.getDouble(0.0);

    double xP = absx / 10;
    // double yP = y / 2 * y;
    // post to smart dashboard periodically

      return xP;
    }


    public void LimelightDashboard(){
      x = tx.getDouble(0.0);
      y = ty.getDouble(0.0);
      area = ta.getDouble(0.0);
      v = tv.getDouble(0.0);
      SmartDashboard.putNumber("LimelightX", x);
      SmartDashboard.putNumber("LimelightY", y);
      SmartDashboard.putNumber("LimelightArea", area);
      SmartDashboard.putNumber("LimelightTarget", v);

  
    }
}