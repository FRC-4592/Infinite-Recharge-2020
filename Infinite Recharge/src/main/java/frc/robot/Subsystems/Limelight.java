package frc.robot.Subsystems;

import edu.wpi.first.networktables.*;
import frc.robot.Lib.SubsystemFramework;



public class Limelight implements SubsystemFramework{

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);

    private double camtran[];

    public Limelight(){
    }

    @Override
    public void update(){

        Update_Limelight_Tracking();

    }

    @Override
    public void outputToSmartDashboard(){

    }

    @Override 
    public void setupSensors(){
    
    }


    public void Update_Limelight_Tracking() 
    {
        this.tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        this.ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        this.camtran = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDoubleArray(new double[]{});
	}
    public double getLimelightCamtran(int x) {
        return this.camtran[x];
    }
    public int getTy() {
        return (int)this.ty;
    }
}