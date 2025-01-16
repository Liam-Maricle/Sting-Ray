package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import teamCode.GoBildaPinpointDriver;

public class PinPointOdometrySubsystem extends SubsystemBase
{
    GoBildaPinpointDriver odo;
    private double oldTime = 0;

    private Pose2D testPose = new Pose2D(DistanceUnit.MM, 100, 100, AngleUnit.DEGREES, 0.0);

    private final GoBildaPinpointDriver m_odo;

    public PinPointOdometrySubsystem(GoBildaPinpointDriver odo)
    {
        this.m_odo = odo;
        odo.setOffsets(68,-178);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }
    public void resetOdo()
    {
        this.m_odo.resetPosAndIMU();
    }

    public double[] getDeltaHeading()
    {
        double deltaX = testPose.getX(DistanceUnit.MM) - this.m_odo.getPosX();
        double deltaY = testPose.getY(DistanceUnit.MM) - this.m_odo.getPosY();
        double deltaHeading = testPose.getHeading(AngleUnit.DEGREES) - this.m_odo.getHeading();

//        double returnX;
//        double returnY;
//        double returnHeading;
//
//        if (returnX > 25)

        return new double[]{deltaX, deltaY, deltaHeading};
    }
}