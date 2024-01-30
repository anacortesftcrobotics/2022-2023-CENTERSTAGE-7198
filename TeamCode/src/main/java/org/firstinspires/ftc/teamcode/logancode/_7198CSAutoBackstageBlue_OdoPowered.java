package org.firstinspires.ftc.teamcode.logancode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Odo CS 7198 Auto Blue Backstage", group="Autos")
public class _7198CSAutoBackstageBlue_OdoPowered extends _7198CSAuto_OdoPowered {
    @Override
    public void initializeGameConfig() {
        THIS_ALLIANCE = _7198CSAuto.ALLIANCE.BLUE;
    }
}
