package org.firstinspires.ftc.teamcode.logancode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="CS 7198 Auto Blue Backstage", group="Autos")
public class _7198CSAutoBackstageBlue extends _7198CSAuto {
    @Override
    public void initializeGameConfig() {
        THIS_ALLIANCE = ALLIANCE.BLUE;
    }
}
