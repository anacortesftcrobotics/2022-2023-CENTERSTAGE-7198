package org.firstinspires.ftc.teamcode.logancode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.CenterStageAutoBackstage;

@Autonomous(name="CS 7198 Auto Blue Backstage", group="Autos")
public class _7198CSAutoBackstageBlue extends _7198CSAuto {
    @Override
    public void initializeGameConfig() {
        THIS_ALLIANCE = ALLIANCE.BLUE;
    }
}
