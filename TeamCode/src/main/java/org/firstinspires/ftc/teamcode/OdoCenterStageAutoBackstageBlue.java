package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="CS 8934 Auto Blue Backstage ODO", group="CenterStage")
public class OdoCenterStageAutoBackstageBlue extends CenterStageAutoBackstageOdoPowered {
    @Override
    public void initializeGameConfig() {
        STAGE_LOCATION = STAGE_LOCATION.BACK;
        THIS_ALLIANCE = CenterStageAutoBackstage.ALLIANCE.BLUE;
    }
}
