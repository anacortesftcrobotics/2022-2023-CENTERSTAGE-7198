package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="CS 8934 Auto Red Backstage", group="CenterStage")
public class CenterStageAutoBackstageRed extends CenterStageAutoBackstage {
        @Override
        public void initializeGameConfig() {
                THIS_ALLIANCE = ALLIANCE.RED;
        }
}
