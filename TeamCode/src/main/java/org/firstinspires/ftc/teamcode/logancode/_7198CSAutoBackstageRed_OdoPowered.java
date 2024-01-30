package org.firstinspires.ftc.teamcode.logancode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Odo CS 7198 Auto Red Backstage", group="Autos")
public class _7198CSAutoBackstageRed_OdoPowered extends _7198CSAuto_OdoPowered {
        @Override
        public void initializeGameConfig() {
                THIS_ALLIANCE = _7198CSAuto.ALLIANCE.RED;
        }
}
