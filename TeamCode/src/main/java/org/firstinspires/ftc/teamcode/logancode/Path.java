package org.firstinspires.ftc.teamcode.logancode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.*;
import java.util.ArrayList;

public class Path
{

    private ArrayList<Position2D> positions = new ArrayList<Position2D>();

    public Path(String pathDataFileLocation, Telemetry telem)
    {
        loadPathFile(pathDataFileLocation, telem);
        telem.addLine("Path Data Loaded from: " + pathDataFileLocation);
        telem.update();
    }

    public Path(InputStream inputStream, Telemetry telem)
    {
        loadPathFile(inputStream, telem);
        telem.addLine("Path Data Loaded from: " + inputStream.toString());
        telem.update();
    }

    public Position2D getPosition(int index)
    {
        if(index >=0 && index < positions.size())
            return positions.get(index);
        return null;
    }

    private void loadPathFile(InputStream inputStream, Telemetry telem)
    {
        try (InputStreamReader r = new InputStreamReader(inputStream);
             BufferedReader br = new BufferedReader(r))
        {

            String line;
            while ((line = br.readLine()) != null) {
                String[] xyString;
                xyString = line.split(" ", 2);

                if(xyString.length == 2)
                    if(!xyString[0].equals("") && !xyString[1].equals(""))
                        positions.add(new Position2D(Double.parseDouble(xyString[0]), Double.parseDouble(xyString[1])));
            }
        }
        catch (IOException ioException)
        {
            telem.addLine(ioException.toString());
        }
    }

    private void loadPathFile(String pathDataFileLocation, Telemetry telem)
    {
        try (FileReader fr = new FileReader(pathDataFileLocation);
             BufferedReader br = new BufferedReader(fr))
        {

            String line;
            while ((line = br.readLine()) != null) {
                String[] xyString = new String[2];
                xyString = line.split(" ", 2);

                positions.add(new Position2D(Double.parseDouble(xyString[0]), Double.parseDouble(xyString[1])));
            }
        }
        catch (IOException ioException)
        {
            telem.addLine(ioException.toString());
        }
    }
}
