package org.firstinspires.ftc.teamcode.stryke;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

/**
 * Created by Steven on 10/20/2016.
 *
 * Credit to FTC 4545
 * https://github.com/stev3nlo/Ouroboros2016-2017
 */


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.ArrayList;

/**
 * Created by Steven on 10/20/2016.
 */


public class MRRangeSensor {
    private ElapsedTime runtime = new ElapsedTime();
    byte[] rangeCache; //The read will return an array of bytes.

    I2cAddr rangeAddress = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    public static final int RANGE_REG_START = 0x04; //Register to start reading
    public static final int RANGE_READ_LENGTH = 2; //Number of byte to read

    I2cDevice rangeSensor;
    I2cDeviceSynch rangeReader;

    int ultraSonicDistanceValue;

    public MRRangeSensor(I2cDevice rangeSensor, I2cAddr addr) {
        this.rangeSensor = rangeSensor;
        this.rangeAddress = addr;
        rangeCache = new byte[RANGE_READ_LENGTH];
        initReader();

        if(!rangeReader.isArmed()) throw new IllegalStateException("Range reader isnt armed!!");
    }


    public void setI2cAddress(int i) {
        rangeReader.setI2cAddress(I2cAddr.create8bit(i));
    }

    public int getRawUltraSonicDistance() {
        return rangeCache[0] & 0xFF;
    }
    public void initReader() {
        rangeReader = new I2cDeviceSynchImpl(rangeSensor, rangeAddress, false);
        rangeReader.engage();
    }

    public int getUltraSonicDistance() {
        getRangeCache();
        return rangeCache[0] & 0xFF;
    }

    //
    public int getRawOptic() {
        getRangeCache();
        return rangeCache[1] & 0xFF;
    }

    public ArrayList<String> getRangeCache() {
        rangeCache = rangeReader.read(RANGE_REG_START, RANGE_READ_LENGTH);
        ArrayList output = new ArrayList<String>();

        output.add(rangeCache[0] & 0xFF);
        output.add(rangeCache[1] & 0xFF);
        output.add("Run Time: " + runtime.toString());

        return output;
    }

    public boolean inFrontOfBeacon() {
        if (getUltraSonicDistance() < 8) {		//value needs to be tested/calculated
            return true;
        }
        return false;
    }

    @Override
    public String toString() {
        ArrayList<String> cache = getRangeCache();
        String output = "";

        //output += "Ultra Sonic " + String.valueOf(cache.get(0));
        output += "\nRaw USD: " + getRawUltraSonicDistance() + "\n";
        output += "Ultra Sonic: " + ultraSonicDistanceValue;
        output += "\nODS: " + String.valueOf(cache.get(1));
        output += "\nStatus: " + String.valueOf(cache.get(2));

        return output;
    }

    public double pParam = -1.02001;
    public double qParam = 0.00311326;
    public double rParam = -8.39366;
    public int    sParam = 10;

    protected double cmFromOptical(int opticalReading)
    {
        if (opticalReading < sParam)
            return 0;
        else
            return pParam * Math.log(qParam * (rParam + opticalReading));
    }

    public double cmOptical()
    {
        return cmFromOptical(getRawOptic());
    }

    public double distanceCm() {
        double optical = cmOptical();
        return Math.floor(optical > 0 ? optical : getUltraSonicDistance());
    }
}