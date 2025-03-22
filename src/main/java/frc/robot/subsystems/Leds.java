package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase{

    public AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    
    public Leds(){
        m_led = new AddressableLED(0);
        m_ledBuffer = new AddressableLEDBuffer(60);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public void setNode(int node, int r, int g, int b){
        m_ledBuffer.setRGB(node, r, g, b);
        
    }

    public double getStripLength(){
        return m_ledBuffer.getLength();
    }

    public void rainfallSeqence(){
        for(var i = 0;i < m_ledBuffer.getLength(); i++){
            m_ledBuffer.setRGB(i, 0, 0, 0);    //set all leds to black
            
        }
        m_ledBuffer.setRGB(0, 0, 0, 255);//start rainfall
        /* 
        for(var i = 0; i < m_ledBuffer.getLength(); i++){
            Commands.waitSeconds(0.25);
            if(m_ledBuffer.getBlue(i) == 255){
                try{m_ledBuffer.setRGB(i+1,0,0,255);}catch(Exception e){}
                for(var j = 0; j < 5; j++){// 5 trailing leds
                    try{m_ledBuffer.setRGB(i-j, 0,0,m_ledBuffer.getBlue(i-j) - 51);}catch(Exception e){}//fade value is equal to inital blue divided by number of trailing leds
                }

            }
        }
            */
        Timer.delay(1);
        m_ledBuffer.setRGB(1, 0, 0, 255);//start rainfall

        

    }

    /*private void wave(Color c1, Color c2, double cycleLength, double duration) {
    double x = (1 - ((Timer.getTimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = m_ledBuffer.getLength() - 1; i >= 0; i--) {
      x += xDiffPerLed;
      double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
      if (Double.isNaN(ratio)) {
        ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
      }
      if (Double.isNaN(ratio)) {
        ratio = 0.5;
      }
      double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
      double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
      double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
      m_ledBuffer.setLED(i, new Color(red, green, blue));
    }
  } */

    

    public void green(){
        for(var i = 0;i < m_ledBuffer.getLength(); i++){
            m_ledBuffer.setRGB(i, 0, 255, 0);
        }
    }

    public void red(){
        for(var i = 0;i < m_ledBuffer.getLength(); i++){
            m_ledBuffer.setRGB(i, 255, 0, 0);
        }
    }

    public void black(){
        for(var i = 0;i < m_ledBuffer.getLength(); i++){
            m_ledBuffer.setRGB(i, 0, 0, 0);
        }
    }

    public void yellow(){
        for(var i = 0; i < m_ledBuffer.getLength(); i++){
            m_ledBuffer.setRGB(i, 255, 255, 0);
        }
    }

    public void flashGreen(){
        for(var i = 0; i < 5; i++){
            green();
            Commands.waitSeconds(0.5);
            black();
            Commands.waitSeconds(0.2);
        }
        
    }

    public void spiritColors(){
        for(var i = 0;i < m_ledBuffer.getLength(); i++){
            if(i % 2 == 0){
            m_ledBuffer.setRGB(i, 255, 255, 255); //white
            }
            else{
                m_ledBuffer.setRGB(i, 0, 0, 255); //blue
            }
        }
    }

    public void redAndGreen(){
        for(var i = 0;i < m_ledBuffer.getLength(); i++){
            if(i % 2 == 0){
            m_ledBuffer.setRGB(i, 255, 0, 0); //white
            }
            else{
                m_ledBuffer.setRGB(i, 0, 255, 0); //blue
            }
        }
    }

    @Override
    public void periodic() {
        m_led.setData(m_ledBuffer);
    }
}