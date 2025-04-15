package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase{

    //m_led is the actual thing plugged into the rio, m_ledBuffer is a controller object
    public AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    
    public Leds(){
        m_led = new AddressableLED(0);//GPIO port
        m_ledBuffer = new AddressableLEDBuffer(60);//Length of strip
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

    //Does not work
    public void rainfallSeqence(){
        for(var i = 0;i < m_ledBuffer.getLength(); i++){
            m_ledBuffer.setRGB(i, 0, 0, 0);    //set all leds to black
            
        }
        m_ledBuffer.setRGB(0, 0, 0, 255);//start rainfall
        Timer.delay(1);
        m_ledBuffer.setRGB(1, 0, 0, 255);//start rainfall
    }

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
            m_ledBuffer.setRGB(i, 255, 0, 0); //red
            }
            else{
                m_ledBuffer.setRGB(i, 0, 255, 0); //green
            }
        }
    }

    @Override
    public void periodic() {
        m_led.setData(m_ledBuffer);
    }
}