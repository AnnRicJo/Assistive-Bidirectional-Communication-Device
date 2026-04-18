# Assistive Bidirectional Communication Device

A wireless bidirectional communication system using **ESP32** and **nRF24L01**, capable of transmitting messages in both **text and Morse code**. The project also includes an interactive **Morse learning mode** with audio feedback.

## Features
- Wireless communication using nRF24L01  
-  Dual input: Text (rotary encoder) & Morse code (buttons)  
-  Bidirectional message transmission  
-  Buzzer for Morse audio feedback  
-  Learn Morse mode for practice  
-  16×2 LCD display

## Components Used
- ESP32 DOIT DEVKIT V1  
- nRF24L01+ PA LNA Module  
- 16×2 LCD Display  
- PCF8574 I2C Module  
- Rotary Encoder  
- Push Buttons  
- Active Buzzer  
- Capacitors (100nF, 100µF)

## Circuit Diagram
<img width="1600" height="1027" alt="WhatsApp Image 2026-04-15 at 7 39 43 PM" src="https://github.com/user-attachments/assets/55cf4d2f-7271-4d25-af46-f38e1732dda1" />

## Project Images
<img width="1230" height="813" alt="WhatsApp Image 2026-04-17 at 6 53 07 PM" src="https://github.com/user-attachments/assets/e116bfdf-8c8d-4fae-b6ef-31153fafeca9" />

## Working
The device operates in three modes:

- **Letter Mode**: Select characters using the rotary encoder  
- **Morse Mode**: Input dots and dashes using buttons  
- **Learn Mode**: Plays Morse code using buzzer for learning  

Messages are transmitted wirelessly via nRF24L01 and displayed on the LCD of the receiving device.

## System Flow
1. Initialize ESP32, LCD, RF module, and inputs  
2. User composes message (Text/Morse)  
3. Message transmitted via RF module  
4. Receiver displays message and plays Morse (optional)  

## Code
> Full code and explanation available in the project files / report.

## 🎯 Applications
- Emergency communication  
- Assistive communication devices  
- Morse code learning tool  
- Low-power embedded systems  

## Authors
- Paris Pratheesh  
- Pranav Nair  
- Rashid Rahman  
- Richu Mathew Shaji  

## License
This project is for academic and educational purposes.
