# RPN Calculator
MSP430-based RPN scientific calculator

This calculator has two MSP430s working in tandem to control the memory, screen, and keyboard. The main memory is 128k of parallel SRAM, which the MSP430 cannot address directly as memory since it lacks an external bus. Instead, a converter program [(link)](https://github.com/JoeyShepard/MSP430_memory_manager) allows the program to refer to memory stored externally by using arrays. The convert translates array accesses at compile time to function calls that use the GPIO to read and write data to the chip. The calculator software supports basic math functions, exponents, logarithms, roots, and trig functions. The accuracy of calculation can be set from 12 to 32 decimal places in the settings screen. With 32 decimal places, the calculator produces extremely accurate results. The display is a simple 4x20 HD44780 compatible LCD. While the systems runs at around 3.6v, the LCD expects 5v and sets the contrast relative to the voltage difference between the supply voltage and contrast pin. Luckily, the logic of the LCD will run at 3.5v, so a voltage pump built with a 555 timer supplies around -2v on the contrast pin to make the display visible. The keypad is soldered from 42 tactile switches, and the button labels were cut on a laser printer into stamp rubber and then painted. The case was cut out of a large single sheet of copper clad circuit board material and soldered together into a box. This project was part of [Hacklet 70 on Hackaday](https://hackaday.com/2015/08/14/hacklet-70-calculator-projects/).<br/><br/>


![RPN Calculator](/images/calculator_keypad.JPG)
![RPN Calculator internals](/images/calc_back.jpg)

![Benchmark result of arcsin(arccos(arctan(tan(cos(sin(9))))))](/images/benchmark.jpg)
![Settings page](/images/settings.jpg)
