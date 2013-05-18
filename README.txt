This project is a Demo to demonstrate how to connect a graphical display to the Launchpad microcontroller. The display is low cost and high resolution, 160 x 100, and each pixel has 16 levels of grey.

The hardware has the following characteristics:
  - Circuit for controlling the NHD-C160100DIZ display from Newhaven
  - Graphic controller ST7528
  - Platform MSP430 Launchpad
  - Processor MSP430G2553 @ 8MHz

The display is hard wire connected to the display using a brekout board (Digikey NHD-COG14-36-ND) and a FPC connector (Digikey A100284CT-ND)
  - P1.7	SDA 	(pin 5)
  - P1.6	SCL 	(pin 4)
  - P2.5	RST	(pin 2)
  - P2.5	CSB	(pin 1)

Remaining connections are for decoupling capacitors that you may fand in the datasheets.
	http://www.newhavendisplay.com/specs/NHD-C160100DiZ-FSW-FBW.pdf
	http://www.newhavendisplay.com/app_notes/ST7528.pdf

The software is capable of
  - Clear display
  - Clear line
  - Write fixed length text line.
  - Only black level shade

To create the font library I used the software "The Dot Factory". This software creates two arrays, one with the bitmaps and other with the index of each char for indirect addressing. Due to the complexity of the generated characters, high resolution and small size of the screen, I decided to create 2 byte height characters using this software.

	http://www.pavius.net/2009/07/the-dot-factory-an-lcd-font-and-image-generator/




Have fun and remember to use at your own risk, given that I don't guarantee that this crap won't burn your house.