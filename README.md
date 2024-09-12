# FPGA-Morse-Code-Translator

For this project, I developed an International Morse Code Translator capable of interpreting keys from A to Z and 0 to 9. The system utilizes UART for both input and output. When a character is received through the UART RX, the translator converts it into its Morse code equivalent and sends this output through UART TX to a terminal.

To enhance user interaction, the device has the following:

Volume Control: Adjusted using 5 switches (SW15-11), allowing users to modify the amplitude of the sound output. Speed Control: Governed by 9 switches (SW10-2), this feature lets users set the pace of Morse code signals (one time unit). Mode Selection: A single switch (SW1) toggles between two output modes: Mode 0: Sound output Mode 1: LED output
