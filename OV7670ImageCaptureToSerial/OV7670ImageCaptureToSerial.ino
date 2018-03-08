/***
 * OV7670ImageCaptureToSerial
 * Luiz C. A. Albuquerque
 * lcaalbuquerque@uol.com.br
 * 23/03/2017
 * Captures an image from an OV7670 camera with FIFO memory to a serial line.
 ***/

#include <SPI.h>
#define USE_SDFAT
//#include <SD.h>
#include <SdFat.h>
SdFatSoftSpi<12, 11, 13> SD;  // Bit-Bang SD_SPI_CONFIGURATION == 3

#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv tft;
//#define SD_CS 10

#include <Wire.h>

/***
 * Pin connections for an Arduino Due
 * ----------------------------------
 * OV + FIFO Pins           Due Pins
 *    1 - 3.3V  <--------->   3.3V
 *    2 - GND   <--------->   GND
 *    3 - SIOC  <---------> 21 - SCL
 *    4 - SIOD  <---------> 20 - SDA
 *    5 - VSYNC <---------> 43
 *    6 - HREF  <--------->    NC
 *    7 - D7    <---------> 30
 *    8 - D6    <---------> 31
 *    9 - D5    <---------> 32
 *   10 - D4    <---------> 33
 *   11 - D3    <---------> 34
 *   12 - D2    <---------> 35
 *   13 - D1    <---------> 36
 *   14 - D0    <---------> 37
 *   15 - RST   <--------->   3.3V
 *   16 - PWDN  <--------->   GND
 *   17 - STR   <---------> 13 - LED
 *   18 - RCK (RCLK) <----> 42
 *   19 - WR (WEN) <------> 44
 *   20 - OE    <--------->   GND
 *   21 - WRST  <---------> 45
 *   22 - RRST  <---------> 46
 * ---------------------------------
 ***/

const byte ChipSelect    = 10;  // for TFT LCD shield
const byte HardwareSSPin = 53;  // for Arduino Mega
int photoTakenCount = 0;

// Serial input:
const int BUFFERLENGTH = 255;
char incomingByte[BUFFERLENGTH+1];  // for incoming serial data

// VGA default:
int PHOTO_WIDTH           = 640;
int PHOTO_HEIGHT          = 480;
int PHOTO_BYTES_PER_PIXEL = 2;

// Command and parameter related strings
String rawCommandLine = "";
String command        = "QQVGA";
String FPSparam       = "ThirtyFPS";
String AWBparam       = "SAWB";
String AECparam       = "HistAEC";
String YUVmatrixParam = "YUVMatrixOn";
String denoiseParam   = "DenoiseNo";
String edgeParam      = "EdgeNo";
String ABLCparam      = "AblcON";

enum ResolutionType {None, VGA, VGAP, QVGA, QQVGA};
ResolutionType resolution = None;

enum CaptureMode {SDcard, TFTdisplay, SerialLine};

// Camera input/output pin connection to Arduino Due
#define WRST  45    // 27 Output Write Pointer Reset
#define RRST  46    // 28 Output Read Pointer Reset
#define WEN   44    // 29 Output Write Enable
#define VSYNC 43    // Input Vertical Sync marking frame capture
#define RCLK  42    // Output FIFO buffer output clock
#define STR   13    // Output LED

// FIFO memory input pins
#define DO7   30
#define DO6   31
#define DO5   32
#define DO4   33
#define DO3   34
#define DO2   35
#define DO1   36
#define DO0   37

// Register addresses and values
#define CLKRC  0x11
#define CLKRC_VALUE_VGA     0x01  // raw Bayer
#define CLKRC_VALUE_QVGA    0x01
#define CLKRC_VALUE_QQVGA   0x01
#define CLKRC_VALUE_NIGHTMODE_FIXED 0x03  // fixed frame
#define CLKRC_VALUE_NIGHTMODE_AUTO  0x80  // auto frame rate adjust
#define COM7   0x12
#define COM7_VALUE_VGA      0x01  // raw Bayer
#define COM7_VALUE_VGA_COLOR_BAR    0x03  // raw Bayer
#define COM7_VALUE_VGA_PROCESSED_BAYER  0x05  // processed Bayer
#define COM7_VALUE_QVGA     0x00
#define COM7_VALUE_QVGA_COLOR_BAR   0x02
#define COM7_VALUE_PREDEFINED_COLOR_BAR 0x12
#define COM7_VALUE_QQVGA    0x00
#define COM7_VALUE_QQVGA_COLOR_BAR  0x02 
#define COM7_VALUE_RESET    0x80
#define COM3   0x0C
#define COM3_VALUE_VGA      0x00  // raw Bayer
#define COM3_VALUE_QVGA     0x04
#define COM3_VALUE_QQVGA    0X04  // from docs
#define COM3_VALUE_QQVGA_SCALE_ENABLED  0x0C  // enable scale and DCW
#define COM14  0x3E
#define COM14_VALUE_VGA     0x00  // raw Bayer
#define COM14_VALUE_QVGA    0x19
#define COM14_VALUE_QQVGA   0x1A
#define COM14_VALUE_MANUAL_SCALING    0x08  // manual scaling enabled
#define COM14_VALUE_NO_MANUAL_SCALING 0x00  // manual scaling disabled
#define SCALING_XSC         0x70
#define SCALING_XSC_VALUE_VGA           0x3A  // raw Bayer
#define SCALING_XSC_VALUE_QVGA          0x3A
#define SCALING_XSC_VALUE_QQVGA         0x3A
#define SCALING_XSC_VALUE_QQVGA_SHIFT1  0x3A
#define SCALING_XSC_VALUE_COLOR_BAR     0xBA
#define SCALING_YSC         0x71
#define SCALING_YSC_VALUE_VGA               0x35  // raw Bayer
#define SCALING_YSC_VALUE_QVGA              0x35
#define SCALING_YSC_VALUE_QQVGA             0x35
#define SCALING_YSC_VALUE_COLOR_BAR         0x35  // 8 color bar
#define SCALING_YSC_VALUE_COLOR_BAR_GREY    0xB5  // fade to grey color bar
#define SCALING YSC_VALUE_COLOR_BAR_SHIFT1  0xB5
#define SCALING_DCWCTR      0x72
#define SCALING_DCWCTR_VALUE_VGA      0x11  // raw Bayer
#define SCALING_DCWCTR_VALUE_QVGA     0x11
#define SCALING_DCWCTR_VALUE_QQVGA    0x22
#define SCALING_PCLK_DIV    0x73
#define SCALING_PCLK_DIV_VALUE_VGA    0xF0  // raw Bayer
#define SCALING_PCLK_DIV_VALUE_QVGA   0xF1
#define SCALING_PCLK_DIV_VALUE_QQVGA  0xF2
#define SCALING_PCLK_DELAY  0xA2
#define SCALING_PCLK_DELAY_VALUE_VGA    0x02  // raw Bayer
#define SCALING_PCLK_DELAY_VALUE_QVGA   0x02
#define SCALING_PCLK_DELAY_VALUE_QQVGA  0x02

// Controls YUV order used with COM13
// Need YUYV format for Android decoding - default value is 0xD
#define TSLB   0x3A
#define TSLB_VALUE_YUYV_AUTO_OUTPUT_WINDOW_ENABLED  0x01  // no custom scaling
#define TSLB_VALUE_YUYV_AUTO_OUTPUT_WINDOW_DISABLED 0x00  // for adjusting
                                                          // HSTART, etc.
                                                          // YUYV format
#define TSLB_VALUE_UYVY_AUTO_OUTPUT_WINDOW_DISABLED 0x08
#define TSLB_VALUE_TESTVALUE  0x04  // from YCbCr reference

// Default value is 0x88
// Ok if you want YUYV order, no need to change
#define COM13  0x3D
#define COM13_VALUE_DEFAULT       0x88
#define COM13_VALUE_NOGAMMA_YUYV  0x00
#define COM13_VALUE_GAMMA_YUYV    0x80
#define COM13_VALUE_GAMMA_YVYU    0x82
#define COM13_VALUE_YUYV_UVSATAUTOADJ_ON  0x40 // works with COM4
#define COM17  0x42
#define COM17_VALUE_AEC_NORMAL_NO_COLOR_BAR 0x00
#define COM17_VALUE_AEC_NORMAL_COLOR_BAR    0x08 // activate color bar for DSP
#define COM4   0x0D

// RGB settings and data format
#define COM15  0x40

// Night mode
#define COM11  0x3B
#define COM11_VALUE_NIGHTMODE_ON        0x80  // night mode
#define COM11_VALUE_NIGHTMODE_OFF       0x00
#define COM11_VALUE_NIGHTMODE_ON_EIGHTH 0xE0  // night mode 1/8 frame rate min
#define COM11_VALUE_NIGHTMODE_FIXED     0x0A
#define COM11_VALUE_NIGHTMODE_AUTO      0xEA  // night mode auto frame rate adj

// Color matrix control YUV
#define MTX1    0x4F
#define MTX1_VALUE    0x80
#define MTX2    0x50
#define MTX2_VALUE    0x80
#define MTX3    0x51
#define MTX3_VALUE    0x00
#define MTX4    0x52
#define MTX4_VALUE    0x22
#define MTX5    0x53
#define MTX5_VALUE    0x5E
#define MTX6    0x54
#define MTX6_VALUE    0x80
#define CONTRAS 0x56
#define CONTRAS_VALUE 0x40
#define MTXS    0x58
#define MTXS_VALUE    0x9E

// COM8
#define COM8    0x13
#define COM8_VALUE_AWB_OFF  0xE5
#define COM8_VALUE_AWB_ON   0xE7

// Automatic white balance
#define AWBC1   0x43
#define AWBC1_VALUE   0x14
#define AWBC2   0x44
#define AWBC2_VALUE   0xF0
#define AWBC3   0x45
#define AWBC3_VALUE   0x34
#define AWBC4   0x46
#define AWBC4_VALUE   0x58
#define AWBC5   0x47
#define AWBC5_VALUE   0x28
#define AWBC6   0x48
#define AWBC6_VALUE   0x3A
#define AWBC7   0x59
#define AWBC7_VALUE   0x88
#define AWBC8   0x5A
#define AWBC8_VALUE   0x88
#define AWBC9   0x5B
#define AWBC9_VALUE   0x44
#define AWBC10  0x5C
#define AWBC10_VALUE  0x67
#define AWBC11  0x5D
#define AWBC11_VALUE  0x49
#define AWBC12  0x5E
#define AWBC12_VALUE  0x0E
#define AWBCTR3 0x6C
#define AWBCTR3_VALUE 0x0A
#define AWBCTR2 0x6D
#define AWBCTR2_VALUE 0x55
#define AWBCTR1 0x6E
#define AWBCTR1_VALUE 0x11
#define AWBCTR0 0x6F
#define AWBCTR0_VALUE_NORMAL    0x9F
#define AWBCTR0_VALUE_ADVANCED  0x9E

// Gain
#define COM9    0x14
#define COM9_VALUE_MAX_GAIN_128X  0x6A
#define COM9_VALUE_4XGAIN         0x10
#define BLUE    0x01  // AWB blue channel gain
#define BLUE_VALUE    0x40
#define RED     0x02  // AWB red channel gain
#define RED_VALUE     0x40
#define GGAIN   0x6A  // AWB green channel gain
#define GGAIN_VALUE   0x40
#define COM16   0x41
#define COM16_VALUE   0x08  // AWB gain on 
#define GFIX    0x69
#define GFIX_VALUE    0x00

// Edge Enhancement Adjustment
#define EDGE    0x3F
#define EDGE_VALUE    0x00
#define REG75   0x75
#define REG75_VALUE   0x03
#define REG76   0x76
#define REG76_VALUE   0xE1

// Denoise
#define DNSTH   0x4C
#define DNSTH_VALUE   0x00
#define REG77   0x77
#define REG77_VALUE   0x00

// Denoise and Edge Enhancement
#define COM16_VALUE_DENOISE_OFF_EDGE_ENHANCEMENT_OFF_AWBGAIN_ON 0x08
#define COM16_VALUE_DENOISE_ON_EDGE_ENHANCEMENT_OFF_AWBGAIN_ON  0x18
#define COM16_VALUE_DENOISE_OFF_EDGE_ENHANCEMENT_ON_AWBGAIN_ON  0x28
#define COM16_VALUE_DENOISE_ON_EDGE_ENHANCEMENT_ON_AWBGAIN_ON   0x38

// 30FPS frame rate, PCLK = 24 MHz
#define CLKRC_VALUE_30FPS   0x80
#define DBLV    0x6B
#define DBLV_VALUE_30FPS    0x0A
#define EXHCH   0x2A
#define EXHCH_VALUE_30FPS   0x00
#define EXHCL   0x2B
#define EXHCL_VALUE_30FPS   0x00
#define DM_LNL  0x92
#define DM_LNL_VALUE_30FPS  0x00
#define DM_LNH  0x93
#define DM_LNH_VALUE_30FPS  0x00
#define COM11_VALUE_30FPS   0x0A

// Saturation control
#define SATCTR  0xC9
#define SATCTR_VALUE    0x60

// AEC/AGC - Automatic Exposure/Gain Control
#define GAIN    0x00
#define GAIN_VALUE      0x00
#define AEW     0x24
#define AEW_VALUE       0x95
#define AEB     0x25
#define AEB_VALUE       0x33
#define VPT     0x26
#define VPT_VALUE       0xE3

// AEC/AGC control histogram
#define HAECC1  0x9F
#define HAECC1_VALUE    0x78
#define HAECC2  0xA0
#define HAECC2_VALUE    0x68
#define HAECC3  0xA6
#define HAECC3_VALUE    0xD8
#define HAECC4  0xA7
#define HAECC4_VALUE    0xD8
#define HAECC5  0xA8
#define HAECC5_VALUE    0xF0
#define HAECC6  0xA9
#define HAECC6_VALUE    0x90
#define HAECC7  0xAA  // AEC algorithm selection
#define HAECC7_VALUE_HISTOGRAM_AEC_ON 0x94
#define HAECC7_VALUE_AVERAGE_AEC_ON   0x00

// Array control
#define CHLF    0x33
#define CHLF_VALUE      0x0B
#define ARBLM   0x34
#define ARBLM_VALUE     0x11

// ADC control
#define ADCCTR1 0x21
#define ADCCTR1_VALUE   0x02
#define ADCCTR2 0x22
#define ADCCTR2_VALUE   0x91
#define ADC     0x37
#define ADC_VALUE       0x1D
#define ACOM    0x38
#define ACOM_VALUE      0x71
#define OFON    0x39
#define OFON_VALUE      0x2A

// Black level calibration
#define ABLC1   0xB1
#define ABLC1_VALUE     0x0C
#define THL_ST  0xB3
#define THL_ST_VALUE    0x82

// Window Output
#define HSTART  0x17
#define HSTART_VALUE_DEFAULT  0x11
#define HSTART_VALUE_VGA      0x13
#define HSTART_VALUE_QVGA     0x13
#define HSTART_VALUE_QQVGA    0x13  // it works
#define HSTOP   0x18
#define HSTOP_VALUE_DEFAULT   0x61
#define HSTOP_VALUE_VGA       0x01
#define HSTOP_VALUE_QVGA      0x01
#define HSTOP_VALUE_QQVGA     0x01  // it works
#define HREF    0x32
#define HREF_VALUE_DEFAULT    0x80
#define HREF_VALUE_VGA        0xB6
#define HREF_VALUE_QVGA       0x24
#define HREF_VALUE_QQVGA      0xA4
#define VSTRT   0x19
#define VSTRT_VALUE_DEFAULT   0x03
#define VSTRT_VALUE_VGA       0x02
#define VSTRT_VALUE_QVGA      0x02
#define VSTRT_VALUE_QQVGA     0x02
#define VSTOP   0x1A
#define VSTOP_VALUE_DEFAULT   0x7B
#define VSTOP_VALUE_VGA       0x7A
#define VSTOP_VALUE_QVGA      0x7A
#define VSTOP_VALUE_QQVGA     0x7A
#define VREF    0x03
#define VREF_VALUE_DEFAULT    0x03
#define VREF_VALUE_VGA        0x0A
#define VREF_VALUE_QVGA       0x0A
#define VREF_VALUE_QQVGA      0x0A

// Strobe
#define STR     0xAC
#define STR_VALUE_ENTER_LED2_MODE 0x82
#define STR_VALUE_EXIT            0x00

// I2C
#define OV7670_I2C_ADDRESS  0x21
#define I2C_ERROR_WRITING_START_ADDRESS  11
#define I2C_ERROR_WRITING_DATA  22
#define DATA_TOO_LONG  1  // data too long to fit in transmit buffer
#define NACK_ON_TRANSMIT_OF_ADDRESS  2  // received NACK on transmit of address
#define NACK_ON_TRANSMIT_OF_DATA  3  // received NACK on transmit of data
#define OTHER_ERROR  4
#define I2C_READ_START_ADDRESS_ERROR  33
#define I2C_READ_DATA_SIZE_MISMATCH_ERROR  44

void executeCommand(String command, CaptureMode show = SDcard);

void setup()
{
  uint16_t ID;
  Serial.begin(115200);
  Serial1.begin(460800);
  Serial.println(F("Arduino Due Serial Monitor Controlled Camera "
                   "--- Using OV7670 Camera with FIFO Memory"));
  Serial.println();
  
  // Setup the OV7670 Camera for use in taking still photos.
  Wire.begin();
  Serial.println(F("------------------------------ Camera Registers "
                   "------------------------------"));
  resetCameraRegisters();
  readRegisters();
  Serial.println(F("------------------------------------------------"
                   "------------------------------"));
  setupCamera();
  Serial.println(F("Finished initializing camera..."));
  Serial.println();

  // Initialize TFT LCD.
  tft.reset();
  delay(500);
  ID = tft.readID();
  Serial.print(F("Initializing TFT LCD with ID: 0x"));
  Serial.println(ID, HEX);
  if (ID == 0x0D3D3) ID = 0x9481;
  tft.begin(ID);
  tft.fillScreen(0x001F);
  Serial.print(F("Screen width:  "));
  Serial.println(tft.width());
  Serial.print(F("Screen height: "));
  Serial.println(tft.height());
  if (tft.height() > tft.width()) tft.setRotation(1); // landscape
  tft.setTextColor(0xFFFF, 0x0000);
  Serial.println();

  // Initialize SD card.
  Serial.print(F("Initializing SD card... "));
  pinMode(HardwareSSPin, OUTPUT);
  if (!SD.begin(ChipSelect))
  {
    Serial.println(F("Initialization failed...\nThings to check:"));
    Serial.println(F("- Is a card inserted?"));
    Serial.println(F("- Is your wiring correct?"));
    Serial.println(F("- Did you change the chipSelect pin to match "
                     "your shield or module?"));
    return;
  }
  else
  {
    Serial.println(F("Wiring is correct and a card is present..."));
  }

  Serial.println();
  Serial.println(F("========================================================"
                   "=========="));
  Serial.println();
  Serial.println(F("Omnivision OV7670 Camera Image Capture Software v. 1.0"));
  Serial.println(F("Copyright 2015 by Robert Chin. All Rights Reserved."));
  Serial.println(F("Modified by Luiz C. A. Albuquerque in 02/2017."));
  Serial.println();
  Serial.println(F("========================================================"
                   "=========="));
  Serial.println();
  Serial.println(F("Type h or help for Main Help Menu..."));
} // end setup

void loop()
{
  // Serial monitor input.
  Serial.println(F("------------------------------------------"
                   "------------------------"));
  Serial.println(F("\nReady to accept new command =>"));
  while (true)
  {
    if (Serial.available() > 0)
    {
      int noCharsRead = Serial.readBytesUntil('\n', incomingByte,
                                              BUFFERLENGTH);
      for (int i = 0; i < noCharsRead; i++)
      {
        rawCommandLine += incomingByte[i];
      }
      break;
    }
  }

  // Print out the the command from serial monitor.
  Serial.print(F("Raw command from Serial Monitor: "));
  Serial.println(rawCommandLine);
  if (rawCommandLine == "h" || rawCommandLine == "help")
  {
    displayHelpMenu();
  }
  else if (rawCommandLine == "help cam")
  {
    displayHelpCommandsParams();
  }
  else if (rawCommandLine == "d")
  {
    displayCurrentCommand();
  }
  else if (rawCommandLine == "t")
  {
    // Take photo.
    Serial.println(F("\nGoing to take photo with current command:"));
    displayCurrentCommand();
    executeCommand(command);
    Serial.println(F("Photo taken and saved to Arduino SD card..."));
    String testFile = createPhotoFilename();
    Serial.print(F("Image output filename: "));
    Serial.println(testFile);
    photoTakenCount++;
  }
  else if (rawCommandLine == "s")
  {
    // Show photo.
    Serial.println(F("\nGoing to show photo with current command:"));
    displayCurrentCommand();
    executeCommand(command, TFTdisplay);
    Serial.println(F("Photo taken and shown on TFT LCD..."));
    //photoTakenCount++;
  }
  else if (rawCommandLine == "u")
  {
    // Send photo through Serial Line.
    Serial.println(F("Going to send photo with current command:"));
    displayCurrentCommand();
    executeCommand(command, SerialLine);
    Serial.println(F("Photo taken and sent to Serial Line..."));
  }
  else if (rawCommandLine == "testread")
  {
    readPrintFile("TEST.TXT");
  }
  else if (rawCommandLine == "testwrite")
  {
    checkRemoveFile("TEST.TXT");
    writeFileTest("TEST.TXT");
  }
  else
  {
    //Serial.println(F("ERROR... Command does not exist!"));
    Serial.println(F("Changing command or parameters according to your "
                     "input:"));
    // Parse command line and set command line elements.
    // Parse raw command into command and parameters.
    parseRawCommand(rawCommandLine);
    // Display new changed camera command with parameters.
    displayCurrentCommand();
  }

  // Reset raw command line.
  rawCommandLine = "";
  Serial.println();
} // end loop

void displayHelpMenu()
{
  Serial.println();
  Serial.println(F("--------------------------- Help Menu "
                   "---------------------------"));
  Serial.println(F("d - display current camera command"));
  Serial.println(F("s - show photograph using current command and parameters"));
  Serial.println(F("t - take photograph using current command and parameters"));
  Serial.println(F("u - send photograph using current command and parameters"));
  Serial.println(F("testread - test reading files from SD card by reading and"
                   "\n           printing the contents of TEST.TXT"));
  Serial.println(F("testwrite - test writing files to SD card"));
  Serial.println(F("help cam - display camera's commands and parameters"));
  Serial.println();
} // end displayHelpMenu

void displayHelpCommandsParams()
{
  Serial.println();
  Serial.println(F("---------------- Help Menu Camera Commands/Params "
                   "----------------"));
  Serial.println(F("Resolution change commands: VGA, VGAP, QVGA, QQVGA"));
  Serial.println(F("FPS parameters: ThirtyFPS, NightMode"));
  Serial.println(F("AWB parameters: SAWB, AAWB"));
  Serial.println(F("AEC parameters: AveAEC, HistAEC"));
  Serial.println(F("YUV matrix parameters: YUVMatrixOn, YUVMatrixOff"));
  Serial.println(F("Denoise parameters: DenoiseYes, DenoiseNo"));
  Serial.println(F("Edge enhancement: EdgeYes, EdgeNo"));
  Serial.println(F("Automatic black level calibration: AblcON, AblcOFF"));
  Serial.println();
} // end displayHelpCommandsParams

void parseRawCommand(String rcl)
{
  String entries[10];
  boolean success = false;
  int noElements = parseCommand(rcl.c_str(), ' ', entries);

  for (int i = 0; i < noElements; i++)
  {
    success = processRawCommandElement(entries[i]);
    if (!success)
    {
      Serial.print(F("Invalid command or parameter: "));
      Serial.println(entries[i]);
    }
    else
    {
      Serial.print(F("Command or parameter \'"));
      Serial.print(entries[i]);
      Serial.println(F("\' succesfully set..."));
    }
  }

  // Assume parameter change since user is setting parameters on command
  // line manually. Tell the camera to re-initialize and set up camera
  // according to new parameters.
  resolution = None;
  // Reset and reloads registers.
  resetCameraRegisters();
} // end parseRawCommand

int parseCommand(const char *commandline, char splitchar, String *result)
{
  int resultIndex = 0;
  int length = strlen(commandline);
  String temp = "";
  char tempchar;

  for (int i = 0; i < length; i++)
  {
    tempchar = commandline[i];
    if (tempchar == splitchar)
    {
      result[resultIndex] += temp;
      resultIndex++;
      temp = "";
    }
    else
    {
      temp += tempchar;
    }
  }

  // Put in end part of string.
  result[resultIndex] = temp;
  return resultIndex + 1;
} // end parseCommand

boolean processRawCommandElement(String element)
{
  boolean result = false;
  element.toLowerCase();
  if (element == "vga" || element == "vgap" || element == "qvga" ||
      element == "qqvga")
  {
    element.toUpperCase();
    command = element;
    result = true;
  }
  else if (element == "thirtyfps")
  {
    FPSparam = "ThirtyFPS";
    result = true;
  }
  else if (element == "nightmode")
  {
    FPSparam = "NightMode";
    result = true;
  }
  else if (element == "sawb")
  {
    AWBparam = "SAWB";
    result = true;
  }
  else if (element == "aawb")
  {
    AWBparam = "AAWB";
    result = true;
  }
  else if (element == "aveaec")
  {
    AECparam = "AveAEC";
    result = true;
  }
  else if (element == "histaec")
  {
    AECparam = "HistAEC";
    result = true;
  }
  else if (element == "yuvmatrixon")
  {
    YUVmatrixParam = "YUVMatrixOn";
    result = true;
  }
  else if (element == "yuvmatrixoff")
  {
    YUVmatrixParam = "YUVMatrixOff";
    result = true;
  }
  else if (element == "denoiseyes")
  {
    denoiseParam = "DenoiseYes";
    result = true;
  }
  else if (element == "denoiseno")
  {
    denoiseParam = "DenoiseNo";
    result = true;
  }
  else if (element == "edgeyes")
  {
    edgeParam = "EdgeYes";
    result = true;
  }
  else if (element == "edgeno")
  {
    edgeParam = "EdgeNo";
    result =true;
  }
  else if (element == "ablcon")
  {
    ABLCparam = "AblcON";
    result = true;
  }
  else if (element == "ablcoff")
  {
    ABLCparam = "AblcOFF";
    result = true;
  }
  return result;
} // end processRawCommandElement

// Main call to setup the OV7670 camera.
void setupCamera()
{
  Serial.println(F("In setupCamera..."));
  initializeOV7670Camera();
} // end setupCamera

void displayCurrentCommand()
{
  // Print out command and parameters.
  Serial.println();
  Serial.println(F("--------- Current camera command and parameters setting "
                   "---------"));
  Serial.print(F("Command: "));
  Serial.println(command);
  Serial.print(F("FPSparam: "));
  Serial.println(FPSparam);
  Serial.print(F("AWBparam: "));
  Serial.println(AWBparam);
  Serial.print(F("AECparam: "));
  Serial.println(AECparam);
  Serial.print(F("YUVmatrixParam: "));
  Serial.println(YUVmatrixParam);
  Serial.print(F("DenoiseParam: "));
  Serial.println(denoiseParam);
  Serial.print(F("EdgeParam: "));
  Serial.println(edgeParam);
  Serial.print(F("ABLCparam: "));
  Serial.println(ABLCparam);
  Serial.println();
} // end displayCurrentCommand

void writeFileTest(String fileName)
{
  File tempFile;
  tempFile = SD.open(fileName.c_str(), FILE_WRITE);
  if (tempFile)
  {
    Serial.print(F("Writing to testfile..."));
    tempFile.print(F("TEST CAMERA SDCARD HOOKUP at time..."));
    tempFile.print(millis()/1000);
    tempFile.println(F(" seconds"));
    tempFile.print(F("Photo info filename: "));
    tempFile.println(createPhotoInfoFilename());
    tempFile.print(F("Photo info: "));
    tempFile.println(createPhotoInfo());
    tempFile.close();
    Serial.println(F("Writing file done..."));
  }
  else
  {
    Serial.print(F("ERROR opening file "));
    Serial.println(fileName);
  }
} // end writeFileTest

byte readRegisterValue(int registerAddress)
{
  byte data = 0;
  Wire.beginTransmission(OV7670_I2C_ADDRESS);
  Wire.write(registerAddress);
  Wire.endTransmission();
  Wire.requestFrom(OV7670_I2C_ADDRESS, 1);
  while (Wire.available() < 1);
  data = Wire.read();
  return data;
} // end readRegisterValue

int OV7670Write(int start, const byte *pData, int size)
{
  int n, error;
  Wire.beginTransmission(OV7670_I2C_ADDRESS);
  n = Wire.write(start);  // write the start address
  if (n != 1)
  {
    return I2C_ERROR_WRITING_START_ADDRESS;
  }
  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
  {
    return I2C_ERROR_WRITING_DATA;
  }
  error = Wire.endTransmission(true);  // release the I2C bus
  if (error != 0)
  {
    return error;
  }
  return 0;  // no error
} // end OV7670Write

void resetCameraRegisters()
{
  // Reading needed to prevent error; this is most likely a bug
  // in the OV7670.
  byte data      = readRegisterValue(COM7);
  int result     = OV7670WriteReg(COM7, COM7_VALUE_RESET);
  String sresult = parseI2Cresult(result);
  Serial.println("Resetting all registers by setting COM7 "
                 "register to 0x80: " + sresult);
  delay(500);
} // end resetCameraRegisters

int OV7670WriteReg(int reg, byte data)
{
  int error;
  error = OV7670Write(reg, &data, 1);
  return error;
} // end OV7670WriteReg

String parseI2Cresult(int result)
{
  String sresult = "";
  switch (result)
  {
    case 0:
      sresult = "I2C operation OK...";
      break;
    case I2C_ERROR_WRITING_START_ADDRESS:
      sresult = "I2C_ERROR_WRITING_STARTING_ADDRESS";
      break;
    case I2C_ERROR_WRITING_DATA:
      sresult = "I2C_ERROR_WRITING_DATA";
      break;
    case DATA_TOO_LONG:
      sresult = "DATA_TOO_LONG";
      break;
    case NACK_ON_TRANSMIT_OF_ADDRESS:
      sresult = "NACK_ON_TRANSMIT_OF_ADDRESS";
      break;
    case NACK_ON_TRANSMIT_OF_DATA:
      sresult = "NACK_ON_TRANSMIT_OF_DATA";
      break;
    case OTHER_ERROR:
      sresult = "OTHER_ERROR";
      break;
    default:
      sresult = "I2C ERROR TYPE NOT FOUND...";
      break;
  }
  return sresult;
} // end parseI2Cresult

void readRegisters()
{
  byte data = 0;
  data = readRegisterValue(CLKRC);
  Serial.print(F("CLKRC = "));
  Serial.println(data, HEX);
  data = readRegisterValue(COM7);
  Serial.print(F("COM7  = "));
  Serial.println(data, HEX);
  data = readRegisterValue(COM3);
  Serial.print(F("COM3  = "));
  Serial.println(data, HEX);
  data = readRegisterValue(COM14);
  Serial.print(F("COM14 = "));
  Serial.println(data, HEX);
  data = readRegisterValue(SCALING_XSC);
  Serial.print(F("SCALING_XSC = "));
  Serial.println(data, HEX);
  data = readRegisterValue(SCALING_YSC);
  Serial.print(F("SCALING_YSC = "));
  Serial.println(data, HEX);
  data = readRegisterValue(SCALING_DCWCTR);
  Serial.print(F("SCALING_DCWCTR = "));
  Serial.println(data, HEX);
  data = readRegisterValue(SCALING_PCLK_DIV);
  Serial.print(F("SCALING_PCLK_DIV = "));
  Serial.println(data, HEX);
  data = readRegisterValue(SCALING_PCLK_DELAY);
  Serial.print(F("SCALING_PCLK_DELAY = "));
  Serial.println(data, HEX);
  // default value D
  data = readRegisterValue(TSLB);
  Serial.print(F("TSLB  (YUV higher order bit, Bit[3]) = "));
  Serial.println(data, HEX);
  // default value 88
  data = readRegisterValue(COM13);
  Serial.print(F("COM13 (YUV lower order bit, Bit[1]) = "));
  Serial.println(data, HEX);
  data = readRegisterValue(COM17);
  Serial.print(F("COM17 (DSP color bar selection) = "));
  Serial.println(data, HEX);
  data = readRegisterValue(COM4);
  Serial.print(F("COM4  (works with COM17) = "));
  Serial.println(data, HEX);
  data = readRegisterValue(COM15);
  Serial.print(F("COM15 (COLOR FORMAT SELECTION) = "));
  Serial.println(data, HEX);
  data = readRegisterValue(COM11);
  Serial.print(F("COM11 (night mode) = "));
  Serial.println(data, HEX);
  data = readRegisterValue(COM8);
  Serial.print(F("COM8  (color control, AWB) = "));
  Serial.println(data,HEX);
  data = readRegisterValue(HAECC7);
  Serial.print(F("HAECC7 (AEC algorithm selection) = "));
  Serial.println(data,HEX);
  data = readRegisterValue(GFIX);
  Serial.print(F("GFIX = "));
  Serial.println(data,HEX);
  // Window output
  data = readRegisterValue(HSTART);
  Serial.print(F("HSTART = "));
  Serial.println(data,HEX);
  data = readRegisterValue(HSTOP);
  Serial.print(F("HSTOP = "));
  Serial.println(data,HEX);
  data = readRegisterValue(HREF);
  Serial.print(F("HREF = "));
  Serial.println(data,HEX);
  data = readRegisterValue(VSTRT);
  Serial.print(F("VSTRT = "));
  Serial.println(data,HEX);
  data = readRegisterValue(VSTOP);
  Serial.print(F("VSTOP = "));
  Serial.println(data,HEX);
  data = readRegisterValue(VREF);
  Serial.print(F("VREF = "));
  Serial.println(data,HEX);
} // end readRegisters

void initializeOV7670Camera()
{
  Serial.println(F("Initializing OV7670 camera..."));
  // Set WRST to 0 and RRST to 0, 0.1ms after power on.
  int durationMicroSecs = 1;
  // Set mode for pins whether input or output.
  pinMode(WRST, OUTPUT);
  pinMode(RRST, OUTPUT);
  pinMode(WEN, OUTPUT);
  pinMode(VSYNC, INPUT);
  pinMode(RCLK, OUTPUT);
  pinMode(STR, OUTPUT);
  // FIFO memory output pins:
  pinMode(DO7, INPUT);
  pinMode(DO6, INPUT);
  pinMode(DO5, INPUT);
  pinMode(DO4, INPUT);
  pinMode(DO3, INPUT);
  pinMode(DO2, INPUT);
  pinMode(DO1, INPUT);
  pinMode(DO0, INPUT);
  // Delay 1 ms.
  delay(1);
  pulseLowEnabledPin(WRST, durationMicroSecs);
  // Need to clock the FIFO manually to get it to reset.
  digitalWrite(RRST, LOW);
  pulsePin(RCLK, durationMicroSecs);
  pulsePin(RCLK, durationMicroSecs);
  digitalWrite(RRST, HIGH);
} // end initializeOV7670Camera

void pulseLowEnabledPin(int pinNo, int durationMicroSecs)
{
  // For low enabled pins: 0 = on and 1 = off.
  digitalWrite(pinNo, LOW);  // sets the pin on
  delayMicroseconds(durationMicroSecs);  // pauses for durationMicroSecs
  digitalWrite(pinNo, HIGH);  // ses the pin off
  delayMicroseconds(durationMicroSecs);  // pauses for durationMicroSecs
} // end pulseLowEnabledPin

void pulsePin(int pinNo, int durationMicroSecs)
{
  digitalWrite(pinNo, HIGH);
  delayMicroseconds(durationMicroSecs);
  digitalWrite(pinNo, LOW);
  delayMicroseconds(durationMicroSecs);
}  // end pulsePin

void executeCommand(String command, CaptureMode show)
{
  // Setup camera for VGA, QVGA or QQVGA modes.
  if (command == "VGA")
  {
    Serial.println(F("Taking a VGA photo..."));
    if (resolution != VGA)
    {
      resetCameraRegisters();
      resolution = VGA;
      setupOV7670ForVGArawRGB();
      Serial.println(F("----------------------------- Camera Registers "
                       "-----------------------------"));
      readRegisters();
      Serial.println(F("-----------------------------------------------"
                       "-----------------------------"));
    }
  }
  else if (command == "VGAP")
  {
    Serial.println(F("Taking a VGAP photo..."));
    if (resolution != VGAP)
    {
      resetCameraRegisters();
      resolution = VGAP;
      setupOV7670ForVGAprocessedBayerRGB();
      Serial.println(F("----------------------------- Camera Registers "
                       "-----------------------------"));
      readRegisters();
      Serial.println(F("-----------------------------------------------"
                       "-----------------------------"));
    }
  }
  else if (command == "QVGA")
  {
    Serial.println(F("Taking a QVGA photo..."));
    if (resolution != QVGA)
    {
      resetCameraRegisters();
      resolution = QVGA;
      setupOV7670ForQVGAYUV();
      Serial.println(F("----------------------------- Camera Registers "
                       "-----------------------------"));
      readRegisters();
      Serial.println(F("-----------------------------------------------"
                       "-----------------------------"));
    }
  }
  else if (command == "QQVGA")
  {
    Serial.println(F("Taking a QQVGA photo..."));
    if (resolution != QQVGA)
    {
      resetCameraRegisters();
      resolution = QQVGA;
      setupOV7670ForQQVGAYUV();
      Serial.println(F("----------------------------- Camera Registers "
                       "-----------------------------"));
      readRegisters();
      Serial.println(F("-----------------------------------------------"
                       "-----------------------------"));
    }
  }
  else
  {
    Serial.print(F("The command "));
    Serial.print(command);
    Serial.println(F(" is not recognized..."));
  }
  delay(100);
  if (show == TFTdisplay)
    showPhoto();
  else if (show == SDcard)
    takePhoto();
  else
    sendPhoto();
} // end executeCommand

void setupOV7670ForVGArawRGB()
{
  int result = 0;
  String sresult = "";
  Serial.println(F("-------------- Setting camera for VGA (Raw RGB) "
                   "--------------"));
  PHOTO_WIDTH  = 640;
  PHOTO_HEIGHT = 480;
  PHOTO_BYTES_PER_PIXEL = 1;
  Serial.print(F("Photo width = "));
  Serial.println(PHOTO_WIDTH);
  Serial.print(F("Photo height = "));
  Serial.println(PHOTO_HEIGHT);
  Serial.print(F("Bytes per pixel = "));
  Serial.println(PHOTO_BYTES_PER_PIXEL);

  // Basic registers:
  result = OV7670WriteReg(CLKRC, CLKRC_VALUE_VGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("CLKRC: "));
  Serial.println(sresult);
  result = OV7670WriteReg(COM7, COM7_VALUE_VGA);
  //result = OV7670WriteReg(COM7, COM7_VALUE_VGA_COLOR_BAR);
  sresult = parseI2Cresult(result);
  Serial.print(F("COM7: "));
  Serial.println(sresult);
  result = OV7670WriteReg(COM3, COM3_VALUE_VGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("COM3: "));
  Serial.println(sresult);
  result = OV7670WriteReg(COM14, COM14_VALUE_VGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("COM14: "));
  Serial.println(sresult);
  result = OV7670WriteReg(SCALING_XSC, SCALING_XSC_VALUE_VGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("SCALING_XSC: "));
  Serial.println(sresult);
  result = OV7670WriteReg(SCALING_YSC, SCALING_YSC_VALUE_VGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("SCALING_YSC: "));
  Serial.println(sresult);
  result = OV7670WriteReg(SCALING_DCWCTR, SCALING_DCWCTR_VALUE_VGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("SCALING_DCWCTR: "));
  Serial.println(sresult);
  result = OV7670WriteReg(SCALING_PCLK_DIV, SCALING_PCLK_DIV_VALUE_VGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("SCALING_PCLK_DIV: "));
  Serial.println(sresult);
  result = OV7670WriteReg(SCALING_PCLK_DELAY, SCALING_PCLK_DELAY_VALUE_VGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("SCALING_PCLK_DELAY: "));
  Serial.println(sresult);

  // COM17 - DSP color bar enable/disable
  // COM17_VALUE 0x08
  // Activate color bar for DSP
  //result = OV7670WriteReg(COM17, COM17_VALUE_AEC_NORMAL_COLOR_BAR);
  result = OV7670WriteReg(COM17, COM17_VALUE_AEC_NORMAL_NO_COLOR_BAR);
  sresult = parseI2Cresult(result);
  Serial.print(F("COM17: "));
  Serial.println(sresult);

  // Set additional parameters:
  // Set camera frames per second.
  setCameraFPSmode();
  // Set camera automatic exposure control.
  setCameraAEC();
  // Needed color correction: green to red.
  result = OV7670WriteReg(0xB0, 0x8C);
  Serial.print(F("Setting B0 UNDOCUMENTED register to 0x84 or 0x8C?: "));
  Serial.println(sresult);

  // Set camera saturation.
  setCameraSaturationControl();

  // Set up camera array control.
  setupCameraArrayControl();

  // Setup ADC control.
  setupCameraADCcontrol();

  // Setup automatic black level calibration.
  setupCameraABLC();

  // Change window output parameters after custom scaling.
  Serial.println(F("------------ Setting camera window output parameters "
                   "------------"));
  result = OV7670WriteReg(HSTART, HSTART_VALUE_VGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("HSTART: "));
  Serial.println(sresult);
  result = OV7670WriteReg(HSTOP, HSTOP_VALUE_VGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("HSTOP: "));
  Serial.println(sresult);
  result = OV7670WriteReg(HREF, HREF_VALUE_VGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("HREF: "));
  Serial.println(sresult);
  result = OV7670WriteReg(VSTRT, VSTRT_VALUE_VGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("VSTRT: "));
  Serial.println(sresult);
  result = OV7670WriteReg(VSTOP, VSTOP_VALUE_VGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("VSTOP: "));
  Serial.println(sresult);
  result = OV7670WriteReg(VREF, VREF_VALUE_VGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("VREF: "));
  Serial.println(sresult);
} // end setupOV7670ForVGArawRGB

void setCameraFPSmode()
{
  // Set FPS for camera.
  if (FPSparam == "ThirtyFPS")
    setupCameraFor30FPS();
  else if (FPSparam == "NightMode")
    setupCameraNightMode();
} // end setCameraFPSmode

void setupCameraFor30FPS()
{
  int result = 0;
  String sresult = "";

  Serial.println(F("------------------- Setting camera to 30 FPS "
                   "-------------------"));
  result = OV7670WriteReg(CLKRC, CLKRC_VALUE_30FPS);
  sresult = parseI2Cresult(result);
  Serial.print(F("CLKRC: "));
  Serial.println(sresult);
  result = OV7670WriteReg(DBLV, DBLV_VALUE_30FPS);
  sresult = parseI2Cresult(result);
  Serial.print(F("DBLV: "));
  Serial.println(sresult);
  result = OV7670WriteReg(EXHCH, EXHCH_VALUE_30FPS);
  sresult = parseI2Cresult(result);
  Serial.print(F("EXHCH: "));
  Serial.println(sresult);
  result = OV7670WriteReg(EXHCL, EXHCL_VALUE_30FPS);
  sresult = parseI2Cresult(result);
  Serial.print(F("EXHCL: "));
  Serial.println(sresult);
  result = OV7670WriteReg(DM_LNL, DM_LNL_VALUE_30FPS);
  sresult = parseI2Cresult(result);
  Serial.print(F("DM_LNL: "));
  Serial.println(sresult);
  result = OV7670WriteReg(DM_LNH, DM_LNH_VALUE_30FPS);
  sresult = parseI2Cresult(result);
  Serial.print(F("DM_LNH: "));
  Serial.println(sresult);
  result = OV7670WriteReg(COM11, COM11_VALUE_30FPS);
  sresult = parseI2Cresult(result);
  Serial.print(F("COM11: "));
  Serial.println(sresult);
} // end setupCameraFor30FPS

void setupCameraNightMode()
{
  int result = 0;
  String sresult = "";

  Serial.println(F("------------------- Turning Night Mode ON "
                   "-------------------"));
  result = OV7670WriteReg(CLKRC, CLKRC_VALUE_NIGHTMODE_AUTO);
  sresult = parseI2Cresult(result);
  Serial.print(F("CLKRC: "));
  Serial.println(sresult);
  result = OV7670WriteReg(COM11, COM11_VALUE_NIGHTMODE_AUTO);
  sresult = parseI2Cresult(result);
  Serial.print(F("COM11: "));
  Serial.println(sresult);
} // end setupCameraNightMode

void setCameraAEC()
{
  // Process AEC (Automatic Exposure Control).
  if (AECparam == "AveAEC")
    // Set camera's average AEC/AGC parameters.
    setupCameraAverageBasedAECAGC();
  else if (AECparam == "HistAEC")
    // Set camera AEC algorithm to histogram
    setupCameraHistogramBasedAECAGC();
} // end setCameraAEC

void setupCameraAverageBasedAECAGC()
{
  int result = 0;
  String sresult = "";

  Serial.println(F("------- Setting camera average based AEC/AGC registers "
                   "-------"));
  result = OV7670WriteReg(AEW, AEW_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("AEW: "));
  Serial.println(sresult);
  result = OV7670WriteReg(AEB, AEB_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("AEB: "));
  Serial.println(sresult);
  result = OV7670WriteReg(VPT, VPT_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("VPT: "));
  Serial.println(sresult);
  result = OV7670WriteReg(HAECC7, HAECC7_VALUE_AVERAGE_AEC_ON);
  sresult = parseI2Cresult(result);
  Serial.print(F("HAECC7: "));
  Serial.println(sresult);
} // end setupCameraAverageBasedAECAGC

void setupCameraHistogramBasedAECAGC()
{
  int result = 0;
  String sresult = "";

  Serial.println(F("------- Setting camera histogram based AEC/AGC registers "
                   "-------"));
  result = OV7670WriteReg(AEW, AEW_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("AEW: "));
  Serial.println(sresult);
  result = OV7670WriteReg(AEB, AEB_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("AEB: "));
  Serial.println(sresult);
  result = OV7670WriteReg(HAECC1, HAECC1_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("HAECC1: "));
  Serial.println(sresult);
  result = OV7670WriteReg(HAECC2, HAECC2_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("HAECC2: "));
  Serial.println(sresult);
  result = OV7670WriteReg(HAECC3, HAECC3_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("HAECC3: "));
  Serial.println(sresult);
  result = OV7670WriteReg(HAECC4, HAECC4_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("HAECC4: "));
  Serial.println(sresult);
  result = OV7670WriteReg(HAECC5, HAECC5_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("HAECC5: "));
  Serial.println(sresult);
  result = OV7670WriteReg(HAECC6, HAECC6_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("HAECC6: "));
  Serial.println(sresult);
  result = OV7670WriteReg(HAECC7, HAECC7_VALUE_HISTOGRAM_AEC_ON);
  sresult = parseI2Cresult(result);
  Serial.print(F("HAECC7: "));
  Serial.println(sresult);
}  // end setupCameraHistogramBasedAECAGC

void setCameraSaturationControl()
{
  int result = 0;
  String sresult = "";

  Serial.println(F("-------------- Setting camera saturation level ----------"
                   "------"));
  result = OV7670WriteReg(SATCTR, SATCTR_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("SATCTR: "));
  Serial.println(sresult);
} // end setCameraSaturationControl

void setupCameraArrayControl()
{
  int result = 0;
  String sresult = "";

  Serial.println(F("--------------- Setting camera array control ------------"
                   "------"));
  result = OV7670WriteReg(CHLF, CHLF_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("CHLF: "));
  Serial.println(sresult);
  result = OV7670WriteReg(ARBLM, ARBLM_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("ARBLM: "));
  Serial.println(sresult);
} // end setupCameraArrayControl

void setupCameraADCcontrol()
{
  int result = 0;
  String sresult = "";

  Serial.println(F("---------------- Setting camera ADC control "
                   "----------------"));
  result = OV7670WriteReg(ADCCTR1, ADCCTR1_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("ADCCTR1: "));
  Serial.println(sresult);
  result = OV7670WriteReg(ADCCTR2, ADCCTR2_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("ADCCTR2: "));
  Serial.println(sresult);
  result = OV7670WriteReg(ADC, ADC_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("ADC: "));
  Serial.println(sresult);
  result = OV7670WriteReg(ACOM, ACOM_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("ACOM: "));
  Serial.println(sresult);
  result = OV7670WriteReg(OFON, OFON_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("OFON: "));
  Serial.println(sresult);
} // end setupCameraADCcontrol

void setupCameraABLC()
{
  int result = 0;
  String sresult = "";

  // If ABLC is off then return; otherwise turn on ABLC.
  if (ABLCparam == "AblcOFF")
    return;
  Serial.println(F("-------------------- Setting camera ABLC "
                   "--------------------"));
  result = OV7670WriteReg(ABLC1, ABLC1_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("ABLC1: "));
  Serial.println(sresult);
  result = OV7670WriteReg(THL_ST, THL_ST_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("THL_ST: "));
  Serial.println(sresult);
} // end setupCameraABLC

void setupOV7670ForVGAprocessedBayerRGB()
{
  int result = 0;
  String sresult = "";

  // Call base for VGA raw Bayer RGB mode.
  setupOV7670ForVGArawRGB();
  Serial.println(F("--------- Setting camera for VGA (Processed Bayer RGB) "
                   "---------"));
  // Set key register for selecting processed Bayer RGB output.
  result = OV7670WriteReg(COM7, COM7_VALUE_VGA_PROCESSED_BAYER);
  //result = OV7670WriteReg(COM7, COM7_VALUE_VGA_COLOR_BAR);
  sresult = parseI2Cresult(result);
  Serial.print(F("COM7: "));
  Serial.println(sresult);
  result = OV7670WriteReg(TSLB, 0x04);
  sresult = parseI2Cresult(result);
  Serial.print(F("Initializing TSLB register result = "));
  Serial.println(sresult);
  // Needed color correction: green to red.
  result = OV7670WriteReg(0xB0, 0x8C);
  sresult = parseI2Cresult(result);
  Serial.print(F("Setting B0 UNDOCUMENTED register to 0x8C? = "));
  Serial.println(sresult);
  // Set camera automatic white balance.
  setupCameraAWB();
  // Denoise and edge enhancement.
  setupCameraDenoiseEdgeEnhancement();
} // end setupOV7670ForVGAprocessedBayerRGB

void setupCameraAWB()
{
  // set AWB mode.
  if (AWBparam == "SAWB")
  {
    // Set simple automatic white balance.
    setupCameraSimpleAutomaticWhiteBalance();
    // Set gain config.
    setupCameraGain();
  }
  else if (AWBparam == "AAWB")
  {
    // Set advanced automatic white balance.
    setupCameraAdvancedAutomaticWhiteBalance();
    // Set camera automatic whie balance config.
    setupCameraAdvancedAutoWhiteBalanceConfig();
    // Set gain config.
    setupCameraGain();
  }
} // end setupCameraAWB

void setupCameraSimpleAutomaticWhiteBalance()
{
  int result = 0;
  String sresult = "";

  Serial.println(F("----------------- Setting camera to simple AWB "
                   "-----------------"));
  // Set register COM8.
  result = OV7670WriteReg(COM8, COM8_VALUE_AWB_ON);
  sresult = parseI2Cresult(result);
  Serial.print(F("COM8 (0x13): "));
  Serial.println(sresult);
  // Set control register AWBCTR0.
  result = OV7670WriteReg(AWBCTR0, AWBCTR0_VALUE_NORMAL);
  sresult = parseI2Cresult(result);
  Serial.print(F("AWBCTR0 control register (0x6F): "));
  Serial.println(sresult);
} // end setupCameraSimpleAutomaticWhiteBalance()

void setupCameraGain()
{
  int result = 0;
  String sresult = "";

  Serial.println(F("------------------------ Setting camera gain "
                   "------------------------"));
  // Set maximum gain.
  result = OV7670WriteReg(COM9, COM9_VALUE_4XGAIN);
  sresult = parseI2Cresult(result);
  Serial.print(F("COM9: "));
  Serial.println(sresult);
  // Set blue gain.
  result = OV7670WriteReg(BLUE, BLUE_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("BLUE gain: "));
  Serial.println(sresult);
  // Set red gain.
  result = OV7670WriteReg(RED, RED_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("RED gain: "));
  Serial.println(sresult);
  // Set green gain.
  result = OV7670WriteReg(GGAIN, GGAIN_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("GREEN gain: "));
  Serial.println(sresult);
  // Enable AWB gain.
  result = OV7670WriteReg(COM16, COM16_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("COM16 (enable gain): "));
  Serial.println(sresult);
} // end setupCameraGain

void setupCameraAdvancedAutomaticWhiteBalance()
{
  int result = 0;
  String sresult = "";
  Serial.println(F("------------- Setting camera to advanced AWB "
                   "-------------"));
  // AGC, AWB, and AEC enable.
  result = OV7670WriteReg(COM8, COM8_VALUE_AWB_ON);
  sresult = parseI2Cresult(result);
  Serial.print(F("COM8 (0x13): "));
  Serial.println(sresult);
  // AWBCTR0
  result = OV7670WriteReg(AWBCTR0, AWBCTR0_VALUE_ADVANCED);
  sresult = parseI2Cresult(result);
  Serial.print(F("AWB control register 0 (0x6F): "));
  Serial.println(sresult);
} // end setupCameraAdvancedAutomaticWhiteBalance

void setupCameraAdvancedAutoWhiteBalanceConfig()
{
  int result = 0;
  String sresult = "";

  Serial.println(F("----- Setting camera advanced auto white balance configs "
                   "-----"));
  result = OV7670WriteReg(AWBC1, AWBC1_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("AWBC1: "));
  Serial.println(sresult);
  result = OV7670WriteReg(AWBC2, AWBC2_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("AWBC2: "));
  Serial.println(sresult);
  result = OV7670WriteReg(AWBC3, AWBC3_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("AWBC3: "));
  Serial.println(sresult);
  result = OV7670WriteReg(AWBC4, AWBC4_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("AWBC4: "));
  Serial.println(sresult);
  result = OV7670WriteReg(AWBC5, AWBC5_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("AWBC5: "));
  Serial.println(sresult);
  result = OV7670WriteReg(AWBC6, AWBC6_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("AWBC6: "));
  Serial.println(sresult);
  result = OV7670WriteReg(AWBC7, AWBC7_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("AWBC7: "));
  Serial.println(sresult);
  result = OV7670WriteReg(AWBC8, AWBC8_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("AWBC8: "));
  Serial.println(sresult);
  result = OV7670WriteReg(AWBC9, AWBC9_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("AWBC9: "));
  Serial.println(sresult);
  result = OV7670WriteReg(AWBC10, AWBC10_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("AWBC10: "));
  Serial.println(sresult);
  result = OV7670WriteReg(AWBC11, AWBC11_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("AWBC11: "));
  Serial.println(sresult);
  result = OV7670WriteReg(AWBC12, AWBC12_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("AWBC12: "));
  Serial.println(sresult);
  result = OV7670WriteReg(AWBCTR3, AWBCTR3_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("AWBCTR3: "));
  Serial.println(sresult);
  result = OV7670WriteReg(AWBCTR2, AWBCTR2_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("AWBCTR2: "));
  Serial.println(sresult);
  result = OV7670WriteReg(AWBCTR1, AWBCTR1_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("AWBCTR1: "));
  Serial.println(sresult);
} // end setupCameraAdvancedAutoWhiteBalanceConfig

void setupCameraDenoiseEdgeEnhancement()
{
  int result = 0;
  String sresult = "";

  if (denoiseParam == "DenoiseYes" && edgeParam == "EdgeYes")
  {
    setupCameraDenoise();
    setupCameraEdgeEnhancement();
    result = OV7670WriteReg(COM16,
               COM16_VALUE_DENOISE_ON_EDGE_ENHANCEMENT_ON_AWBGAIN_ON);
    sresult = parseI2Cresult(result);
    Serial.print(F("COM16: "));
    Serial.println(sresult);
  }
  else if (denoiseParam == "DenoiseYes" && edgeParam == "EdgeNo")
  {
    setupCameraDenoise();
    result = OV7670WriteReg(COM16,
               COM16_VALUE_DENOISE_ON_EDGE_ENHANCEMENT_OFF_AWBGAIN_ON);
    sresult = parseI2Cresult(result);
    Serial.print(F("COM16: "));
    Serial.println(sresult);
  }
  else if (denoiseParam == "DenoiseNo" && edgeParam == "EdgeYes")
  {
    setupCameraEdgeEnhancement();
    result = OV7670WriteReg(COM16,
               COM16_VALUE_DENOISE_OFF_EDGE_ENHANCEMENT_ON_AWBGAIN_ON);
    sresult = parseI2Cresult(result);
    Serial.print(F("COM16: "));
    Serial.println(sresult);
  }
} // end setupCameraDenoiseEdgeEnhancement

void setupCameraDenoise()
{
  int result = 0;
  String sresult = "";

  Serial.println(F("-------------------- Setting camera denoise "
                   "--------------------"));
  result = OV7670WriteReg(DNSTH, DNSTH_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("DNSTH: "));
  Serial.println(sresult);
  result = OV7670WriteReg(REG77, REG77_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("REG77: "));
  Serial.println(sresult);
} // end setupCameraDenoise

void setupCameraEdgeEnhancement()
{
  int result = 0;
  String sresult = "";

  Serial.println(F("----------------- Setting camera edge enhancement "
                   "-----------------"));
  result = OV7670WriteReg(EDGE, EDGE_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("EDGE: "));
  Serial.println(sresult);
  result = OV7670WriteReg(REG75, REG75_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("REG75: "));
  Serial.println(sresult);
  result = OV7670WriteReg(REG76, REG76_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("REG76: "));
  Serial.println(sresult);
} // end setupCameraEdgeEnhancement

void setupOV7670ForQVGAYUV()
{
  int result = 0;
  String sresult = "";

  Serial.println(F("------------------ Setting camera for QVGA (YUV) "
                   "------------------"));
  PHOTO_WIDTH  = 320;
  PHOTO_HEIGHT = 240;
  PHOTO_BYTES_PER_PIXEL = 2;
  Serial.print(F("Photo width  = "));
  Serial.println(PHOTO_WIDTH);
  Serial.print(F("Photo height = "));
  Serial.println(PHOTO_HEIGHT);
  Serial.print(F("Bytes per pixel = "));
  Serial.println(PHOTO_BYTES_PER_PIXEL);

  // Basic registers:
  result = OV7670WriteReg(CLKRC, CLKRC_VALUE_QVGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("CLKRC: "));
  Serial.println(sresult);
  result = OV7670WriteReg(COM7, COM7_VALUE_QVGA);
  //result = OV7670WriteReg(COM7, COM7_VALUE_QVGA_COLOR_BAR);
  sresult = parseI2Cresult(result);
  Serial.print(F("COM7: "));
  Serial.println(sresult);
  result = OV7670WriteReg(COM3, COM3_VALUE_QVGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("COM3: "));
  Serial.println(sresult);
  result = OV7670WriteReg(COM14, COM14_VALUE_QVGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("COM14: "));
  Serial.println(sresult);
  result = OV7670WriteReg(SCALING_XSC, SCALING_XSC_VALUE_QVGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("SCALING_XSC: "));
  Serial.println(sresult);
  result = OV7670WriteReg(SCALING_YSC, SCALING_YSC_VALUE_QVGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("SCALING_YSC: "));
  Serial.println(sresult);
  result = OV7670WriteReg(SCALING_DCWCTR, SCALING_DCWCTR_VALUE_QVGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("SCALING_DCWCTR: "));
  Serial.println(sresult);
  result = OV7670WriteReg(SCALING_PCLK_DIV, SCALING_PCLK_DIV_VALUE_QVGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("SCALING_PCLK_DIV: "));
  Serial.println(sresult);
  result = OV7670WriteReg(SCALING_PCLK_DELAY, SCALING_PCLK_DELAY_VALUE_QVGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("SCALING_PCLK_DELAY: "));
  Serial.println(sresult);

  // YUV order control change from default use with COM13.
  result = OV7670WriteReg(TSLB, 0x04);
  sresult = parseI2Cresult(result);
  Serial.print(F("TSLB: "));
  Serial.println(sresult);

  // COM13
  result = OV7670WriteReg(COM13, 0xC2); // from YCbCr reference specs
  sresult = parseI2Cresult(result);
  Serial.print(F("COM13: "));
  Serial.println(sresult);

  // COM17 - DSP color bar enable/disable.
  // COM17_VALUE 0x08
  // Activate color bar for DSP
  //result = OV7670WriteReg(COM17, COM17_VALUE_AEC_NORMAL_COLOR_BAR);
  result = OV7670WriteReg(COM17, COM17_VALUE_AEC_NORMAL_NO_COLOR_BAR);
  sresult = parseI2Cresult(result);
  Serial.print(F("COM17: "));
  Serial.println(sresult);

  // Set additional parameters:
  // Set camera frames per second.
  setCameraFPSmode();
  // Set camera automatic exposure control.
  setCameraAEC();
  // Set camera automatic white balance.
  setupCameraAWB();
  // Set camera undocumented registers -- Needed minimum.
  setCameraUndocumentedRegisters();
  // Set color matrix for YUV.
  if (YUVmatrixParam == "YUVMatrixOn")
  {
    setCameraColorMatrixYUV();
  }
  // Set camera saturation.
  setCameraSaturationControl();
  // Denoise and edge enhancement.
  setupCameraDenoiseEdgeEnhancement();
  // Set up camera array control.
  setupCameraArrayControl();
  // Set ADC control.
  setupCameraADCcontrol();
  // Set automatic black level calibration.
  setupCameraABLC();

  Serial.println(F("----------- Setting camera window output parameters "
                   "-----------"));
  // Change window output parameters after custom scaling.
  result = OV7670WriteReg(HSTART, HSTART_VALUE_QVGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("HSTART: "));
  Serial.println(sresult);
  result = OV7670WriteReg(HSTOP, HSTOP_VALUE_QVGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("HSTOP: "));
  Serial.println(sresult);
  result = OV7670WriteReg(HREF, HREF_VALUE_QVGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("HREF: "));
  Serial.println(sresult);
  result = OV7670WriteReg(VSTRT, VSTRT_VALUE_QVGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("VSTRT: "));
  Serial.println(sresult);
  result = OV7670WriteReg(VSTOP, VSTOP_VALUE_QVGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("VSTOP: "));
  Serial.println(sresult);
  result = OV7670WriteReg(VREF, VREF_VALUE_QVGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("VREF: "));
  Serial.println(sresult);
} // end setupOV7670ForQVGAYUV

void setCameraUndocumentedRegisters()
{
  int result = 0;
  String sresult = "";

  Serial.println(F("------------ Setting camera undocumented registers "
                   "------------"));
  result = OV7670WriteReg(0xB0, 0x84);
  sresult = parseI2Cresult(result);
  Serial.print(F("Setting B0 UNDOCUMENTED register to 0x84: "));
  Serial.println(sresult);
} // end setupCameraUndocumentedRegisters

void setCameraColorMatrixYUV()
{
  int result = 0;
  String sresult = "";

  Serial.println(F("------------- Setting camera color matrix for YUV "
                   "-------------"));
  result = OV7670WriteReg(MTX1, MTX1_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("MTX1: "));
  Serial.println(sresult);
  result = OV7670WriteReg(MTX2, MTX2_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("MTX2: "));
  Serial.println(sresult);
  result = OV7670WriteReg(MTX3, MTX3_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("MTX3: "));
  Serial.println(sresult);
  result = OV7670WriteReg(MTX4, MTX4_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("MTX4: "));
  Serial.println(sresult);
  result = OV7670WriteReg(MTX5, MTX5_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("MTX5: "));
  Serial.println(sresult);
  result = OV7670WriteReg(MTX6, MTX6_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("MTX6: "));
  Serial.println(sresult);
  result = OV7670WriteReg(CONTRAS, CONTRAS_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("CONTRAS: "));
  Serial.println(sresult);
  result = OV7670WriteReg(MTXS, MTXS_VALUE);
  sresult = parseI2Cresult(result);
  Serial.print(F("MTXS: "));
  Serial.println(sresult);
} // end setCameraColorMatrixYUV

void setupOV7670ForQQVGAYUV()
{
  int result = 0;
  String sresult = "";

  Serial.println(F("----------------- Setting camera for QQVGA YUV "
                   "-----------------"));
  PHOTO_WIDTH = 160;
  PHOTO_HEIGHT = 120;
  PHOTO_BYTES_PER_PIXEL = 2;
  Serial.print(F("Photo width = "));
  Serial.println(PHOTO_WIDTH);
  Serial.print(F("Photo height = "));
  Serial.println(PHOTO_HEIGHT);
  Serial.print(F("Bytes per pixel = "));
  Serial.println(PHOTO_BYTES_PER_PIXEL);

  Serial.println(F("---------------- Setting basic QQVGA parameters "
                   "----------------"));
  result = OV7670WriteReg(CLKRC, CLKRC_VALUE_QQVGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("CLKRC: "));
  Serial.println(sresult);
  result = OV7670WriteReg(COM7, COM7_VALUE_QQVGA);
  //result = OV7670WriteReg(COM7, COM7_VALUE_QQVGA_COLOR_BAR);
  sresult = parseI2Cresult(result);
  Serial.print(F("COM7: "));
  Serial.println(sresult);
  result = OV7670WriteReg(COM3, COM3_VALUE_QQVGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("COM3: "));
  Serial.println(sresult);
  result = OV7670WriteReg(COM14, COM14_VALUE_QQVGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("COM14: "));
  Serial.println(sresult);
  result = OV7670WriteReg(SCALING_XSC, SCALING_XSC_VALUE_QQVGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("SCALING_XSC: "));
  Serial.println(sresult);
  result = OV7670WriteReg(SCALING_YSC, SCALING_YSC_VALUE_QQVGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("SCALING_YSC: "));
  Serial.println(sresult);
  result = OV7670WriteReg(SCALING_DCWCTR, SCALING_DCWCTR_VALUE_QQVGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("SCALING_DCWCTR: "));
  Serial.println(sresult);
  result = OV7670WriteReg(SCALING_PCLK_DIV, SCALING_PCLK_DIV_VALUE_QQVGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("SCALING_PCLK_DIV: "));
  Serial.println(sresult);
  result = OV7670WriteReg(SCALING_PCLK_DELAY, SCALING_PCLK_DELAY_VALUE_QQVGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("SCALING_PCLK_DELAY: "));
  Serial.println(sresult);

  // YUV order control change from default use with COM13.
  result = OV7670WriteReg(TSLB, TSLB_VALUE_YUYV_AUTO_OUTPUT_WINDOW_DISABLED);
  sresult = parseI2Cresult(result);
  Serial.print(F("TSLB: "));
  Serial.println(sresult);
  // COM13
  result = OV7670WriteReg(COM13, 0xC8); // Gamma enabled, UV auto adj on
  sresult = parseI2Cresult(result);
  Serial.print(F("COM13: "));
  Serial.println(sresult);
  // COM17 -- DSP color bar enable/disable
  //result = OV7670WriteReg(COM17, COM17_VALUE_AEC_NORMAL_COLOR_BAR);
  result = OV7670WriteReg(COM17, COM17_VALUE_AEC_NORMAL_NO_COLOR_BAR);
  sresult = parseI2Cresult(result);
  Serial.print(F("COM17: "));
  Serial.println(sresult);

  // Set additional parameters:
  // Set camera frames per second.
  setCameraFPSmode();
  // Set camera automatic exposure control.
  setCameraAEC();
  // Set camera automatic white balance.
  setupCameraAWB();
  // Setup undocumented registers -- Needed minimum.
  setCameraUndocumentedRegisters();
  // Set color matrix for YUV.
  if (YUVmatrixParam == "YUVMatrixOn")
  {
    setCameraColorMatrixYUV();
  }
  // Set camera saturation.
  setCameraSaturationControl();
  // Denoise and edge enhancement.
  setupCameraDenoiseEdgeEnhancement();
  // Set array control.
  setupCameraArrayControl();
  // Set ADC control.
  setupCameraADCcontrol();
  // Set automatic black control level calibration.
  setupCameraABLC();

  Serial.println(F("--------- Setting camera window output parameters "
                   "---------"));
  // Change window output parameters after custom scaling.
  result = OV7670WriteReg(HSTART, HSTART_VALUE_QQVGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("HSTART: "));
  Serial.println(sresult);
  result = OV7670WriteReg(HSTOP, HSTOP_VALUE_QQVGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("HSTOP: "));
  Serial.println(sresult);
  result = OV7670WriteReg(HREF, HREF_VALUE_QQVGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("HREF: "));
  Serial.println(sresult);
  result = OV7670WriteReg(VSTRT, VSTRT_VALUE_QQVGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("VSTRT: "));
  Serial.println(sresult);
  result = OV7670WriteReg(VSTOP, VSTOP_VALUE_QQVGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("VSTOP: "));
  Serial.println(sresult);
  result = OV7670WriteReg(VREF, VREF_VALUE_QQVGA);
  sresult = parseI2Cresult(result);
  Serial.print(F("VREF: "));
  Serial.println(sresult);
} // end setupOV7670ForQQVGAYUV

///////////////////////////// Main Function /////////////////////////////
void takePhoto()
{
  unsigned long startTime   = 0;
  unsigned long endTime     = 0;
  float elapsedTime         = 0;

  startTime = millis();
  captureOV7670Frame();
  readTransmitCapturedFrame();
  endTime = millis();
  elapsedTime = (endTime - startTime)/1000.0; // convert to seconds
  Serial.print(F("Elapsed time for taking and sending photo (sec) = "));
  Serial.println(elapsedTime);
} // end takePhoto

/***
 * The captureOV7670Frame() function captures an image from the OV7670 camera 
 * and saves it to the camera's FIFO frame buffer memory. The function does
 * the following:
 * 1. Waits for the VSYNC input from the camera to pulse to indicate the
 *    start of the image by calling the pulseIn(VSYNC, HIGH) function. The
 *    pulseIn() function makes the Arduino wait for thr pin VSYNC to pulse
 *    from LOW to HIGH and then to LOW again before returning and continuing
 *    program execution.
 * 2. Resets the FIFO memory frame buffer's write pointer to 0 that represents
 *    the beginning of the frame. The pulseLowEnabledPin(WRST, 6) function
 *    resets the write pointer by sending a LOW pulse to pin WRST for 6 usec.
 * 3. Sets the FIFO write enable to active (HIGH) so that image can be written
 *    to memory. Tjis done by calling the digitalWrite(WEN, HIGH) function
 *    that sets the output pin WEN to HIGH.
 * 4. Waits for VSYNC to pulse again to indicate the end of the frame capture.
 *    The pulseIn(VSYNC, HIGH) functionis called and waits for a positive
 *    pulse on the VSYNC pin.
 * 5. Sets the FIFO write enable to nonactive (LOW) so that no more images can
 *    be written to the camera's memory. The digitalWrite(WEN, LOW) function
 *    is called to write a LOW value to pin WEN which is connected to the
 *    camera's write enable pin WR.
 * 6. Prints out the elapsed time from the start of the image capture to the 
 *    end of the image capture.
 * 7. Program execution is halted for 2 msecs so that new data can appear on
 *    output pins. The delay() function is used to do this.
 ***/
void captureOV7670Frame()
{
  unsigned long durationStart       = 0;
  unsigned long durationStop        = 0;
  unsigned long timeForCaptureStart = 0;
  unsigned long timeForCaptureEnd   = 0;
  unsigned long elapsedTime         = 0;

  int result = 0;
  String sresult = "";

  result = OV7670WriteReg(STR, STR_VALUE_ENTER_LED2_MODE);
  sresult = parseI2Cresult(result);

  // Capture one frame into FIFO memory:
  // 0. Initialization.
  Serial.println();
  Serial.println(F("Starting capture of photo..."));

  Serial.print(F("STR enter: "));
  Serial.println(sresult);

  timeForCaptureStart = millis();
  // 1. Wait for VSYNC to pulse to indicate the start of the image.
  durationStart = pulseIn(VSYNC, HIGH);
  // 2. Reset write pointer to 0 which is the beginning of frame.
  pulseLowEnabledPin(WRST, 6);  // 3usec + 3usec for error factor on Arduino
  // 3. Set FIFO write enable to active (HIGH) so that the image can be
  //    written to memory.
  digitalWrite(WEN, HIGH);
  // 4. Wait for VSYNC to pulse again to indicate the end of the frame capture.
  durationStop = pulseIn(VSYNC, HIGH);
  // 5. Set FIFO write enable to nonactive (LOW) so that no more images can be
  //    written to the memory.
  digitalWrite(WEN, LOW);
  // 6. Print out statistics.
  timeForCaptureEnd = millis();
  elapsedTime = timeForCaptureEnd - timeForCaptureStart;
  Serial.print(F("Time for frame capture (msec) = "));
  Serial.println(elapsedTime);
  Serial.print(F("VSYNC beginning duration (usec) = "));
  Serial.println(durationStart);
  Serial.print(F("VSYNC end duration (usec) = "));
  Serial.println(durationStop);
  // 7. Wait so that new data can appear on output pins to read new data.
  delay(2);

  result = OV7670WriteReg(STR, STR_VALUE_EXIT);
  sresult = parseI2Cresult(result);
  Serial.print(F("STR exit: "));
  Serial.println(sresult);

} // end captureOV7670Frame

void readTransmitCapturedFrame()
{
  byte pixelData = 0;
  byte pinVal7   = 0,
       pinVal6   = 0,
       pinVal5   = 0,
       pinVal4   = 0,
       pinVal3   = 0,
       pinVal2   = 0,
       pinVal1   = 0,
       pinVal0   = 0;
  unsigned long byteCounter = 0;

  Serial.println(F("Starting transmission of photo to SD card..."));
  ////////////////////// Code for SD card //////////////////////////
  File imageOutputFile;
  String fileName = createPhotoFilename();
  checkRemoveFile(fileName);
  imageOutputFile = SD.open(fileName.c_str(), FILE_WRITE);
  if (!imageOutputFile)
  {
    Serial.println(F("\nCritical ERROR... Cannot open imageOutputFile for "
                     "output..."));
    return;
  }
  ///////////////////////////////////////////////////////////////////
  // Set read buffer pointer to start of frame.
  digitalWrite(RRST, LOW);
  pulsePin(RCLK, 1);
  pulsePin(RCLK, 1);
  pulsePin(RCLK, 1);
  digitalWrite(RRST, HIGH);
  for (int height = 0; height < PHOTO_HEIGHT; height++)
  for (int width = 0; width < PHOTO_WIDTH; width++)
  for (int byteno = 0; byteno < PHOTO_BYTES_PER_PIXEL; byteno++)
  {
    // Pulse the read clock RCLK to bring in new byte of data.
    pulsePin(RCLK, 1);
    // Convert pin values to byte values for pins 0-7 of incoming pixel byte.
    pinVal7 = convertPinValueToByteValue(digitalRead(DO7), 7);
    pinVal6 = convertPinValueToByteValue(digitalRead(DO6), 6);
    pinVal5 = convertPinValueToByteValue(digitalRead(DO5), 5);
    pinVal4 = convertPinValueToByteValue(digitalRead(DO4), 4);
    pinVal3 = convertPinValueToByteValue(digitalRead(DO3), 3);
    pinVal2 = convertPinValueToByteValue(digitalRead(DO2), 2);
    pinVal1 = convertPinValueToByteValue(digitalRead(DO1), 1);
    pinVal0 = convertPinValueToByteValue(digitalRead(DO0), 0);
    // Combine individual data from each pin into composite data in the form
    // of a single byte.
    pixelData = pinVal7 | pinVal6 | pinVal5 | pinVal4 | pinVal3 | pinVal2 |
                pinVal1 | pinVal0;
    /////////////////////////// SD card ////////////////////////////////
    byteCounter += imageOutputFile.write(pixelData);
    ////////////////////////////////////////////////////////////////////
  }
  // Close SD card file.
  imageOutputFile.close();
  Serial.print(F("Total bytes saved to SD card = "));
  Serial.println(byteCounter);
  // Write photo's info file to SD card.
  Serial.println(F("Writing photo's info file (.txt) to SD card..."));
  createPhotoInfoFile();
} // end readTransmitCapturedFrame

String createPhotoFilename()
{
  String fileName = "";
  String ext      = "";

  // Create filename and extension suffix that the photo will be saved under.
  // If command = QQVGA or QVGA then the extension is '.yuv'.
  if (command == "QQVGA" || command == "QVGA")
    ext = ".yuv";
  else if (command == "VGA" || command == "VGAP")
    ext = ".raw";
  // Create filename from 'resolution + photo number + extension'.
  fileName = command + photoTakenCount + ext;
  return fileName;
} // end createPhotoFilename

void checkRemoveFile(String fileName)
{
  // Check if file already exists and remove it if it does.
  char tempChar[256];
  strcpy(tempChar, fileName.c_str());
  if (SD.exists(tempChar))
  {
    Serial.print(F("Filename \'"));
    Serial.print(tempChar);
    Serial.println(F("\' already exists. Removing it..."));
    SD.remove(tempChar);
  }
  // If file still exists then new image file cannot be saved to SD card.
  if (SD.exists(tempChar))
  {
    Serial.println(F("ERROR: image output file cannot be created..."));
    return;
  }
} // end checkRemoveFile

void createPhotoInfoFile()
{
  // Creates the photo information file based on current settings.
  // '.txt' information file for photo.
  File infoFile;
  String fileName = createPhotoInfoFilename();
  // Check if file already exists and remove it if it does.
  checkRemoveFile(fileName);

  infoFile = SD.open(fileName.c_str(), FILE_WRITE);
  if (!infoFile)
  {
    Serial.println(F("\nCritical ERROR... Cannot open photo info file for "
                     "output..."));
    return;
  }
  // Write info to file.
  String data = createPhotoInfo();
  infoFile.println(data);
  infoFile.close();
} // end createPhotoInfoFile

String createPhotoInfoFilename()
{
  String fileName = "";
  String ext      = "";

  // Creates filename that the information about the photo will be saved under.
  // Create the file extension.
  ext = ".txt";
  // Create filename from:
  // Resolution + PhotoNumber + Extension
  fileName = command + photoTakenCount + ext;
  return fileName;
} // end createPhotoInfoFilename

String createPhotoInfo()
{
  String info = "";
  info = command + " " + FPSparam + " " + AWBparam + " " + AECparam + " " +
         YUVmatrixParam + " " + denoiseParam + " " + edgeParam + " " +
         ABLCparam;
  return info;
} // end createPhotoInfo

byte convertPinValueToByteValue(int pinValue, int pinPosition)
{
  byte byteValue = 0;
  if (pinValue == HIGH)
  {
    byteValue = 0x01 << pinPosition;
  }
  return byteValue;
} // end convertPinValueToByteValue

void readPrintFile(String fileName)
{
  File tempFile;

  // Reads in file and prints it to screen via serial monitor.
  tempFile = SD.open(fileName.c_str());
  if (tempFile)
  {
    Serial.print(fileName);
    Serial.println(F(":"));
    // Read from the file until there's nothing else in it.
    while (tempFile.available())
    {
      Serial.write(tempFile.read());
    }
    tempFile.close();
  }
  else
  {
    // Error opening file.
    Serial.print(F("ERROR opening \'"));
    Serial.print(fileName);
    Serial.println(F("\'"));
  }
} // end readPrintFile

void showPhoto()
{
  unsigned long startTime   = 0;
  unsigned long endTime     = 0;
  float elapsedTime         = 0.0;

  startTime = millis();
  captureOV7670Frame();
  showTransmitCapturedFrame();
  endTime = millis();
  elapsedTime = (endTime - startTime)/1000.0; // convert to seconds
  Serial.print(F("Elapsed time for taking and showing photo (sec) = "));
  Serial.println(elapsedTime);
} // end showPhoto

void showTransmitCapturedFrame()
{
  byte pixelData[2] = {0, 0};
  byte pinVal7   = 0,
       pinVal6   = 0,
       pinVal5   = 0,
       pinVal4   = 0,
       pinVal3   = 0,
       pinVal2   = 0,
       pinVal1   = 0,
       pinVal0   = 0;
  unsigned long byteCounter = 0;

  byte Y, U, Y1, V;
  uint8_t R, G, B;
  uint16_t color;

  Serial.println(F("Starting transmission of photo to TFT LCD..."));
  tft.fillScreen(0x001F);
  // Set read buffer pointer to start of frame.
  digitalWrite(RRST, LOW);
  pulsePin(RCLK, 1);
  pulsePin(RCLK, 1);
  pulsePin(RCLK, 1);
  digitalWrite(RRST, HIGH);
  for (int height = 0; height < PHOTO_HEIGHT; height++)
  for (int width = 0; width < PHOTO_WIDTH/2; width++)
  {
    for (int byteno = 0; byteno < PHOTO_BYTES_PER_PIXEL; byteno++)
    {
      // Pulse the read clock RCLK to bring in new byte of data.
      pulsePin(RCLK, 1);
      // Convert pin values to byte values for pins 0-7 of incoming pixel byte.
      pinVal7 = convertPinValueToByteValue(digitalRead(DO7), 7);
      pinVal6 = convertPinValueToByteValue(digitalRead(DO6), 6);
      pinVal5 = convertPinValueToByteValue(digitalRead(DO5), 5);
      pinVal4 = convertPinValueToByteValue(digitalRead(DO4), 4);
      pinVal3 = convertPinValueToByteValue(digitalRead(DO3), 3);
      pinVal2 = convertPinValueToByteValue(digitalRead(DO2), 2);
      pinVal1 = convertPinValueToByteValue(digitalRead(DO1), 1);
      pinVal0 = convertPinValueToByteValue(digitalRead(DO0), 0);
      // Combine individual data from each pin into composite data in the form
      // of a single byte.
      pixelData[byteno] = pinVal7 | pinVal6 | pinVal5 | pinVal4 | pinVal3 |
                          pinVal2 | pinVal1 | pinVal0;
      byteCounter += 1;
    }
    Y = pixelData[0];
    U = pixelData[1];

    for (int byteno = 0; byteno < PHOTO_BYTES_PER_PIXEL; byteno++)
    {
      // Pulse the read clock RCLK to bring in new byte of data.
      pulsePin(RCLK, 1);
      // Convert pin values to byte values for pins 0-7 of incoming pixel byte.
      pinVal7 = convertPinValueToByteValue(digitalRead(DO7), 7);
      pinVal6 = convertPinValueToByteValue(digitalRead(DO6), 6);
      pinVal5 = convertPinValueToByteValue(digitalRead(DO5), 5);
      pinVal4 = convertPinValueToByteValue(digitalRead(DO4), 4);
      pinVal3 = convertPinValueToByteValue(digitalRead(DO3), 3);
      pinVal2 = convertPinValueToByteValue(digitalRead(DO2), 2);
      pinVal1 = convertPinValueToByteValue(digitalRead(DO1), 1);
      pinVal0 = convertPinValueToByteValue(digitalRead(DO0), 0);
      // Combine individual data from each pin into composite data in the form
      // of a single byte.
      pixelData[byteno] = pinVal7 | pinVal6 | pinVal5 | pinVal4 | pinVal3 |
                          pinVal2 | pinVal1 | pinVal0;
      byteCounter += 1;
    }
    Y1 = pixelData[0];
    V  = pixelData[1];

    // Convert YUYV to RGB.
    R = Y + 1.4075 * (V - 128);
    G = Y - 0.3455 * (U - 128) - (0.7169 * (V - 128));
    B = Y + 1.7790 * (U - 128);
    color = tft.color565(R, G, B);
    tft.fillRect(2*width, height, 1, 1, color);  // draw first pixel

    R = Y1 + 1.4075 * (V - 128);
    G = Y1 - 0.3455 * (U - 128) - (0.7169 * (V - 128));
    B = Y1 + 1.7790 * (U - 128);
    color = tft.color565(R, G, B);
    tft.fillRect(2*width+1, height, 1, 1, color);  // draw second pixel    
  }
  Serial.print(F("Total bytes sent to TFT LCD = "));
  Serial.println(byteCounter);
} // end showTransmitCapturedFrame

void sendPhoto()
{
  unsigned long startTime   = 0;
  unsigned long endTime     = 0;
  float elapsedTime         = 0.0;

  startTime = millis();
  captureOV7670Frame();
  sendTransmitCapturedFrame();
  endTime = millis();
  elapsedTime = (endTime - startTime)/1000.0; // convert to seconds
  Serial.print(F("Elapsed time for taking and sending photo (sec) = "));
  Serial.println(elapsedTime);
} // end sendPhoto

void sendTransmitCapturedFrame()
{
  byte pixelData = 0;
  byte pinVal7   = 0,
       pinVal6   = 0,
       pinVal5   = 0,
       pinVal4   = 0,
       pinVal3   = 0,
       pinVal2   = 0,
       pinVal1   = 0,
       pinVal0   = 0;
  unsigned long byteCounter = 0;

  Serial.println(F("Starting transmission of photo to Serial Line..."));
  Serial1.flush();
  // Set read buffer pointer to start of frame.
  digitalWrite(RRST, LOW);
  pulsePin(RCLK, 1);
  pulsePin(RCLK, 1);
  pulsePin(RCLK, 1);
  digitalWrite(RRST, HIGH);
  for (int height = 0; height < PHOTO_HEIGHT; height++)
  for (int width = 0; width < PHOTO_WIDTH; width++)
  for (int byteno = 0; byteno < PHOTO_BYTES_PER_PIXEL; byteno++)
  {
    // Pulse the read clock RCLK to bring in new byte of data.
    pulsePin(RCLK, 1);
    // Convert pin values to byte values for pins 0-7 of incoming pixel byte.
    pinVal7 = convertPinValueToByteValue(digitalRead(DO7), 7);
    pinVal6 = convertPinValueToByteValue(digitalRead(DO6), 6);
    pinVal5 = convertPinValueToByteValue(digitalRead(DO5), 5);
    pinVal4 = convertPinValueToByteValue(digitalRead(DO4), 4);
    pinVal3 = convertPinValueToByteValue(digitalRead(DO3), 3);
    pinVal2 = convertPinValueToByteValue(digitalRead(DO2), 2);
    pinVal1 = convertPinValueToByteValue(digitalRead(DO1), 1);
    pinVal0 = convertPinValueToByteValue(digitalRead(DO0), 0);
    // Combine individual data from each pin into composite data in the form
    // of a single byte.
    pixelData = pinVal7 | pinVal6 | pinVal5 | pinVal4 | pinVal3 | pinVal2 |
                pinVal1 | pinVal0;
    /////////////////////////// Serial Line ////////////////////////////////
    byteCounter += Serial1.write(pixelData);
    ////////////////////////////////////////////////////////////////////
  }
  Serial.print(F("Total bytes sent to Serial Line = "));
  Serial.println(byteCounter);
} // end sendTransmitCapturedFrame

