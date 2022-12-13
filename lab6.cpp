/**********************************************************************************************************************
Course: ROBT1270

Program: Lab6: SCARA Robot Simulator Advanced Control

Purpose: To demonstrate advanced control over the SCARA Robot Simulator by using process abstraction.
Programming methods: formatted I/O, conditional statements, functions, loops, arrays, data parsing.

Author(s): Alejandro Fajardo and Edwing Cheung
A01268695 and A01272795 (respectively)
Both Set A

Declaration: I(We), Alejandro Fajardo and Edwing Cheung, declare that the following program was written by me(us).

Date Created: May 18, 2021.

**********************************************************************************************************************/

#pragma warning(disable : 6011)  // $$$$$$$$$$$$$ DISABLE NONSENSE WARNINGS $$$$$$$$$$$$$$$
#pragma warning(disable : 6001)  // $$$$$$$$$$$$$ DISABLE NONSENSE WARNINGS $$$$$$$$$$$$$$$

//-------------------------- Standard library prototypes --------------------------------------------------------------
#include <stdio.h>  // formatted i/o
#include <math.h>   // math functions
#include <stdlib.h> // system function
#include "robot.h"  // robot functions

CRobot robot;       // the global robot Class.  Can be used everywhere

//---------------------------- Program Constants ----------------------------------------------------------------------
const double PI = 3.14159265358979323846;
const int MAX_POINTS = 81;                // global constant for max available points between 2 points
const int MAX_ARGS = 7;                         // maximum number of command arguments
const size_t MAX_ARG_STRING_LENGTH = 20;        // for sting arguments, i.e., "HIGH", "DOWN", "ON"
const size_t MAX_COMMAND_LENGTH = 256;          // maximum number of characters in command string
const size_t MAX_MESSAGE_LENGTH = 256;          // maximum number of characters in error message string
const size_t MAX_FILENAME_LENGTH = 256;         // maximum number of characters in a filename (includes path)


const int NO_FILE_LINE = 0;  // for parseCommand to differentiate between file and keyboard input

const double L1 = 350.0;                  // length of inner arm
const double L2 = 250.0;                  // length of outer arm
const double MAX_ABS_THETA1_DEG = 150.0;  // maximum shoulder angle (CW or CCW)
const double MAX_ABS_THETA2_DEG = 170.0;  // maximum elbow bend angle (CW or CCW)

// max/min reach of the robot
const double LMAX = L1 + L2;
const double LMIN = sqrt(L1 * L1 + L2 * L2 - 2.0 * L1 * L2 * cos(PI - MAX_ABS_THETA2_DEG * PI / 180.0));

// motor speed constants
const char *STR_MOTOR_SPEED_LOW = "LOW";
const char *STR_MOTOR_SPEED_MEDIUM = "MEDIUM";
const char *STR_MOTOR_SPEED_HIGH = "HIGH";
enum MOTOR_SPEED { MOTOR_SPEED_LOW, MOTOR_SPEED_MEDIUM, MOTOR_SPEED_HIGH };  // enum sets 1st value to 0, 2nd to 1, ..

// object drawing resolution constants (for rectangle, triangle, arc, line) 
const char *STR_RESOLUTION_LOW = "LOW";
const char *STR_RESOLUTION_MEDIUM = "MEDIUM";
const char *STR_RESOLUTION_HIGH = "HIGH";
enum RESOLUTION { RESOLUTION_LOW, RESOLUTION_MEDIUM, RESOLUTION_HIGH };

// pen position constants
const char *STR_PEN_UP = "UP";
const char *STR_PEN_DOWN = "DOWN";
enum PEN_POSITION { PEN_UP, PEN_DOWN };

// cycle pen colors constants
const char *STR_CYCLE_PEN_COLORS_ON = "ON";
const char *STR_CYCLE_PEN_COLORS_OFF = "OFF";
enum CYCLE_PEN_COLORS { CYCLE_PEN_COLORS_ON, CYCLE_PEN_COLORS_OFF };

// limits for colors
int COLOR_MIN = 0;
int COLOR_MAX = 255;

// arm position constants
enum ARM_POSITION { LEFT_ARM, RIGHT_ARM, NO_ARM }; // no arm means can't reach a point.  Used in checkPath.

// index of each SCARA command

enum COMMAND_LIST_INDEX
{
   INDEX_MOTOR_SPEED, INDEX_PEN_POS, INDEX_PEN_COLOR, INDEX_CYCLE_PEN_COLORS, INDEX_CLEAR_TRACE,
   INDEX_CLEAR_REMOTE_COMMAND_LOG, INDEX_CLEAR_POSITION_LOG, INDEX_SHUTDOWN_SIMULATION,
   INDEX_END_REMOTE_CONNECTION, INDEX_HOME, INDEX_MOVE_TO, INDEX_DRAW_LINE, INDEX_DRAW_ARC,
   INDEX_DRAW_RECTANGLE, INDEX_DRAW_TRIANGLE, INDEX_ADD_ROTATION, INDEX_ADD_TRANSLATION, INDEX_ADD_SCALING,
   INDEX_RESET_TRANSFORMATION_MATRIX, INDEX_QUERY_STATE, NUM_COMMANDS
};
const int NUM_SCARA_COMMANDS = NUM_COMMANDS; // number of abstracted SCARA commands.  *** NOTE THE TRICK TO GET THIS

enum INPUT_MODE { KEYBOARD_INPUT, FILE_INPUT };  // for users choice

//----------------------------- Program Structures --------------------------------------------------------------------

typedef struct RGB_COLOR // for describing pen RGB color
{
   int r, g, b;   // red, green, blue components (0-255 each)
}
RGB_COLOR;

typedef struct SCARA_POSITION  // to store the current position of the pen.  Note that armPos is redundant.
{
   double x, y, theta1Deg, theta2Deg;
   int armPos;
}
SCARA_POSITION;

typedef struct SCARA_STATE  // used to store the current state of the robot.
{
   SCARA_POSITION currentPos;
   int motorSpeed, penPos, cyclePenColors;
   RGB_COLOR penColor;
}
SCARA_STATE;

// encapsulates both x and y for forward solution 
// forward solution is NEVER TRANSFORMED!!
typedef struct FORWARD_SOLUTION
{
   double x, y;
   bool bHasSolution;
}
FORWARD_SOLUTION;

// encapsulates both right/left arm angles for inverse solution
typedef struct INVERSE_SOLUTION
{
   double theta1DegLeft, theta1DegRight;  // right arm solution
   double theta2DegLeft, theta2DegRight;  // left arm solution
   bool bLeft, bRight;                    // true if has solution
}
INVERSE_SOLUTION;

// a union is used to save space.   
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!  ONLY ONE PARAMETER CAN BE USED AT A TIME BECAUSE THE MEMORY IS SHARED !!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
typedef union COMMAND_ARGUMENT
{
   char strValue[MAX_ARG_STRING_LENGTH];  // to store values for string parameters (like "HIGH", "MEDIUM", "LOW")
   double dValue; // to store floating point values
   int iValue;    // to store integer values
}
COMMAND_ARGUMENT;

// struct to hold data for abstracted commands like drawRectangle
typedef struct SCARA_COMMAND
{
   const char *cmdName;       // name of the command.  Pointer points at hardcoded string constant.
   const char *strArgs;       // names of all arguments.  Pointer points at hardcoded string constant.
   int nArgs;                 // number of input arguments for the command
   COMMAND_ARGUMENT *args;    // dynamic array used to store all the argument values.  Note: must use malloc 
}
SCARA_COMMAND;

// struct to hold info related for drawing an arc
typedef struct ARC_INFO
{
   double thetaARad, thetaBRad, Xc, Yc, R;
   RGB_COLOR rgb;
}ARC_INFO;


//structure that holds the line information of a straightline
typedef struct LINE_INFO
{
   double X0, Y0, X1, Y1;
   RGB_COLOR rgb;
   int N;

}LINE_INFO;


//----------------------------- Local Function Prototypes -------------------------------------------------------------
bool flushInputBuffer();            // flushes any characters left in the standard input buffer
void waitForEnterKey();             // waits for the Enter key to be pressed
double degToRad(double);            // returns angle in radians from input angle in degrees
double radToDeg(double);            // returns angle in degrees from input angle in radians
void closeAndExit(const char *);    // prints a message and closes program
int nint(double d);                 // find int nearest to double
char *makeUpper(char *);            // changes a string to all upper case
bool isBlankLine(const char *);     // checks if a string is composed entiredly of whitespace characters
bool isCommentLine(const char *);   // check if a string is considered a comment string
void pauseAndClearRobotAndConsole();      // asks the user to press ENTER and then clears the robot interface
void resetTransformMatrix(double TM[][3]);                     // resets the transform matrix to the identity matrix
void transformMatrixMultiply(double TM[][3], double M[][3]);    // premultiplies the transform matrix TM by matrix M
FORWARD_SOLUTION forwardKinematics(double, double);  // implements forward kinematics
int getN(double len, int resolution);  // number of points to use for a line for drawLine or arc for drawArc.
bool initSCARAcommands(SCARA_COMMAND *cmdList);  //initiate all the commands for the robot assigning values, names...
int getDataInputMode();                // get an input from the user so the program know where is it gonna get the data
int parseCommand(char *strCommand, SCARA_COMMAND *cmdList, char *strErrorMsg, int lineNumber); //Compare the input com
void freeDynamicMemory(SCARA_COMMAND *);   // will free all the dynamic memery before closing the pogram
void help(SCARA_COMMAND *);      // function that will print all SCARA COMMANDS, arguments, and any needed info
void runKeyboardCommands(SCARA_COMMAND *cmdList, SCARA_STATE *state, double transformMatrix[3][3]);      //fun keyboard
void runFileCommands(SCARA_COMMAND *cmdList, SCARA_STATE *state, double transformMatrix[3][3]); //runcommands from a file
void executeCommand(SCARA_COMMAND *cmdList, SCARA_STATE *state, int index, double transformMatrix[3][3]);  //commds exe
void drawArc(SCARA_COMMAND *cmdList, int index, double transformMatrix[3][3], SCARA_STATE *state);//calc starting/end
void drawStraightLine(SCARA_COMMAND *cmdList, int index, double transformMatrix[3][3], SCARA_STATE *state);//for line
INVERSE_SOLUTION inverseKinematics(double, double, double transformMatrix[3][3]);          // funtion for inverseKinem
bool checkPad(double, double);     //will check that a full pad can be draw prior to the drawing. 

//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------
// demonstrates advanced control of the robot simulator using all facets of C learned in the course
// INPUTS: none
// RETURN: an integer - signals to the O/S how the program terminated.
int main()
{
   // open connection with robot
   if(!robot.Initialize()) return 0;

   int dataInputMode; // stores the input mode (keyboard or file)

   SCARA_COMMAND cmdList[NUM_SCARA_COMMANDS] = {}; // holds the list of all abstracted SCARA command

   // current state of the robot (position, pen, and motor states).
   SCARA_STATE state = {600.0, 0.0, 0.0, 0.0, LEFT_ARM, CYCLE_PEN_COLORS_OFF, MOTOR_SPEED_MEDIUM, 255, 0, 0, PEN_DOWN};

   // all points sent to inverseKinematics will be transformed using transformMatrix BEFORE 
   // the motor angle values are calculated
   double transformMatrix[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};

   // initialize the list of commands and associated data
   if(!initSCARAcommands(cmdList)) closeAndExit("Can't initialize the SCARA command list!");
   dataInputMode = getDataInputMode(); // ask the user if they want to get commands from the keyboard or a file

   if(dataInputMode == KEYBOARD_INPUT)

      runKeyboardCommands(cmdList, &state, transformMatrix); // get/run commands interactively from the keyboard
   else
      runFileCommands(cmdList, &state, transformMatrix); // get/run commands from a specified file

   freeDynamicMemory(cmdList); // free memory allocated inside the cmdList array.
   closeAndExit("Thanks for playing!"); // that's all folks!
}




//---------------------------------------------------------------------------------------------------------------------
// This function processes a user-inputted or file-read command string, i.e., "moveTo 300.0 400.0\n". then is tokenized 
// to extract the command name (i.e., "moveTo") and the command arguments (i.e.,"300.0"  "400.0"). If name isnt found
// in the cmdList array or all arguments are not valid, the function formats an error message and returns -1. The error 
// message must contain the file line number if the command was read from a file.  If all arguments are valid, their
//  values are stored in the appropriate element of the cmdList array.
// INPUTS: strCommand - The user-inputted or file-read command string
//         cmdList - The array of SCARA_COMMAND structures.
//         strError - a pointer to a string used to store an error message
//         lineNumber - the line number of the command string if read from a file or -1 if user-inputted
// RETURN: the index of the command in cmdList, or -1 if the command and/or arguments was not valid
int parseCommand(char *strCommand, SCARA_COMMAND *cmdList, char *strErrorMsg, int lineNumber)
{
   int index = -1;  // stores the index of the command from cmdList if found and has valid argument data
   char *tok = NULL, *nextTok = NULL;  // for tokenizing strCommand
   const char *seps = " \t\n\r,;:\\/_"; // for tokenzing strCommand.  Possibly delimeters (can be altered if desired)
   RGB_COLOR RGBcol = {}; // structure that will store the color of the tok.
   char *pGarbage = NULL;  //  will store if there is any trailing garbage


   // tokenizing the first command and compare it with the Scara Commands
   tok = strtok_s(strCommand, seps, &nextTok); // tokenize the command name (i.e., "moveTo")
   if(tok != NULL)  // if got something, search for matching command in cmdList (case-insenstive match!)
   {
      for(index = 0; index < NUM_COMMANDS; index++)
      {
         if(_stricmp(tok, cmdList[index].cmdName) == 0) break;
      }
   }

   if(tok == NULL || index == NUM_COMMANDS)  // command not found, format error message and return
   {
      if(lineNumber == -1) // user-inputted command
         sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH, "%s is not a valid command", tok);
      else // file-read command
         sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH, "%s is not a valid command (line %d)", tok, lineNumber);

      return -1;
   }

   // command name ok.  Process argument values using a swtich based on the index.   
   // Must be done individually because number of arguments, argument types, and argument limits are
   // generally different for each argument (some can be grouped, i.e., those with no arguments)

   char storeArg1[MAX_ARG_STRING_LENGTH];    //store the value of arg 1
   int i;   // variable for a counter

   switch(index)
   {

      // first case motor speed
   case INDEX_MOTOR_SPEED:

      tok = strtok_s(NULL, seps, &nextTok);  // get the one and only argument
      // if argument not found or doesn't match the required text, format error message and return
      if(tok == NULL ||
         _stricmp(tok, STR_MOTOR_SPEED_LOW) != 0 &&   // use exisiting global constants!!!
         _stricmp(tok, STR_MOTOR_SPEED_MEDIUM) != 0 &&
         _stricmp(tok, STR_MOTOR_SPEED_HIGH) != 0)
      {
         sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
            "expecting %d parameter(s).  Should be: %s",
            cmdList[index].nArgs, cmdList[index].strArgs);  // strArgs should contain insightful text.
         return -1;
      }

      strcpy_s(storeArg1, MAX_ARG_STRING_LENGTH, tok);
      // checking for no extra arguments
      tok = strtok_s(NULL, seps, &nextTok);
      if(tok != NULL)
      {
         sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH, "expecting %d parameter(s), you have entered more",
            cmdList[index].nArgs);
         return -1;
      }

      //if everything okay, we can send to main the commands
      strcpy_s(cmdList[index].args[0].strValue, MAX_ARG_STRING_LENGTH, storeArg1);
      break;

      // argument is valid, so store it's value in the approriate element of cmdList.  Note that args is 
      // an array of COMMAND_ARGUMENT unions, so you need to store the argument value in the appropriate 
      // spot in that array and in the appropriate element of args.  Here the single argument is a string
      // therefore it is stored in the strValue union member.  If it was an int, you would need to convert
      // the token to an int (i.e., using strtol) and store the resulting int in the iValue union member.
      // In the cases of ints and doubles, they must be garbage-free and in the valid range (i.e., pen color 
      // components must be in the range 0-255.
      // check for pen pos it should have just one parameter
   case INDEX_PEN_POS:
      tok = strtok_s(NULL, seps, &nextTok);  // get only one argument for penPos either UP or DOWN

   // if argument not found or doesn't match the required text, format error message and return
      if(tok == NULL ||
         _stricmp(tok, STR_PEN_DOWN) != 0 &&   // use exisiting global constants!!!
         _stricmp(tok, STR_PEN_UP) != 0)
      {
         sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
            "expecting %d parameter(s).  Should be: %s",
            cmdList[index].nArgs, cmdList[index].strArgs);  // strArgs should contain insightful text.
         return -1;
      }
      strcpy_s(storeArg1, MAX_ARG_STRING_LENGTH, tok);
      // checking for no extra arguments
      tok = strtok_s(NULL, seps, &nextTok);
      if(tok != NULL)
      {
         sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH, "expecting %d parameter(s), you have entered more",
            cmdList[index].nArgs);
         return -1;
      }
      //if everything okay, we can send to main the commands
      strcpy_s(cmdList[index].args[0].strValue, MAX_ARG_STRING_LENGTH, storeArg1);
      break;

   case INDEX_PEN_COLOR:
      // create a loop that will tokenize each remaining argument and will check its free of garbage and its complete
      for(i = 0; i < 3; i++)
      {
         tok = strtok_s(NULL, seps, &nextTok);  // get only one argument for penPos either UP or DOWN
         if(tok == NULL)
         {
            sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
               "expecting %d parameter(s).  Should be: %s",
               cmdList[index].nArgs, cmdList[index].strArgs);  // strArgs should contain insightful text.
            return -1;
         }

         cmdList[index].args[i].iValue = (int)strtol(tok, &pGarbage, 10);
         if(*pGarbage != '\0')
         {

            sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
               "You have entered trailing garbage, try again with no garbage this time");
            return -1;
         }
         if(cmdList[index].args[i].iValue < COLOR_MIN || cmdList[index].args[i].iValue > COLOR_MAX)   //glob const
         {
            printf("Sorry values most be between %d and %d, try again", COLOR_MIN, COLOR_MAX);
            return -1;
         }
      }

      // check for no extra arguments
      tok = strtok_s(NULL, seps, &nextTok);
      if(tok != NULL)
      {
         sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
            "You have entered extra parameters, try again");
         return -1;
      }

      break;

   case INDEX_CYCLE_PEN_COLORS:
      tok = strtok_s(NULL, seps, &nextTok);  // get only one argument for penPos either ON or OFF
      if(tok == NULL ||
         _stricmp(tok, STR_CYCLE_PEN_COLORS_ON) != 0 &&   // use exisiting global constants!!!
         _stricmp(tok, STR_CYCLE_PEN_COLORS_OFF) != 0)
      {
         sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
            "expecting %d parameter(s).  Should be: %s",
            cmdList[index].nArgs, cmdList[index].strArgs);  // strArgs should contain insightful text.
         return -1;
      }

      strcpy_s(storeArg1, MAX_ARG_STRING_LENGTH, tok);
      // checking for no extra arguments
      tok = strtok_s(NULL, seps, &nextTok);
      if(tok != NULL)
      {
         sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH, "expecting %d parameter(s), you have entered more",
            cmdList[index].nArgs);
         return -1;
      }

      //if everything okay, we can send to main the commands
      strcpy_s(cmdList[index].args[0].strValue, MAX_ARG_STRING_LENGTH, storeArg1);
      break;

   case INDEX_CLEAR_TRACE:
      tok = strtok_s(NULL, seps, &nextTok);  // not getting any nextTok since the function has no args
      if(tok != NULL)
      {
         sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
            "expecting no parameter(s), you have entered more");  // strArgs should contain insightful text.
         return -1;
      }
      break;

   case INDEX_CLEAR_REMOTE_COMMAND_LOG:
      tok = strtok_s(NULL, seps, &nextTok);  // not getting any nextTok since the function has no args
      if(tok != NULL)
      {
         sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
            "expecting no parameter(s), you have entered more");  // strArgs should contain insightful text.
         return -1;
      }
      break;

   case INDEX_CLEAR_POSITION_LOG:
      tok = strtok_s(NULL, seps, &nextTok);  // not getting any nextTok since the function has no args
      if(tok != NULL)
      {
         sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
            "expecting no parameter(s), you have entered more");  // strArgs should contain insightful text.
         return -1;
      }
      break;

   case INDEX_SHUTDOWN_SIMULATION:
      tok = strtok_s(NULL, seps, &nextTok);  // not getting any nextTok since the function has no args
      if(tok != NULL)
      {
         sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
            "expecting no parameter(s), you have entered more");  // strArgs should contain insightful text.
         return -1;
      }
      break;

   case INDEX_END_REMOTE_CONNECTION:
      tok = strtok_s(NULL, seps, &nextTok);  // not getting any nextTok since the function has no args
      if(tok != NULL)
      {
         sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
            "expecting no parameter(s), you have entered more");  // strArgs should contain insightful text.
         return -1;
      }
      break;

   case INDEX_HOME:
      tok = strtok_s(NULL, seps, &nextTok);  // not getting any nextTok since the function has no args
      if(tok != NULL)
      {
         sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
            "expecting no parameter(s), you have entered more");  // strArgs should contain insightful text.
         return -1;
      }
      break;

   case INDEX_MOVE_TO:
      // create a loop that will tokenize each remaining argument and will check its free of garbage and its complete
      for(i = 0; i < 2; i++)
      {
         tok = strtok_s(NULL, seps, &nextTok);  // get only one argument for penPos either UP or DOWN
         if(tok == NULL)
         {
            sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
               "expecting %d parameter(s).  Should be: %s",
               cmdList[index].nArgs, cmdList[index].strArgs);  // strArgs should contain insightful text.
            return -1;
         }
         cmdList[index].args[i].dValue = strtod(tok, &pGarbage);
         if(*pGarbage != '\0')
         {
            sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
               "You have entered trailing garbage, try again with no garbage this time");
            return -1;
         }
      }
      // check for no extra arguments
      tok = strtok_s(NULL, seps, &nextTok);
      if(tok != NULL)
      {
         sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
            "You have entered extra parameters, try again");
         return -1;
      }
      break;

   case INDEX_DRAW_LINE:
      // create a loop that will tokenize each remaining argument and will check its free of garbage and its complete
      for(i = 0; i < 4; i++)
      {
         tok = strtok_s(NULL, seps, &nextTok);  // get only one argument for penPos either UP or DOWN
         if(tok == NULL)
         {
            sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
               "expecting %d parameter(s).  Should be: %s",
               cmdList[index].nArgs, cmdList[index].strArgs);  // strArgs should contain insightful text.
            return -1;
         }

         cmdList[index].args[i].dValue = strtod(tok, &pGarbage);
         if(*pGarbage != '\0')
         {
            sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
               "You have entered trailing garbage, try again with no garbage this time");
            return -1;
         }

      }

      // Checking for resolutions
      tok = strtok_s(NULL, seps, &nextTok);
      if(tok == NULL)
      {
         sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH, "expecting Resolution: %s or %s or %s",
            STR_RESOLUTION_HIGH, STR_RESOLUTION_MEDIUM, STR_RESOLUTION_LOW);   // global constantes
         return -1;
      }

      if(_stricmp(tok, STR_MOTOR_SPEED_HIGH) != 0 && _stricmp(tok, STR_MOTOR_SPEED_MEDIUM) != 0 &&
         _stricmp(tok, STR_MOTOR_SPEED_LOW) != 0)
      {
         sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH, "expecting Resolution: %s or %s or %s",
            STR_RESOLUTION_HIGH, STR_RESOLUTION_MEDIUM, STR_RESOLUTION_LOW);
         return -1;
      }

      strcpy_s(cmdList[index].args[i].strValue, MAX_ARG_STRING_LENGTH, tok);

      // check for no extra arguments
      tok = strtok_s(NULL, seps, &nextTok);
      if(tok != NULL)
      {
         sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
            "You have entered extra parameters, try again");
         return -1;
      }
      break;

   case INDEX_DRAW_ARC:
      // create a loop that will tokenize each remaining argument and will check its free of garbage and its complete
      for(i = 0; i < 5; i++)
      {
         tok = strtok_s(NULL, seps, &nextTok);  // get only one argument for penPos either UP or DOWN
         if(tok == NULL)
         {
            sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
               "expecting %d parameter(s).  Should be: %s",
               cmdList[index].nArgs, cmdList[index].strArgs);  // strArgs should contain insightful text.
            return -1;
         }

         cmdList[index].args[i].dValue = strtod(tok, &pGarbage);
         if(*pGarbage != '\0')
         {
            sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
               "You have entered trailing garbage, try again with no garbage this time");
            return -1;
         }
      }

      // Checking for resolutions
      tok = strtok_s(NULL, seps, &nextTok);
      if(tok == NULL)
      {
         sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH, "expecting Resolution: %s or %s or %s",
            STR_RESOLUTION_HIGH, STR_RESOLUTION_MEDIUM, STR_RESOLUTION_LOW);   // global constantes
         return -1;
      }

      if(_stricmp(tok, STR_MOTOR_SPEED_HIGH) != 0 && _stricmp(tok, STR_MOTOR_SPEED_MEDIUM) != 0 &&
         _stricmp(tok, STR_MOTOR_SPEED_LOW) != 0)
      {
         sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH, "expecting Resolution: %s or %s or %s",
            STR_RESOLUTION_HIGH, STR_RESOLUTION_MEDIUM, STR_RESOLUTION_LOW);
         return -1;
      }
      strcpy_s(cmdList[index].args[5].strValue, tok);

      // check for no extra arguments
      tok = strtok_s(NULL, seps, &nextTok);
      if(tok != NULL)
      {
         sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
            "You have entered extra parameters, try again");
         return -1;
      }
      break;

   case INDEX_DRAW_RECTANGLE:
      // create a loop that will tokenize each remaining argument and will check its free of garbage and its complete
      for(i = 0; i < 4; i++)
      {
         tok = strtok_s(NULL, seps, &nextTok);  // get only one argument for penPos either UP or DOWN
         if(tok == NULL)
         {
            sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
               "expecting %d parameter(s).  Should be: %s",
               cmdList[index].nArgs, cmdList[index].strArgs);  // strArgs should contain insightful text.
            return -1;
         }

         cmdList[index].args[i].dValue = strtod(tok, &pGarbage);
         if(*pGarbage != '\0')
         {
            sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
               "You have entered trailing garbage, try again with no garbage this time");
            return -1;
         }
      }

      // Checking for resolutions
      tok = strtok_s(NULL, seps, &nextTok);
      if(tok == NULL)
      {
         sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH, "expecting Resolution: %s or %s or %s",
            STR_RESOLUTION_HIGH, STR_RESOLUTION_MEDIUM, STR_RESOLUTION_LOW);   // global constantes
         return -1;
      }

      if(_stricmp(tok, STR_MOTOR_SPEED_HIGH) != 0 && _stricmp(tok, STR_MOTOR_SPEED_MEDIUM) != 0 &&
         _stricmp(tok, STR_MOTOR_SPEED_LOW) != 0)
      {
         sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH, "expecting Resolution: %s or %s or %s",
            STR_RESOLUTION_HIGH, STR_RESOLUTION_MEDIUM, STR_RESOLUTION_LOW);
         return -1;
      }
      strcpy_s(cmdList[index].args[4].strValue, tok);


      // check for no extra arguments
      tok = strtok_s(NULL, seps, &nextTok);
      if(tok != NULL)
      {
         sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
            "You have entered extra parameters, try again");
         return -1;
      }
      break;

   case INDEX_DRAW_TRIANGLE:
      // create a loop that will tokenize each remaining argument and will check its free of garbage and its complete
      for(i = 0; i < 6; i++)
      {
         tok = strtok_s(NULL, seps, &nextTok);  // get only one argument for penPos either UP or DOWN
         if(tok == NULL)
         {
            sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
               "expecting %d parameter(s).  Should be: %s",
               cmdList[index].nArgs, cmdList[index].strArgs);  // strArgs should contain insightful text.
            return -1;
         }

         cmdList[index].args[i].dValue = strtod(tok, &pGarbage);
         if(*pGarbage != '\0')
         {
            sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
               "You have entered trailing garbage, try again with no garbage this time");
            return -1;
         }
      }

      // Checking for resolutions
      tok = strtok_s(NULL, seps, &nextTok);
      if(tok == NULL)
      {
         sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH, "expecting Resolution: %s or %s or %s",
            STR_RESOLUTION_HIGH, STR_RESOLUTION_MEDIUM, STR_RESOLUTION_LOW);   // global constantes
         return -1;
      }
      if(_stricmp(tok, STR_MOTOR_SPEED_HIGH) != 0 && _stricmp(tok, STR_MOTOR_SPEED_MEDIUM) != 0 &&
         _stricmp(tok, STR_MOTOR_SPEED_LOW) != 0)
      {
         sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH, "expecting Resolution: %s or %s or %s",
            STR_RESOLUTION_HIGH, STR_RESOLUTION_MEDIUM, STR_RESOLUTION_LOW);
         return -1;
      }
      strcpy_s(cmdList[index].args[6].strValue, tok);

      // check for no extra arguments
      tok = strtok_s(NULL, seps, &nextTok);
      if(tok != NULL)
      {
         sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
            "You have entered extra parameters, try again");
         return -1;
      }
      break;

   case INDEX_ADD_ROTATION:
      // create a loop that will tokenize each remaining argument and will check its free of garbage and its complete
      tok = strtok_s(NULL, seps, &nextTok);  // get only one argument for penPos either UP or DOWN
      if(tok == NULL)
      {
         sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
            "expecting %d parameter(s).  Should be: %s",
            cmdList[index].nArgs, cmdList[index].strArgs);  // strArgs should contain insightful text.
         return -1;
      }
      cmdList[index].args[0].dValue = strtod(tok, &pGarbage);
      if(*pGarbage != '\0')
      {
         sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
            "You have entered trailing garbage, try again with no garbage this time");
         return -1;
      }

      // check for no extra arguments
      tok = strtok_s(NULL, seps, &nextTok);
      if(tok != NULL)
      {
         sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
            "You have entered extra parameters, try again");
         return -1;
      }
      break;

   case INDEX_ADD_TRANSLATION:
      // create a loop that will tokenize each remaining argument and will check its free of garbage and its complete
      for(i = 0; i < 2; i++)
      {
         tok = strtok_s(NULL, seps, &nextTok);  // get only one argument for penPos either UP or DOWN
         if(tok == NULL)
         {
            sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
               "expecting %d parameter(s).  Should be: %s",
               cmdList[index].nArgs, cmdList[index].strArgs);  // strArgs should contain insightful text.
            return -1;
         }
         cmdList[index].args[i].dValue = strtod(tok, &pGarbage);
         if(*pGarbage != '\0')
         {
            sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
               "You have entered trailing garbage, try again with no garbage this time");
            return -1;
         }
      }

      // check for no extra arguments
      tok = strtok_s(NULL, seps, &nextTok);
      if(tok != NULL)
      {
         sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
            "You have entered extra parameters, try again");
         return -1;
      }
      break;

   case INDEX_ADD_SCALING:
      // create a loop that will tokenize each remaining argument and will check its free of garbage and its complete
      for(i = 0; i < 2; i++)
      {
         tok = strtok_s(NULL, seps, &nextTok);  // get only one argument for penPos either UP or DOWN
         if(tok == NULL)
         {
            sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
               "expecting %d parameter(s).  Should be: %s",
               cmdList[index].nArgs, cmdList[index].strArgs);  // strArgs should contain insightful text.
            return -1;
         }

         cmdList[index].args[i].dValue = strtod(tok, &pGarbage);
         if(*pGarbage != '\0')
         {
            sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
               "You have entered trailing garbage, try again with no garbage this time");
            return -1;
         }

      }

      // check for no extra arguments
      tok = strtok_s(NULL, seps, &nextTok);
      if(tok != NULL)
      {
         sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
            "You have entered extra parameters, try again");
         return -1;
      }

      break;

   case INDEX_RESET_TRANSFORMATION_MATRIX:
      tok = strtok_s(NULL, seps, &nextTok);  // shouldnt get any nextTok since the function has no args
      if(tok != NULL)
      {
         sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
            "expecting no parameter(s), you have entered more");  // strArgs should contain insightful text.
         return -1;
      }
      break;

   case INDEX_QUERY_STATE:
      tok = strtok_s(NULL, seps, &nextTok);  // shouldnt get any nextTok since the function has no args
      if(tok != NULL)
      {
         sprintf_s(strErrorMsg, MAX_MESSAGE_LENGTH,
            "expecting no parameter(s), you have entered more");  // strArgs should contain insightful text.
         return -1;
      }
      break;
   }

   return index;  // command is valid and has valid arguments so return the index
}


//---------------------------------------------------------------------------------------------------------------------
// Resets the tranform matrix to the unit matrix.  x, y points will no longer be transformed in inverseKinematics
// INPUTS:  the 3x3 transform matrix
// RETURN:  nothing
void resetTransformMatrix(double TM[][3])  // resets to unit matrix
{
   int r, c;  // matrix row, column indexes

   for(r = 0; r < 3; r++)
   {
      for(c = 0; c < 3; c++)
      {
         TM[r][c] = (r == c ? 1.0 : 0.0);  // same as:  if(r==c) 
      }                                    //               TM[r][c] = 1.0;
   }                                       //           else
}                                          //               TM[r][c] = 0.0;

//---------------------------------------------------------------------------------------------------------------------
// Premultiplies the transform matrix by matrix M.  M is normally the rotation mat, translation mat, or the scaling mat
// INPUTS:  TM: the 3x3 transform matrix, M the premultiplier matrix
// RETURN:  nothing
void transformMatrixMultiply(double TM[][3], double M[][3])
{
   int r, c, cc;     // row, column indexes
   double TMM[3][3]; // temp matrix

   for(r = 0; r < 3; r++)
   {
      for(c = 0; c < 3; c++)
      {
         TMM[r][c] = 0.0;  // set element to zero

         for(cc = 0; cc < 3; cc++) // accumulate the multiples
         {
            TMM[r][c] += M[r][cc] * TM[cc][c];
         }
      }
   }

   for(r = 0; r < 3; r++)  // copy temp matrix to TM
   {
      for(c = 0; c < 3; c++)
      {
         TM[r][c] = TMM[r][c];
      }
   }
}


//---------------------------------------------------------------------------------------------------------------------
// Calculates the number of points to be drawn using drawLine or drawArc.  Number is based on length of line/arc- resol
// INPUTS:  len:  the length of the line or arc to be drawn. resolution: integer parameter to control the point spacing
// RETURN:  the number of points
int getN(double len, int resolution)
{
   int N; // calculated number of points

   if(resolution == RESOLUTION_HIGH)
      N = nint(40.0 * (double)len / 600.0);        // i.e., 80 points across the workspace
   else if(resolution == RESOLUTION_MEDIUM)
      N = nint(20.0 * (double)len / 600.0);        // i.e., 40 points across the workspace
   else
      N = nint(10.0 * (double)len / 600.0);        // i.e., 20 points across the workspace

   return N;
}


//---------------------------------------------------------------------------------------------------------------------
// // Calculates the integer value nearest to a given floating point value (i.e., 6.2 would give 6, -7.9 would give -8)
// INPUTS:  d:  the double
// RETURN:  the nearest integer
int nint(double d)
{
   return (int)floor(d + 0.5);
}

//---------------------------------------------------------------------------------------------------------------------
// Sets the robot to its starting state
// INPUTS:  none
// RETURN:  none
//---------------------------------------------------------------------------------------------------------------------
void pauseAndClearRobotAndConsole()
{
   printf("Press ENTER to continue...");
   waitForEnterKey();
   system("cls");
   robot.Send("MOTOR_SPEED HIGH\n");
   robot.Send("HOME\n");
   robot.Send("CLEAR_TRACE\n");
   robot.Send("CLEAR_POSITION_LOG\n");
   robot.Send("CLEAR_REMOTE_COMMAND_LOG\n");  // must be last or all unprocessed commands are lost!
}

//---------------------------------------------------------------------------------------------------------------------
// This function computes the SCARA forward solution for a given pair of joint angles.
// INPUTS:  The joint angles theta1Deg and theta2Deg (in DEGREES!!!!!!!!)
// RETURN:  an FORWARD_SOLUTION structure containing coordinate position x,y corresponding
//          to the input joint angles
FORWARD_SOLUTION forwardKinematics(double theta1Deg, double theta2Deg)
{
   FORWARD_SOLUTION fsol = {0.0, 0.0, true};

   if(fabs(theta1Deg) > MAX_ABS_THETA1_DEG) fsol.bHasSolution = false;
   if(fabs(theta2Deg) > MAX_ABS_THETA2_DEG) fsol.bHasSolution = false;

   if(fsol.bHasSolution)
   {
      double theta1 = degToRad(theta1Deg), theta2 = degToRad(theta2Deg);
      fsol.x = L1 * cos(theta1) + L2 * cos(theta1 + theta2);
      fsol.y = L1 * sin(theta1) + L2 * sin(theta1 + theta2);
   }

   return fsol;
}

//---------------------------------------------------------------------------------------------------------------------
// Checks if a string (normally got from a line in a file) contains only whitespace
// INPUTS:  strLine:  the string
// RETURN:  true strLine only contains whitespace, false if not.
bool isBlankLine(const char *strLine)
{
   size_t n, len = strlen(strLine);  // n = temp index.  len = # of characters in the string
   char c;

   for(n = 0; n < len; n++)
   {
      c = strLine[n];
      if(c != ' ' && c != '\t' && c != '\n' && c != '\r') return false;  // if non-whitespace found, return false
   }

   return true;  // no whitespace characters found!
}

//---------------------------------------------------------------------------------------------------------------------
// Checks if a string (normally got from a line in a file) is meant to be a comment string.  Comment lines start 
// with // or \\.
// INPUTS:  strLine:  the string
// RETURN:  true if considered a comment string, false if not.
bool isCommentLine(const char *strLine)
{
   size_t n, len = strlen(strLine);
   char c;

   for(n = 0; n < len; n++)
   {
      c = strLine[n];
      if(c == ' ' || c == '\t' || c == '\n' || c == '\r') continue;  // skip leading whitespace

      if((c == '\\' || c == '/') && n == len - 1) // note that '\\' is c for single '\'
         return false;  // only one possible because at end of the string (special case)
      else if((c == '\\' || c == '/') && (strLine[n + 1] == '\\' || strLine[n + 1] == '/'))
         return true; // it's a comment line!
   }
   return false; // if here than line doesn't start with \\ or //
}

//---------------------------------------------------------------------------------------------------------------------
// Turns an entire input string to upper case characters
// INPUTS:  str:  the string
// RETURN:  the address of the string (useful for strcpy_s, strcmp, strcat_s functions)
char *makeUpper(char *str)
{
   size_t n, len;

   len = strlen(str);

   for(n = 0; n < len; n++)
   {
      str[n] = (char)toupper(str[n]);
   }

   return str;
}

//---------------------------------------------------------------------------------------------------------------------
// Converts an angle in degrees to radians
// INPUTS:  angDeg: the angle in degrees
// RETURN:  the angle in radians
double degToRad(double angDeg)
{
   return (PI / 180.0) * angDeg;
}

//---------------------------------------------------------------------------------------------------------------------
// Converts an angle in radians to degrees
// INPUTS:  angRad: the angle in radian
// RETURN:  the angle in degrees
double radToDeg(double angRad)
{
   return (180.0 / PI) * angRad;
}

//---------------------------------------------------------------------------------------------------------------------
// Closes the robot and the program.  Outputs a message to the console and asks the user to press ENTER before closing.
// INPUTS:  message:  the message string
// RETURN:  none
void closeAndExit(const char *message)
{
   printf("\n%s\n", message);
   printf("Press ENTER to end this program...");
   waitForEnterKey();
   robot.Close(); // close remote connection
   exit(0);  // exit terminates a console program immediately
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  This function flushes the input buffer to avoid scanf issues
// ARGUMENTS:    none
// RETURN VALUE: false if nothing or only '\n' in stdin. true if extra keystrokes precede the '\n'.
//               Good for detecting left over garbage from scanf_s in the input buffer
bool flushInputBuffer()
{
   unsigned char ch; // temp character variable
   bool bHasGarbage = false;

   // exit loop when all characters are flushed
   while((ch = (unsigned char)getchar()) != '\n' && ch != EOF)
   {
      if(!bHasGarbage) bHasGarbage = true;
   }
   return bHasGarbage;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Waits for user to press enter.  flushes stdin if keystrokes precede enter
// ARGUMENTS:    none
// RETURN VALUE: none
void waitForEnterKey()
{
   unsigned char ch;
   if((ch = (unsigned char)getchar()) != EOF && ch != '\n') flushInputBuffer();
}

//---------------------------------------------------------------------------------------------------------------------
// Initialize the array of SCARA_COMMAND structures.  The array elements each hold the command name, argument 
// descriptions, and an array of COMMAND_ARGUMENT unions used to store argument values for the command
// INPUTS: cmdList:  The array of SCARA_COMMAND structures.
// RETURN: true if all argument memory was allocated, false if not
bool initSCARAcommands(SCARA_COMMAND *cmdList)
{
   int n; //variable to store the number of arguments and then use it to compute dynamic memory allocation

   // SCARA_COMMAND_0 motorSpeed:
   cmdList[INDEX_MOTOR_SPEED].cmdName = "motorSpeed";
   cmdList[INDEX_MOTOR_SPEED].strArgs = "Arg that should be either HIGH / MEDIUM / LOW";
   n = cmdList[INDEX_MOTOR_SPEED].nArgs = 1;
   cmdList[INDEX_MOTOR_SPEED].args = (COMMAND_ARGUMENT *)malloc(n * sizeof(COMMAND_ARGUMENT));
   if(cmdList[INDEX_MOTOR_SPEED].args == NULL) return false;

   // SCARA_COMMAND_1 penPos:
   cmdList[INDEX_PEN_POS].cmdName = "penPos";
   cmdList[INDEX_PEN_POS].strArgs = "Arg that should be either UP / DOWN";
   n = cmdList[INDEX_PEN_POS].nArgs = 1;
   cmdList[INDEX_PEN_POS].args = (COMMAND_ARGUMENT *)malloc(n * sizeof(COMMAND_ARGUMENT));
   if(cmdList[INDEX_PEN_POS].args == NULL) return false;

   // SCARA_COMMAND_2 penColor:
   cmdList[INDEX_PEN_COLOR].cmdName = "penColor";
   cmdList[INDEX_PEN_COLOR].strArgs = "3 int numbers between 0 and 255 r g b";
   n = cmdList[INDEX_PEN_COLOR].nArgs = 3;
   cmdList[INDEX_PEN_COLOR].args = (COMMAND_ARGUMENT *)malloc(n * sizeof(COMMAND_ARGUMENT));
   if(cmdList[INDEX_PEN_COLOR].args == NULL) return false;

   // SCARA_COMMAND_3 cyclePenColors:
   cmdList[INDEX_CYCLE_PEN_COLORS].cmdName = "cyclePenColors";
   cmdList[INDEX_CYCLE_PEN_COLORS].strArgs = "Arg that should be either ON / OFF";
   n = cmdList[INDEX_CYCLE_PEN_COLORS].nArgs = 1;
   cmdList[INDEX_CYCLE_PEN_COLORS].args = (COMMAND_ARGUMENT *)malloc(n * sizeof(COMMAND_ARGUMENT));
   if(cmdList[INDEX_CYCLE_PEN_COLORS].args == NULL) return false;

   // SCARA_COMMAND_4 clearTrace:
   cmdList[INDEX_CLEAR_TRACE].cmdName = "clearTrace";
   cmdList[INDEX_CLEAR_TRACE].strArgs = "NONE";
   n = cmdList[INDEX_CLEAR_TRACE].nArgs = 0;
   cmdList[INDEX_CLEAR_TRACE].args = NULL;

   // SCARA_COMMAND_5 clearRemoteCommandLog:
   cmdList[INDEX_CLEAR_REMOTE_COMMAND_LOG].cmdName = "clearRemoteCommandLog";
   cmdList[INDEX_CLEAR_REMOTE_COMMAND_LOG].strArgs = "NONE";
   n = cmdList[INDEX_CLEAR_REMOTE_COMMAND_LOG].nArgs = 0;
   cmdList[INDEX_CLEAR_REMOTE_COMMAND_LOG].args = NULL;

   // SCARA_COMMAND_6 clearPositionLog:
   cmdList[INDEX_CLEAR_POSITION_LOG].cmdName = "clearPositionLog";
   cmdList[INDEX_CLEAR_POSITION_LOG].strArgs = "NONE";
   n = cmdList[INDEX_CLEAR_POSITION_LOG].nArgs = 0;
   cmdList[INDEX_CLEAR_POSITION_LOG].args = NULL;

   // SCARA_COMMAND_7 shutdownSimulation:
   cmdList[INDEX_SHUTDOWN_SIMULATION].cmdName = "shutdownSimulation";
   cmdList[INDEX_SHUTDOWN_SIMULATION].strArgs = "NONE";
   n = cmdList[INDEX_SHUTDOWN_SIMULATION].nArgs = 0;
   cmdList[INDEX_SHUTDOWN_SIMULATION].args = NULL;

   // SCARA_COMMAND_8 endRemoteConnection:
   cmdList[INDEX_END_REMOTE_CONNECTION].cmdName = "endRemoteConnection";
   cmdList[INDEX_END_REMOTE_CONNECTION].strArgs = "NONE";
   n = cmdList[INDEX_END_REMOTE_CONNECTION].nArgs = 0;
   cmdList[INDEX_END_REMOTE_CONNECTION].args = NULL;

   // SCARA_COMMAND_9 home:
   cmdList[INDEX_HOME].cmdName = "home";
   cmdList[INDEX_HOME].strArgs = "NONE";
   n = cmdList[INDEX_HOME].nArgs = 0;
   cmdList[INDEX_HOME].args = NULL;

   // SCARA_COMMAND_10 moveTo:
   cmdList[INDEX_MOVE_TO].cmdName = "moveTo";
   cmdList[INDEX_MOVE_TO].strArgs = "x , y";
   n = cmdList[INDEX_MOVE_TO].nArgs = 2;
   cmdList[INDEX_MOVE_TO].args = (COMMAND_ARGUMENT *)malloc(n * sizeof(COMMAND_ARGUMENT));
   if(cmdList[INDEX_MOVE_TO].args == NULL) return false;

   // SCARA_COMMAND_11 drawLine:
   cmdList[INDEX_DRAW_LINE].cmdName = "drawLine";
   cmdList[INDEX_DRAW_LINE].strArgs = "x1, y1, x1, y1, resolution";
   n = cmdList[INDEX_DRAW_LINE].nArgs = 5;
   cmdList[INDEX_DRAW_LINE].args = (COMMAND_ARGUMENT *)malloc(n * sizeof(COMMAND_ARGUMENT));
   if(cmdList[INDEX_DRAW_LINE].args == NULL) return false;

   // SCARA_COMMAND_12 drawArc:
   cmdList[INDEX_DRAW_ARC].cmdName = "drawArc";
   cmdList[INDEX_DRAW_ARC].strArgs = "Xc, Yc, r, thetaDegStart, thetaDegEnd, resolution";
   n = cmdList[INDEX_DRAW_ARC].nArgs = 6;
   cmdList[INDEX_DRAW_ARC].args = (COMMAND_ARGUMENT *)malloc(n * sizeof(COMMAND_ARGUMENT));
   if(cmdList[INDEX_DRAW_ARC].args == NULL) return false;

   // SCARA_COMMAND_13 drawRectangle:
   cmdList[INDEX_DRAW_RECTANGLE].cmdName = "drawRectangle";
   cmdList[INDEX_DRAW_RECTANGLE].strArgs = "Xbl,Ybl, Xtr, Ytr, resolution";
   n = cmdList[INDEX_DRAW_RECTANGLE].nArgs = 5;
   cmdList[INDEX_DRAW_RECTANGLE].args = (COMMAND_ARGUMENT *)malloc(n * sizeof(COMMAND_ARGUMENT));
   if(cmdList[INDEX_DRAW_RECTANGLE].args == NULL) return false;

   // SCARA_COMMAND_14 drawTriangle:
   cmdList[INDEX_DRAW_TRIANGLE].cmdName = "drawTriangle";
   cmdList[INDEX_DRAW_TRIANGLE].strArgs = "Xbl, Ybl, Xt, Yt, Xbr, Ybr, resolution";
   n = cmdList[INDEX_DRAW_TRIANGLE].nArgs = 7;
   cmdList[INDEX_DRAW_TRIANGLE].args = (COMMAND_ARGUMENT *)malloc(n * sizeof(COMMAND_ARGUMENT));
   if(cmdList[INDEX_DRAW_TRIANGLE].args == NULL) return false;

   // SCARA_COMMAND_15 addRotation:
   cmdList[INDEX_ADD_ROTATION].cmdName = "addRotation";
   cmdList[INDEX_ADD_ROTATION].strArgs = "rotationDeg";
   n = cmdList[INDEX_ADD_ROTATION].nArgs = 1;
   cmdList[INDEX_ADD_ROTATION].args = (COMMAND_ARGUMENT *)malloc(n * sizeof(COMMAND_ARGUMENT));
   if(cmdList[INDEX_ADD_ROTATION].args == NULL) return false;

   // SCARA_COMMAND_16 addTranslation:
   cmdList[INDEX_ADD_TRANSLATION].cmdName = "addTranslation";
   cmdList[INDEX_ADD_TRANSLATION].strArgs = "dx, dy";
   n = cmdList[INDEX_ADD_TRANSLATION].nArgs = 2;
   cmdList[INDEX_ADD_TRANSLATION].args = (COMMAND_ARGUMENT *)malloc(n * sizeof(COMMAND_ARGUMENT));
   if(cmdList[INDEX_ADD_TRANSLATION].args == NULL) return false;

   // SCARA_COMMAND_17 addScaling:
   cmdList[INDEX_ADD_SCALING].cmdName = "addScaling";
   cmdList[INDEX_ADD_SCALING].strArgs = "SX, SY";
   n = cmdList[INDEX_ADD_SCALING].nArgs = 2;
   cmdList[INDEX_ADD_SCALING].args = (COMMAND_ARGUMENT *)malloc(n * sizeof(COMMAND_ARGUMENT));
   if(cmdList[INDEX_ADD_SCALING].args == NULL) return false;

   // SCARA_COMMAND_18 resetTransform:
   cmdList[INDEX_RESET_TRANSFORMATION_MATRIX].cmdName = "resetTransformMatrix";
   cmdList[INDEX_RESET_TRANSFORMATION_MATRIX].strArgs = "NONE";
   n = cmdList[INDEX_RESET_TRANSFORMATION_MATRIX].nArgs = 0;
   cmdList[INDEX_RESET_TRANSFORMATION_MATRIX].args = NULL;

   // SCARA_COMMAND_19 queryState:
   cmdList[INDEX_QUERY_STATE].cmdName = "queryState";
   cmdList[INDEX_QUERY_STATE].strArgs = "NONE";
   n = cmdList[INDEX_QUERY_STATE].nArgs = 0;
   cmdList[INDEX_QUERY_STATE].args = NULL;

   return true;
}

//----------------------------------------------------------------------------------------------------------------
//Thus function  will ask the user if he/she want to get the that from keyboard or file
//Arguments: None
//return value: an integer of 0 if its from keyboard or 1 if file

int getDataInputMode()
{
   char inputData[MAX_ARG_STRING_LENGTH]; // a varialbe to store input value from the user
   char kb[MAX_ARG_STRING_LENGTH] = "keyboard\n"; // to compare with the inputData and kyboard
   char f[MAX_ARG_STRING_LENGTH] = "file\n";       // to compare with the inputData and file
   int ret; // to store the return value of stricmp
   size_t strLength;    // to calculate the length of the input string

   do
   {
      printf("From where do you want to get the data: \"file\" or \"keyboard\"\t");
      fgets(inputData, MAX_ARG_STRING_LENGTH, stdin);

      strLength = strlen(inputData);

      if(strLength == strlen(kb))
      {
         ret = _stricmp(inputData, kb);
         if(ret != 0) printf("Sorry didnt type the right word try again \n");
         else if(ret == 0)
         {
            printf("Good you have type %s", inputData);
            return KEYBOARD_INPUT;
         }
      }

      else if(strLength == strlen(f))

      {
         ret = _stricmp(inputData, f);
         if(ret != 0) printf("Sorry didnt type the right word try again \n");
         else if(ret == 0)
         {
            printf("Good you have type %s", inputData);
            return FILE_INPUT;
         }

      }
      else printf("You didnt type either keboard or file try again!\n");

   }
   while(true);
   return 0;
}

//---------------------------------------------------------------------------------------------------------------------
// function that will free the dynamic memory
// return value . void
// arguments structure that contain scara command args
void freeDynamicMemory(SCARA_COMMAND *cmdList)
{
   int i;  // counter to free all of the dynamic memory of every command

   for(i = 0; i < NUM_COMMANDS; i++) free(cmdList[i].args);
}



//---------------------------------------------------------------------------------------------------------------------
// function that when called, will print all the SCARA COMMANDS and Arguments for each command
// return value . void
// arguments structure that contain scara command args
void help(SCARA_COMMAND *cmdList)
{
   int i;      // for counting in for loops

   for(i = 0; i < NUM_COMMANDS; i++)
   {
      printf("SCARA_COMMAND_%d\n", i);
      printf("Command Name: %s. Number of Arguments: %d. And str Arguments are: %s.\n\n",
         cmdList[i].cmdName, cmdList[i].nArgs, cmdList[i].strArgs);
   }

}


//---------------------------------------------------------------------------------------------------------------------
// This function will run a series of commands typed by the user using the keyboard and will call executCommand funtion
// arguments: structre cmdList, the current state and tranformMatrix
// return value: none.
void runKeyboardCommands(SCARA_COMMAND *cmdList, SCARA_STATE *state, double transformMatrix[3][3])
{
   int index;   // to store the return value of parseCommand which is the index of the found command
   char strCommand[MAX_COMMAND_LENGTH];         // string that stores the input command
   char strErrorMsg[MAX_MESSAGE_LENGTH] = {};    // string that will return the error message if the command isnt found

   while(true)
   {
      printf("Please enter a command with argument values (type Q to quit or H for help): ");
      fgets(strCommand, MAX_COMMAND_LENGTH, stdin);
      if(toupper(strCommand[0]) == 'Q' && strlen(strCommand) == 2) break;
      else if(toupper(strCommand[0]) == 'H' && strlen(strCommand) == 2)
      {
         help(cmdList);
      }
      else
      {
         index = parseCommand(strCommand, cmdList, strErrorMsg, -1);
         if(index == -1) printf("%s\n", strErrorMsg);
         else
         {
            printf("%s is a valid command! (index = %d)\n", strCommand, index);
            executeCommand(cmdList, state, index, transformMatrix);
         }
      }
   }

}


//---------------------------------------------------------------------------------------------------------------------
// This function will open and read the commands for the robot from a file. 
// Inputs: scaara commandList, the memory address to update the state of the robot and the matrix
// Return Value: None.
void runFileCommands(SCARA_COMMAND *cmdList, SCARA_STATE *state, double transformMatrix[3][3])
{
   FILE *fi = NULL;  // variable for the address of the location of the file
   char fileName[MAX_FILENAME_LENGTH] = {};     //variable to store the file name typed by the userr
   errno_t err = NULL;  // variable to see of the return value of fopen_s is valid
   char strCommand[MAX_COMMAND_LENGTH];  // variable used as a buffer to store the input line from a file
   char strErrorMsg[MAX_MESSAGE_LENGTH] = {};  // string that will return the error message if the command isnt found
   int index;   // to store the return value of parseCommand which is the index of the found command


   printf("Please enter the name of the file where you want to get the data from: \n");

   //asked the user for a file name, check for errors and if okay, then used to open the file
   if(fgets(fileName, MAX_FILENAME_LENGTH, stdin) == NULL)
   {
      printf("Sorry but the name of the File isnt valid, the program has finished.");
      waitForEnterKey();
      return;
   }

   fileName[strlen(fileName) - 1] = '\0';  // to substitute the \n char for \0


   // opening the file in read mode and check the file exists and has valid data
   err = fopen_s(&fi, "test.txt", "r");

   if(err != 0 || fi == NULL)
   {
      printf("Sorry the file could not be open, the program has finished.");
      waitForEnterKey();
      return;
   }


   // checking for emptylines and comment lines
   while(fgets(strCommand, MAX_COMMAND_LENGTH, fi) != NULL)
   {
      if(isBlankLine(strCommand) == true) continue;
      if(isCommentLine(strCommand) == true) continue;
      index = parseCommand(strCommand, cmdList, strErrorMsg, -1);
      if(index == -1) printf("%s\n", strErrorMsg);

      else
      {
         printf("%s is a valid command! (index = %d)\n", strCommand, index);
         executeCommand(cmdList, state, index, transformMatrix);
      }

   }

   fclose(fi);  //closing the file.
}


//---------------------------------------------------------------------------------------------------------------------
// This function is where already checked and cleaned input is sent and real action happens
// arguments: command list, actual state of the SCARA robot, the index of the command and the transform Matrix
// return value: none
void executeCommand(SCARA_COMMAND *cmdList, SCARA_STATE *state, int index, double transformMatrix[3][3])
{
   char cmdStg[MAX_COMMAND_LENGTH] = {};  // local variable to change numbers to strings used for sprintf
   double rotationMatrix[3][3];  // matrix to perform rotations
   double translationMatrix[3][3];  // matrix to perform rotations
   double scalingMatrix[3][3];  // matrix to perform rotations
   double theta1Rad;   // variable for converting typed angle which is in degrees and convert to rad
   INVERSE_SOLUTION isol = {}; // contain the return value when called inversekinematics to know the current state
   double xbl, ybl, xtr, ytr, xt, yt, xbr, ybr;    // for bottom left, top right, bottom right top x and y coordenates

   switch(index)
   {
      // use and if statment to see which alternative it is and then update the state of the robot
   case INDEX_MOTOR_SPEED:
      if(_stricmp(cmdList[index].args[0].strValue, STR_RESOLUTION_HIGH) == 0)
      {
         robot.Send("MOTOR_SPEED HIGH\n");
         state->motorSpeed = MOTOR_SPEED_HIGH;
      }

      else if(_stricmp(cmdList[index].args[0].strValue, STR_RESOLUTION_MEDIUM) == 0)
      {
         robot.Send("MOTOR_SPEED MEDIUM\n");
         state->motorSpeed = MOTOR_SPEED_MEDIUM;
      }

      else if(_stricmp(cmdList[index].args[0].strValue, STR_RESOLUTION_LOW) == 0)

      {
         robot.Send("MOTOR_SPEED LOW\n");
         state->motorSpeed = MOTOR_SPEED_LOW;
      }

      break;

   case INDEX_PEN_POS:
      if(_stricmp(cmdList[index].args[0].strValue, STR_PEN_UP) == 0)
      {
         robot.Send("PEN_UP\n");
         state->penPos = PEN_UP;
      }
      else if(_stricmp(cmdList[index].args[0].strValue, STR_PEN_DOWN) == 0)
      {
         robot.Send("PEN_DOWN\n");
         state->penPos = PEN_DOWN;
      }

      break;

   case INDEX_PEN_COLOR:
      sprintf_s(cmdStg, "PEN_COLOR %d %d %d\n", cmdList[index].args[0].iValue,
         cmdList[index].args[1].iValue, cmdList[index].args[2].iValue);
      robot.Send(cmdStg);
      state->penColor.r = cmdList[index].args[0].iValue;
      state->penColor.g = cmdList[index].args[1].iValue;
      state->penColor.b = cmdList[index].args[2].iValue;
      break;

   case INDEX_CYCLE_PEN_COLORS:
      if(_stricmp(cmdList[index].args[0].strValue, STR_CYCLE_PEN_COLORS_OFF) == 0)
      {
         robot.Send("CYCLE_PEN_COLOR OFF\n");
         state->cyclePenColors = CYCLE_PEN_COLORS_OFF;
      }
      else if(_stricmp(cmdList[index].args[1].strValue, STR_CYCLE_PEN_COLORS_ON) == 0)
      {
         robot.Send("CYCLE_PEN_COLOR ON\n");
         state->cyclePenColors = CYCLE_PEN_COLORS_ON;
      }
      break;

   case INDEX_CLEAR_TRACE:
      robot.Send("CLEAR_TRACE\n");
      break;

   case INDEX_CLEAR_REMOTE_COMMAND_LOG:
      sprintf_s(cmdStg, "CLEAR_REMOTE_COMMAND_LOG\n");
      robot.Send(cmdStg);
      break;

   case INDEX_SHUTDOWN_SIMULATION:
      sprintf_s(cmdStg, "SHUTDOWN_SIMULATION\n");
      robot.Send(cmdStg);
      break;

   case INDEX_HOME:
      sprintf_s(cmdStg, "HOME\n");
      robot.Send(cmdStg);
      state->currentPos.x = 600.0;
      state->currentPos.y = 0.0;
      break;

   case INDEX_MOVE_TO:
      drawStraightLine(cmdList, index, transformMatrix, state);
      state->currentPos.x = cmdList[index].args[0].dValue;
      state->currentPos.y = cmdList[index].args[1].dValue;
      break;

   case INDEX_DRAW_LINE:
      drawStraightLine(cmdList, index, transformMatrix, state);
      state->currentPos.x = cmdList[index].args[2].dValue;
      state->currentPos.y = cmdList[index].args[3].dValue;
      break;

   case INDEX_DRAW_ARC:
      drawArc(cmdList, index, transformMatrix, state);
      break;

   case INDEX_DRAW_RECTANGLE:
      xbl = cmdList[index].args[0].dValue;
      ybl = cmdList[index].args[1].dValue;
      xtr = cmdList[index].args[2].dValue;
      ytr = cmdList[index].args[3].dValue;

      cmdList[index].args[0].dValue = xbl;
      cmdList[index].args[1].dValue = ybl;
      cmdList[index].args[2].dValue = xbl;
      cmdList[index].args[3].dValue = ytr;
      drawStraightLine(cmdList, index, transformMatrix, state);

      cmdList[index].args[0].dValue = xbl;
      cmdList[index].args[1].dValue = ytr;
      cmdList[index].args[2].dValue = xtr;
      cmdList[index].args[3].dValue = ytr;
      drawStraightLine(cmdList, index, transformMatrix, state);

      cmdList[index].args[0].dValue = xtr;
      cmdList[index].args[1].dValue = ytr;
      cmdList[index].args[2].dValue = xtr;
      cmdList[index].args[3].dValue = ybl;
      drawStraightLine(cmdList, index, transformMatrix, state);

      cmdList[index].args[0].dValue = xtr;
      cmdList[index].args[1].dValue = ybl;
      cmdList[index].args[2].dValue = xbl;
      cmdList[index].args[3].dValue = ybl;
      drawStraightLine(cmdList, index, transformMatrix, state);
      state->currentPos.x = cmdList[index].args[2].dValue;
      state->currentPos.y = cmdList[index].args[3].dValue;
      break;

   case INDEX_DRAW_TRIANGLE:
      xbl = cmdList[index].args[0].dValue;
      ybl = cmdList[index].args[1].dValue;
      xt = cmdList[index].args[2].dValue;
      yt = cmdList[index].args[3].dValue;
      xbr = cmdList[index].args[4].dValue;
      ybr = cmdList[index].args[5].dValue;

      cmdList[index].args[0].dValue = xbl;
      cmdList[index].args[1].dValue = ybl;
      cmdList[index].args[2].dValue = xt;
      cmdList[index].args[3].dValue = yt;
      drawStraightLine(cmdList, index, transformMatrix, state);

      cmdList[index].args[0].dValue = xt;
      cmdList[index].args[1].dValue = yt;
      cmdList[index].args[2].dValue = xbr;
      cmdList[index].args[3].dValue = ybr;
      drawStraightLine(cmdList, index, transformMatrix, state);

      cmdList[index].args[0].dValue = xbr;
      cmdList[index].args[1].dValue = ybr;
      cmdList[index].args[2].dValue = xbl;
      cmdList[index].args[3].dValue = ybl;
      drawStraightLine(cmdList, index, transformMatrix, state);
      state->currentPos.x = cmdList[index].args[2].dValue;
      state->currentPos.y = cmdList[index].args[3].dValue;
      break;

   case INDEX_ADD_ROTATION:
      theta1Rad = degToRad(cmdList[index].args[0].dValue);
      rotationMatrix[0][0] = cos(theta1Rad);
      rotationMatrix[0][1] = -sin(theta1Rad);
      rotationMatrix[0][2] = 0.0;
      rotationMatrix[1][0] = sin(theta1Rad);
      rotationMatrix[1][1] = cos(theta1Rad);
      rotationMatrix[1][2] = 0.0;
      rotationMatrix[2][0] = 0.0;
      rotationMatrix[2][1] = 0.0;
      rotationMatrix[2][2] = 1.0;
      transformMatrixMultiply(transformMatrix, rotationMatrix);
      break;

   case INDEX_ADD_TRANSLATION:
      translationMatrix[0][0] = 1.0;
      translationMatrix[0][1] = 0.0;
      translationMatrix[0][2] = cmdList[index].args[0].dValue;
      translationMatrix[1][0] = 0.0;
      translationMatrix[1][1] = 1.0;
      translationMatrix[1][2] = cmdList[index].args[1].dValue;
      translationMatrix[2][0] = 0.0;
      translationMatrix[2][1] = 0.0;
      translationMatrix[2][2] = 1.0;
      transformMatrixMultiply(transformMatrix, translationMatrix);
      break;

   case INDEX_ADD_SCALING:
      scalingMatrix[0][0] = cmdList[index].args[0].dValue;
      scalingMatrix[0][1] = 0.0;
      scalingMatrix[0][2] = 0.0;
      scalingMatrix[1][0] = 0.0;
      scalingMatrix[1][1] = cmdList[index].args[1].dValue;
      scalingMatrix[1][2] = 0.0;
      scalingMatrix[2][0] = 0.0;
      scalingMatrix[2][1] = 0.0;
      scalingMatrix[2][2] = 1.0;
      transformMatrixMultiply(transformMatrix, scalingMatrix);
      break;

   case INDEX_RESET_TRANSFORMATION_MATRIX:
      resetTransformMatrix(transformMatrix);
      break;

   case INDEX_QUERY_STATE:
      printf_s("Current Position x: %.2lf y: %.2lf\n", state->currentPos.x, state->currentPos.y);
      printf_s("Current Angles Theta1: %.2lf Theta2: %.2lf\n", state->currentPos.theta1Deg, state->currentPos.theta2Deg);
      if(state->currentPos.armPos == LEFT_ARM) printf_s("Current Arm Configuration: LEFT_ARM\n");
      else if(state->currentPos.armPos == RIGHT_ARM) printf_s("Current Arm Configuration: RIGHT_ARM\n");
      else if(state->currentPos.armPos == NO_ARM) printf_s("Current Arm Configuration: NO_ARM\n");
      break;
   }

}


//---------------------------------------------------------------------------------------------------------------------
// This function will be called from executeCommand to calculate the n intermediate points of a straight lines and then
// call inverseKinematics to send the angles to the SCARA robot
// arguments structure that contain cmdList, the index where to command was found and the transformMatrix
// return value: none since we are sending pointers 
void drawStraightLine(SCARA_COMMAND *cmdList, int index, double transformMatrix[3][3], SCARA_STATE *state)
{
   char commandString[MAX_COMMAND_LENGTH];   // for using sprintf_s and send commands to the robot
   double adderLeft = 0, adderRight = 0;     // to store the accumulation of angles for the most efficient path
   double len, x0, x1, y0, y1;               // to copy values of cmdList array and work with smaller commands
                                             // those variables are for length, inicital and final coordanates for x/y
   int n;                                    // n is number if intermediate points                                         
   int i;                                    // used for counting
   bool bRight, bLeft;                       // to store the return values fo left and right solutions
   double arrayX[MAX_POINTS] = {0}, arrayY[MAX_POINTS] = {0};     // to store in the array the intermediate coordanates in x/y
   double arrayTheta1R[MAX_POINTS] = {0}, arrayTheta2R[MAX_POINTS] = {0}, arrayTheta1L[MAX_POINTS] = {0},
      arrayTheta2L[MAX_POINTS] = {0};
   // to store in array the theta1 and 2 degrees and check for the shortest / fastest solution
   INVERSE_SOLUTION isol;                    // for the retun value of inverseKinematics

   x0 = cmdList[index].args[0].dValue;
   y0 = cmdList[index].args[1].dValue;
   x1 = cmdList[index].args[2].dValue;
   y1 = cmdList[index].args[3].dValue;
   len = sqrt(pow(x1 - x0, 2) + pow(y1 - y0, 2));
   n = 0;

   int b;
   b = cmdList[index].nArgs;

   if(_stricmp(cmdList[index].args[cmdList[index].nArgs - 1].strValue, STR_RESOLUTION_HIGH) == 0)
   {
      n = getN(len, RESOLUTION_HIGH);
   }
   else  if(_stricmp(cmdList[index].args[cmdList[index].nArgs - 1].strValue, STR_RESOLUTION_MEDIUM) == 0)

   {
      n = getN(len, RESOLUTION_MEDIUM);
   }
   else if(_stricmp(cmdList[index].args[cmdList[index].nArgs - 1].strValue, STR_RESOLUTION_LOW) == 0)
   {
      n = getN(len, RESOLUTION_LOW);
   }

   if(n == 0)
   {
      arrayX[0] = x0 + ((x1 - x0) * (double)0 / ((double)n + 1.0));
      arrayY[0] = y0 + ((y1 - y0) * (double)0 / ((double)n + 1.0));
      n = -1;
   }
   else
   {
      for(i = 0; i <= n + 1; i++)
      {
         arrayX[i] = x0 + ((x1 - x0) * (double)i / ((double)n + 1.0));
         arrayY[i] = y0 + ((y1 - y0) * (double)i / ((double)n + 1.0));
      }
   }
   bLeft = true;
   bRight = true;

   // calulating theta1 and theta2 for each pair of coordanates prevously stored in arrayx and arrayy
   // use inverseKinematics and send the angles to the robot and print angles and position

   for(i = 0; i <= n + 1; i++)
   {
      isol = inverseKinematics(arrayX[i], arrayY[i], transformMatrix);
      bLeft = bLeft && isol.bLeft;                              // validating left solution
      bRight = bRight && isol.bRight;                            // validating right solution

      // add al angles together to see the most optimal solution
      adderLeft = isol.theta1DegLeft + isol.theta2DegLeft + adderLeft;
      adderRight = isol.theta1DegRight + isol.theta2DegRight + adderRight;

      arrayTheta1R[i] = isol.theta1DegRight;
      arrayTheta2R[i] = isol.theta2DegRight;
      arrayTheta1L[i] = isol.theta1DegLeft;
      arrayTheta2L[i] = isol.theta2DegLeft;
   }


   // creating the most efficient path
   if(bLeft == true && bRight == true)
   {
      if(adderLeft < adderRight) bRight = false;
      else if(adderLeft >= adderRight) bLeft = false;
   }


   for(i = 0; i <= n + 1; i++)
   {
      if(i == 0)
      {
         sprintf_s(commandString, "PEN_UP\n");
         robot.Send(commandString);
      }
      if(bRight == true)
      {
         sprintf_s(commandString, "ROTATE_JOINT ANG1 %lf ANG2 %lf\n", arrayTheta1R[i], arrayTheta2R[i]);
         robot.Send(commandString);
      }
      else if(bLeft == true)
      {
         sprintf_s(commandString, "ROTATE_JOINT ANG1 %lf ANG2 %lf\n", arrayTheta1L[i], arrayTheta2L[i]);
         robot.Send(commandString);
      }
      if(i == 0)
      {
         sprintf_s(commandString, "PEN_DOWN\n");
         robot.Send(commandString);
      }

      if(i == n + 1 && bRight == true)
      {
         state->currentPos.theta1Deg = arrayTheta1R[i];
         state->currentPos.theta2Deg = arrayTheta2R[i];
      }
      else if(i == n + 1 && bLeft == true)
      {
         state->currentPos.theta1Deg = arrayTheta1L[i];
         state->currentPos.theta2Deg = arrayTheta2L[i];
      }
   }
}




//----------------------------------------------------------------------------------------------------------------
// This function computes the SCARA inverse solution for a given x,y position.
// both right arm and left arm solutions are returned in the INVERSE_SOLUTION structure.
// INPUTS:  The position coordinates x, y and the tranformMatrix
// RETURN:  an INVERSE_SOLUTION structure containing joint angles for both left and 
//          right arms.  RETURN ANGLES ARE IN DEGREES!!!!!!!!!!!!!!

INVERSE_SOLUTION inverseKinematics(double x, double y, double transformMatrix[3][3])
{
   INVERSE_SOLUTION isol = {0.0, 0.0, 0.0, 0.0, false, false};  // structure for solution left/right arm angles
   double len, beta, alfa, thetaRad1Right, thetaRad2Right, thetaRad1Left, thetaRad2Left;
   // to calculate length, angles beta and alfa (from SCARA Robot and Theta1 and 2 in Rad a for both configuration

   // x, y are the original (non-transformed) coordinates.
   double xt = x, yt = y;
   x = xt * transformMatrix[0][0] + yt * transformMatrix[0][1] + transformMatrix[0][2];
   y = xt * transformMatrix[1][0] + yt * transformMatrix[1][1] + transformMatrix[1][2];


   len = sqrt(x * x + y * y);
   beta = atan2(y, x);
   alfa = acos((L2 * L2 - len * len - L1 * L1) / (-2 * len * L1));


   // right arm configuration
   thetaRad1Right = beta - alfa;
   thetaRad2Right = (atan2((y - L1 * sin(thetaRad1Right)), (x - L1 * cos(thetaRad1Right)))) - thetaRad1Right;
   if(thetaRad2Right <= -PI) thetaRad2Right = thetaRad2Right + 2 * PI;
   else if(thetaRad2Right >= PI) thetaRad2Right = -2 * PI + thetaRad2Right;
   isol.theta1DegRight = radToDeg(thetaRad1Right);
   isol.theta2DegRight = radToDeg(thetaRad2Right);

   if(isol.theta1DegRight >= -MAX_ABS_THETA1_DEG && isol.theta1DegRight <= MAX_ABS_THETA1_DEG &&
      isol.theta2DegRight >= -MAX_ABS_THETA2_DEG && isol.theta2DegRight <= MAX_ABS_THETA2_DEG)

   {
      isol.bRight = true;
   }

   // left arm configuration
   thetaRad1Left = beta + alfa;
   thetaRad2Left = (atan2((y - L1 * sin(thetaRad1Left)), (x - L1 * cos(thetaRad1Left)))) - thetaRad1Left;
   if(thetaRad2Left <= -PI) thetaRad2Left = thetaRad2Left + 2 * PI;
   else if(thetaRad2Left >= PI) thetaRad2Left = -2 * PI + thetaRad2Left;
   isol.theta1DegLeft = radToDeg(thetaRad1Left);
   isol.theta2DegLeft = radToDeg(thetaRad2Left);

   if(isol.theta1DegLeft >= -MAX_ABS_THETA1_DEG && isol.theta1DegLeft <= MAX_ABS_THETA1_DEG &&
      isol.theta2DegLeft >= -MAX_ABS_THETA2_DEG && isol.theta2DegLeft <= MAX_ABS_THETA2_DEG)

   {
      isol.bLeft = true;
   }
   return isol;
}



//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  calculate x and y coordanates and store them in arrays, then print the values as well as draw them
// ARGUMENTS:    structure
// RETURN VALUE: void since we are working with pointers adn passing info by referencing

void drawArc(SCARA_COMMAND *cmdList, int index, double transformMatrix[3][3], SCARA_STATE *state)
{
   int N = 0, i;     // N is for intermediate points and i is used as a counter
   const int SIZE = 200;  // SIZE variable to store the coordanates of the x and y points in all the n intermediate
   double x[SIZE] = {}, y[SIZE] = {}, theta;    //to store the coordanates of all n intermediate points
   char commandString[MAX_COMMAND_LENGTH];      //to store and send the information to the scara robor
   INVERSE_SOLUTION isol = {0.0, 0.0, 0.0, 0.0, false, false};  // structure for solution left/right arm angles
   bool bLeft = true, bRight = true;      // to check the righ and left arm configuration
   double adderLeft = 0, adderRight = 0;  // to accumulate the angles of every point across an arc to check fastest way
   double theta1Left[SIZE] = {}, theta2Left[SIZE] = {}, theta1Right[SIZE] = {}, theta2Right[SIZE] = {};
   double len, xc, yc, radius;               // to copy values of cmdList array and work with smaller commands xc / yc
                                             // are the coordanates of the center of the arc.
   double thetaStart, thetaEnd;              // to copy the values of cmdList to to the start and end angles

   xc = cmdList[index].args[0].dValue;
   yc = cmdList[index].args[1].dValue;
   radius = cmdList[index].args[2].dValue;
   thetaStart = degToRad(cmdList[index].args[3].dValue);
   thetaEnd = degToRad(cmdList[index].args[4].dValue);


   // length of arc formular is s times (delta theta)
   len = abs(cmdList[index].args[2].dValue * (thetaEnd - thetaStart));

   if(_stricmp(cmdList[index].args[cmdList[index].nArgs - 1].strValue, STR_RESOLUTION_HIGH) == 0)
   {
      N = getN(len, RESOLUTION_HIGH);
   }
   else  if(_stricmp(cmdList[index].args[cmdList[index].nArgs - 1].strValue, STR_RESOLUTION_MEDIUM) == 0)

   {
      N = getN(len, RESOLUTION_MEDIUM);
   }
   else if(_stricmp(cmdList[index].args[cmdList[index].nArgs - 1].strValue, STR_RESOLUTION_LOW) == 0)
   {
      N = getN(len, RESOLUTION_LOW);
   }



   // to calculate x and y coordinate across the circumference
   for(i = 0; i <= N - 1; i++)
   {
      theta = thetaStart + (thetaEnd - thetaStart) * ((double)i / (N - 1.0));
      x[i] = xc + radius * cos(theta);
      y[i] = yc + radius * sin(theta);

      isol = inverseKinematics(x[i], y[i], transformMatrix);

      if(i == N - 1)
      {
         state->currentPos.theta1Deg = x[i];
         state->currentPos.theta2Deg = y[i];
      }

      theta1Left[i] = isol.theta1DegLeft;
      theta2Left[i] = isol.theta2DegLeft;
      theta1Right[i] = isol.theta1DegRight;
      theta2Right[i] = isol.theta2DegRight;
      bLeft = bLeft && isol.bLeft;                              // validating left solution
      bRight = bRight && isol.bRight;                            // validating right solution

 // add al angles together to see the most optimal solution
      adderLeft = isol.theta1DegLeft + isol.theta2DegLeft + adderLeft;
      adderRight = isol.theta1DegRight + isol.theta2DegRight + adderRight;
   }


   //creating the most efficient path
   if(bLeft == true && bRight == true)
   {
      if(adderLeft < adderRight) bRight = false;
      else if(adderLeft >= adderRight) bLeft = false;
   }
   sprintf_s(commandString, MAX_COMMAND_LENGTH, "PEN_UP\n");
   robot.Send(commandString);


   for(i = 0; i < N; i++) // removed -1 from N
   {
      if(bRight == true)
      {
         sprintf_s(commandString, "ROTATE_JOINT ANG1 %lf ANG2 %lf\n", theta1Right[i], theta2Right[i]);
         robot.Send(commandString);
      }
      else if(bLeft == true)
      {
         sprintf_s(commandString, "ROTATE_JOINT ANG1 %lf ANG2 %lf\n", theta1Left[i], theta2Left[i]);
         robot.Send(commandString);
      }
      if(i == 0)
      {
         sprintf_s(commandString, MAX_COMMAND_LENGTH, "PEN_DOWN\n");
         robot.Send(commandString);
      }

      if(i == N)
      {
         sprintf_s(commandString, "PEN_UP\n");
         robot.Send(commandString);
      }
   }
}

//---------------------------------------------------------------------------------------------------------------------
//This function will get x and y coordinates and check that the whole path can be drawn
//Arguments, the x and y coordinates of every point
//Return Value, true if its inside the range or false if not.
bool checkPad(double x, double y)
{

   return false;
}