/***************************************************
 * Pares a string from serial port, process it
 * based on the command definitions.
 * command template: <command>,<integer value>
 ***************************************************/
#include <CheapStepper.h>

/***********************************************
 * structures, types
 ***********************************************/

typedef void (* GenericFP)(int);
struct cmdSet 
{
  const String cmd;               /*command string*/
  GenericFP fp;                   /*function to call*/
  
};

/***********************************************
 * preprocessor directives, constants, variables
 ***********************************************/
#define FIRST_PORT 2            /*excludes serial ports*/
#define LAST_PORT 13            /*include on board led*/
#define SERIAL_SPEED 9600       /*serial speed*/
#define STEPPER0_SPEED 8        /*delay btween steps, usually 5-20*/
#define STEPPER1_SPEED 8        /*delay btween steps, usually 5-20*/
#define STEPPER0_PINS 8,9,10,11 /*ogly way to defne of pins used to control the stepper*/
#define STEPPER1_PINS 2,3,4,5   /*ogly way to defne of pins used to control the stepper*/
#define MAX_STEPS 4096

CheapStepper stepper0 (STEPPER0_PINS); 
CheapStepper stepper1 (STEPPER1_PINS);

String readString;            /*holds the string recived from serial port*/
String cmd = "";              /*substring for command*/
String val = "";              /*substring for value*/
bool processString = false;   /*indicats the end of string received on serial*/
int setHomeM0 = 0;            /*motor 0 position*/
int setHomeM1 = 0;            /*motor 1 position*/
//bool limitM0Reached = false;  /*indicate if motor0 position jumps from 0 to MAX_STEPS*/
//bool limitM1Reached = false;  /*indicate if motor1 position jumps from 0 to MAX_STEPS*/

/*command set declaration*/
struct cmdSet cmdControl[] = {{"m0mtp",&m0mtp},  /*stepper 0 move to plus (clockwhise) n steps*/
                              {"m0mtn",&m0mtm},  /*stepper 0 move to minus (anti-clockwhise) n steps*/
                              {"m1mtp",&m1mtp},  /*stepper 1 move to plus (clockwhise) n steps*/
                              {"m1mtn",&m1mtm},  /*stepper 1 move to minus (anti-clockwhise) n steps*/
                              {"pon",&pon},      /*port x on*/
                              {"poff",&poff},    /*port x off*/
                              {"m0p",&m0p},      /*stepper 0, current position plus n steps*/
                              {"m0m",&m0m},      /*stepper 0, current position minus n steps*/                             
                              {"m1p",&m1p},      /*stepper 1, current position plus n steps*/
                              {"m1m",&m1m},      /*stepper 1, current position minus n steps*/
                              {"ls",&ls},        /*ls,val - list available commands*/
                              {"sh",&sh},         /*sh,[0,1] - set home posiion for motor [0,1]*/
                              {"gh",&gohome}         /*gh,[0,1] - go home motor [0,1]*/
                              };  


/*command description, stored in flash*/
const char cmddesc[] PROGMEM = {
  "m0mtp,val - stepper 0 move to position plus (clockwhise) n steps\n"
  "m0mtm,val - stepper 0 move to position minus (anti-clockwhise) n steps\n"
  "m1mtp,val - stepper 1 move plus (clockwhise) n steps\n"
  "m1mtn,val - stepper 1 move minus (anti-clockwhise) n steps\n"
  "pon,val - port x on\n"
  "poff,val - port x off\n"
  "m0p,val - stepper 0, current position plus n steps\n"
  "m0m,val - stepper 0, current position minus n steps\n"
  "m1p,val - stepper 1, current position plus n steps\n"
  "m1m,val - stepper 1, current position minus n steps\n"
  "ls,val - list commands over serial\n"
  "sh,[0,1] - set home posiion for motor [0,1]\n"
  "gh,[0,1] - go home motor [0,1]\n"
  };

  
/*init*/
void setup()
{
  stepper0.setRpm(STEPPER0_SPEED); 
  stepper1.setRpm(STEPPER1_SPEED); 
  Serial.begin(SERIAL_SPEED);
}

/*main loop*/
void loop()
{
  char c = ""; /*holds the received characters*/
  int sindex = 0; /*used to pharse the string*/
  int strend = 0; /*used to pharse the string*/
  

  /*non blocking serial read*/
  if (Serial.available() > 0) 
  {
    c = Serial.read();
    
    if (c != '\n')
    {
      /*read characters till new lne received*/
      readString +=c;
    }
    else
    {
      /*string ready to be processed*/
      processString = true;
    }
    
  }

  /*********************************************
   * we have the string let's pharse it, 
   *command template: <command>,<integer value>
   *********************************************/  
  if(processString == true)
  {

    processString = false;

    /*put back to serial the received string*/
    Serial.println(readString);

    /*Find the location of separator*/
    strend = readString.length();
    sindex = readString.indexOf(',');

    /*process only if we have a separator and not on the first place*/
    if (sindex > 0)
    {

      /*get cmd string and value*/
      cmd = readString.substring(0, sindex);
      val = readString.substring(sindex+1, strend);

      /*search for the command, execute it*/
      for (byte i = 0; i< sizeof (cmdControl)/sizeof (cmdControl[0]); i++)
      {

        /*check command*/
        if (cmd == cmdControl[i].cmd)
        {
          /*call processing function, if val is not an integer, 0 will be returned*/
          cmdControl[i].fp(val.toInt());
        }
      }
    }
    
    /*clear string cotent*/
    readString = "";
    Serial.flush();
    
  }

}


/***********************
 * command implementation
 ***********************/

/*stepper 0 command*/
void m0mtp(int val)
{
  stepper0.moveTo (true, val);
}

/*stepper 0 command*/
void m0mtm(int val)
{
  stepper0.moveTo (true, val);
}


/*stepper 0, plus n steps*/
void m0p(int val)
{
  for (int i = 0; i<val; i++)
  {
    stepper0.step(true);
  }

}


/*stepper 0, minus n steps*/
void m0m(int val)
{
  
  for (int i = 0; i<val; i++)
  {
    stepper0.step(false);
  }
  
}

/*stepper 1, plus n steps*/
void m1mtp(int val)
{
  stepper1.moveTo (true, val);
}

/*stepper 1, takes steps*/
void m1mtm(int val)
{
  stepper1.moveTo (true, val);
}

/*stepper 1, plus n steps*/
void m1p(int val)
{  
  for (int i = 0; i<val; i++)
  {
    stepper1.step(true);
  }
}

/*stepper 1, minus n steps*/
void m1m(int val)
{  
  for (int i = 0; i<val; i++)
  {
    stepper1.step(false);
  }
}


/*set home motor [0,1]*/
void sh(int val)
{

  /*handle motor 0*/
  if (val == 0)
  {
    setHomeM0 = stepper0.getStep();
    Serial.println(setHomeM0);
  }

  /*hadle motor 1*/
  if (val == 1)
  {
    setHomeM1 = stepper1.getStep();
    Serial.println(setHomeM1);
  }

  
}

/*go home motor [0,1]*/
void gohome(int val)
{
  /*handle the specific motors*/
  if (val == 0)
  {
    /*compute the shortest roote (clockwhise or anti)*/
    if ((MAX_STEPS - stepper0.getStep())< (stepper0.getStep()))
    {
      stepper0.moveTo(true, setHomeM0);
    }
    else
    {
      stepper0.moveTo(false, setHomeM0);
    }
  }

  /*hadle motor 1*/
  if (val == 1)
  {
    /*compute the shortest roote (clockwhise or anti)*/
    if ((MAX_STEPS - stepper1.getStep())< (stepper1.getStep()))
    {
      stepper1.moveTo(true, setHomeM1);
    }
    else
    {
      stepper1.moveTo(false, setHomeM1);
    }
  }
  
}


/*port on, takes prot number*/
void pon(int val)
{
  if (val<=LAST_PORT && val>=FIRST_PORT)
  {
    digitalWrite(val,HIGH);
  }
}

/*port off, takes port number*/
void poff(int val)
{
  if (val<=LAST_PORT && val>=FIRST_PORT)
  {
    digitalWrite(val,LOW);
  }
}

/*list the available commands*/
void ls(int val)
{
  char ct='\0';
  
  /*read back string from flash*/
  for (unsigned int i = 0; i < strlen_P(cmddesc); i++) 
  {
    ct = pgm_read_byte_near(cmddesc + i);
    Serial.print(ct);
  }

}
