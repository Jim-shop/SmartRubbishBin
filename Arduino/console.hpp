#ifndef _CONSOLE_HPP
#define _CONSOLE_HPP

#include "config.hpp"
#include "constants.hpp"

enum Command
{
    ANALOG_READ = 'a',
    GET_BAUDRATE = 'b',
    DIGITAL_READ = 'd',
    READ_ENCODERS = 'e',
    MOTOR_SPEEDS = 'm',
    PIN_MODE = 'p',
    RESET_ENCODERS = 'r',
    UPDATE_PID = 'u',
    DIGITAL_WRITE = 'w',
    ANALOG_WRITE = 'x',
};

class Console
{
private:
    // A pair of varibles to help parse serial commands (thanks Fergs)
    int arg_count = 0;
    int index = 0;

    // Variable to hold an input character
    char temp_chr;

    // Variable to hold the current single-character command
    char cmd;

    // Character arrays to hold the first and second arguments
    char argv1[16];
    char argv2[16];

    // The arguments converted to integers
    long arg1;
    long arg2;

    /* Clear the current command parameters. */
    void resetCommand()
    {
        cmd = 0;
        memset(argv1, 0, sizeof(argv1));
        memset(argv2, 0, sizeof(argv2));
        arg1 = 0;
        arg2 = 0;
        arg_count = 0;
        index = 0;
    }

    /* Run a command. */
    int runCommand()
    {
        int i = 0;
        char *p = argv1;
        char *str;
        int pid_args[4];
        arg1 = atoi(argv1);
        arg2 = atoi(argv2);

        switch (cmd)
        {
        case GET_BAUDRATE:
            Serial.println(BAUDRATE);
            break;
        case ANALOG_READ:
            Serial.println(analogRead(arg1));
            break;
        case DIGITAL_READ:
            Serial.println(digitalRead(arg1));
            break;
        case ANALOG_WRITE:
            analogWrite(arg1, arg2);
            Serial.println("OK");
            break;
        case DIGITAL_WRITE:
            if (arg2 == 0)
                digitalWrite(arg1, LOW);
            else if (arg2 == 1)
                digitalWrite(arg1, HIGH);
            Serial.println("OK");
            break;
        case PIN_MODE:
            if (arg2 == 0)
                pinMode(arg1, INPUT);
            else if (arg2 == 1)
                pinMode(arg1, OUTPUT);
            Serial.println("OK");
            break;
#if false
        case READ_ENCODERS:
            Serial.print(encoderVal1);
            Serial.print(" ");
            Serial.println(encoderVal2);
            break;
        case RESET_ENCODERS:
            resetEncoders();
            resetPID();
            Serial.println("OK");
            break;
        case MOTOR_SPEEDS:
            /* Reset the auto stop timer */
            lastMotorCommand = millis();
            if (arg1 == 0 && arg2 == 0)
            {
                setMotorSpeeds(0, 0);
                resetPID();
                moving = false;
            }
            else
                moving = true;
            leftPID.TargetTicksPerFrame = arg1;
            rightPID.TargetTicksPerFrame = arg2;
            Serial.println("OK");
            break;
        case UPDATE_PID:
            while ((str = strtok_r(p, ":", &p)) != '\0')
            {
                pid_args[i] = atoi(str);
                i++;
            }
            Kp = pid_args[0];
            Kd = pid_args[1];
            Ki = pid_args[2];
            Ko = pid_args[3];
            Serial.println("OK");
            break;
#endif
        default:
            Serial.println("Invalid Command");
            break;
        }
    }

public:
    void parse()
    {
        while (Serial.available() > 0)
        {
            // Read the next character
            temp_chr = Serial.read();
            // Terminate a command with a CR
            if (temp_chr == '\r')
            {
                if (arg_count == 1)
                    argv1[index] = 0;
                else if (arg_count == 2)
                    argv2[index] = 0;
                runCommand();
                resetCommand();
            }
            // Use spaces to delimit parts of the command
            else if (temp_chr == ' ')
            {
                // Step through the arguments
                if (arg_count == 0)
                    arg_count = 1;
                else if (arg_count == 1)
                {
                    argv1[index] = 0;
                    arg_count = 2;
                    index = 0;
                }
            }
            else
            {
                if (arg_count == 0)
                    // The first arg is the single-letter command
                    cmd = temp_chr;
                else if (arg_count == 1)
                    // Subsequent arguments can be more than one character
                    argv1[index++] = temp_chr;
                else if (arg_count == 2)
                    argv2[index++] = temp_chr;
            }
        }
    }
};

#endif