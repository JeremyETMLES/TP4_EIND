/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "Mc32DriverAdc.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    S_ADCResults valADC;    // Sauvegarde des valeurs des ADC
    float Uk;               // Facteur globale
    static uint8_t overCurrent = 0;    // Sauvegarde si il y a eu un surcourant
    static uint32_t overcurrentTimer = 0;
    
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            BSP_InitADC10(); //Initialisation de l'ADC 
            //DRV_ADC_Open();
            
            // Active les timers 
            DRV_TMR0_Start();
            DRV_TMR1_Start();
            DRV_OC0_Start();
            //DRV_OC1_Start();
        
            // Activer le driver de gate
            PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_5, 1);
            
            APP_UpdateState(APP_STATE_WAIT);
            break;
        }
        
        case APP_STATE_WAIT :
          // nothing to do
        break;
        
        case APP_STATE_SERVICE_TASKS:
        {
            valADC = BSP_ReadAllADC();    // Récupération des valeurs de tension et de courant
            
            // S'il y n'y apas de surcourant
            if((valADC.Chan0 <= COURANTMAX) && (!overCurrent))
            {
                Uk = APP_CalculePI(valADC.Chan1);   // Calcul du PI par rapport à la tension lue
                APP_ReglagePWM(Uk);     // Réglage du PWM
            }
            else
            {
                // Surcourant
                // Arrêter le convertisseur
                DRV_OC0_PulseWidthSet(0);
                overCurrent = 1;    // Il y a un surcourant
                overcurrentTimer++;
                // Si le courant est plus bas que la limite basse
                if((valADC.Chan0 <= COURANTLIMITBASSE) && (overcurrentTimer > OVERCURRENTWAIT))
                {
                    overCurrent = 0;    // Il n'y a plus de surrocurant
                    overcurrentTimer = 0;
                }
            }
            APP_UpdateState(APP_STATE_WAIT);
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

void APP_UpdateState ( APP_STATES NewState )
{
    appData.state = NewState;
}

// Fonction de calcul du PI par rapport à l'erreur
// Auteur: JAR & DBS
// Date: 22.05.2024
// Entrées: uint16_t valLue
// Sortie: float
float APP_CalculePI(uint16_t valLue)
{
    //-- variable liée aux différents correcteur PID --// 
    float Up_k = 0;        // facteur proportionnelle; 
    float Ui_k = 0;        // facteur intégrateur => actuel 
    static float Ui_k_1 = 0;      // facteur intégrateur => passé  
    float Uk;              // facteur globale
    
    float erreur;
    
    erreur = (CONSIGNE - (float)valLue)/1024;
    
    //-- proportionnel --//
    Up_k = KP * erreur;
    
    //-- intégrateur --// 
    Ui_k = KI * erreur + Ui_k_1; 
    // Antiwind-up
    if(Ui_k > 1)
    {
        Ui_k = 1;
    }
    else if(Ui_k < 0)
    {
        Ui_k = 0;
    }
    
    //-- final --// 
    Uk = Up_k + Ui_k;
    
    Ui_k_1 = Ui_k;  // Sauvegarde de l'ancien Ui
    
    if(Uk > 0.9)
    {
        Uk = 0.9;
    }
    else if(Uk < 0.1)
    {
        Uk = 0.1;
    }
    
    return Uk;
}

// Fonction de réglage du PWM par rapport au facteur PI
// Auteur: JAR & DBS
// Date: 22.05.2024
// Entrées: float Uk
// Sortie: -
void APP_ReglagePWM(float Uk)
{
    static float pulseWidht = CONSIGNE;
    
    //pulseWidht = pulseWidht - Uk;
    pulseWidht = Uk*1999;
    // Limitation de la largeur de pulse pour avoir toujours le bootstrap qui peut fonctionner
//    if(pulseWidht > PULSWIGHTMAX)
//    {
//        pulseWidht = PULSWIGHTMAX;
//    }
//    else if(pulseWidht < PULSWIGHTMIN)
//    {
//        pulseWidht = PULSWIGHTMIN;
//    }
    DRV_OC0_PulseWidthSet(pulseWidht);  // *65535/1023 = 64
}

/*******************************************************************************
 End of File
 */
