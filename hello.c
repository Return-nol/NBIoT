#include <msp430.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

//#include "HAL_UCS.h"

#define MHZ20 1
#define SMCLK_115200     0
#define SMCLK_9600      1
#define ACLK_9600       2

#define UART_MODE       ACLK_9600//SMCLK_115200//
int prev_val=99;

void InitIoConfig(void) {
    P3SEL = BIT4 | BIT5;                        // P3.4,5 = USCI_A0 TXD/RXD
    //P5SEL = BIT6 | BIT7;                          // p5.6,7 = USCI_A0 TXD/RXD
    //P9SEL = BIT4 | BIT5;                          // p5.6,7 = USCI_A0 TXD/RXD
    //P10SEL = BIT4 | BIT5;                          // p5.6,7 = USCI_A0 TXD/RXD
    P7SEL |= BIT0 | BIT1;                         // Select XT1
}

void InitClock() {

    UCSCTL3 |= SELREF_2;                      // Set DCO FLL reference = REFO
        UCSCTL4 |= SELA_0;                        // Set ACLK = XT1CLK
        __bis_SR_register(SCG0);                  // Disable the FLL control loop
        UCSCTL0 = 0x0000;                         // Set lowest possible DCOx, MODx
        UCSCTL1 = DCORSEL_5;                      // Select DCO range 16MHz operation
        UCSCTL2 = FLLD_0 + 487;                   // Set DCO Multiplier for 16MHz
                                                  // (N + 1) * FLLRef = Fdco
                                                  // (487 + 1) * 32768 = 16MHz
                                                  // Set FLL Div = fDCOCLK
        __bic_SR_register(SCG0);                  // Enable the FLL control loop

        // Worst-case settling time for the DCO when the DCO range bits have been
        // changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 5xx
        // UG for optimization.
        // 32 x 32 x 16 MHz / 32,768 Hz = 500000 = MCLK cycles for DCO to settle
        __delay_cycles(500000);//
        // Loop until XT1,XT2 & DCO fault flag is cleared
        do
        {
            UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG); // Clear XT2,XT1,DCO fault flags
            SFRIFG1 &= ~OFIFG;                          // Clear fault flags
        }while (SFRIFG1&OFIFG);                         // Test oscillator fault flag
}

void InitUart() {
    // Configure USCI_A0 for UART mode
        UCA0CTLW0 = UCSWRST;                      // Put eUSCI in reset
    #if UART_MODE == SMCLK_115200

        UCA0CTLW0 |= UCSSEL__SMCLK;               // CLK = SMCLK
        // Baud Rate calculation
        // 16000000/(16*115200) = 8.6805
        // Fractional portion = 0.6805
        // Use Table 24-5 in Family User Guide
        UCA0BR0 = 8;                               // 16000000/16/9600
        UCA0BR1 = 0x00;
        UCA0MCTL |= UCOS16 | UCBRF_11 | UCBRS_0;

    #elif UART_MODE == SMCLK_9600

        UCA0CTLW0 |= UCSSEL__SMCLK;               // CLK = SMCLK
        // Baud Rate calculation
        // 16000000/(16*9600) = 104.1667
        // Fractional portion = 0.1667
        // Use Table 24-5 in Family User Guide
        UCA0BR0 = 104;                            // 16000000/16/9600
        UCA0BR1 = 0x00;
        UCA0MCTL |= UCOS16 | UCBRF_3 | UCBRS_0;

    #elif UART_MODE == ACLK_9600

        UCA0CTLW0 |= UCSSEL__ACLK;               // CLK = ACLK
        // Baud Rate calculation
        // 32768/(9600) = 3.4133
        // Fractional portion = 0.4133
        // Use Table 24-5 in Family User Guide
        UCA0CTL0 = 0x00; // default mode...8 data bits, 1 stop //bit, async
        UCA0BR0 = 3;                             // 32768/9600
        UCA0BR1 = 0x00;
        UCA0MCTL |= UCBRS_3;    //0x0300 is UCBRSx = 0x03

    #else
        # error "Please specify baud rate to 115200 or 9600"
    #endif

        UCA0CTLW0 &= ~UCSWRST;                    // Initialize eUSCI
        UCA0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt
}

void send(char c) {
    while (!(UCA0IFG & UCTXIFG))
        ;             // USCI_A0 TX buffer ready?
    UCA0TXBUF = c;                  // TX -> RXed character
}

void sendZ(char* p) {
    while (*p) {
        send(*p);
        p++;
    }
}

void sendN(short c) {
    __no_operation();
    if (c==0) {
        sendZ("0");
    } else {
        char buffer[7];
        char* p=p=buffer+6;
        while(c>0) {
            char n=c%10;
            c/=10;
            --p;
            *p='0'+n;
        }
        __no_operation();
        char* end=buffer+6;
        while(end!=p) {
            __no_operation();
            send(*p);
            ++p;
        }
        __no_operation();
    }
}

volatile short counter=0;

volatile short readFlag=0;
volatile short readflags2=0;
int gotChar() {
    short f=readFlag;
    if (f!=readflags2) {
        readflags2=f;
        return 1;
    }
    return 0;
}

void init_nbiot(){
    /*sendZ("AT+NRB"); // COMMAND FOR INITIALIZING GSM
    send(0x0A);//ENTER
    send(0x0D);//CARRIAGE RETURN*/
    //sendZ("\r\nAT+NRB\r\n"); // COMMAND FOR INITIALIZING GSM

    __delay_cycles(30000000);//DELAY...WAIT FOR OK FROM GSM
    sendZ("\r\nAT+CMEE=1\r\n"); // COMMAND FOR ENABLING CHECK ERROR CODE
    //send(0x0A);//ENTER
    //send(0x0D);//CARRIAGE RETURN

    __delay_cycles(30000000);//DELAY...WAIT FOR OK FROM GSM
    sendZ("\r\nAT+CSCON=1\r\n");//INDICATE CONNECTION
    __delay_cycles(30000000);//DELAY...WAIT FOR OK FROM GSM
    sendZ("\r\nAT+CSQ\r\n");//CHECK SIGNAL
    //send(0x0A);
    //send(0x0D);
    __delay_cycles(30000000);//DELAY...WAIT FOR OK FROM GSM
    sendZ("\r\nAT+CEREG?\r\n");//CHECK NETWORK REGISTERED OR NOT
    __delay_cycles(30000000);//WAIT FOR OK
    sendZ("\r\nAT+NPING=8.8.8.8\r\n");//SEND A MESSAGE TO GOOGLE DNS
    __delay_cycles(50000000);//WAIT FOR OK
    //send(0x0A);
    //send(0x0D);
    __delay_cycles(30000000);//WAIT FOR OK
    sendZ("\r\nAT+NSOCR=DGRAM,17,5678,1\r\n");//CREATE SOCKET
    __delay_cycles(30000000);//WAIT FOR OK
    sendZ("\r\nAT+NSORF=0,100\r\n");//CHECK MESSAGE RECEIVE IN SOCKET 0
    //send(0x0A);
    //send(0x0D);
    __delay_cycles(30000000);//WAIT FOR OK
    //AFTER HARDWARE CONFIGURATION THE MESSAGE WILL GET SEND
    //ATTACH THE UART FILES OR WRITE THE CODE FOR INIT AND SENDING MESSAGE IN THE SAME FILE...
}

void up_nbiot(int value){
    if (value==1&&prev_val!=1){
        sendZ("\r\nAT+NSOST=0,211.20.181.199,5678,32,a28C08115101000000801300444b425a47473746594d4d4b325a583548353199\r\n"); // COMMAND FOR INITIALIZING GSM
        prev_val=1;
    }
    if (value==0&&prev_val!=0){
        sendZ("\r\nAT+NSOST=0,211.20.181.199,5678,32,a28C08115101000000801300444b425a47473746594d4d4b325a58354835309a\r\n"); // COMMAND FOR INITIALIZING GSM
        prev_val=0;
    }
    //send(0x0A);//ENTER
    //send(0x0D);//CARRIAGE RETURN
    __delay_cycles(10000000);//DELAY...WAIT FOR OK FROM GSM
    //sendZ("\r\nAT+NPING=8.8.8.8\r\n");//SEND A MESSAGE TO GOOGLE DNS
}

uint16_t setVCoreUp(uint8_t level){
    uint32_t PMMRIE_backup, SVSMHCTL_backup, SVSMLCTL_backup;

    //The code flow for increasing the Vcore has been altered to work around
    //the erratum FLASH37.
    //Please refer to the Errata sheet to know if a specific device is affected
    //DO NOT ALTER THIS FUNCTION

    //Open PMM registers for write access
    PMMCTL0_H = 0xA5;

    //Disable dedicated Interrupts
    //Backup all registers
    PMMRIE_backup = PMMRIE;
    PMMRIE &= ~(SVMHVLRPE | SVSHPE | SVMLVLRPE |
                SVSLPE | SVMHVLRIE | SVMHIE |
                SVSMHDLYIE | SVMLVLRIE | SVMLIE |
                SVSMLDLYIE
                );
    SVSMHCTL_backup = SVSMHCTL;
    SVSMLCTL_backup = SVSMLCTL;

    //Clear flags
    PMMIFG = 0;

    //Set SVM highside to new level and check if a VCore increase is possible
    SVSMHCTL = SVMHE | SVSHE | (SVSMHRRL0 * level);

    //Wait until SVM highside is settled
    while((PMMIFG & SVSMHDLYIFG) == 0)
    {
        ;
    }

    //Clear flag
    PMMIFG &= ~SVSMHDLYIFG;

    //Check if a VCore increase is possible
    if((PMMIFG & SVMHIFG) == SVMHIFG)
    {
        //-> Vcc is too low for a Vcore increase
        //recover the previous settings
        PMMIFG &= ~SVSMHDLYIFG;
        SVSMHCTL = SVSMHCTL_backup;

        //Wait until SVM highside is settled
        while((PMMIFG & SVSMHDLYIFG) == 0)
        {
            ;
        }

        //Clear all Flags
        PMMIFG &= ~(SVMHVLRIFG | SVMHIFG | SVSMHDLYIFG |
                     SVMLVLRIFG | SVMLIFG |
                     SVSMLDLYIFG
                     );

        //Restore PMM interrupt enable register
        PMMRIE = PMMRIE_backup;
        //Lock PMM registers for write access
        PMMCTL0_H = 0x00;
        //return: voltage not set
        return false;
    }

    //Set also SVS highside to new level
    //Vcc is high enough for a Vcore increase
    SVSMHCTL |= (SVSHRVL0 * level);

    //Wait until SVM highside is settled
    while((PMMIFG & SVSMHDLYIFG) == 0)
    {
        ;
    }

    //Clear flag
    PMMIFG &= ~SVSMHDLYIFG;

    //Set VCore to new level
    PMMCTL0_L = PMMCOREV0 * level;

    //Set SVM, SVS low side to new level
    SVSMLCTL = SVMLE | (SVSMLRRL0 * level) |
               SVSLE | (SVSLRVL0 * level);

    //Wait until SVM, SVS low side is settled
    while((PMMIFG & SVSMLDLYIFG) == 0)
    {
        ;
    }

    //Clear flag
    PMMIFG &= ~SVSMLDLYIFG;
    //SVS, SVM core and high side are now set to protect for the new core level

    //Restore Low side settings
    //Clear all other bits _except_ level settings
    SVSMLCTL &= (SVSLRVL0 + SVSLRVL1 + SVSMLRRL0 +
                 SVSMLRRL1 + SVSMLRRL2
                 );

    //Clear level settings in the backup register,keep all other bits
    SVSMLCTL_backup &=
        ~(SVSLRVL0 + SVSLRVL1 + SVSMLRRL0 + SVSMLRRL1 + SVSMLRRL2);

    //Restore low-side SVS monitor settings
    SVSMLCTL |= SVSMLCTL_backup;

    //Restore High side settings
    //Clear all other bits except level settings
    SVSMHCTL &= (SVSHRVL0 + SVSHRVL1 +
                 SVSMHRRL0 + SVSMHRRL1 +
                 SVSMHRRL2
                 );

    //Clear level settings in the backup register,keep all other bits
    SVSMHCTL_backup &=
        ~(SVSHRVL0 + SVSHRVL1 + SVSMHRRL0 + SVSMHRRL1 + SVSMHRRL2);

    //Restore backup
    SVSMHCTL |= SVSMHCTL_backup;

    //Wait until high side, low side settled
    while(((PMMIFG & SVSMLDLYIFG) == 0) &&
          ((PMMIFG & SVSMHDLYIFG) == 0))
    {
        ;
    }

    //Clear all Flags
    PMMIFG &= ~(SVMHVLRIFG | SVMHIFG | SVSMHDLYIFG |
                SVMLVLRIFG | SVMLIFG | SVSMLDLYIFG
                );

    //Restore PMM interrupt enable register
    PMMRIE = PMMRIE_backup;

    //Lock PMM registers for write access
    PMMCTL0_H = 0x00;

    return true;
}

bool increaseVCoreToLevel2()
{
    uint8_t level = 2;
    uint8_t actlevel;
    bool status = true;

    //Set Mask for Max. level
    level &= PMMCOREV_3;

    //Get actual VCore
    actlevel = PMMCTL0 & PMMCOREV_3;

    //step by step increase or decrease
    while((level != actlevel) && (status == true))
    {
        if(level > actlevel)
        {
            status = setVCoreUp(++actlevel);
        }
    }

    return (status);
}

int main(void) {
    //int c;
    WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

    InitIoConfig();
    increaseVCoreToLevel2();
    InitClock();
    InitUart();

    __enable_interrupt();
    init_nbiot();

    printf("Testing\n");
        while(1){
            if ((P1IN & 0x01) == 0){
                //printf("0 Nothing\n");
                up_nbiot(0);

            } else {
                //printf("1 DETECTED!!\n");
                up_nbiot(1);
            }
            /*sendZ("AT\n");
            __bis_SR_register(LPM3_bits + GIE);*/
        }
}
unsigned char RX;
unsigned char TX;
// Echo back RXed character, confirm TX buffer is ready first
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch (__even_in_range(UCA0IV, 4)) {
    case 0:
        break;                             // Vector 0 - no interrupt
    case 2:                                   // Vector 2 - RXIFG
        while (!(UCA0IFG&UCTXIFG));             // USCI_A0 TX buffer ready?
        UCA0TXBUF = UCA0RXBUF;                  // TX -> RXed character
        if (UCA0RXBUF==0x0A){
            send(UCA0RXBUF);
            RX = UCA0RXBUF;
                    printf("%c",RX);
        }
        send(UCA0RXBUF);
        RX = UCA0RXBUF;
        printf("%c",RX);
        readFlag++;
        break;
    case 4:                                 // Vector 4 - TXIFG
        break;
    default:
        break;
    }
    //MarkInterruptInt(INT_USART_RS232);
    //LPM0_EXIT;
}
/*
#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR(void)
{
    GreenOn();
    __no_operation();
    int iv=P1IV;
    counter++;
    RedToggle();
    GreenOff();
}*/
