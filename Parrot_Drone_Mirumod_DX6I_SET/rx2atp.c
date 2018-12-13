#define  AT2SO 1
/*
 * what: rx2at - RC receiver servo signal to ASCII conversion
 * who:  miru
 * when: April 2011...
 */
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
const char version[] PROGMEM = "0.22 20140720";

#define RX_CON  0 /* 0 for standard mount, 1 for reversed mount */
#define IN_AUX  1 /* 1 for AUX1 channel inversion (legacy...) */
#define NL_AIR  0 /* 1 to turn off Arduino LED when airborne */

/* switch setup, S_GEAR-(2 pos), S_AUX1-(2 or 3 pos) *
 * S_LAND S_FMOD
 * S_AUX1 S_AUX1 one 3 position switch (e.g. DX7)
 * S_GEAR S_GEAR one 2 position switch
 * S_GEAR S_AUX1 two switches (e.g. DX6)
 * S_AUX1 S_GEAR two switches */
#define S_LAND  S_GEAR
#define S_FMOD  S_AUX1

/* transmitter mode, channel assignments of the sticks
 * T_MODE 1 -> left: V-ELEV H-RUDD right: V-THRO H-AILE
 * T_MODE 4 -> left: V-THRO H-AILE right: V-ELEV H-RUDD
 * T_MODE 2 -> left: V-THRO H-RUDD right: V-ELEV H-AILE, US common mode
 * T_MODE 3 -> left: V-ELEV H-AILE right: V-THRO H-RUDD, US reversed */
#define T_MODE  2

/* drone configuration choices
 * outdoor:     FALSE or TRUE, use (0 or 1)
 * no_shell:    FALSE or TRUE, use (0 or 1)
 * max_euler:   0.0 ... 0.52 max pitch/roll angle [rad]
 * max_vz:      200 ... 2000 max climb speed [mm/s]
 * max_yaw      0.7 ... 6.11 max yaw speed [rad/s]
 * max_alt:     altitude limit [m] (0 is off)
 * animation:   0 - disable, enabled otherwise
 * videorecord: 0 - disable, 1 - enable (NOOP on Drone 1 or Drone 2 without USB stick)
 * dcat:        TX/RX disconnect action deferral [sec]
 *            outdoor,no_shell,max_euler,max_vz,max_yaw,max_alt,anim,rvid,dcat */
const char cfg1[] PROGMEM = "1,1,0.35,2000,3.5,0,1,1,30"; /* S standard */
const char cfg2[] PROGMEM = "1,1,0.52,2000,6.1,0,1,1,30"; /* W wild (max) */
const char cfg3[] PROGMEM = "0,0,0.21,700,1.75,2,0,0,5";  /* N normal */
const char cfg4[] PROGMEM = "0,0,0.10,700,1.50,2,0,0,5";  /* E easy */

/* VLBA output, Visible Low Battery Alert
 * the program can watch the battery capacity (percentage) on the drone, if it
 * goes below VLBA_THR the VLBA mechanism triggers. The drone's LEDs flash in RED
 * and VLBA output gets activated (blinking or steady), set to 0 to turn VLBA off */
#define VLBA_THR  15  /* 0 or > 60 turns it off */
#define VLBA_POL  1 /* 1-active high, 0-active low */
#define VLBA_BLINK  1 /* 1-blink, 0-no blink */

/* GSWO output, you can route the GEAR channel status to this output to switch
 * something ON/OFF or use it as the output of the BEEP sequencer */
#define GSWO_ENA  -1  /* -1 disable, 0 active low, 1 active high */
#define GSWO_SRC  0 /* 0 - GEAR channel, 1 - BEEP output */


#define LOOPHZ  30  /* loop frequency */
#define S2LTIC(s) (u08_t)((double)LOOPHZ*(double)(s)+0.5)


#if defined(__AVR_ATmega328P__)
#define U4  0
/* Arduino Nano and ProMini */
/* LABEL PORT  DIR description
 *   RX0 PD.0  IN  UART receive
 *   TX1 PD.1  OUT UART transmit
 *   D2  PD.2  IN  RX AUX1 channel
 *   D3  PD.3  IN  RX GEAR channel
 *   D4  PD.4  IN  RX RUDD channel
 *   D5  PD.5  IN  RX ELEV channel
 *   D6  PD.6  IN  RX AILE channel
 *   D7  PD.7  IN  RX THRO channel
 * LABEL PORT  DIR description
 *   D8  PB.0
 *   D9  PB.1  OUT VLBA
 *   D10 PB.2  OUT GSWO (if GSWO_ENA >= 0)
 *   D11 PB.3  IN  SETUP input
 *   D12 PB.4
 *   D13 PB.5  OUT (LED,   Arduino comes this way)
 *   -   PB.6  -   (quarz, Arduino comes this way)
 *   -   PB.7  -   (quarz, Arduino comes this way)
 * LABEL PORT  DIR description
 *   A0  PC.0  IN  GPS RX
 *   A1  PC.1  OUT GPS TX
 *   A2  PC.2
 *   A3  PC.3  
 *   A4  PC.4  I/O (SDA two wire interface)
 *   A5  PC.5  I/O (SCL two wire interface)
 *   A6  -         (analog input 6, Nano and newer ProMini)
 *   A7  -         (analog input 7, Nano and newer ProMini)
 *
 * TIMER0: 'soft' serial port for GPS
 * TIMER1: system clock
 * TIMER2: not used
 */
#define SIO_UDR   UDR0
#define SIO_RX_VECT USART_RX_vect
#define SIO_TX_VECT USART_UDRE_vect
#define SIO_UCSRA UCSR0A
#define SIO_UCSRB UCSR0B
#define SIO_UCSRC UCSR0C
#define SIO_UBRR  UBRR0
#define SIO_U2X   U2X0
#define SIO_UDRIE UDRIE0
#define SIO_RXCIE RXCIE0
#define SIO_RXEN  RXEN0
#define SIO_TXEN  TXEN0
#define PIN_CHG_IRQEN() PCICR |= _BV(PCIE2)|_BV(PCIE1)

#elif defined(__AVR_ATmega32U4__)
#define U4  1
/* MiruPCB board layout
 * LABEL PORT  notes         DIR description
 *       PB.0
 * B1    PB.1  (SCK)         IN  RX THRO
 * B2    PB.2  (MOSI)        IN  RX AILE
 * B3    PB.3  (MISO)        IN  RX ELEV
 * B4    PB.4                IN  RX RUDD
 * B5    PB.5                IN  RX GEAR
 * B6    PB.6                IN  RX AUX1
 * RST   PB.7                (O) RESET*
 * LABEL PORT  notes         DIR description
 *       PC.6
 * LED   PC.7  LED green     OUT activity LED
 * LABEL PORT  notes         DIR description
 * SCL   PD.0  (SCL)         I/O (two wire interface)
 * SDA   PD.1  (SDA)         I/O (two wire interface)
 *       PD.2  (UART RX)     IN  UART receive
 *       PD.3  (UART TX)     OUT UART transmit
 * GTX   PD.4                OUT GPS  transmit
 *       PD.5
 * VLBA  PD.6                OUT VLBA
 * GSWO  PD.7                OUT GSWO
 * LABEL PORT  notes         DIR description
 *       PE.2  (HWB)
 * GRX   PE.6  (INT6)        IN  GPS receive
 * LABEL PORT  notes         DIR description
 *       PF.0
 *       PF.1
 *       PF.4
 *       PF.5
 *       PF.6               (OUT debug signal 1)
 *       PF.7               (OUT debug signal 0)
 *
 * TIMER0: 'soft' serial port for GPS
 * TIMER1: system clock
 * TIMER2: not used
 */
#define SIO_UDR   UDR1
#define SIO_RX_VECT USART1_RX_vect
#define SIO_TX_VECT USART1_UDRE_vect
#define SIO_UCSRA UCSR1A
#define SIO_UCSRB UCSR1B
#define SIO_UCSRC UCSR1C
#define SIO_UBRR  UBRR1
#define SIO_U2X   U2X1
#define SIO_UDRIE UDRIE1
#define SIO_RXCIE RXCIE1
#define SIO_RXEN  RXEN1
#define SIO_TXEN  TXEN1
#define PIN_CHG_IRQEN() PCICR |= _BV(PCIE0), EIFR |= _BV(6)

#else
#error "sorry, processor not supported"
#endif

/* NOOPs, get redefined if needed */
#define usb_sq()
#define i2c_setup()
#define bmp_sq()
#define bmp_rp(upd) 0
#define m56_sq()
#define m56_rp(upd) 0
#define hmc_sq()
#define hmc_rp(upd) 0

void port_setup(void)
{
#if U4 == 0   /* is Arduino Nano or ProMini */
  PORTB = 0b00000000;
  DDRB  = 0b00000000;

/* (label 13 on Arduino) HW comes with a LED on this pin */
#define LED _BV(5)
#define LEDON() PORTB |=  LED
#define LEDOF() PORTB &= ~LED
#define LEDTG() PORTB ^=  LED
  DDRB |= LED;  /* output */

/* (label 11 on Arduino) ground for SETUP */
  PORTB |= _BV(3);  /* enable pull-up */
#define SETUP() ((PINB & _BV(3)) == 0)

/* (label 10 on Arduino) reflects GSWO_SRC if enabled */
#define GSWO  _BV(2)
#if GSWO_ENA >= 0
  #if GSWO_ENA > 0
  #define GSWON() PORTB |=  GSWO
  #define GSWOF() PORTB &= ~GSWO
  #else
  #define GSWON() PORTB &= ~GSWO
  #define GSWOF() PORTB |=  GSWO
  #endif
  DDRB |= GSWO; /* output */
  GSWOF();
#else
  #define GSWON()
  #define GSWOF()
#endif

/* (label  9 on Arduino) VLBA output pin */
#define VLBA  _BV(1)
#if VLBA_POL == 1
#define VLBON() PORTB |=  VLBA
#define VLBOF() PORTB &= ~VLBA
#else
#define VLBOF() PORTB |=  VLBA
#define VLBON() PORTB &= ~VLBA
#endif
#define VLBTG() PORTB ^=  VLBA
  VLBOF();
  DDRB |= VLBA; /* output */

/* I2C interface */
  PORTC = 0b00000000;
  PORTC |= _BV(5)|_BV(4); /* enable pullups for SDA/SCL */
  DDRC  = 0b00000000;

/* (label A0 on Arduino) GPS serial port in */
#define GPS_RXI_VECT  PCINT1_vect
#define GPS_RXI _BV(0)
#define GPS_RXI_INPUT() (PINC & GPS_RXI)
#define GPS_RXI_IRQON() PCMSK1 |=  GPS_RXI
#define GPS_RXI_IRQOF() PCMSK1 &= ~GPS_RXI
  PORTC |=  GPS_RXI;  /* enable pullup on RXI */

/* (label A1 on Arduino) GPS serial port out */
#define GPS_TXO _BV(1)
#define GPS_TXO_SET() PORTC |=  GPS_TXO
#define GPS_TXO_CLR() PORTC &= ~GPS_TXO
#define GPS_TXO_VAL() (PORTC & GPS_TXO)
  DDRC  |=  GPS_TXO;  /* TXO is output */
  PORTC |=  GPS_TXO;  /* set TXO idle */

#define RXSIG_VECT  PCINT2_vect
#define RXSIG   PIND
#define RXMSK   (_BV(7)|_BV(6)|_BV(5)|_BV(4)|_BV(3)|_BV(2))
  PCMSK2 = RXMSK;
  PORTD =  RXMSK|03;    /* enable pull-ups for receiver inputs, and UART signals */
  DDRD  = 0b00000000;


#else /* is MiruPCB */
#define RXSIG_VECT  PCINT0_vect
#define RXSIG   PINB
#define RXMSK   (_BV(6)|_BV(5)|_BV(4)|_BV(3)|_BV(2)|_BV(1))
  PCMSK0 = RXMSK;
  PORTB  = RXMSK; /* enable pull-ups for receiver and GPS inputs */
  DDRB   = 0b00000000;

  PORTC = 0b00000000;
  DDRC  = 0b00000000;
/* HW comes with a LED on this pin */
#define LED _BV(7)
#define LEDON() PORTC |=  LED
#define LEDOF() PORTC &= ~LED
#define LEDTG() PORTC ^=  LED
  DDRC  |= LED; /* output */

/* I2C interface */
  PORTD = 0b00001111;     /* enable pull-ups for UART and I2C signals */
  DDRD  = 0b00000000;
/* GSWO output pin */
#define GSWO  _BV(7)
#if GSWO_ENA >= 0
  #if GSWO_ENA > 0
  #define GSWON() PORTD |=  GSWO
  #define GSWOF() PORTD &= ~GSWO
  #else
  #define GSWON() PORTD &= ~GSWO
  #define GSWOF() PORTD |=  GSWO
  #endif
  DDRD  |= GSWO;  /* output */
#else
  #define GSWON()
  #define GSWOF()
#endif
  GSWOF();
/* VLBA output pin */
#define VLBA  _BV(6)
#if VLBA_POL == 1
#define VLBON() PORTD |=  VLBA
#define VLBOF() PORTD &= ~VLBA
#else
#define VLBOF() PORTD |=  VLBA
#define VLBON() PORTD &= ~VLBA
#endif
#define VLBTG() PORTD ^=  VLBA
  VLBOF();
  DDRD  |= VLBA;    /* output */
/* GPS serial port out */
#define GPS_TXO _BV(4)
#define GPS_TXO_SET() PORTD |=  GPS_TXO
#define GPS_TXO_CLR() PORTD &= ~GPS_TXO
#define GPS_TXO_VAL() (PORTD & GPS_TXO)
  PORTD |= GPS_TXO; /* set TXO idle */
  DDRD  |= GPS_TXO; /* output */

  PORTE = 0b00000000;
/* GPS serial port in */
#define GPS_RXI_VECT  INT6_vect
#define GPS_RXI _BV(6)
#define GPS_RXI_INPUT() (PINE & GPS_RXI)
#define GPS_RXI_IRQON() EIMSK |=  GPS_RXI
#define GPS_RXI_IRQOF() EIMSK &= ~GPS_RXI
  PORTE |= GPS_RXI; /* enable pullup on RXI */
  DDRE  = 0b00000000;
  EICRB = 0b00010000; /* -,-,isc61,ISC60,-,-,-,- */

  PORTF = 0b00000000;
  DDRF  = 0b00000000;
#endif

  MCUCR = 0b00000000; /* PUD=0 clear global pull-up disable */
}

void sio_setup(void)
{
  SIO_UCSRB = 0b00000000; /* rxcie0,txcie0,udrie0,rxen0,txen0,ucsz02,rxb80,txb80 */
  SIO_UCSRC = 0b00000110; /* umsel01,umsel00,upm01,upm00,usbs0,UCSZ01,UCSZ00,ucpol0 */
  SIO_UCSRA = 0b00000000; /* rxc0,txc0,udre0,fe0,dor0,upe0,u2x0,mpcm0 */
#if F_CPU == 16000000UL /* (see NOTES USART) */
  SIO_UBRR  = 16;
  SIO_UCSRA |= _BV(SIO_U2X);
#else
  SIO_UBRR  = (F_CPU/(16UL*115200UL)-1);
#endif
}

void tmr1_setup(void)
{
#define F_TM1   (F_CPU/8UL) /* (see NOTES TIMER1) */
#define MS2TIC(ms)  (u32_t)((double)F_TM1*(double)(ms)*1e-3)
#define TIC2MS(tic) (u32_t)((tic)/(F_TM1/1000L))
#define TIC2MS10(tic) (u32_t)((tic)/(F_TM1/10000L))
#define TIC2US(tic) (u32_t)((tic)/(F_TM1/1000000L))

  TCCR1A = 0b00000000;  /* com1a1,com1a0,com1b1,com1b0,-,-,wgm11,wgm10 */
  TCCR1B = 0b00000010;  /* icnc1,ices1,-,wgm13,wgm12,cs12,CS11,cs10 F_CPU/8 */
  TCCR1C = 0b00000000;  /* foc1a,foc1b,-,-,-,-,-,- */
  TIMSK1 = 0b00000001;  /* -,-,icie1,-,-,ocie1b,ocie1a,TOIE1 */
}

#define NEL(x)    (sizeof(x)/sizeof(x[0]))
#define ABS(x)    ((x)>=0?(x):-(x))
#define MIN(a,b)  ((a)<=(b)?(a):(b))
/* circular buffers */
#define CB_FILL(r,w,s)  (((w)-(r)+(s))%(s)) /* available elements for get */
#define CB_FREE(r,w,s)  (((r)-(w)+(s)-1)%(s)) /* available elements for put */
#define CB_HASD(r,w)  ((r)!=(w))    /* has data */
#define CB_NO_D(r,w)  ((r)==(w))    /* has no data */

typedef char    s08_t;
typedef volatile s08_t  vs08_t;
typedef unsigned char u08_t;
typedef volatile u08_t  v08_t;
typedef unsigned short  u16_t;
typedef volatile u16_t  v16_t;
typedef unsigned long u32_t;

typedef struct {
  u32_t dcst; /* drone status */
  char  cbat; /* battery percentage left */
  char  beep;
  char  sgps; /* GPS status */
} __attribute__ ((packed)) m2a_t;

typedef struct {  /* radio signal sample */
  u08_t smp;
  u32_t tmr;
} __attribute__ ((packed)) rsm_t;

struct {
  v16_t t1ov; /* timer 1 overflow count */
  v16_t dcnt; /* delay count for ms_dly() */
  u08_t exit;
  u08_t alin;

  struct { /* UART is connected to drone */
    /* circular receive buffer (r == w -> empty) */
    v08_t rxw;
    v08_t rxr;
    u08_t rxb[32];
    /* circular transmit buffer (r == w -> empty) */
    v08_t txw;
    v08_t txr;
    u08_t txb[200];
  } sio;

#if U4
  struct { /* FTDI simulation */
    u08_t ep0a; /* interface active */
    u08_t ep0r; /* EP0 received */
    union { /* setup request packet */
      u08_t b[8];
      struct {
        u08_t bmRequestType;
        #define REQ_DIR 0x80
        #define  REQ_H2D  0x00
        #define  REQ_D2H  0x80
        #define REQ_TYP 0x60
        #define  REQ_STD  0x00
        #define  REQ_CLS  0x20
        #define  REQ_VEN  0x40
        #define REQ_DST 0x03
        #define  REQ_DEV  0x00
        #define  REQ_IFC  0x01
        #define  REQ_EPT  0x02
        #define  REQ_OTH  0x03
        u08_t bRequest;
        #define GET_STATUS  0
        #define CLR_FEA   1
        #define SET_FEA   3
        #define SET_ADR   5
        #define GET_DSC   6
        #define SET_DSC   7
        #define GET_CFG   8
        #define SET_CFG   9
        #define GET_IFC   10
        #define SET_IFC   11
        u16_t wValue;
        u16_t wIndex;
        u16_t wLength;
      };
    } ep0b;
    /* EP1 management dev->host */
    u08_t sofi;
    v08_t ep1h;
    v08_t ep1r;
    v08_t ep1w;
    u08_t ep1b[200];
    /* EP2 management host->dev */
    v08_t ep2r;

    /* FTDI simulator */
    u08_t mdst; /* modem status */
    #define MDST_RLSD _BV(7)  /* receive line signal */
    #define MDST_RI   _BV(6)  /* ring indicator */
    #define MDST_DSR  _BV(5)  /* data set ready */
    #define MDST_CTS  _BV(4)  /* clear to send */
    #define RST_MDST  (MDST_DSR|MDST_CTS|_BV(0))
    u08_t lnst; /* line status */
    #define LNST_FIFO _BV(7)  /* fifo error */
    #define LNST_TEMT _BV(6)  /* transmitter empty */
    #define LNST_THRE _BV(5)  /* transmit holding register enpty */
    #define LNST_BI   _BV(4)  /* break interrupt */
    #define LNST_FE   _BV(3)  /* framing error */
    #define LNST_PE   _BV(2)  /* parity error */
    #define LNST_OE   _BV(1)  /* overrun */
    #define LNST_DTR  _BV(0)  /* data terminal ready */
    #define RST_LNST  (LNST_TEMT|LNST_THRE|LNST_DTR)
    u08_t rbsq; /* reboot sequencer */
    u16_t tlat; /* latency timer */
  } usb;
#endif

  struct { /* messages from drone companion program */
    v08_t ack;
    v08_t cst;
    v08_t nda;
    union {
      u08_t b[0];
      m2a_t m2a;
    } dat;
    m2a_t m2a;
  } drm;

  struct { /* 'soft' serial port for GPS */
    int b100;
    u08_t tcnt;
    u08_t thbt;
    u16_t lstp; /* sample of timer 1 at last stop bit */
    /* receiver */
    v08_t rxi;  /* last value of input GPS_RXI */
    u08_t rxm;  /* data bit mask register */
    u08_t rxd;  /* data shift register */
    v08_t rxw;
    v08_t rxr;
    /* transmitter */
    u08_t txm;
    u08_t txd;
    v08_t txe;
    v08_t txw;
    v08_t txr;
    /* GPS data assembly */
    u08_t est;
    u08_t ech;
    u08_t cst;
    u08_t cks;
    u08_t seq;
    u08_t b_r;
    u08_t b_w;
    u08_t b_s;
    u08_t b_n;
    /* buffers */
    union {
      u32_t rxt[95];  /* for baudrate detection */
      struct {
      u08_t rxb[96];  /* buffers 24.7 ms on 38400 baud stream */
      u08_t txb[32];
      char  buf[252];
      };
    };
  } gps;

  struct {
    u08_t stat;
    u08_t tick;
  } vlba;
#if GSWO_ENA >= 0 && GSWO_SRC == 1
  struct {
    u08_t nbp;
    u08_t tmr;
    u08_t seq;
  } beep;
#endif

  struct { /* RC receiver signals */
    u08_t err;  /* signals with errors */
    u08_t erx;  /* consecutive error count rx_read() */

    u08_t fms;  /* flight mode (rx_read()) */
#define FMS_LAND  0
#define FMS_FM_1  1 /* use camera for stabilization on ROL/PTC sticks centered */
#define FMS_FM_2  2 /* idle up, don't use camera on ROL/PTC stick centered */
#define FMS_FM_3  3 /* auto pilot mode */
    u08_t fsq;  /* flight mode sequencer */
    u08_t fst;  /* flight mode sequencer tic */
    u08_t fsa;  /* flight mode sequencer animation trigger */

    u08_t stk;  /* stick status (rx_read()) */
    u08_t chg;  /* stick status change (dr_loop() private) */
#define TS_RT 0x01
#define TS_LF 0x02
#define TS_UP 0x04
#define TS_DN 0x08
#define TS_MSK  0x0f
#define ES_RT 0x10
#define ES_LF 0x20
#define ES_UP 0x40
#define ES_DN 0x80
#define ES_MSK  0xf0

    u08_t smp;  /* last signal sample */
    u08_t rsr;  /* read index radio sampling buffer */
    u08_t rsw;  /* write index radio sampling buffer */
    rsm_t rsb[32];
#define S_AILE  0
#define S_ELEV  1
#define S_THRO  2
#define S_RUDD  3
#define S_AUX1  4
#define S_GEAR  5
#define S_NCHN  6
/* Note: independent of TX mode 1-4, throttle and elevator channels are never on the
 * same physical stick and are always controlled by vertical movement of the stick */
#if   T_MODE == 1 || T_MODE == 4
  #define SH_THRO S_AILE
  #define SH_ELEV S_RUDD
#else
  #define SH_THRO S_RUDD
  #define SH_ELEV S_AILE
#endif
#define STKTHR  500 /* throttle/elevator stick threshold for switching */
    struct chn {
      u08_t msk; /* port mask for signal */
      u32_t tup; /* timer1 when it went up */
      u32_t ftm; /* time between the last two ups */
      int dbn; /* signal dead band */
      int dur; /* how long signal was up */
      int val; /* interpreted value (rx_read()) */
    } chn[S_NCHN];
  } rxs;
/* RX channel mapping to drone */
#define S_ROL S_AILE
#define S_PTC S_ELEV
#define S_GAZ S_THRO
#define S_YAW S_RUDD
/* RX channels used */
#if GSWO_ENA >= 0
#define S_SIG (_BV(S_ROL)|_BV(S_PTC)|_BV(S_GAZ)|_BV(S_YAW)|_BV(S_AUX1)|_BV(S_GEAR))
#else
#define S_SIG (_BV(S_ROL)|_BV(S_PTC)|_BV(S_GAZ)|_BV(S_YAW)|_BV(S_LAND)|_BV(S_FMOD))
#endif

  /* drone companion program in flash */
  struct {
    u16_t adr;
    u16_t siz;
    u16_t cks;
  } arm;

  u08_t dsnd; /* snd_sc() destination, 0 sio, 1 usb */
  char  pad;  /* drone control <0 emergency, >0 fly, else land */
  char  cfw;  /* configuration wanted by telling from the sticks */
  u08_t eewen;  /* EEROM */
  union {
    u08_t eedat[6];
    struct {
      u16_t eemag;
      u16_t bid;  /* boot id */
      u08_t cfg;
      u08_t eecks;
    };
  };
#define EE_MAG  0x2701
} gl;

#if GSWO_ENA >= 0 && GSWO_SRC == 1
void beep_sq(void)
{
  if (gl.beep.tmr && (gl.beep.tmr -= 1)) return;
  switch (gl.beep.seq) {
  case 0: if (gl.beep.nbp == 0) break;
    gl.beep.nbp -= 1;
    gl.beep.tmr = S2LTIC(0.25);
    gl.beep.seq = 1;
    GSWON();
    break;
  case 1: gl.beep.tmr = S2LTIC(0.25);
    gl.beep.seq = 0;
    GSWOF();
    break;
  }
}
#else
#define beep_sq()
#endif

/*
 * How the 'soft' serial port for GPS works:
 * TIMER0 (8bit) is setup so it rolls on RS232 bit boundaries, this provides all that's
 * needed to do a transmitter with the timer rollover interrupt. The interrupt is turned
 * off when there is nothing to transmit.
 * To build a receiver at the same baudrate, the 1->0 transition of the RX input is used
 * to set the second compare register of the timer with the current timer value+some,
 * after that the compare interrupt becomes the sampling point for the RX input. Once a
 * byte is received, the RX transition interrupt is reenabled to wait for the next byte.
 */
void tmr0_setup(int b100) /* argument is baudrate/100 */
{
  u16_t cnt;
  u08_t ccr;

  ccr = SREG; cli();
  TIMSK0 = 0b00000000;  /* -,-,-,-,-,ocie0b,ocie0a,toie0 */
  TCCR0A = 0b00000000;  /* com0a1,com0a0,com0b1,com0b0,-,-,wgm01,wgm00  */
  TCCR0B = 0b00000000;  /* foc0a,foc0b,-,-,wgm02,cs02,cs01,cs00  OFF */
  SREG = ccr;
  gl.gps.rxw = gl.gps.rxr = 0;
  gl.gps.txw = gl.gps.txr = 0;
  gl.gps.txe = 0;
  gl.gps.b100 = 0;
  gl.gps.tcnt = 0;
  gl.gps.thbt = 0;
  if (b100 <= 0) return;
  /* turn it on for baudrate b100 * 100 */
  gl.gps.txe = 1;
  cnt = (F_CPU/100UL)/(long)b100;
  for (ccr = 1; cnt >= 210L; cnt >>= 3) ccr++;
  if (ccr >= 5) return;
  gl.gps.b100 = b100;
  gl.gps.tcnt = cnt - 1;
  /* sampling offset relative to 1->0 transition of start bit */
  if (gl.gps.tcnt > 80)
    gl.gps.thbt = (cnt*50)/100;
  else  gl.gps.thbt = (cnt*15)/100;
  OCR0A  = gl.gps.tcnt;
  TCCR0B = ccr;
  TCCR0A = 0b00000010;  /* com0a1,com0a0,com0b1,com0b0,-,-,WGM01,wgm00  CTC mode */
}

ISR(TIMER1_OVF_vect)
{
  gl.t1ov++;
}

inline void itic(u16_t *t)  /* called with interrupts disabled */
{
  t[1] = gl.t1ov;
  t[0] = TCNT1;
  if ((TIFR1 & _BV(TOV1)) && t[0] < 0xffff) t[1]++;
}

u32_t tic(void)
{
  union { u16_t uw[2]; u32_t ul; } t;
  u08_t srg;

  srg = SREG; cli();
  itic(t.uw);
  SREG = srg;
  return t.ul;
}

void ms_dly(int ndcnt)
{
  int i;

  for (; ndcnt; ndcnt--)
    for (i = gl.dcnt; --i >= 0; )
      __asm__ __volatile__ (" nop");
}

void msdly_cali(void)
{
  u32_t tmr;

  gl.dcnt = 2000;
  tmr = tic();
  ms_dly(100);
  tmr = tic() - tmr;
  gl.dcnt = (int)((200UL*F_TM1)/tmr);
}

void blip(unsigned char nb)
{
  while (nb--) {
    LEDON(); ms_dly( 20);
    LEDOF(); ms_dly(180);
  }
}

/*
 * EEPROM
 */
void eepr_save(void)
{
  u08_t i;

  gl.eemag = EE_MAG;
  gl.eecks = 0;
  for (i = 0; i < (NEL(gl.eedat)-1); i++) gl.eecks += gl.eedat[i];
  gl.eewen = 0;
}

void eepr_updt(void)
{
  u08_t t;

  if ((EECR & _BV(EEPE)) == 0) {
    EEAR = (int)gl.eewen;
    EEDR = gl.eedat[gl.eewen];
    t = SREG; cli();
    EECR = _BV(EEMPE);
    EECR |= _BV(EEPE);
    SREG = t;
    gl.eewen++;
  }
}

/*
 * uart receiver
 */
ISR(SIO_RX_VECT) /* receiver interrupt */
{
  u08_t i;

  gl.sio.rxb[i = gl.sio.rxw] = SIO_UDR;
  if (++i >= NEL(gl.sio.rxb)) i = 0;
  if (i != gl.sio.rxr) gl.sio.rxw = i;
}

int sio_rc(void) /* read next character received */
{
  u08_t i;
  int b;

  if (CB_NO_D(gl.sio.rxr,gl.sio.rxw)) return -1;
  b = (int)gl.sio.rxb[i = gl.sio.rxr];
  gl.sio.rxr = ++i >= NEL(gl.sio.rxb) ? 0 : i;
  return b;
}

char sio_rf(void) /* flush sio receive buffer */
{
  if (gl.sio.rxr == gl.sio.rxw) return 0;
  gl.sio.rxr = gl.sio.rxw;
  return 1;
}

/*
 * uart transmitter
 */
ISR(SIO_TX_VECT) /* tx empty interrupt */
{
  u08_t i;

  if (CB_HASD(gl.sio.txr,gl.sio.txw)) {
    SIO_UDR = gl.sio.txb[i = gl.sio.txr];
    gl.sio.txr = ++i >= NEL(gl.sio.txb) ? 0 : i;
  }
  if (CB_NO_D(gl.sio.txr,gl.sio.txw))
    SIO_UCSRB &= ~_BV(SIO_UDRIE); /* no more, disable TX IRQ */
}

void sio_sc(char c) /* send character */
{
  u08_t i;

  gl.sio.txb[i = gl.sio.txw] = c;
  if (++i >= NEL(gl.sio.txb)) i = 0;
  if (i != gl.sio.txr) {
    gl.sio.txw = i;
    SIO_UCSRB |= _BV(SIO_UDRIE); /* enable TX IRQ */
  }
}

u08_t sio_tf(void) /* txb free space */
{
  return CB_FREE(gl.sio.txr,gl.sio.txw,NEL(gl.sio.txb));
}

void gps_rcv(void);

void sio_tw(void) /* wait for tx empty */
{
  do gps_rcv();
  while (CB_HASD(gl.sio.txr,gl.sio.txw));
}

void sio_ss(char *s) /* send string from ram */
{
  char  c;

  while ((c = *s++)) sio_sc(c);
}

void sio_sp(const char *p) /* send string from flash */
{
  char  c;

  while ((c = pgm_read_byte(p++))) sio_sc(c);
}

#define SIO_SP(str) sio_sp(PSTR(str))

void sio_rev(void)
{
  u08_t n, k;

  for (n = 0; n < 5; n++) {
    k = pgm_read_byte(&version[n]);
    if (k == '.') continue;
    if (k == ' ') break;
    sio_sc(k);
  }
}

void snd_sc(char c)
{
#if U4
  void usb_sc(char c);
  if (gl.dsnd) usb_sc(c); else
#endif
  sio_sc(c);
}

/*
 * convert to ASCII and transmit
 */
/* send hex */
void snd_snx(u08_t n) { n &= 0x0f; if (n <= 9) snd_sc('0'+n); else snd_sc('A'-10+n); }
void snd_sbx(u08_t b) { snd_snx(b>>4); snd_snx(b>>0); }
void snd_swx(u16_t w) { snd_sbx(w>>8); snd_sbx(w>>0); }

void snd_sid(char lc, int i) /* integer decimal */
{
  u08_t fw, nd, sn, b[16];

  fw = sn = nd = 0;
  if (lc > 0) snd_sc(lc);   /* 'lc' is leading character */
  else if (lc < 0) fw = -lc;  /* 'lc' is field width */
  if (i < 0) i = -i, sn = '-';  /* sign */
  do b[nd++] = (i%10) + '0'; while ((i /= 10));
  if (sn) b[nd++] = sn;
  for (sn = nd; sn < fw; sn++) snd_sc(' ');
  do snd_sc(b[--nd]); while (nd);
}

void snd_spd(u08_t fw, u08_t pr, long i) /* pseudo decimal */
{
  u08_t nd, sn, b[20];

  if (pr > 16) pr = 16;
  sn = nd = 0;
  if (i < 0) i = -i, sn = '-';
  do {
    b[nd++] = (i%10) + '0';
    if (nd == pr) b[nd++] = '.';
  }
  while ((i /= 10) || nd < pr);
  if (pr && (nd - pr) == 1) b[nd++] = '0';
  if (sn) b[nd++] = '-';
  while (nd < fw) fw--, snd_sc(' ');
  do snd_sc(b[--nd]); while (nd);
}

void snd_sp(const char *p) /* string from flash */
{
  char  c;

  while ((c = pgm_read_byte(p++))) snd_sc(c);
}

#define SND_SP(str) snd_sp(PSTR(str))

/*
 * 'soft' serial port for GPS
 */
ISR(TIMER0_COMPA_vect)
{
  u08_t i;

  /* transmit bit interrupt */
  if (gl.gps.txm == 0) {
    if (GPS_TXO_VAL()) { /* was stop bit or pause */
      if (gl.gps.txr != gl.gps.txw) {
        gl.gps.txd = gl.gps.txb[i = gl.gps.txr];
        gl.gps.txr = ++i >= NEL(gl.gps.txb) ? 0 : i;
        GPS_TXO_CLR();  /* send start bit */
      }
      else {
        TIMSK0 &= ~_BV(OCIE0A); /* compa interrupt OFF */
        gl.gps.txe = 1;
      }
    }
    else { /* was start bit */
      gl.gps.txm = 1;
      if (gl.gps.txd & 01) GPS_TXO_SET();
    }
  }
  else {
    gl.gps.txe = 0;
    gl.gps.txm <<= 1;
    if (gl.gps.txm == 0 || (gl.gps.txd & gl.gps.txm))
      GPS_TXO_SET();
    else  GPS_TXO_CLR();
  }
}

ISR(TIMER0_COMPB_vect)
{
  u08_t i;

  /* receive bit interrupt */
  i = GPS_RXI_INPUT();
  if (gl.gps.rxm == 0xff) { /* start bit */
    if (i == 0) {
      gl.gps.rxd = 0;
      gl.gps.rxm = 1;
    }
  }
  else {
    if (i) gl.gps.rxd |= gl.gps.rxm;
    if (gl.gps.rxm) gl.gps.rxm <<= 1; /* more bits to sample */
    else {          /* just sampled stop bit */
      TIMSK0 &= ~_BV(OCIE0B);   /* compb interrupt OFF */
      if (i) {      /* stop bit is valid */
        gl.gps.rxb[i = gl.gps.rxw] = gl.gps.rxd;
        if (++i >= NEL(gl.gps.rxb)) i = 0;
        if (i != gl.gps.rxr) gl.gps.rxw = i;
        gl.gps.rxi |= GPS_RXI;
      }
      GPS_RXI_IRQON();
      gl.gps.lstp = TCNT1;
    }
  }
}

ISR(GPS_RXI_VECT)
{
  u08_t rxi, tmp;

  rxi = GPS_RXI_INPUT();    /* sample input */
  tmp = rxi ^ gl.gps.rxi;   /* compare to last sample */
  gl.gps.rxi = rxi;   /* save input */
  if (gl.gps.tcnt == 0) {   /* sample timer for figuring out baudrate */
    if (gl.gps.rxw < NEL(gl.gps.rxt)) {
      itic((u16_t *)&gl.gps.rxt[gl.gps.rxw]);
      gl.gps.rxw++;
    }
  }
  else if (tmp && rxi == 0) { /* it changed and is 0 now -> received start bit */
    tmp = TCNT0;
    tmp += gl.gps.thbt;
    if (tmp == gl.gps.tcnt) tmp++;
    if (tmp > gl.gps.tcnt) tmp -= gl.gps.tcnt;
    if (tmp < 1) tmp++;
    OCR0B = tmp;    /* set sampling point */
    TIFR0  |= _BV(OCF0B); /* clear in case it is set */
    TIMSK0 |= _BV(OCIE0B);  /* compB interrupt ON */
    GPS_RXI_IRQOF();
    gl.gps.rxm = 0xff;  /* tell timer irq COMPB, next sample is start bit */
  }
}

void gps_sc(char c) /* send character */
{
  u08_t i;

  gl.gps.txb[i = gl.gps.txw] = c;
  if (++i >= NEL(gl.gps.txb)) i = 0;
  if (i == gl.gps.txr) return;
  gl.gps.txw = i;
  if (TIMSK0 & _BV(OCIE0A)) return;
  TIFR0  |= _BV(OCF0A);   /* clear compa flag */
  TIMSK0 |= _BV(OCIE0A);    /* compa interrupt ON */
}

void gps_snx(u08_t n) { if ((n &= 0x0f) <= 9) gps_sc('0'+n); else gps_sc('A'-10+n); }

void nmea_snd(const char *p)
{
  u08_t b, cks;

  gps_sc('$');
  for (cks = 0; (b = pgm_read_byte(p)); p++) cks ^= b, gps_sc(b);
  gps_sc('*');
  gps_snx(cks>>4); gps_snx(cks>>0);
  gps_sc('\r'); gps_sc('\n');
}

#define NMEA_SND(str) nmea_snd(PSTR(str))

void gps_bdt(void)
{
  int b;
  u08_t i;

  /* figure out baudrate from single bit timing, 4800...57600 is MAX! */
  if (gl.gps.rxw == NEL(gl.gps.rxt)) {
    b = 1000;
    for (i = NEL(gl.gps.rxt); --i > 5; ) { /* ignore first couple of samples */
      gl.gps.rxt[i] -= gl.gps.rxt[i-1];
      if (gl.gps.rxt[i] < b) b = gl.gps.rxt[i];
    }
         if (b >= (int)MS2TIC(0.016) && b <= (int)MS2TIC(0.018)) b = 576;
    else if (b >= (int)MS2TIC(0.025) && b <= (int)MS2TIC(0.027)) b = 384;
    else if (b >= (int)MS2TIC(0.051) && b <= (int)MS2TIC(0.053)) b = 192;
    else if (b >= (int)MS2TIC(0.103) && b <= (int)MS2TIC(0.106)) b =  96;
    else if (b >= (int)MS2TIC(0.205) && b <= (int)MS2TIC(0.210)) b =  48;
    else b = 0;
    tmr0_setup(b);
    gl.gps.cst = 0;
    gl.gps.b_r = gl.gps.b_w = 0;
  }
}

void gps_rcv(void)
{
  u08_t c, i;

  if (gl.gps.tcnt == 0) {
    gps_bdt();
    return;
  }
  while (1) {
    if (CB_NO_D(gl.gps.rxr,gl.gps.rxw)) break;
    c = (int)gl.gps.rxb[i = gl.gps.rxr];
    gl.gps.rxr = ++i >= NEL(gl.gps.rxb) ? 0 : i;
    /* NMEA: $..........*hh\r\n */
    switch (gl.gps.cst) {
    default:
GPSDROP0:   gl.gps.ech = c;
      gl.gps.est = gl.gps.est ? 0x20 : 0x00;
      gl.gps.est |= gl.gps.cst;
      if (i == gl.gps.b_r) gl.gps.est |= 0x10; /* overflow */
GPSDROP1:   gl.gps.cst = 0;
      break;
    case 6: /* stays here until next '$' arrives */
    case 0: if (c != '$') break;
      gl.gps.cks = 0;
      i = gl.gps.b_s = gl.gps.b_w;
      gl.gps.buf[i] = c;
      if (++i >= NEL(gl.gps.buf)) i = 0;
      if (i == gl.gps.b_r) goto GPSDROP0;
      gl.gps.b_n = 1;
      gl.gps.b_s = i;
      gl.gps.cst = 1;
      break;
    case 1:
      gl.gps.buf[i = gl.gps.b_s] = c;
      if (++i >= NEL(gl.gps.buf)) i = 0;
      if (c < ' ' || c > '~' || i == gl.gps.b_r) goto GPSDROP0;
      if (gl.gps.b_n == 1 && c != 'G') goto GPSDROP1;
      if (gl.gps.b_n == 2 && c != 'P' && c != 'N') goto GPSDROP1;
      gl.gps.b_n++;
      gl.gps.b_s = i;
      if (c == '*') gl.gps.cst = 2;
      else gl.gps.cks ^= c;
      break;
    case 2: case 3:
      gl.gps.buf[i = gl.gps.b_s] = c;
      if (++i >= NEL(gl.gps.buf)) i = 0;
           if (c >= '0' && c <= '9') c -= '0';
      else if (c >= 'A' && c <= 'F') c -= 'A'-10;
      if (c > 0x0f || i == gl.gps.b_r) goto GPSDROP0;
      gl.gps.b_n++;
      gl.gps.b_s = i;
      gl.gps.cst++;
      if (gl.gps.cst == 3) gl.gps.cks ^= c << 4;
      else if (gl.gps.cks != c) goto GPSDROP0;
      break;
    case 4:
      gl.gps.buf[i = gl.gps.b_s] = c;
      if (++i >= NEL(gl.gps.buf)) i = 0;
      if (c != '\r' || i == gl.gps.b_r) goto GPSDROP0;
      gl.gps.b_n++;
      gl.gps.b_s = i;
      gl.gps.cst = 5;
      break;
    case 5: if (c == '\n' && gl.gps.seq >= 10) gl.gps.b_w = gl.gps.b_s;
      gl.gps.cst = 6;
      break;
    }
  }
  if (gl.gps.cst !=  6) return; /* message incomplete */
  if (gl.gps.seq >= 10) return; /* setup done, good to go */
  gl.gps.b_r = gl.gps.b_w = 0;  /* kill received messages during setup */
  if (gl.gps.txr != gl.gps.txw || gl.gps.txe == 0) return; /* not done sending last cmd */
  if ((TCNT1 - gl.gps.lstp) < (u16_t)MS2TIC(10.0)) return; /* wait for gap in data stream */
  switch (gl.gps.seq) {
  case 0: /* check/set baudrate */
    if (gl.gps.b100 == 384)         gl.gps.seq = 2;
    else NMEA_SND("PMTK251,38400"), gl.gps.seq = 1;
    break;
  case 1: tmr0_setup(0);
    gl.gps.seq = 2;
    break;
  /* get RMC,GGA sentences only */
  case 2: NMEA_SND("PMTK314,0,5,0,1,0,0,0,0"); gl.gps.seq = 3; break;
  /* set rate to 200ms, 5Hz */
  case 3: NMEA_SND("PMTK220,200");             gl.gps.seq = 4; break;
  /* done */
  case 4: gl.gps.seq = 10; break;
  }
}

/*
 * I2C devices support. Nobody is really doing this, you would have
 * to install the companion program on the drone to have enough room
 * for this in the FLASH.
 */
#define EN_BMP  0 /* 1 for BMP085 support */
#define EN_M56  0 /* 1 for MS5611 support */
#define EN_HMC  0 /* 1 for HMC5883L support */

#if EN_BMP || EN_HMC || EN_M56
#undef  i2c_setup
/*
 * Two Wire Interface (I2C)
 * 400000 HZ, 9 bit/b -> 0.0225 ms/b
 * 100000 HZ, 9 bit/b -> 0.0900 ms/b
 * write: <adr>|0,<reg>,<dat>
 *  read: <adr>|0,<reg>,<adr>|1,<dat>[,<dat>[...]]
 */
struct {
  vs08_t  act;
  v08_t srg;
  u08_t adr;
  u08_t reg;
  u08_t ndc;
  u08_t dat;
  v08_t irc;
  u08_t nrc;
  v08_t rcv[23];
} gltwi;
#define TWI_RD  _BV(0)

#define TWICLK  400000  /* (Hz) */
void i2c_setup(void)
{
  TWSR = 0;       /* PRSV prescaler value = 1 */
  TWBR = (F_CPU/(2L*TWICLK)-8-1/2); /* TWBR = F_CPU/(2*TWICLK) - 8 - PRSV/2 */
}

ISR(TWI_vect)
{
  u08_t srg;

  srg = gltwi.srg;
  gltwi.srg = TWSR & 0xf8;
  switch (gltwi.act) {
  default:
    gltwi.srg = srg;
ERROR:
    if (gltwi.act > 0) gltwi.act *= -1;
    TWCR = _BV(TWINT)|_BV(TWSTO)|_BV(TWEN)|_BV(TWIE);
    break;
  case 1: /* START sent */
    if (gltwi.srg != 0x08) goto ERROR;
    TWDR = (gltwi.adr & ~TWI_RD);
    gltwi.act = 2;
    TWCR = _BV(TWINT)|_BV(TWEN)|_BV(TWIE);
    break;
  case 2: /* address-W sent */
    if (gltwi.srg != 0x18) goto ERROR;
    TWDR = gltwi.reg;
    gltwi.act = gltwi.ndc ? 4 : 3;
    gltwi.ndc = 0;
    TWCR = _BV(TWINT)|_BV(TWEN)|_BV(TWIE);
    break;
  case 3: /* register sent */
    if (gltwi.srg != 0x28) goto ERROR;
    if (gltwi.adr & TWI_RD) { /* read transaction */
      gltwi.act = 5;
      TWCR = _BV(TWINT)|_BV(TWSTA)|_BV(TWEN)|_BV(TWIE);
      break;
    }
    TWDR = gltwi.dat;
    gltwi.act = 4;
    TWCR = _BV(TWINT)|_BV(TWEN)|_BV(TWIE);
    break;

  case 4: /* data byte sent */
    if (gltwi.srg != 0x28) goto ERROR;
    gltwi.act = 8;
    TWCR = _BV(TWINT)|_BV(TWSTO)|_BV(TWEN)|_BV(TWIE);
    break;

  case 5: /* repeated START sent */
    if (gltwi.srg != 0x10) goto ERROR;
    TWDR = gltwi.adr;
    gltwi.act = 6;
    TWCR = _BV(TWINT)|_BV(TWEN)|_BV(TWIE);
    break;
  case 6: /* address-R sent */
    if (gltwi.srg != 0x40) goto ERROR;
    gltwi.act = 7;
    if (gltwi.nrc > 1)
      TWCR = _BV(TWINT)|_BV(TWEN)|_BV(TWIE)|_BV(TWEA);
    else  TWCR = _BV(TWINT)|_BV(TWEN)|_BV(TWIE);
    break;
  case 7: /* data byte received */
    if (gltwi.srg == 0x50) {
      gltwi.rcv[gltwi.irc++] = TWDR;
      if ((gltwi.nrc - gltwi.irc) > 1)
        TWCR = _BV(TWINT)|_BV(TWEN)|_BV(TWIE)|_BV(TWEA);
      else  TWCR = _BV(TWINT)|_BV(TWEN)|_BV(TWIE);
    }
    else if (gltwi.srg == 0x58) {
      gltwi.rcv[gltwi.irc++] = TWDR;
      gltwi.act = 8;
      TWCR = _BV(TWINT)|_BV(TWSTO)|_BV(TWEN)|_BV(TWIE);
    }
    else goto ERROR;
    break;
  }
}

void i2c_strt(void)
{
  gltwi.irc = 0;
  gltwi.srg = 0;
  gltwi.act = 1;
  TWCR = _BV(TWINT)|_BV(TWSTA)|_BV(TWEN)|_BV(TWIE);
}

void i2c_send(u08_t adr, u08_t reg, u08_t dat)
{
  gltwi.adr = (adr<<1);
  gltwi.reg = reg;
  gltwi.dat = dat;
  i2c_strt();
}

void i2c_recv(u08_t adr, u08_t reg, u08_t nda)
{
  gltwi.adr = (adr<<1)|TWI_RD;
  gltwi.reg = reg;
  gltwi.nrc = nda;
  i2c_strt();
}

void i2c_snd(u08_t adr, u08_t cmd)
{
  gltwi.adr = (adr<<1);
  gltwi.reg = cmd;
  gltwi.ndc = 1;
  i2c_strt();
}
#endif

#if EN_BMP
#undef  bmp_sq
#undef  bmp_rp
  /* BMP085 pressure/temperature sensor on twi */
#define A_BMP 0b1110111
#define OSS 3
struct {
  vs08_t  seq;  
  u08_t cid;  /* chip id: [0xd0] 0x55 */
  u08_t idx;
  u08_t cdn;
  u16_t tmr;
  u08_t upd;
  u08_t ut[2];  /* raw temperature */
  u08_t up[3];  /* raw pressure */
  u08_t cal[22];
} glbmp;

u08_t bmp_rp(u08_t upd)
{
  u08_t i;

  if (glbmp.upd & upd & 01) {
    SND_SP("BMPR");
    snd_sid(',',glbmp.seq);
    for (snd_sc(','), i = 0; i < NEL(glbmp.ut); i++) snd_sbx(glbmp.ut[i]);
    for (snd_sc(','), i = 0; i < NEL(glbmp.up); i++) snd_sbx(glbmp.up[i]);
    glbmp.upd &= ~01;
  }
  if (glbmp.upd & upd & 02) {
    SND_SP("BMPC,"); snd_snx(OSS);
    for (i = 0; i < NEL(glbmp.cal); i += 2)
      snd_sc(','), snd_sbx(glbmp.cal[i]), snd_sbx(glbmp.cal[i+1]);
    glbmp.upd &= ~02;
  }
  return glbmp.upd;
}

#define BMPTIC    MS2TIC(0.25)
#define MS2BMP(ms)  (MS2TIC(ms)/BMPTIC)

void bmp_sq(void)
{
  u16_t dtm;
  u08_t i;

  if (glbmp.cdn) {
    dtm = TCNT1 - glbmp.tmr;
    if (dtm < (u16_t)BMPTIC) return;
    glbmp.tmr = TCNT1;
    if ((glbmp.cdn -= 1)) return;
  }
  switch (glbmp.seq) {
  case  1: case  3: case  5: case  7:
    if (gltwi.act != 8) {
      glbmp.seq = -glbmp.seq;
      gltwi.act = 0;
      glbmp.upd |= 01;
      break;
    }
    if (glbmp.seq != 1) gltwi.act = 0;
    break;
  case  0: case  4: case  6:
    if (gltwi.act) return;
    break;
  }
  switch (glbmp.seq) {
  case 0: /* read chip id */
    i2c_recv(A_BMP,0xd0,1);
    glbmp.cdn = MS2BMP(0.25);
    glbmp.seq = 1;
    break;
  case 1: glbmp.cid = gltwi.rcv[0];
    if (glbmp.cid != 0x55) {
      gltwi.act = 0;
      glbmp.seq = -100;
      glbmp.upd |= 01;
      break;
    }
    /* read calibration registers */
    i2c_recv(A_BMP,0xaa,NEL(glbmp.cal));
    glbmp.cdn = MS2BMP(3.0);
    glbmp.seq = 3;
    break;
  case 3: for (i = 0; i < NEL(glbmp.cal); i++) glbmp.cal[i] = gltwi.rcv[i];
    glbmp.upd |= 02;
    glbmp.seq = 4;
    break;

  case 4: /* request data */
    i2c_send(A_BMP,0xf4,glbmp.idx?(0x34+(OSS<<6)):0x2e);
    glbmp.cdn = MS2BMP(0.5);
    glbmp.seq = 5;
    break;
  case 5: glbmp.cdn = glbmp.idx?MS2BMP(28.0):MS2BMP(6.0);
    glbmp.seq = 6;
    break;
  case 6: /* read result register */
    i2c_recv(A_BMP,0xf6,glbmp.idx?3:2);
    glbmp.cdn = MS2BMP(0.5);
    glbmp.seq = 7;
    break;
  case 7: glbmp.idx++;
    if (glbmp.idx == 1) { /* temperature data */
      glbmp.ut[0] = gltwi.rcv[0];
      glbmp.ut[1] = gltwi.rcv[1];
      glbmp.seq = 4;
      break;
    }
    /* pressure data */
    glbmp.up[0] = gltwi.rcv[0];
    glbmp.up[1] = gltwi.rcv[1];
    glbmp.up[2] = gltwi.rcv[2];
    glbmp.upd |= 01;
    if (glbmp.idx >= 10) glbmp.idx = 0;
    glbmp.cdn = MS2BMP(10.0);
    glbmp.seq = 8;
    break;
  case 8: if (glbmp.upd == 0) glbmp.seq = 4;
    break;
  }
  if (glbmp.cdn) glbmp.tmr = TCNT1;
}

#undef  BMPTIC
#undef  MS2BMP

#endif

#if EN_M56
#undef  m56_sq
#undef  m56_rp
  /* MS5611 pressure/temperature sensor on twi */
#define A_M56 0b1110111 
struct {
  vs08_t  seq;  
  u08_t cdn;
  u16_t tmr;
  u08_t idx;
  u08_t upd;
  u08_t ut[3];  /* raw temperature */
  u08_t up[3];  /* raw pressure */
  u08_t cal[16];
} glm56;

u08_t m56_rp(u08_t upd)
{
  u08_t i;

  if (glm56.upd & upd & 01) {
    SND_SP("M56R");
    snd_sid(',',glm56.seq);
    for (snd_sc(','), i = 0; i < 3; i++) snd_sbx(glm56.ut[i]);
    for (snd_sc(','), i = 0; i < 3; i++) snd_sbx(glm56.up[i]);
    glm56.upd &= ~01;
  }
  if (glm56.upd & upd & 02) {
    SND_SP("M56C");
    for (i = 0; i < NEL(glm56.cal); i += 2)
      snd_sc(','), snd_sbx(glm56.cal[i]), snd_sbx(glm56.cal[i+1]);
    glm56.upd &= ~02;
  }
  return glm56.upd;
}

#define M56TIC    MS2TIC(0.25)
#define MS2M56(ms)  (MS2TIC(ms)/M56TIC)

void m56_sq(void)
{
#define M56_RST 0x1e
#define M56_UP  0x48  /* OSR = 4096 */
#define M56_UT  0x58  /* OSR = 4096 */
#define M56_ROM 0xa0
  u16_t dtm;

  if (glm56.cdn) {
    dtm = TCNT1 - glm56.tmr;
    if (dtm < (u16_t)M56TIC) return;
    glm56.tmr = TCNT1;
    if ((glm56.cdn -= 1)) return;
  }
  switch (glm56.seq) {
  case  1: case  3: case  5: case  7:
    if (gltwi.act != 8) {
      glm56.seq = -glm56.seq;
      gltwi.act = 0;
      glm56.upd |= 01;
      break;
    }
    gltwi.act = 0;
    break;
  case  0: case 2: case  4: case  6:
    if (gltwi.act) return;  /* i2c busy */
    break;
  }
  switch (glm56.seq) {
  case 0: /* reset sensor */
    i2c_snd(A_M56,M56_RST);
    glm56.cdn = MS2M56(0.5);
    glm56.seq = 1;
    glm56.idx = 0;
    break;
  case 1: /* wait a while for reset */
    glm56.cdn = MS2M56(60);
    glm56.seq = 2;
    break;
  case 2: /* read calibration registers */
    i2c_recv(A_M56,M56_ROM+glm56.idx,2);
    glm56.cdn = MS2M56(0.5);
    glm56.seq = 3;
    break;
  case 3: glm56.cal[glm56.idx++] = gltwi.rcv[0];
    glm56.cal[glm56.idx++] = gltwi.rcv[1];
    if (glm56.idx < NEL(glm56.cal)) glm56.seq = 2;
    else {
      glm56.idx = 0;
      glm56.seq = 4;
      glm56.upd |= 02;
    }
    break;

  case 4: /* inititate conversion */
    i2c_snd(A_M56,glm56.idx?M56_UP:M56_UT);
    glm56.cdn = MS2M56(0.5);
    glm56.seq = 5;
    break;
  case 5: /* wait for conversion */
    glm56.cdn = MS2M56(9.0);
    glm56.seq = 6;
    break;
  case 6: /* read result */
    i2c_recv(A_M56,0,3);
    glm56.cdn = MS2M56(0.5);
    glm56.seq = 7;
    break;
  case 7: /* transfer result */
    if (++glm56.idx == 1) {
      glm56.ut[0] = gltwi.rcv[0];
      glm56.ut[1] = gltwi.rcv[1];
      glm56.ut[2] = gltwi.rcv[2];
      glm56.seq = 4;
      break;
    }
    glm56.up[0] = gltwi.rcv[0];
    glm56.up[1] = gltwi.rcv[1];
    glm56.up[2] = gltwi.rcv[2];
    if (glm56.idx >= 2) glm56.idx = 0;
    glm56.upd |= 01;
    glm56.seq  = 8;
    break;
  case 8: /* wait for data pickup */
    if (glm56.upd == 0) glm56.seq = 4;
    break;
  }
  if (glm56.cdn) glm56.tmr = TCNT1;
#undef  M56_RST
#undef  M56_UP
#undef  M56_UT
#undef  M56_ROM
}

#undef  M56TIC
#undef  MS2M56

#endif

#if EN_HMC
#undef  hmc_sq
#undef  hmc_rp
#define A_HMC 0b0011110
struct {
  vs08_t  seq;  
  u08_t upd;
  u08_t twi;
  u08_t cdn;
  u16_t tmr;
  int bia[3];
  int raw[3];
} glhmc;

u08_t hmc_rp(u08_t upd)
{
  u08_t i;

  if (glhmc.upd & upd & 01) {
    SND_SP("HMCR");
    snd_sid(',',glhmc.seq);
    for (i = 0; i < NEL(glhmc.raw); i++)
      snd_sc(','), snd_spd(0,0,glhmc.raw[i]);
    glhmc.upd &= ~01;
  }
  if (glhmc.upd & upd & 02) {
    SND_SP("HMCC");
    SND_SP(",1160,"), snd_spd(0,0,glhmc.bia[0]);
    SND_SP(",1160,"), snd_spd(0,0,glhmc.bia[1]);
    SND_SP(",1080,"), snd_spd(0,0,glhmc.bia[2]);
    glhmc.upd &= ~02;
  }
  return glhmc.upd;
}

#define HMCTIC    MS2TIC(0.5)
#define MS2HMC(ms)  (MS2TIC(ms)/HMCTIC)

void hmc_sq(void)
{
  union { int i; u16_t w; struct { u08_t lsb, msb; }; } t;

  if (glhmc.cdn) {
    t.w = TCNT1 - glhmc.tmr;
    if (t.w < (u16_t)HMCTIC) return;
    glhmc.tmr = TCNT1;
    if ((glhmc.cdn -= 1)) return;
  }
  if (glhmc.twi&01) { /* release i2c */
    if (gltwi.act != 8) {
      glhmc.seq = -glhmc.seq;
      glhmc.upd |= 01;
    }
    glhmc.twi = 0;
    gltwi.act = 0;
  }
  if (glhmc.seq == 0 || (glhmc.twi&02)) { /* acquire i2c */
    if (gltwi.act) return;
    glhmc.twi = 0;
  }
  switch (glhmc.seq) {
  case  0: /* read chip id */
    i2c_recv(A_HMC,10,3);
    glhmc.cdn = MS2HMC(0.5);
    glhmc.twi = 1;
    glhmc.seq = 1;
    break;
  case  1:
    if (gltwi.rcv[0] != 'H' || gltwi.rcv[1] != '4' || gltwi.rcv[2] != '3') {
      glhmc.seq = -100;
      glhmc.upd |= 01;
      break;
    }
    glhmc.twi = 2;
    glhmc.seq = 2;
    break;

  case  2: /* set CFG A */
    i2c_send(A_HMC,0x00,0b01110001); /* -,MA1,MA0,DO2,do1,do0,ms1,MS2  avg 8, upd 15 Hz, +bias */
    glhmc.cdn = MS2HMC(1.0);
    glhmc.twi = 1;
    glhmc.seq++;
    break;
  case  3: glhmc.seq++; glhmc.twi = 2; break;
  case  4: /* set CFG B */
    i2c_send(A_HMC,0x01,1<<5);  /* gain +-1.3Ga */
    glhmc.cdn = MS2HMC(1.0);
    glhmc.twi = 1;
    glhmc.seq++;
    break;
  case  5: glhmc.seq++; glhmc.twi = 2; break;

  case  6: case 26: /* single measurement */
    i2c_send(A_HMC,0x02,0x01);
    glhmc.cdn = MS2HMC(1.0);
    glhmc.twi = 1;
    glhmc.seq++;
    break;
  case  7: case 27: /* wait a while */
    glhmc.cdn = glhmc.seq == 7 ? MS2HMC(100.0) : MS2HMC(68);
    glhmc.twi = 2;
    glhmc.seq++;
    break;
  case  8: case 28: /* read result register */
    i2c_recv(A_HMC,0x03,6);
    glhmc.cdn = MS2HMC(1.0);
    glhmc.seq++;
    break;
  case  9: case 29: /* transfer raw values */
    t.msb = gltwi.rcv[0]; t.lsb = gltwi.rcv[1]; glhmc.raw[0] = t.i; /* X */
    t.msb = gltwi.rcv[2]; t.lsb = gltwi.rcv[3]; glhmc.raw[2] = t.i; /* Z */
    t.msb = gltwi.rcv[4]; t.lsb = gltwi.rcv[5]; glhmc.raw[1] = t.i; /* Y */
    glhmc.twi = 1;
    glhmc.seq++;
    break;
  case 10: /* transfer bias */
    glhmc.bia[0] = glhmc.raw[0];
    glhmc.bia[1] = glhmc.raw[1];
    glhmc.bia[2] = glhmc.raw[2];
    glhmc.upd |= 02;
    glhmc.twi = 2;
    glhmc.seq = 20;
    break;

  case 20: /* set CFG A */
    i2c_send(A_HMC,0x00,0b01110000); /* -,MA1,MA0,DO2,do1,do0,ms1,MS2  avg 8, upd 15 Hz, norm */
    glhmc.cdn = MS2HMC(1.0);
    glhmc.twi = 1;
    glhmc.seq++;
    break;
  case 21: glhmc.seq++; break;
  case 22: /* set CFG B */
    i2c_send(A_HMC,0x01,1<<5);  /* gain +-1.3Ga */
    glhmc.cdn = MS2HMC(1.0);
    glhmc.twi = 1;
    glhmc.seq++;
    break;
  case 23:
    glhmc.twi = 2;
    glhmc.seq = 26;
    break;

  case 30: /* measurements valid */
    glhmc.upd |= 01;
    glhmc.cdn = MS2HMC(1.0);
    glhmc.seq++;
    break;
  case 31: /* wait for measurement digest */
    if (glhmc.upd & 01) break;
    glhmc.seq = 26;
    glhmc.twi = 2;
    break;
  }
  if (glhmc.cdn) glhmc.tmr = TCNT1;
}

#undef  HMCTIC
#undef  MS2BMP

#endif

/*
 * RC receiver
 */
#define PTMAX MS2TIC(2.05)  /* pulse should be shorter than that */
#define PTMIN MS2TIC(0.95)  /* pulse should be longer than that */
#define PTREF MS2TIC(1.50)  /* nominal center position */
#define PTRNG MS2TIC(0.40)  /* +- nominal spread */

#define FTMIN MS2TIC( 8.0)  /* min frametime */
#define FTMAX MS2TIC(35.0)  /* max frametime */
#define FTTOT MS2TIC(250.0) /* max frame time out */

ISR(RXSIG_VECT) /* RX signal change interrupt */
{
  union { rsm_t *r; u08_t *b; } p;
  u08_t i;

  p.r = gl.rxs.rsb;
  i = gl.rxs.rsw;
  p.b += i * sizeof(*p.r);
  p.r->smp = RXSIG;
  itic((u16_t *)&p.r->tmr);
  if (++i >= NEL(gl.rxs.rsb)) i = 0;
  if (i != gl.rxs.rsr) gl.rxs.rsw = i;
}

void rx_proc(void)
{
  u32_t tmr, dtm;
  char  k;
  u08_t i, smp, chg, lsm;

  lsm = gl.rxs.smp;
  smp = SREG; cli(); i = gl.rxs.rsw; SREG = smp;
  k = CB_FILL(gl.rxs.rsr,i,NEL(gl.rxs.rsb));
  while (k--) {
    i = gl.rxs.rsr;
    smp = gl.rxs.rsb[i].smp;
    tmr = gl.rxs.rsb[i].tmr;
    gl.rxs.rsr = ++i >= NEL(gl.rxs.rsb) ? 0 : i;
    chg = (smp ^ lsm) & RXMSK;
    lsm = smp;
    for (i = 0; chg && i < NEL(gl.rxs.chn); i++)
      if (chg & gl.rxs.chn[i].msk) {
        dtm = tmr - gl.rxs.chn[i].tup;
        if (smp & gl.rxs.chn[i].msk)
          gl.rxs.chn[i].ftm = dtm, gl.rxs.chn[i].tup = tmr;
        else  gl.rxs.chn[i].dur = dtm;
        chg &= ~gl.rxs.chn[i].msk;
      }
  }
  gl.rxs.smp = lsm;
}

int rx_tk2v(int dur, int dbn)
{
  int val;

  dur -= PTREF;
  if (dur >= 0) { if ((dur -= dbn) <= 0) return 0; }
  else          { if ((dur += dbn) >= 0) return 0; }
  val = (int)((long)(dur * 1000L + PTRNG/2)/(long)(PTRNG - dbn));
  val = val < -1000 ? -1000 : val > 1000 ? 1000 : val;
  return val;
}

u08_t rx_esel(int thr)
{
  int v, h;
  u08_t isp;

  isp = 0;
  if ((v = gl.rxs.chn[ S_ELEV].val) < 0) v *= -1; else isp |= 01;
  if ((h = gl.rxs.chn[SH_ELEV].val) < 0) h *= -1; else isp |= 02;
  if (v < thr && h < thr) return 0;
  return v >= h ? (isp & 01) ? 1 : 3 : (isp & 02) ? 4 : 2;
}

u08_t rx_read(void)
{
  u08_t err, mch;
  u32_t tot;
  int val;
  struct chn *c;

  rx_proc();
  err = 0;
  tot = tic();
  mch = _BV(NEL(gl.rxs.chn)-1);
  for (c = gl.rxs.chn + NEL(gl.rxs.chn); --c >= gl.rxs.chn; mch >>= 1)
    if (!(mch & S_SIG)) continue; /* bit/channel is not used */
    else if ((tot - c->tup) > FTTOT) err |= mch;
    else if (c->ftm < FTMIN || c->ftm > FTMAX) err |= mch;
    else if (c->dur < PTMIN || c->dur > PTMAX) err |= mch;
  if ((gl.rxs.err = err)) { /* there is a problem with one or more RX signals */
    if (gl.rxs.erx < S2LTIC(0.5)) {
      /* delay error reporting */
      gl.rxs.erx++;
      return 0;
    }
    for (c = gl.rxs.chn + NEL(gl.rxs.chn); --c >= gl.rxs.chn; ) c->val = 0;
    return err;
  }
  /* all RX signals ok */
  gl.rxs.erx = 0;
  mch = _BV(NEL(gl.rxs.chn)-1);
  for (c = gl.rxs.chn + NEL(gl.rxs.chn); --c >= gl.rxs.chn; mch >>= 1)
    c->val = rx_tk2v(c->dur,c->dbn);

  if (IN_AUX) gl.rxs.chn[S_AUX1].val *= -1;

  val = gl.rxs.chn[S_LAND].val;
  switch (S_LAND) {
  case S_GEAR: gl.rxs.fms =       val < 0 ? FMS_LAND : FMS_FM_1; break;
  case S_AUX1: gl.rxs.fms = val ? val < 0 ? FMS_LAND : FMS_FM_2 : FMS_FM_1; break;
  }
  if (S_FMOD != S_LAND && gl.rxs.fms != FMS_LAND) /* 2 switches and in the air */
    gl.rxs.fms = gl.rxs.chn[S_FMOD].val < 0 ? FMS_FM_1 : FMS_FM_2;

  /* extended flightmode sequencer */
  switch (gl.rxs.fsq) {
  default: gl.rxs.fsq = 0; break;
  case 0: if (gl.rxs.fms == FMS_FM_2) gl.rxs.fsq = 1; break;
  case 1: if (gl.rxs.fms == FMS_LAND) gl.rxs.fsq = 0;
    else if (gl.rxs.fms == FMS_FM_1) {
      /* switch to FM_1, arms trigger mechanism */
      gl.rxs.fst = S2LTIC(0.5);
      gl.rxs.fsq = 2;
      gl.rxs.fms = FMS_FM_2;
    }
    break;
  case 2: gl.rxs.fsq = 0;
    if ((gl.rxs.fst -= 1) == 0) break; /* timeout */
    if (gl.rxs.fms == FMS_LAND) break; /* land */
    if (gl.rxs.fms == FMS_FM_1) {
      /* keep FM_2 during timeout period */
      gl.rxs.fsq = 2;
      gl.rxs.fms = FMS_FM_2;
      break;
    }
    if (gl.rxs.fms != FMS_FM_2) break;
    /* user switched back to FM_2 before timeout */
    gl.rxs.fst = S2LTIC(0.40);  /* animation trigger window */
    gl.rxs.fsq = 3;
    break;
  case 3: if (gl.rxs.fms != FMS_FM_2) { /* switch changed */
      gl.rxs.fsq = 0;
      break;
    }
    if ((gl.rxs.fst -= 1) == 0) { /* enter FM_3 */
      gl.rxs.fms = FMS_FM_3;
      gl.rxs.fsq = 5;
      break;
    }
    if ((mch = rx_esel(100)) == 0) break;
    /* trigger animation */
    gl.rxs.fst = S2LTIC(0.5);
    gl.rxs.fsa = mch;
    gl.rxs.fsq = 4;
    break;
  case 4: if ((gl.rxs.fst -= 1) == 0 || gl.rxs.fsa == 0)
      gl.rxs.fsq = gl.rxs.fsa = 0;
    break;
  case 5: gl.rxs.fsq = 0;
    if (gl.rxs.fms != FMS_FM_2) break;  /* switch changed */
    if (rx_esel(150)) break;    /* stick moved */
    gl.rxs.fsq = 5;
    gl.rxs.fms = FMS_FM_3;
    break;
  }

#if GSWO_ENA >= 0 && GSWO_SRC == 0
  if (gl.rxs.chn[S_GEAR].val <= 0) GSWOF(); else GSWON();
#endif

  if (gl.rxs.fms == FMS_LAND) {
    mch = 0;
    val = gl.rxs.chn[ S_THRO].val; if (ABS(val) > STKTHR) mch |= val < 0 ? TS_DN : TS_UP;
    val = gl.rxs.chn[SH_THRO].val; if (ABS(val) > STKTHR) mch |= val < 0 ? TS_LF : TS_RT;
    val = gl.rxs.chn[ S_ELEV].val; if (ABS(val) > STKTHR) mch |= val < 0 ? ES_UP : ES_DN;
    val = gl.rxs.chn[SH_ELEV].val; if (ABS(val) > STKTHR) mch |= val < 0 ? ES_LF : ES_RT;
    gl.rxs.stk = mch;
    /* alternate configuration selection */
    if ((gl.rxs.stk & TS_MSK) == TS_UP)
      gl.cfw = (char)rx_esel(STKTHR) - 1;
    else {
      if (gl.cfw >= 0 && gl.cfg != gl.cfw && gl.eewen >= NEL(gl.eedat)) {
        gl.cfg = gl.cfw;
        eepr_save();
      }
      gl.cfw = -1;
    }
#if GSWO_ENA >= 0 && GSWO_SRC == 1
    if ((gl.rxs.stk & TS_MSK) == (TS_UP|TS_LF) && gl.beep.nbp == 0) gl.beep.nbp = 5;
#endif
  }
  else {
    gl.rxs.stk = 0;
    gl.cfw = -1;
  }
  return 0;
}

/*
 * messages from drone
 */
#define C_ACK 0x01
void sio_rcv(void)
{
  int r;
  char  c;

  while ((r = sio_rc()) >= 0)
    if ((c = r) <= 0) gl.drm.cst = 0;
    else if (c == C_ACK) gl.drm.ack = 1, gl.drm.cst = 0;
    else switch (gl.drm.cst) {
    case 0: if (c == '$') gl.drm.cst = 1, gl.drm.nda = 0; break;
    case 1: if (c == '*') {
        if (gl.drm.nda == sizeof(gl.drm.dat.m2a)) {
#if GSWO_ENA >= 0 && GSWO_SRC == 1
          if (gl.drm.dat.m2a.beep > 0 && gl.beep.nbp == 0)
            gl.beep.nbp = gl.drm.dat.m2a.beep;
#endif
          gl.drm.m2a = gl.drm.dat.m2a;
        }
        gl.drm.cst = 0;
        break;
      }
      /* FALLTHROUGH */
    case 2: if (c >= '0' && c <= '9') c -= '0';
      else if (c >= 'A' && c <= 'F') c -= 'A'-10;
      else if (c >= 'a' && c <= 'f') c -= 'a'-10;
      else c = -1;
      if (c < 0 || gl.drm.nda >= sizeof(gl.drm.dat)) {
        gl.drm.cst = 0;
        break;
      }
      if (gl.drm.cst == 1)
        gl.drm.dat.b[gl.drm.nda] = (c<<4);
      else  gl.drm.dat.b[gl.drm.nda++] |= c;
      gl.drm.cst ^= 3;
      break;
    }
}

#if U4
#undef  usb_sq
/*
 * MiruPCB USB interface
 */
typedef struct { /* Standard Device Descriptor */
  u08_t bLength;
  u08_t bDescriptorType;
  u16_t bcdUSB;
  u08_t bDeviceClass;
  u08_t bDeviceSubClass;
  u08_t bDeviceProtocol;
  u08_t bMaxPacketSize0;
  u16_t idVendor;
  u16_t idProduct;
  u16_t bcdDevice;
  u08_t iManufacturer;
  u08_t iProduct;
  u08_t iSerialNumber;
  u08_t bNumConfigurations;
} usb_sdd_t; /* 18 bytes */

typedef struct { /* Standard Configuration Descriptor */
  u08_t bLength;
  u08_t bDescriptorType;
  u16_t wTotalLength;
  u08_t bNumInterfaces;
  u08_t bConfigurationValue;
  u08_t iConfiguration;
  u08_t bmAttributes;
  u08_t bMaxPower;  /* in 2mA units, 50 ^ 100mA */
} usb_scd_t; /* 9 bytes */

typedef struct { /* Standard Interface Descriptor */
  u08_t bLength;
  u08_t bDescriptorType;
  u08_t bInterfaceNumber;
  u08_t bAlternateSetting;
  u08_t bNumEndPoints;
  u08_t bInterfaceClass;
  u08_t bInterfaceSubClass;
  u08_t bInterfaceProtocol;
  u08_t iInterface;
} usb_sid_t; /* 9 bytes */

typedef struct { /* Standard Endpoint Descriptor */
  u08_t bLength;
  u08_t bDescriptorType;
  u08_t bEndPointAddress;
  u08_t bmAttributes;
  u16_t wMaxPacketSize;
  u08_t bInterval;
} usb_sep_t; /* 7 bytes */

#define DSC_DEV 1 // DEVICE
#define DSC_CFG 2 // CONFIGURATION
#define DSC_STR 3 // STRING
#define DSC_IFC 4 // INTERFACE
#define DSC_EPT 5 // ENDPOINT
#define DSC_DQU 6 // DEVICE_QUALIFIER
#define DSC_OSC 7 // OTHER_SPEED_CONFIGURATION
#define DSC_PWR 8 // INTERFACE_POWER

const usb_sdd_t usb_sdd PROGMEM = {
 .bLength = sizeof(usb_sdd_t), .bDescriptorType = DSC_DEV, .bcdUSB = 0x0200,
 .bDeviceClass       = 0,
 .bDeviceSubClass    = 0,
 .bDeviceProtocol    = 0,
 .bMaxPacketSize0    = 64,
 .idVendor           = 0x0403, /* FTDI */
 .idProduct          = 0x6001, /* FT232 USB-Serial (UART) IC */
 .bcdDevice          = 0x0600,
 .iManufacturer      = 0,
 .iProduct           = 0,
 .iSerialNumber      = 0,
 .bNumConfigurations = 1,
};

#define EP1_LGS   3 /* endpoint size (0 == 8, 1 == 16, 2 == 32 ... 6 == 512) */
#define EP2_LGS   3 /* endpoint size (0 == 8, 1 == 16, 2 == 32 ... 6 == 512) */
const struct {
  usb_scd_t scd;  /* 9 bytes */
  usb_sid_t sid;  /* 9 bytes */
  usb_sep_t ept[2]; /* 7 bytes each */
} usb_cfg PROGMEM = {
 .scd.bLength = sizeof(usb_scd_t), .scd.bDescriptorType = DSC_CFG,
 .scd.wTotalLength        = sizeof(usb_scd_t) + sizeof(usb_sid_t) + 2*sizeof(usb_sep_t),
 .scd.bNumInterfaces      = 1,
 .scd.bConfigurationValue = 1,
 .scd.iConfiguration      = 0,
 .scd.bmAttributes        = 0xa0,
 .scd.bMaxPower           = 100/2,

 .sid.bLength = sizeof(usb_sid_t), .sid.bDescriptorType = DSC_IFC,
 .sid.bInterfaceNumber    = 0,
 .sid.bAlternateSetting   = 0,
 .sid.bNumEndPoints       = 2,
 .sid.bInterfaceClass     = 0xff, /* vendor specific */
 .sid.bInterfaceSubClass  = 0xff,
 .sid.bInterfaceProtocol  = 0xff,
 .sid.iInterface          = 2,

 .ept[0].bLength = sizeof(usb_sep_t), .ept[0].bDescriptorType = DSC_EPT,
 .ept[0].bEndPointAddress = 0x81, /* EP 1 IN dev->host */
 .ept[0].bmAttributes     = 0x02, /* BULK */
 .ept[0].wMaxPacketSize   = (8<<EP1_LGS),
 .ept[0].bInterval        = 0,

 .ept[1].bLength = sizeof(usb_sep_t), .ept[1].bDescriptorType = DSC_EPT,
 .ept[1].bEndPointAddress = 0x02, /* EP 2 OUT dev<-host */
 .ept[1].bmAttributes     = 0x02, /* BULK */
 .ept[1].wMaxPacketSize   = (8<<EP2_LGS),
 .ept[1].bInterval        = 0,
};

u08_t usb_tf(void) /* txb free space */
{
  return CB_FREE(gl.usb.ep1r,gl.usb.ep1w,NEL(gl.usb.ep1b));
}

void usb_sc(char c)
{
  u08_t i;

  gl.usb.ep1b[i = gl.usb.ep1w] = c;
  if (++i >= NEL(gl.usb.ep1b)) i = 0;
  if (i != gl.usb.ep1r) gl.usb.ep1w = i;
}

void usb_sp(const char *p) /* send string from flash */
{
  char  c;

  while ((c = pgm_read_byte(p++))) usb_sc(c);
}

#define USB_SP(str) usb_sp(PSTR(str))

int usb_rc(void) /* read next character received */
{
  int b;

  if (gl.usb.ep2r == 0) return -1;
  UENUM = 2;
  if (!(UEINTX & _BV(FIFOCON))) return -1;
  b = UEDATX;
  if (--gl.usb.ep2r == 0) UEINTX &= ~_BV(FIFOCON);
  return b;
}

ISR(USB_GEN_vect)
{
  u08_t irq, ept;

  irq = UDINT;
  ept = UENUM;
  UDINT = 0b00000000; /* -,uprsmi,eorsmi,wakeup,eorsti,sofi,-,suspi */
  irq &= UDIEN;
  if (irq & _BV(EORSTI)) {
    gl.usb.ep0a = 0;
    gl.usb.ep0r = 0;
    gl.usb.ep1w = 0;
    gl.usb.ep1r = 0;
    gl.usb.sofi = 0;
    gl.usb.ep1h = 1;
    gl.usb.mdst = RST_MDST;
    gl.usb.lnst = RST_LNST;
    UENUM   = 0;
    UECONX  = 0b00000001; /* -,-,stallrq,stallqc,rstdt,-,-,EPEN */
    UECFG0X = 0b00000000; /* ept1,ept0,-,-,-,-,-,epdir */
    UECFG1X = 0b00110010; /* -,eps2,EPS1,EPS0,epbk1,epbk0,ALLOC,- 64 bytes */
    if (UESTA0X & _BV(CFGOK)) {
      UEIENX = _BV(RXSTPE); /* enable IRQs for EP 0 */
      gl.usb.ep0a = 1;  /* enable usb_sq */
    }
  }
  if (irq & _BV(SOFI)) gl.usb.sofi++;
  UENUM = ept;
}

ISR(USB_COM_vect)
{
  u08_t ept = UENUM;

  /* UEINTX: fifocon,nakini,rwal,nakouti,rxstpi,rxouti,stalledi,txini */
  if (UEINT & 01) { /* control endpoint */
    UENUM = 0;
    if ((UEINTX & _BV(RXSTPI))) {
      if (gl.usb.ep0r == 0)
        while (gl.usb.ep0r < 8) gl.usb.ep0b.b[gl.usb.ep0r++] = UEDATX;
      UEINTX = 0;
    }
  }
  if (UEINT & 02) { /* IN endpoint dev->host, outgoing data */
    UENUM = 1;
    if ((UEINTX & _BV(TXINI))) {
      gl.usb.ep1h = 0;
      UEINTX &= ~_BV(TXINI);
    }
  }
  if (UEINT & 04) { /* OUT endpoint dev<-host, incoming data */
    UENUM = 2;
    if ((UEINTX & _BV(RXOUTI))) {
      gl.usb.ep2r += UEBCX;
      UEINTX &= ~_BV(RXOUTI);
    }
  }
  UENUM = ept;
}

void usbd_psnd(const void *buf, u08_t nbu)
{
  const u08_t *d = buf;
  u08_t   n;

  for (n = 0; n < nbu; n++) {
    while (!(UEINTX & (_BV(TXINI)|_BV(RXOUTI))));
    if (UEINTX & _BV(RXOUTI)) break;
    UEDATX = pgm_read_byte(d++);
  }
}

void usb_sq(void)
{
  u08_t i;

  if (gl.usb.ep0a == 0) return;
  /*
   * EP1, IN endpoint dev->host
   */
  if (gl.usb.ep1h == 0 && (CB_HASD(gl.usb.ep1r,gl.usb.ep1w) || gl.usb.sofi >= 40)) {
    UENUM = 1;
    UEDATX = gl.usb.mdst;
    UEDATX = gl.usb.lnst;
    while ((UEINTX & _BV(RWAL))) {
      if (CB_NO_D(gl.usb.ep1r,gl.usb.ep1w)) break;
      UEDATX = gl.usb.ep1b[i = gl.usb.ep1r];
      gl.usb.ep1r = ++i >= NEL(gl.usb.ep1b) ? 0 : i;
    }
    gl.usb.sofi = 0;
    gl.usb.ep1h = 1;
    UEINTX &= ~_BV(FIFOCON);
  }
  /*
   * control endpoint
   */
  if (gl.usb.ep0r == 0) return;
  i = SREG; cli();
  UENUM = 0;
  if (gl.usb.ep0b.bmRequestType & REQ_D2H)
    while (!(UEINTX & _BV(TXINI)));
  else  UEINTX = ~_BV(TXINI);
  switch ((gl.usb.ep0b.bmRequestType & REQ_TYP)) {
  case REQ_STD:
    switch (gl.usb.ep0b.bRequest) {
    case GET_STATUS:
      UEDATX = 0;
      UEDATX = 0;
      UEINTX &= ~_BV(TXINI);
      break;
    case SET_ADR:
      while (!(UEINTX & _BV(TXINI)));
      UDADDR = gl.usb.ep0b.wValue;
      UEINTX &= ~_BV(TXINI);
      UDADDR |= _BV(ADDEN);
      break;
    case GET_DSC:
      switch (gl.usb.ep0b.wValue>>8) {
      case DSC_DEV: /* 1 USB_DEVICE_DESCRIPTOR_TYPE */
        usbd_psnd(&usb_sdd,MIN(gl.usb.ep0b.wLength,sizeof(usb_sdd)));
        UEINTX &= ~_BV(TXINI);
        break;
      case DSC_CFG: /* 2 USB_CONFIGURATION_DESCRIPTOR_TYPE */
        usbd_psnd(&usb_cfg,MIN(gl.usb.ep0b.wLength,sizeof(usb_cfg)));
        UEINTX &= ~_BV(TXINI);
        break;
#if 0
      case DSC_STR: /* 3 USB_STRING_DESCRIPTOR_TYPE */
      case DSC_IFC: /* 4 USB_INTERFACE_DESCRIPTOR_TYPE */
      case DSC_EPT: /* 5 USB_ENDPOINT_DESCRIPTOR_TYPE */
      case DSC_DQU: /* 6 USB_DEVICE_QUALIFIER_TYPE */
      case DSC_OSC: /* 7 USB_OTHER_SPEED_CONFIGURATION_TYPE */
      case DSC_PWR: /* 8 USB_INTERFACE_POWER_TYPE */
#endif
      default: UECONX |= _BV(STALLRQ); break;
      }
      break;
    case GET_CFG:
      UEDATX = 1;
      UEINTX &= ~_BV(TXINI);
      break;
    case SET_CFG:
      switch ((gl.usb.ep0b.bmRequestType & REQ_DST)) {
      case REQ_DEV:
        UENUM   = 1;  /* IN DEV -> HOST */
        UECONX  = 1;
        UECFG0X = 0b10000001; /* ept[1:0],-,-,-,-,-,EPDIR  BULK IN */
        UECFG1X = (EP1_LGS<<4)|0b0110; /* -,epsize[2:0]=010,epbk[1:0]=01,ALLOC,- double */
        UEIENX  = 0b00000001; /* flerre,nakine,-,nakoute,rxstpe,rxoute,stallede,TXINE */
        UENUM   = 2;  /* OUT HOST -> DEV */
        UECONX  = 1;
        UECFG0X = 0b10000000; /* ept[1:0],-,-,-,-,-,epdir  BULK OUT */
        UECFG1X = (EP2_LGS<<4)|0b0110; /* -,epsize[2:0]=010,epbk[1:0]=01,ALLOC,- double */
        UEIENX  = 0b00000100; /* flerre,nakine,-,nakoute,rxstpe,RXOUTE,stallede,txine */
        break;
      case REQ_IFC:
      case REQ_EPT:
      default:
        break;
      }
      break;
    default: UECONX |= _BV(STALLRQ); break;
    }
    break;
  case REQ_VEN:
    switch (gl.usb.ep0b.bRequest) {
    case  0: /* FTDI_SIO_RESET     0 reset port */
      gl.usb.mdst = RST_MDST;
      gl.usb.lnst = RST_LNST;
      gl.usb.tlat = 0;
      UEINTX &= ~_BV(TXINI);
      break;
    case  3: /* FTDI_SIO_SET_BAUD_RATE   3 set baud rate */
#define FTDI_B9600  0x4138
#define FTDI_B1200  0x09c4
      switch (gl.usb.rbsq) {
      case 0: case 1:
        if (gl.usb.ep0b.wValue == FTDI_B9600) gl.usb.rbsq++;
        else gl.usb.rbsq = 0;
        break;
      case 2: if (gl.usb.ep0b.wValue == FTDI_B1200) gl.exit = 1;
        /* FALLTHROUGH */
      default:
        gl.usb.rbsq = 0;
        break;
      }
#undef  FTDI_B9600
#undef  FTDI_B1200
      /* FALLTHROUGH */
    case  1: /* FTDI_SIO_MODEM_CTRL    1 set modem control register */
    case  2: /* FTDI_SIO_SET_FLOW_CTRL   2 set flow control register */
    case  4: /* FTDI_SIO_SET_DATA    4 set data format */
    case  6: /* FTDI_SIO_SET_EVENT_CHAR  6 set event character */
    case  7: /* FTDI_SIO_SET_ERROR_CHAR  7 set error character */
      UEINTX &= ~_BV(TXINI);
      break;
    case  5: /* FTDI_SIO_GET_MODEM_STATUS  5 get modem status register */
      if (gl.usb.ep0b.wLength > 0) UEDATX = gl.usb.mdst;
      if (gl.usb.ep0b.wLength > 1) UEDATX = gl.usb.lnst;
      UEINTX &= ~_BV(TXINI);
      break;
    case  9: /* FTDI_SIO_SET_LATENCY_TIMER   9 set latency timer */
      gl.usb.tlat = gl.usb.ep0b.wValue;
      UEINTX &= ~_BV(TXINI);
      break;
    case 10: /* FTDI_SIO_GET_LATENCY_TIMER  10 get latency timer */
      if (gl.usb.ep0b.wLength > 0) UEDATX = (gl.usb.tlat>>0);
      if (gl.usb.ep0b.wLength > 1) UEDATX = (gl.usb.tlat>>8);
      UEINTX &= ~_BV(TXINI);
      break;
    default: UECONX |= _BV(STALLRQ); break;
    }
    break;
  default:
    UECONX |= _BV(STALLRQ);
    break;
  }
  gl.usb.ep0r = 0;
  SREG = i;
}

void usb_tw(void) /* wait for tx empty */
{
  do usb_sq(); while (gl.usb.sofi < 100 && CB_HASD(gl.usb.ep1r,gl.usb.ep1w));
}

void usb_init(void)
{
  u08_t sreg;

  sreg = SREG; cli();
  USBCON = 0b00000000;  /* usbe,-,frzclk,otgpade,-,-,-,vbuste */
  UHWCON = 0b00000001;  /* -,-,-,-,-,-,-,UVREGE */
  USBCON = 0b10100000;  /* USBE,-,FRZCLK,otgpade,-,-,-,vbuste */
  PLLCSR = 0b00010010;  /* -,-,-,PINDIV,-,-,PLLE,plock */
  while (!(PLLCSR & _BV(PLOCK)));
  UDINT  = 0b00000000;  /* -,uprsme,eorsme,wakeupe,eorste,sofe,-,suspe */
  USBCON = 0b10010000;  /* USBE,-,frzclk,OTGPADE,-,-,-,vbuste */
  UDIEN  = 0b00001100;  /* -,uprsme,eorsme,wakeupe,EORSTE,SOFE,-,suspe */
  UDCON  = 0b00000000;  /* -,-,-,-,rstcpu,lsm,rmwkup,detach */
  SREG = sreg;
}
#endif

#if AT2SO == 1
/*
 * drone companion program piggy back in flash
 */
u16_t rdflash(u16_t adr); /* read flash word */
void _rdflash(void) /* addr (r24:r25), return (r24:r25) */
{
asm("rdflash:     ");
asm(" push r31    ");
asm(" push r30    ");
asm(" movw r30,r24");
asm(" lpm  r24,Z+ ");
asm(" lpm  r25,Z  ");
asm(" pop  r30    ");
asm(" pop  r31    ");
}

void at2so_chck(u16_t adr)
{
  u16_t mag, siz, cks;

  if (adr == 0) return;
  mag = rdflash(adr); adr += 2;
  siz = rdflash(adr); adr += 2;
  cks = rdflash(adr); adr += 2;
  if (mag != 0x55aa) return;
  if (siz < ( 1*1024)) return;
  if (siz > (20*1024)) return;
  gl.arm.adr = adr;
  gl.arm.siz = siz;
  gl.arm.cks = cks;
}

int at2so_load(void)
{
  union { u16_t w; u08_t b[2]; } v;
  u16_t cks, adr;
  int n;

  /* put tty we are talking to into raw mode, will be
   * sending binary data -> NO interpretation please! */
  SIO_SP("stty -echo -iexten pass8 raw\n");
  ms_dly(50); sio_rf();
  /* 'dd' is the program of choice to receive the data, need to
   * tell it how much is coming, so it quits after the transfer.
   * 'dd' on drone is the 'amputated' version without bells and
   * whistles. */
  SIO_SP("dd of=/tmp/$F.gz bs=1 count=");
  snd_sid(0,gl.arm.siz);
  sio_sc('\n');
  ms_dly(100); sio_rf();
  for (adr = gl.arm.adr, n = cks = 0;;) {
    v.w = rdflash(adr+n); /* reads 2 bytes from flash */
    sio_sc(v.b[0]); cks += v.b[0];
    if (++n >= gl.arm.siz) break;
    sio_sc(v.b[1]); cks += v.b[1];
    if (++n >= gl.arm.siz) break;
    if ((n & 0x1f) == 0) {
      LEDTG(); /* makes LED look like it is dimmed */
      while (CB_HASD(gl.sio.txr,gl.sio.txw));
      ms_dly(1);
    }
  }
  ms_dly(50); sio_rf();
  /* put tty back to 'normal' */
  SIO_SP("stty sane\n"); ms_dly(50); sio_rf();
  if (n != gl.arm.siz || cks != gl.arm.cks) return 0; /* upload bad */
  /* unzip it */
  SIO_SP("gunzip /tmp/$F.gz\n"); ms_dly(100); sio_rf();
  ms_dly(50); sio_rf();
  return 1;
}
#endif

#define C_STRT  0x02
#define C_STOP  0x04

void at2so_lstp(u08_t n)
{
  while (1) {
    blip(1); ms_dly(1000);
    blip(n); ms_dly(2000);
  }
}

char sio_w4lf(int nms)
{
  int k;

  for ( ; nms; nms -= 1, ms_dly(1))
    while ((k = sio_rc()) >= 0) if (k == '\n') return 0;
  return -1;
}

char sio_w4ln(int nms, u08_t *lbu, u08_t nlb)
{
  int k;
  u08_t i;

  for (i = 0; nms; nms -= 1, ms_dly(1))
    while ((k = sio_rc()) >= 0) {
      if (k == '\n') return (char)i;
      if (i < nlb) lbu[i++] = k;
    }
  return -1;
}

int at2so_txfr(const char *pnm)
{
  u08_t lbu[4];

  sio_rf(); SIO_SP("F="); sio_sp(pnm); sio_rev(); SIO_SP(".arm\n");
  if (sio_w4lf(99)) return -1; /* communication bad */
  sio_rf(); SIO_SP("if [ -f $D/$F ];then cp $D/$F /tmp;echo 1;else echo 0;fi\n");
  if (sio_w4lf(99)) return -1; /* communication bad */
  return sio_w4ln(99,lbu,NEL(lbu)) < 1 ? -1 : lbu[0] == '1' ? 1 : 0;
}

int ld_comprgr(void)
{
  int n;

  if ((n = at2so_txfr(PSTR("pilot")))) return n;
  if ((n = at2so_txfr(PSTR("at2so")))) return n;
  return 0;
}

void at2so_exec(u08_t blkwifi)
{
  int n;

#if 1
  /* change baudrate since Arduino at 16 Mhz has problems receiving back to
   * back characters at 115200 with half speed receiver
   * 76800 would be nice because it can do it, but stty on drone is amputated... */
  if ((SIO_UCSRA & _BV(SIO_U2X))) {
    /* is using double speed on serial transmitter, which is not good for
     * receiver (half sampling rate) -> change baudrate to 38400 to fix this */
    SIO_SP("stty 38400\n");
    ms_dly(10); sio_rf();
    SIO_UBRR  = (F_CPU/(16UL*38400UL)-1);
    SIO_UCSRA = 0b00000000; /* rxc0,txc0,udre0,fe0,dor0,upe0,u2x0,mpcm0 */
    ms_dly(50); sio_rf();
    SIO_UCSRB |= _BV(SIO_RXCIE)|_BV(SIO_RXEN)|_BV(SIO_TXEN);
    sio_sc('\r');   /* send a character at new baudrate */
    ms_dly(20); sio_rf(); /* wait a while and clear residuals if any */
  }
#endif
  sio_rf(); sio_sc('\n');
  if (sio_w4lf(99)) at2so_lstp(1);  /* communication bad */
  sio_rf(); SIO_SP("D=/data/video/usb\n");
  if (sio_w4lf(99)) at2so_lstp(2);  /* communication bad */
  if ((n = ld_comprgr()) < 0) at2so_lstp(3);
  else if (n == 0) {
    sio_rf(); SIO_SP("D=/data/video\n");
    if (sio_w4lf(99)) at2so_lstp(5);/* communication bad */
    if ((n = ld_comprgr()) < 0) at2so_lstp(6);
  }
#if AT2SO == 1
  if (n == 0 && gl.arm.adr != 0) n = at2so_load();
#endif
  if (n == 0) at2so_lstp(8);    /* no companion program */

  LEDON();
  /* make companion program executable */
  SIO_SP("chmod +x /tmp/$F\n"); ms_dly(50); sio_rf();
  /* launch compagnion program */
  SIO_SP("/tmp/$F");
  if (blkwifi) SIO_SP(" -w");
  SIO_SP(" -l "); gl.bid++; snd_sid(0,gl.bid); /* boot id */
  ms_dly(25); sio_rf(); /* wait out echo */
  sio_sc('\n');   /* and launch */

  /* wait for companion program to send start character */
  while (1) {
    if ((n = sio_rc()) < 0) ms_dly(1);
    else if (n == C_STRT) break;
    while (n == C_STOP) at2so_lstp(9); /* there is another controller active */
  }
  eepr_save();  /* store new 'bid' in EEPROM */
}

/*
 * RC receiver interface debug/setup loop
 */
void csisclr(void)  { SND_SP("\033[H\033[0m\033[2J"); } /* home, clear attributes, clear screen */
void csirclr(void)  { SND_SP("\033[0J"); }
void csilclr(void)  { SND_SP("\033[K"); }
void csirow(int  r) { SND_SP("\033["); snd_sid(0,r); SND_SP(";1H"); }
void csilco(char c) { SND_SP("\033[0;"); snd_sbx(0x30+c); snd_sc('m'); }
void csihco(char c) { SND_SP("\033[1;"); snd_sbx(0x30+c); snd_sc('m'); }

#define DLN0  1
#define DLN1  2
#define DLN2  3
#define DLN3  4
#define DLNRXI  6
#define DLNRXC  7
#define DLNGGA  14
#define DLNRMC  15
#define DLNOTH  16
#define DLNBMPD 18  /* BMP data */
#define DLNBMPC 19  /* BMP calibration */
#define DLNHMCD 20  /* HMC data */
#define DLNHMCC 21  /* HMC calibration */

#define BLACK 0
#define RED 1
#define GREEN 2
#define YELLOW  3
#define BLUE  4
#define MAGENTA 5
#define CYAN  6
#define WHITE 7

u08_t rdbcfg(u08_t spmcsr, u16_t adr);
void _rdbcfg(void) /* spmcsr (r24), addr (r23:r22), return (r24) */
{
asm("rdbcfg:");
asm(" push r31");
asm(" push r30");
asm(" movw r30,r22");
asm(" out  0x37,r24");
asm(" lpm  r24,Z   ");
asm(" pop  r30");
asm(" pop  r31");
}

void rdcfg(u08_t *buf)
{
  u08_t srg;

  srg = SREG; cli();
  buf[0] = rdbcfg(0x09,3); /* high fuse */
  buf[1] = rdbcfg(0x09,0); /* low fuse */
  buf[2] = rdbcfg(0x09,2); /* ext fuse */
  buf[3] = rdbcfg(0x09,1); /* lockbits */
  buf[4] = rdbcfg(0x21,0); /* signature byte */
  buf[5] = rdbcfg(0x21,2); /* signature byte */
  buf[6] = rdbcfg(0x21,4); /* signature byte */
  SREG = srg;
}

void snd_est(void)
{
  SND_SP("$GPERR,");
  snd_sbx(gl.gps.est);
  snd_sc(',');
  snd_sbx(gl.gps.ech);
  gl.gps.est = 0;
}

void su_sgps(void)
{
  char  c;
  int i;

  if (gl.gps.est) csirow(DLNOTH), csihco(RED), snd_est(), csilclr();
  while (CB_HASD(gl.gps.b_r,gl.gps.b_w) && sio_tf() >= 100) {
    if (++gl.gps.b_r >= NEL(gl.gps.buf)) gl.gps.b_r = 0;
    if ((i = gl.gps.b_r + 2) >= NEL(gl.gps.buf)) i -= NEL(gl.gps.buf);
    c = gl.gps.buf[i];
    if (c == 'G') csirow(DLNGGA), csihco(GREEN);
    else if (c == 'R') csirow(DLNRMC), csihco(GREEN);
    else csirow(DLNOTH), csihco(RED);
    while (1) {
      c = gl.gps.buf[gl.gps.b_r];
      if (++gl.gps.b_r >= NEL(gl.gps.buf)) gl.gps.b_r = 0;
      if (c == '\r') break;
      snd_sc(c);
    }
    csilclr();
  }
}

#define HZ2TIC(hz)  ((double)F_TM1/(double)(hz))
#define DRUPD   (u32_t)HZ2TIC(LOOPHZ) /* dr_loop() cycle time in timer 1 ticks */

void su_loop(void)
{
  u08_t i = 0, k, n, msk;
  char  j;
  int rxs;
  long  tmp;
  u32_t nxt;
  u08_t cfg[7];

#if U4 == 0
#define GUI_TW  sio_tw
#define GUI_RC  sio_rc
#define GUI_TF  sio_tf
#else
#define GUI_TW  usb_tw
#define GUI_RC  usb_rc
#define GUI_TF  usb_tf
#endif
  sio_rf();
  while (1) {
    if (gl.exit) return;
    if (++i < 21 && (i & 02)) LEDTG();
    else if (i >= 40) i = 0;
    nxt = tic() + DRUPD;
    do GUI_TW(); while ((tmp = nxt - tic()) > 0);
    if (GUI_RC() == C_STRT) break;
#if U4
    if (CB_NO_D(gl.sio.rxr,gl.sio.rxw)) continue;
    /* USB <-> RS232 serial bridge */
    csisclr();
    while (gl.usb.sofi < 100) {
      while ((rxs = sio_rc()) > 0) usb_sc(rxs);
      usb_tw();
      while ((rxs = usb_rc()) > 0) sio_sc(rxs);
      if ((tmp = nxt - tic()) <= 0)
        LEDTG(), nxt = tic() + MS2TIC(250);
    }
    return;
#endif
  }
  LEDOF();
  gl.dsnd = U4 ? 1 : 0;
AGAIN:
  GUI_TW();
  csisclr();
  csirow(DLN0); csihco(CYAN);
  SND_SP("rx2at ");
  snd_sp(version);
  SND_SP(", at2so ");
  if (gl.arm.adr == 0) SND_SP("not ");
  SND_SP("attached");

  rdcfg(cfg);
  csirow(DLN1);
  csilco(WHITE); SND_SP("cpusg ");
  k = 0x1e;   i = cfg[4]; csihco(i==k?GREEN:RED); snd_sbx(i);
  k = 0x95;   i = cfg[5]; csihco(i==k?GREEN:RED); snd_sbx(i);
  k = U4 ? 0x87:0x0f; i = cfg[6]; csihco(i==k?GREEN:RED); snd_sbx(i);
  csilco(WHITE); SND_SP(" fuses ");
  snd_sbx(cfg[0]); snd_sc('-'); /* high */
  snd_sbx(cfg[1]); snd_sc('-'); /* low */
  snd_sbx(cfg[2]); snd_sc('-'); /* extended */
  snd_sbx(cfg[3]);    /* lockbits */
  csihco(YELLOW);
  if (S_LAND == S_FMOD) SND_SP(" 1sM");
  else SND_SP(" 2sM");
  snd_snx(T_MODE);

  GUI_TW();
  csirow(DLN2); csilco(WHITE);
  SND_SP(" loop ");
  snd_spd(0,1,TIC2MS10(DRUPD));
  SND_SP("ms, dcnt="); snd_sid(0,gl.dcnt);

  csirow(DLNRXI); csihco(CYAN);
  SND_SP("-RX-  f[ms]  p[ms] value");

  tmr0_setup(0);
GPSCHANGE:
  GUI_TW();
  csirow(DLN3); csilco(WHITE);
  SND_SP("  gps ");
  if (gl.gps.b100) {
    snd_sid(0,gl.gps.b100);
    SND_SP("00 bps");
  }
  csilclr();

  nxt = tic() + DRUPD;
  for (n = k = 0; !gl.exit; n ^= 01) {
    /* EEPROM update */
    if (gl.eewen < NEL(gl.eedat)) eepr_updt();
    i = gl.gps.tcnt;
    LEDOF();
    do {
      usb_sq();
      gps_rcv();
      bmp_sq();
      m56_sq();
      hmc_sq();
    }
    while ((tmp = nxt - tic()) > 0);
    LEDON();
    if (i != gl.gps.tcnt) goto GPSCHANGE;
    nxt = tic() + DRUPD;
    GUI_TW();

    rxs = rx_read();
    beep_sq();
    if (n & 01) {
      for (i = 0; i < NEL(gl.rxs.chn); i++) {
        if (!((msk = _BV(i)) & S_SIG)) continue;
        if (GUI_TF() < 48) GUI_TW();
        csirow(DLNRXC+i);
        csihco(rxs&msk?RED:GREEN);
        switch (i) {
        case S_THRO: SND_SP("THRO"); break;
        case S_AILE: SND_SP("AILE"); break;
        case S_ELEV: SND_SP("ELEV"); break;
        case S_RUDD: SND_SP("RUDD"); break;
        case S_GEAR: SND_SP("GEAR"); break;
        case S_AUX1: SND_SP("AUX1"); break;
        }
        csihco(gl.rxs.err&msk?RED:GREEN);
        snd_spd(7,1,TIC2MS10(gl.rxs.chn[i].ftm));
        snd_spd(7,3,TIC2US(gl.rxs.chn[i].dur));
        snd_sid(-6,gl.rxs.chn[i].val);
        if (rxs == 0) {
          if (i == S_LAND)
            switch (gl.rxs.fms) {
            case FMS_LAND: SND_SP(" LAND"); break;
            case FMS_FM_1: SND_SP(" FM_1"); break;
            case FMS_FM_2: SND_SP(" FM_2"); break;
            case FMS_FM_3: SND_SP(" FM_3"); break;
            default:       SND_SP(" ????"); break;
            }
          if (gl.rxs.fms == FMS_LAND) {
            if (i == SH_THRO) {
              if (gl.rxs.stk & TS_LF) {
                if (gl.rxs.stk & TS_DN)
                  SND_SP(" HOME");
#if GSWO_ENA >= 0 && GSWO_SRC == 1
                else if (gl.rxs.stk & TS_UP)
                  SND_SP(" ALRM");
#endif
                else  SND_SP(" TRIM");
              }
              else if (gl.rxs.stk & TS_RT) SND_SP(" ESTP");
            }
            if (i == S_THRO) {
              j = gl.cfg;
              if (gl.rxs.stk & TS_UP) {
                if ((j = gl.cfw) < 0) j = gl.cfg;
                if ((gl.rxs.stk & ES_MSK)) csihco(YELLOW);
              }
              SND_SP(" CFG"); snd_snx(j+1);
            }
            if (i == SH_ELEV && !(gl.rxs.stk & TS_UP)) {
              if (gl.rxs.stk & ES_RT) SND_SP(" VID+");
            }
          }
          else {
            if (i == S_LAND && gl.rxs.fsa) {
                 SND_SP(" ANI"); snd_snx(gl.rxs.fsa);
            }
          }
        }
        csilclr();
      }
      SND_SP("\r\n");
    }
    else {
      if (gl.gps.est || CB_HASD(gl.gps.b_r,gl.gps.b_w)) GUI_TW(), su_sgps();
      else {
        if ((msk = bmp_rp(0))) {
          if (msk & 01) {
            csirow(DLNBMPD); csihco(WHITE);
            msk = bmp_rp(01);
            csilclr();
          }
          if (msk & 02) {
            csirow(DLNBMPC); csihco(WHITE);
            msk = bmp_rp(02);
            csilclr();
          }
        }
        if ((msk = m56_rp(0))) {
          if (msk & 01) {
            csirow(DLNBMPD); csihco(WHITE);
            msk = m56_rp(01);
            csilclr();
          }
          if (msk & 02) {
            csirow(DLNBMPC); csihco(WHITE);
            msk = m56_rp(02);
            csilclr();
          }
        }
        if ((msk = hmc_rp(0))) {
          if (msk & 01) {
            csirow(DLNHMCD); csihco(WHITE);
            msk = hmc_rp(01);
            csilclr();
          }
          if (msk & 02) {
            csirow(DLNHMCC); csihco(WHITE);
            msk = hmc_rp(02);
            csilclr();
          }
        }
      }
      if (GUI_RC() == C_STRT) goto AGAIN;
    }
  }
#undef  GUI_TW
#undef  GUI_RC
#undef  GUI_TF
}

/*
 * DRONE control loop
 */
void dr_scfg(void)
{
  const char  *p;

  SIO_SP("RX=C,");
  switch (gl.cfg) {
  default:
  case 0: p = cfg1; break;
  case 1: p = cfg2; break;
  case 2: p = cfg3; break;
  case 3: p = cfg4; break;
  }
  sio_sp(p);
  sio_sc('\r');
}

void dr_vlba(void)
{
  if (++gl.vlba.tick < 6) {
    if (VLBA_BLINK && gl.vlba.stat && gl.vlba.tick == 2) VLBOF();
    return;
  }
  gl.vlba.tick = 0;
  switch (gl.vlba.stat) {
  case 0: if (VLBA_THR > 0 && VLBA_THR <= 60 && gl.drm.m2a.cbat <= VLBA_THR) {
      VLBON();
      SIO_SP("RX=B,1");
      gl.vlba.stat = 1;
      break;
    }
    break;
  case 1: if (gl.drm.m2a.cbat > VLBA_THR) {
      VLBOF();
      SIO_SP("RX=B,0");
      gl.vlba.stat = 0;
    }
    else VLBON();
    break;
  }
}

void dr_sgps(void) /* send GPS data to drone */
{
  u08_t c, r, w;

  if (gl.gps.est) snd_est(), sio_sc('\r');
  r = gl.gps.b_r;
  w = gl.gps.b_w;
  while (CB_HASD(r,w) && sio_tf() >= 80)
    do {
      c = gl.gps.buf[r];
      if (++r >= NEL(gl.gps.buf)) r = 0;
      sio_sc(c);
      if (c == '\r') break;
    }
    while (CB_HASD(r,w));
  gl.gps.b_r = r;
}

void dr_send(u08_t s)
{
  u08_t rp;

  SIO_SP("RX=");
  if (gl.pad < 0) sio_sc('E');
  else if (s == 9) sio_sc('D');
  else snd_sid(0,s==10?gl.rxs.fms:0);
  snd_sid(',',gl.rxs.chn[S_ROL].val);
  snd_sid(',',gl.rxs.chn[S_PTC].val);
  snd_sid(',',gl.rxs.chn[S_GAZ].val);
  snd_sid(',',gl.rxs.chn[S_YAW].val);
  if (gl.rxs.fsa) { /* animation trigger */
    SIO_SP("\rRX=A");
    snd_sid(',',gl.rxs.fsa);
    gl.rxs.fsa = 0;
  }
  sio_sc('\r');
  if (gl.gps.est || CB_HASD(gl.gps.b_r,gl.gps.b_w)) dr_sgps();
  if ((rp = bmp_rp(0)) && sio_tf() >= 80) {
         if (rp & 02) rp = bmp_rp(02), sio_sc('\r');
    else if (rp & 01) rp = bmp_rp(01), sio_sc('\r');
  }
  if ((rp = m56_rp(0)) && sio_tf() >= 80) {
         if (rp & 02) rp = m56_rp(02), sio_sc('\r');
    else if (rp & 01) rp = m56_rp(01), sio_sc('\r');
  }
  if ((rp = hmc_rp(0)) && sio_tf() >= 80) {
         if (rp & 02) rp = hmc_rp(02), sio_sc('\r');
    else if (rp & 01) rp = hmc_rp(01), sio_sc('\r');
  }
  sio_sc('\n');
}

void dr_loop(void)
{
  u08_t s, i, k, led;
  long  tmp;
  u32_t nxt;

  gl.rxs.stk = 0;
  led = 0;
  s = i = k = 0;
  /* This is it, this program is talking to the program on the drone now.
   * s < 3 -> drone configuration, s >= 3 -> TX/RX in charge */
  sio_rf();
  nxt = tic() + DRUPD;
  tmr0_setup(0);  /* get GPS setup going */
  while (1) {
    /* packet to drone */
    dr_send(s);
    /* EEPROM update */
    if (gl.eewen < NEL(gl.eedat)) eepr_updt();

    /* keep loop on track, don't want to be too pushy or late */
    gl.drm.ack = 0;
    do {
      gps_rcv();
      sio_rcv();
      bmp_sq();
      m56_sq();
      hmc_sq();
    }
    while ((tmp = nxt - tic()) > 0);
    nxt = tic() + DRUPD;

    gl.rxs.chg = gl.rxs.stk;  /* remember previous */
    rx_read();

    if (s < 3) {  /* drone configuration */
      switch (s) {
      case 0: dr_scfg();
        k = S2LTIC(3.0);
        LEDOF();
        s = 1;
        break;
      case 1: if (gl.drm.ack) s = 2;  /* received C_ACK from drone */
        else if (--k == 0) s = 0, LEDON();
        break;
      case 2: i = 0;
        s = 3;
        break;
      }
      continue;
    }
    /* Arduino LED blink */
#if NL_AIR
    if (s == 10) LEDOF(); else
#endif
    if (led == 0) LEDTG();
    else if ((led >>= 1) == 0) {
      led = 1<<(5-1);
      if (i < k) LEDON();
      if (++i >= 6) i = 0;
    }
    else LEDOF();

    /* chores */
    dr_vlba();
    beep_sq();

    /* check radio link */
    if (gl.rxs.err) { /* radio quit or got disconnected from TX */
      led = 0;
      s = 9;
      continue;
    }

    switch (s) {
    /*
     * drone on ground, or getting there
     */
    case 3:
      gl.rxs.chg ^= gl.rxs.stk;
      if (gl.rxs.fms == FMS_LAND) {
        if (gl.rxs.chg & TS_RT) {
          if (gl.rxs.stk & TS_RT) /* EMERGENCY */
            gl.pad = -1;
          else  gl.pad =  0;
        }
        else if (gl.rxs.stk & gl.rxs.chg & TS_LF) {
          if (gl.rxs.stk & TS_DN)
            SIO_SP("RX=H\r"); /* HOME */
          else  SIO_SP("RX=T\r"); /* FTRIM */
        }
        if (gl.rxs.stk & TS_UP) { /* show/change config */
          led = 1;
          k = gl.cfg + 1;
          i = 0;
          s = 5;
        }
        else if (gl.rxs.stk & TS_DN) { /* show GPS status */
          if (gl.drm.m2a.sgps >= 0) {
            if (gl.rxs.chg & TS_DN) led = 1;
            k = gl.drm.m2a.sgps;
          }
        }
        else if (gl.rxs.chg & TS_DN) led = 0;
        if (gl.rxs.stk & gl.rxs.chg & ES_RT) /* VZAP */
          SIO_SP("RX=V,Z\r");
        break;
      }
      if (gl.eewen < NEL(gl.eedat)) break;  /* wait for EEPROM write to complete */
      if (gl.rxs.stk != 0) break;   /* all sticks centered for launch */
      if (gl.rxs.chn[S_THRO].val) break;  /* throttle should be dead 0 */
      gl.pad = 1, s = 10;     /* launch */
      SIO_SP("RX=V,1\r");     /* video record on */
      break;
    case 5: /* display/set configuration */
      k = (gl.cfw < 0 ? gl.cfg : gl.cfw) + 1;
      if (gl.rxs.stk & TS_UP) break;
      if (gl.eewen < NEL(gl.eedat)) /* did get changed */
        s = i = 0;    /* send new configuration to drone */
      else  s = 3;
      led = 0;
      break;
    case 9: /* radio loss recovery */
      if (gl.rxs.fms == FMS_LAND) gl.pad = 0;
      s = gl.pad > 0 ? 10 : 3;
      break;
    /*
     * drone airborne
     */
    case 10:
      if (gl.rxs.fms != FMS_LAND) break;
      gl.pad = 0;
      SIO_SP("RX=V,0\r"); /* video record off */
      s = 3;
      break;
    }
  }
}

void init(void)
{
extern  u08_t __bss_start;
extern  u08_t __bss_end;
  u08_t i, k, *b = &__bss_start;
  int n = &__bss_end - &__bss_start;

  cli();
#if U4
  UDIEN  = 0;
  USBCON = 0;
  UHWCON = 0;
#endif
  while (--n >= 0) *b++ = 0;  /* clear BSS segment */

  for (EECR = 0; gl.eewen < NEL(gl.eedat); gl.eewen++) {
    EEAR = (int)gl.eewen;
    EECR |= _BV(EERE);
    gl.eedat[gl.eewen] = EEDR;
  }
  for (i = k = 0; i < (NEL(gl.eedat)-1); i++) k += gl.eedat[i];
  if (gl.eemag != EE_MAG || gl.eecks != k) gl.cfg = gl.bid = 0;
  gl.cfw = -1;
#if AT2SO == 1
  extern  u16_t flash_pgb(void);
  at2so_chck(flash_pgb());
#endif
}

int main(void)
{
  u08_t i, j, k;

AGAIN:
  init();

#if U4 == 0
  /* Nano, ProMini - RC receiver hardware bits on port D[7:2] */
#if RX_CON == 0
  gl.rxs.chn[S_THRO].msk = _BV(7);
  gl.rxs.chn[S_AILE].msk = _BV(6);
  gl.rxs.chn[S_ELEV].msk = _BV(5);
  gl.rxs.chn[S_RUDD].msk = _BV(4);
  gl.rxs.chn[S_GEAR].msk = _BV(3);
  gl.rxs.chn[S_AUX1].msk = _BV(2);
#elif RX_CON == 1
  gl.rxs.chn[S_THRO].msk = _BV(2);
  gl.rxs.chn[S_AILE].msk = _BV(3);
  gl.rxs.chn[S_ELEV].msk = _BV(4);
  gl.rxs.chn[S_RUDD].msk = _BV(5);
  gl.rxs.chn[S_GEAR].msk = _BV(6);
  gl.rxs.chn[S_AUX1].msk = _BV(7);
#elif RX_CON == 2
  /* put your own bit numbers here */
  gl.rxs.chn[S_THRO].msk = _BV(7);
  gl.rxs.chn[S_AILE].msk = _BV(6);
  gl.rxs.chn[S_ELEV].msk = _BV(5);
  gl.rxs.chn[S_RUDD].msk = _BV(4);
  gl.rxs.chn[S_GEAR].msk = _BV(3);
  gl.rxs.chn[S_AUX1].msk = _BV(2);
#else
#error  "RX_CON bad value"
#endif
#else
  /* MiruPCB - RC receiver hardware bits on port B[6:1] */
#if RX_CON == 0
  gl.rxs.chn[S_THRO].msk = _BV(1);
  gl.rxs.chn[S_AILE].msk = _BV(2);
  gl.rxs.chn[S_ELEV].msk = _BV(3);
  gl.rxs.chn[S_RUDD].msk = _BV(4);
  gl.rxs.chn[S_GEAR].msk = _BV(5);
  gl.rxs.chn[S_AUX1].msk = _BV(6);
#elif RX_CON == 1
  gl.rxs.chn[S_THRO].msk = _BV(6);
  gl.rxs.chn[S_AILE].msk = _BV(5);
  gl.rxs.chn[S_ELEV].msk = _BV(4);
  gl.rxs.chn[S_RUDD].msk = _BV(3);
  gl.rxs.chn[S_GEAR].msk = _BV(2);
  gl.rxs.chn[S_AUX1].msk = _BV(1);
#elif RX_CON == 2
  /* put your own bit numbers here */
  gl.rxs.chn[S_THRO].msk = _BV(1);
  gl.rxs.chn[S_AILE].msk = _BV(2);
  gl.rxs.chn[S_ELEV].msk = _BV(3);
  gl.rxs.chn[S_RUDD].msk = _BV(4);
  gl.rxs.chn[S_GEAR].msk = _BV(5);
  gl.rxs.chn[S_AUX1].msk = _BV(6);
#else
#error  "RX_CON bad value"
#endif
#endif

  /* dead bands */
  gl.rxs.chn[S_THRO].dbn = MS2TIC(0.06);
  gl.rxs.chn[S_AILE].dbn = MS2TIC(0.04);
  gl.rxs.chn[S_ELEV].dbn = MS2TIC(0.04);
  gl.rxs.chn[S_RUDD].dbn = MS2TIC(0.04);
  gl.rxs.chn[S_GEAR].dbn = MS2TIC(0.20);
  gl.rxs.chn[S_AUX1].dbn = MS2TIC(0.20);

  port_setup();
  LEDON();
  tmr0_setup(0);
  tmr1_setup();
  sio_setup();
  i2c_setup();
  gl.rxs.smp = RXSIG;
  gl.gps.rxi = GPS_RXI_INPUT();
  sei();    /* turn interrupts on */
  msdly_cali(); /* calculates propper gl.dcnt */
  SIO_UCSRB |= _BV(SIO_RXCIE)|_BV(SIO_RXEN)|_BV(SIO_TXEN);  /* turn USART on */
  GPS_RXI_IRQON();  /* enable GPS softserial port */
  PIN_CHG_IRQEN();  /* turn pin change interrupts on */
  LEDOF();

#if U4
  usb_init();
  ms_dly(500);
  if (gl.usb.ep0a) {
    su_loop();
    while (1) PORTB = 0, DDRB = 0x80; /* hardware reset (RST connected to this pin) */
    goto AGAIN;
  }
  UDIEN  = 0;
  USBCON = 0;
  UHWCON = 0;
#else
  if (SETUP()) {
    sio_rf();
    ms_dly(500);
    su_loop();
    goto AGAIN;
  }
#endif

  /* wait for drone to boot
   * make sure radio works and has 'sane' state, aka is in LAND */
#define NS_BOOT 18
#define NSQUIET 5
  gl.rxs.erx = 255; /* mark radio down */
  for (i = j = k = 0; 1; ) {
    if (sio_rf() || j < NS_BOOT) i = 0;
    else if (i < NSQUIET) i++;
    if (rx_read()) k = 0;
    else if (gl.rxs.fms != FMS_LAND) k = 1;
    else if (j < NS_BOOT) k = 2;
    else if (i < NSQUIET) k = 3;
    else break; /* all conditions met, go on */
    switch (k) {
    case 0: blip(1); ms_dly(800-50); break; /* radio not ready */
    case 1: blip(2); ms_dly(600-50); break; /* flight mode not LAND */
    case 2: blip(3); ms_dly(400-50); break; /* drone still booting */
    case 3: blip(4); ms_dly(200-50); break; /* drone still talking */
    }
    /* Note: this loop goes on a 1 second pace, need to refresh RX,
     *       otherwise 'dead signal' detector triggers on rx_read() */
    rx_read(); ms_dly(50);
    if (j < NS_BOOT) j++;
  }

  /* start shell on drone */
  sio_sc('\r'); ms_dly(100); sio_rf();
  sio_sc('\r'); ms_dly( 50); sio_rf(); /* do it twice... */

  LEDON();
  /* look at throttlestick, if it is not centered, user wants to lock out WiFi... */
  k = (gl.rxs.stk & (TS_UP|TS_DN)) ? 1 : 0;
  at2so_exec(k);  /* load/launch companion program on drone */
  LEDOF();

  dr_loop();
  /* NOTREACHED */
  return 0;
}

#if AT2SO == 1

void _flash_pgb(void) {
asm("flash_pgb:");
asm(" call eod_flash_pgb");
asm(" .word 0x55aa"); /* magic number */
asm(" .word  12559"); /* size */
asm(" .word 0x8bc7"); /* checksum */
asm(" .byte 31,139,8,0,45,92,204,83,2,3,205,124,125,96,84,197");
asm(" .byte 185,247,156,179,187,73,8,1,54,31,40,70,36,39,49,81");
asm(" .byte 170,73,56,27,2,70,12,186,33,225,75,3,228,11,164,45");
asm(" .byte 154,132,100,129,180,33,89,147,69,241,94,250,178,201,46,31");
asm(" .byte 90,162,145,196,202,229,202,205,90,233,173,175,229,218,84,189");
asm(" .byte 214,107,177,13,31,106,170,216,130,98,181,20,123,119,207,230");
asm(" .byte 188,161,193,190,210,138,138,31,117,223,223,51,51,75,14,234");
asm(" .byte 189,239,189,239,95,111,116,152,103,102,158,153,121,230,153,231");
asm(" .byte 107,230,156,61,91,23,86,46,82,20,133,197,255,84,54,147");
asm(" .byte 81,105,203,253,140,21,35,47,173,162,58,230,40,102,26,75");
asm(" .byte 66,219,149,44,147,218,189,75,42,25,91,114,76,164,51,76");
asm(" .byte 36,187,76,9,76,244,45,246,139,196,7,68,114,200,118,27");
asm(" .byte 181,161,92,220,37,82,58,19,201,62,142,42,254,252,34,233");
asm(" .byte 203,68,114,200,58,106,23,117,10,79,39,80,177,199,37,230");
asm(" .byte 165,118,149,232,94,70,73,225,41,62,104,130,133,198,37,40");
asm(" .byte 47,233,18,73,99,34,197,219,170,77,95,51,251,154,191,120");
asm(" .byte 255,89,173,45,107,103,181,54,23,180,182,180,109,218,92,216");
asm(" .byte 217,94,56,91,212,59,37,237,139,151,175,148,188,20,125,168");
asm(" .byte 62,15,105,166,28,231,26,164,43,37,60,77,182,197,255,38");
asm(" .byte 200,252,50,217,127,146,44,39,34,77,71,202,150,229,41,72");
asm(" .byte 215,34,101,32,93,133,116,57,82,46,251,250,63,77,230,51");
asm(" .byte 36,45,196,255,201,72,153,72,87,179,255,252,207,241,165,114");
asm(" .byte 178,5,78,145,252,160,191,36,235,190,225,111,162,133,103,169");
asm(" .byte 95,51,110,154,5,190,66,238,127,252,47,75,230,83,145,114");
asm(" .byte 190,166,239,205,152,232,216,206,241,113,218,80,126,199,82,30");
asm(" .byte 70,26,177,148,127,130,116,206,82,190,21,248,159,91,202,55");
asm(" .byte 18,253,247,141,151,239,65,123,134,165,188,242,75,116,79,252");
asm(" .byte 18,111,51,191,84,190,30,253,243,45,253,203,81,46,177,148");
asm(" .byte 31,68,185,194,82,126,1,169,202,82,94,133,246,53,150,114");
asm(" .byte 20,105,131,165,252,56,146,207,82,78,6,190,223,82,190,15");
asm(" .byte 229,93,150,242,29,164,31,150,114,51,202,251,45,229,30,164");
asm(" .byte 65,75,57,15,237,7,45,229,34,148,135,45,229,211,72,39");
asm(" .byte 45,229,103,144,194,150,242,20,224,191,103,41,191,133,116,193");
asm(" .byte 82,238,39,185,185,127,188,124,25,240,157,150,242,103,36,235");
asm(" .byte 150,242,33,210,29,75,185,132,108,135,165,76,130,231,182,148");
asm(" .byte 127,131,84,105,41,79,71,251,106,75,185,245,75,251,57,70");
asm(" .byte 60,177,180,255,35,146,215,58,62,244,125,35,233,249,92,86");
asm(" .byte 95,191,126,99,123,91,125,167,175,177,195,87,95,207,234,111");
asm(" .byte 189,187,190,198,179,190,165,211,231,233,40,111,109,236,236,244");
asm(" .byte 116,18,114,135,143,176,93,172,169,181,189,233,187,245,235,61");
asm(" .byte 62,95,203,70,15,53,172,111,106,170,239,20,109,245,245,141");
asm(" .byte 158,198,181,45,245,155,218,238,105,105,107,174,111,242,122,235");
asm(" .byte 189,29,250,215,87,187,168,111,147,160,160,19,67,122,124,172");
asm(" .byte 211,215,209,228,189,151,117,122,59,90,218,124,235,168,216,209");
asm(" .byte 180,161,131,53,181,183,181,121,154,124,108,83,27,140,211,119");
asm(" .byte 209,13,148,181,177,78,79,43,85,54,174,109,239,240,49,144");
asm(" .byte 227,109,105,166,30,190,246,86,202,90,129,177,209,179,177,19");
asm(" .byte 163,214,215,123,58,58,218,218,235,65,119,163,175,165,189,141");
asm(" .byte 173,5,17,204,215,132,182,70,159,175,131,117,120,26,155,9");
asm(" .byte 151,166,238,104,108,233,244,160,166,233,238,117,29,237,27,89");
asm(" .byte 75,123,147,15,195,221,139,9,55,178,181,77,237,192,192,242");
asm(" .byte 129,209,238,37,10,124,156,60,204,77,244,183,53,130,29,157");
asm(" .byte 158,182,102,95,59,107,108,106,242,120,125,152,99,189,156,3");
asm(" .byte 168,32,162,189,3,196,208,170,37,175,55,54,182,180,177,123");
asm(" .byte 58,90,124,30,212,55,109,110,172,111,244,121,54,183,248,216");
asm(" .byte 226,242,242,250,217,133,115,216,226,202,165,11,202,235,139,10");
asm(" .byte 139,185,221,140,255,103,67,178,75,91,252,223,255,207,206,255");
asm(" .byte 37,241,82,216,98,105,59,73,191,87,94,115,239,68,178,124");
asm(" .byte 143,74,131,71,237,133,150,246,140,150,150,73,100,101,247,91");
asm(" .byte 218,3,178,157,93,108,87,47,182,191,179,92,97,83,97,248");
asm(" .byte 151,32,39,93,168,164,28,19,87,81,14,99,93,71,57,150");
asm(" .byte 177,154,114,24,228,53,148,195,176,54,80,14,199,208,76,57");
asm(" .byte 250,111,160,28,148,181,82,14,227,236,165,28,51,249,40,135");
asm(" .byte 193,223,76,57,28,199,22,202,65,140,159,114,24,231,32,229");
asm(" .byte 16,244,157,148,195,16,239,162,28,142,165,151,114,24,225,126");
asm(" .byte 202,225,144,246,80,14,71,243,40,229,112,92,33,202,97,188");
asm(" .byte 247,83,14,35,248,4,229,112,108,7,40,135,179,26,164,28");
asm(" .byte 206,233,89,202,225,124,158,167,28,198,253,32,229,96,212,16");
asm(" .byte 229,112,104,71,41,135,161,31,166,28,14,233,24,229,112,102");
asm(" .byte 199,41,135,179,60,73,57,156,93,146,187,224,79,112,239,103");
asm(" .byte 147,252,47,158,177,135,11,76,123,120,192,156,28,126,48,156");
asm(" .byte 116,238,151,166,239,37,48,242,229,7,141,148,87,119,25,190");
asm(" .byte 15,15,154,113,184,213,2,55,91,224,53,22,184,206,2,87");
asm(" .byte 90,224,10,11,92,106,129,139,45,112,190,5,206,181,192,211");
asm(" .byte 45,112,134,5,78,177,192,118,11,252,249,249,113,248,188,5");
asm(" .byte 126,207,2,143,88,224,211,22,248,164,5,62,102,129,143,90");
asm(" .byte 224,131,22,248,89,11,124,192,2,239,183,192,143,90,224,126");
asm(" .byte 11,188,203,2,7,45,240,22,11,236,179,192,173,22,184,217");
asm(" .byte 2,175,33,120,48,20,101,225,80,212,238,220,55,50,73,11");
asm(" .byte 69,236,26,246,148,21,152,206,161,1,211,62,84,96,166,176");
asm(" .byte 1,51,69,31,48,247,198,98,103,123,144,90,177,199,7,158");
asm(" .byte 135,156,29,133,14,161,62,3,237,54,253,193,48,211,250,70");
asm(" .byte 25,171,137,206,136,205,82,118,199,98,99,190,35,144,81,210");
asm(" .byte 55,224,48,237,77,147,218,20,45,100,99,218,17,7,112,34");
asm(" .byte 246,21,10,244,116,192,36,121,98,122,47,218,107,163,144,169");
asm(" .byte 36,142,47,225,217,192,35,57,43,89,38,148,84,209,143,143");
asm(" .byte 40,218,137,17,149,213,70,108,220,229,215,70,63,137,197,50");
asm(" .byte 21,22,162,121,35,44,158,235,33,106,31,99,67,39,76,197");
asm(" .byte 233,55,82,88,77,4,170,155,169,104,111,142,0,55,66,180");
asm(" .byte 124,140,126,95,238,163,104,53,6,181,163,92,166,12,1,119");
asm(" .byte 232,208,8,52,61,122,65,226,58,137,86,214,101,178,33,224");
asm(" .byte 130,150,249,152,31,52,164,199,105,65,95,91,134,8,5,198");
asm(" .byte 242,81,62,143,126,24,55,170,232,199,64,51,120,60,212,101");
asm(" .byte 34,70,27,203,97,117,168,187,90,117,240,80,174,54,74,253");
asm(" .byte 243,121,93,165,131,29,234,74,84,152,95,85,220,167,49,119");
asm(" .byte 93,84,101,85,169,127,141,197,94,96,90,40,10,179,49,102");
asm(" .byte 215,31,26,97,67,111,154,57,160,75,25,10,168,246,33,197");
asm(" .byte 145,36,194,31,180,129,46,180,41,238,128,65,116,127,74,243");
asm(" .byte 187,9,183,46,202,134,142,56,20,173,27,245,53,145,63,99");
asm(" .byte 60,39,248,10,235,22,165,53,1,126,62,31,124,31,202,9");
asm(" .byte 69,210,128,163,30,234,27,85,220,161,104,150,147,25,25,169");
asm(" .byte 187,34,106,106,96,180,66,235,51,65,11,228,195,31,170,208");
asm(" .byte 2,63,162,254,73,216,195,92,109,192,220,83,188,219,124,52");
asm(" .byte 99,183,169,136,125,73,86,244,128,145,69,188,209,67,71,246");
asm(" .byte 20,119,155,113,30,19,254,133,133,5,127,106,118,15,152,75");
asm(" .byte 176,158,71,235,250,77,22,10,69,118,59,18,194,138,63,20");
asm(" .byte 185,245,127,108,55,236,222,29,97,230,12,69,19,177,15,105");
asm(" .byte 144,183,68,103,40,146,5,220,100,148,31,70,57,253,170,237");
asm(" .byte 134,82,181,205,72,66,89,113,6,195,215,162,237,65,212,103");
asm(" .byte 177,149,209,71,139,49,94,85,232,136,141,173,140,60,90,23");
asm(" .byte 156,122,97,155,160,113,131,107,192,236,196,58,221,57,88,131");
asm(" .byte 30,8,63,116,75,192,168,36,249,112,190,97,170,238,80,100");
asm(" .byte 18,171,198,62,86,167,19,127,72,158,108,208,131,50,208,160");
asm(" .byte 56,15,143,164,59,187,12,197,121,196,180,13,5,140,235,68");
asm(" .byte 157,153,66,122,2,94,168,85,148,179,235,20,39,228,18,184");
asm(" .byte 243,49,14,213,145,108,217,156,97,224,54,133,21,103,119,24");
asm(" .byte 99,71,243,25,59,69,125,88,67,40,162,82,159,240,241,145");
asm(" .byte 124,118,39,151,223,4,180,167,211,30,58,79,155,229,128,225");
asm(" .byte 22,50,29,250,155,230,114,240,16,174,35,243,9,236,141,82");
asm(" .byte 181,192,40,5,253,251,175,234,51,137,206,210,156,128,201,156");
asm(" .byte 61,230,19,40,219,145,59,192,143,160,115,192,12,205,239,51");
asm(" .byte 19,192,15,86,181,45,236,69,63,170,167,126,173,104,139,247");
asm(" .byte 251,226,11,209,14,58,198,20,237,119,35,76,123,201,180,129");
asm(" .byte 238,4,86,7,185,222,70,60,184,31,182,32,74,245,176,252");
asm(" .byte 38,201,137,157,133,112,66,13,166,49,237,56,215,101,200,99");
asm(" .byte 234,83,160,61,5,251,255,80,86,183,97,211,27,195,238,162");
asm(" .byte 110,115,139,148,139,127,129,157,120,253,215,208,127,210,25,119");
asm(" .byte 149,1,151,245,194,9,204,187,6,52,29,141,203,89,106,223");
asm(" .byte 104,150,219,78,58,23,117,86,28,138,168,101,144,51,189,207");
asm(" .byte 36,89,86,116,210,211,218,104,62,198,135,123,61,165,56,203");
asm(" .byte 141,67,135,209,111,8,246,251,208,238,209,44,166,24,107,72");
asm(" .byte 6,152,51,250,65,44,150,92,161,147,156,133,163,113,185,196");
asm(" .byte 9,198,236,252,34,54,246,91,208,81,135,57,19,176,6,117");
asm(" .byte 38,246,11,62,107,9,124,150,45,140,245,148,96,47,50,66");
asm(" .byte 145,156,252,80,164,252,223,151,25,182,163,161,72,246,204,238");
asm(" .byte 200,213,165,152,91,235,129,174,246,24,26,248,54,25,251,157");
asm(" .byte 95,18,136,12,130,126,170,187,3,121,249,191,247,24,246,115");
asm(" .byte 251,70,38,190,12,223,252,10,226,109,228,233,88,43,173,101");
asm(" .byte 18,100,246,54,224,228,146,45,115,239,51,237,88,147,157,33");
asm(" .byte 119,55,133,51,48,222,167,183,132,162,125,118,80,50,145,157");
asm(" .byte 133,231,15,79,151,52,187,23,76,97,94,140,17,132,126,60");
asm(" .byte 113,124,153,113,176,26,242,12,121,113,232,189,17,40,248,173");
asm(" .byte 255,11,251,250,243,249,65,131,233,61,102,51,198,241,179,85");
asm(" .byte 209,4,61,244,51,191,30,122,28,251,17,33,218,118,16,141");
asm(" .byte 206,30,131,202,180,247,183,210,94,99,237,176,53,167,138,65");
asm(" .byte 143,77,235,31,85,153,63,172,56,170,96,119,130,163,144,223");
asm(" .byte 23,50,48,223,65,165,223,108,1,174,162,135,163,7,93,65");
asm(" .byte 204,223,99,120,37,93,119,98,109,7,11,196,94,150,186,201");
asm(" .byte 31,244,155,51,254,119,208,184,0,188,41,236,151,103,186,127");
asm(" .byte 27,52,166,176,3,103,108,236,159,206,144,189,179,177,93,103");
asm(" .byte 82,114,250,205,159,79,10,26,78,208,201,138,130,102,35,198");
asm(" .byte 118,186,95,60,243,50,124,3,141,217,130,49,105,173,44,21");
asm(" .byte 237,217,72,10,82,217,128,121,59,240,94,196,58,31,64,26");
asm(" .byte 102,253,92,246,176,155,167,90,121,57,104,78,83,68,29,100");
asm(" .byte 247,212,122,212,29,195,154,166,41,1,243,40,232,202,87,198");
asm(" .byte 241,27,208,150,175,4,77,187,5,255,219,168,123,20,248,118");
asm(" .byte 224,247,3,255,180,101,252,58,180,157,198,248,231,216,56,254");
asm(" .byte 10,242,119,192,63,199,2,102,115,213,128,233,46,123,216,36");
asm(" .byte 155,140,240,238,20,3,159,234,184,78,133,72,55,34,110,215");
asm(" .byte 54,243,253,191,197,206,218,185,93,10,69,171,103,109,51,166");
asm(" .byte 125,33,202,233,82,22,14,42,125,124,108,240,245,29,166,129");
asm(" .byte 207,217,1,211,11,158,76,132,156,222,13,126,252,25,49,214");
asm(" .byte 240,22,133,17,127,252,40,159,3,127,28,167,151,25,41,144");
asm(" .byte 5,232,184,161,92,31,138,14,130,79,181,180,167,176,91,54");
asm(" .byte 228,251,137,127,220,30,97,255,249,124,61,198,191,130,14,178");
asm(" .byte 161,84,183,151,224,42,209,126,154,96,216,201,213,216,115,146");
asm(" .byte 13,26,115,3,120,255,203,191,137,241,168,238,12,173,65,135");
asm(" .byte 108,187,3,97,178,9,183,145,95,129,61,117,120,73,215,216");
asm(" .byte 188,68,157,219,188,49,94,214,223,26,209,160,163,31,197,98");
asm(" .byte 123,225,35,129,7,251,31,183,111,240,181,26,236,4,217,18");
asm(" .byte 225,3,95,30,73,224,125,129,83,245,245,56,142,233,88,163");
asm(" .byte 254,210,72,2,167,173,203,72,6,111,249,90,192,179,111,144");
asm(" .byte 253,156,179,207,100,250,155,60,62,200,131,93,156,138,250,203");
asm(" .byte 89,85,58,194,214,228,253,224,111,57,35,153,174,74,157,65");
asm(" .byte 126,85,127,219,220,4,60,216,142,204,93,36,247,176,189,211");
asm(" .byte 164,111,34,24,121,26,194,243,76,216,221,49,31,240,176,167");
asm(" .byte 153,27,36,30,194,244,100,85,226,77,180,192,240,179,201,118");
asm(" .byte 9,147,239,86,38,130,79,204,111,20,67,14,18,177,7,180");
asm(" .byte 190,31,115,25,88,25,121,46,22,155,71,54,66,249,67,143");
asm(" .byte 113,14,122,212,141,253,52,144,190,128,131,84,177,248,167,17");
asm(" .byte 211,255,20,233,125,169,7,11,254,184,204,120,16,125,83,176");
asm(" .byte 159,201,110,178,189,224,47,198,79,194,248,187,81,175,92,223");
asm(" .byte 99,156,199,60,73,176,155,164,231,61,127,19,182,104,30,241");
asm(" .byte 7,188,155,86,220,99,190,3,250,167,35,47,187,25,250,15");
asm(" .byte 59,148,4,90,201,86,64,22,34,52,30,197,56,27,105,44");
asm(" .byte 119,157,1,155,153,89,129,126,52,70,154,254,186,57,83,206");
asm(" .byte 177,199,50,135,143,230,208,73,238,64,7,234,252,164,203,13");
asm(" .byte 129,48,237,207,70,57,127,38,173,215,73,190,35,20,137,125");
asm(" .byte 46,100,141,224,50,41,107,9,82,214,200,38,85,192,150,95");
asm(" .byte 109,145,181,101,156,22,159,129,242,108,194,69,254,38,233,23");
asm(" .byte 236,56,184,20,74,95,240,71,97,131,114,96,95,243,145,156");
asm(" .byte 56,127,180,34,223,245,138,224,35,233,94,39,143,31,41,110");
asm(" .byte 237,49,85,148,7,181,67,17,133,203,23,236,144,88,119,52");
asm(" .byte 141,207,89,101,192,134,190,64,126,16,184,134,83,202,21,98");
asm(" .byte 170,76,167,212,97,135,83,216,253,108,217,70,113,85,188,158");
asm(" .byte 98,227,55,63,23,245,170,172,127,10,120,88,223,216,181,132");
asm(" .byte 223,208,107,198,245,236,114,94,222,198,253,17,249,195,236,58");
asm(" .byte 177,199,97,216,55,245,15,208,101,216,36,242,17,42,244,249");
asm(" .byte 223,57,207,170,12,200,211,171,180,143,7,230,11,155,165,147");
asm(" .byte 46,195,126,181,96,252,3,243,251,225,63,106,105,222,228,4");
asm(" .byte 228,205,152,127,5,234,251,27,225,215,244,254,81,155,182,45");
asm(" .byte 156,56,177,134,226,173,199,209,255,71,9,122,240,199,223,102");
asm(" .byte 236,201,205,144,37,7,232,182,57,187,194,147,64,215,239,48");
asm(" .byte 87,2,217,255,155,122,12,21,125,28,85,129,176,82,178,210");
asm(" .byte 76,208,130,163,164,83,203,49,238,146,84,178,105,11,224,147");
asm(" .byte 107,32,207,193,29,58,236,3,209,148,139,24,74,101,125,163");
asm(" .byte 185,40,47,154,228,55,74,32,151,54,248,145,52,242,213,206");
asm(" .byte 55,70,146,221,213,6,98,198,244,44,138,165,220,161,0,237");
asm(" .byte 35,142,195,201,19,80,190,220,217,165,34,208,77,166,56,107");
asm(" .byte 2,143,115,222,48,147,1,39,59,67,233,118,206,75,54,102");
asm(" .byte 99,107,200,190,100,170,220,239,211,152,71,70,40,86,85,17");
asm(" .byte 107,195,239,205,59,192,109,109,57,167,11,251,238,194,56,243");
asm(" .byte 20,103,45,31,7,242,109,99,206,195,14,196,109,138,141,199");
asm(" .byte 212,213,81,192,169,224,71,186,162,31,206,160,56,136,252,165");
asm(" .byte 170,53,133,27,184,15,236,26,37,189,31,4,95,137,119,138");
asm(" .byte 133,119,131,72,176,11,79,86,56,47,229,221,14,240,142,227");
asm(" .byte 235,219,194,192,49,137,135,113,254,17,239,208,215,161,254,97");
asm(" .byte 220,103,62,139,51,253,80,131,194,190,192,89,252,224,26,133");
asm(" .byte 157,91,76,114,122,194,100,176,155,5,116,86,8,145,191,102");
asm(" .byte 153,21,176,231,195,82,174,192,135,228,18,148,41,230,61,34");
asm(" .byte 235,206,237,120,113,26,197,194,223,65,89,123,8,250,135,56");
asm(" .byte 153,98,225,117,40,103,96,79,50,176,31,106,67,223,232,162");
asm(" .byte 101,219,141,18,119,208,176,185,131,97,230,223,25,182,249,119");
asm(" .byte 26,215,83,140,171,255,222,156,0,185,164,120,92,233,189,207");
asm(" .byte 192,153,96,175,35,110,135,123,199,237,112,150,180,195,137,82");
asm(" .byte 222,255,69,206,15,157,73,78,242,111,11,79,240,175,6,31");
asm(" .byte 32,127,13,123,194,160,105,73,226,196,85,68,219,176,93,107");
asm(" .byte 9,39,57,239,11,79,64,191,155,209,199,15,29,247,235,116");
asm(" .byte 174,11,134,109,206,221,163,138,19,180,56,187,71,147,117,161");
asm(" .byte 243,27,164,173,188,3,184,168,3,223,95,26,21,254,161,194");
asm(" .byte 196,120,103,236,236,155,145,19,216,239,4,172,33,113,226,106");
asm(" .byte 154,227,9,194,167,254,10,171,48,242,209,127,57,250,78,167");
asm(" .byte 179,167,70,113,157,31,198,162,123,148,250,238,129,93,216,247");
asm(" .byte 138,224,123,124,15,6,81,254,41,210,185,50,193,127,245,247");
asm(" .byte 203,12,226,191,31,62,149,244,21,50,27,185,23,235,170,135");
asm(" .byte 191,252,228,51,242,155,85,198,38,148,79,99,31,214,74,30");
asm(" .byte 220,197,253,201,91,102,35,250,145,221,247,202,188,19,249,243");
asm(" .byte 217,3,142,22,146,91,146,39,138,99,72,95,145,211,89,236");
asm(" .byte 9,232,146,58,161,199,200,193,56,148,147,79,126,238,51,97");
asm(" .byte 139,214,192,22,113,27,240,141,80,244,143,159,141,251,232,164");
asm(" .byte 207,133,221,172,227,49,134,144,177,73,160,81,189,175,199,88");
asm(" .byte 70,126,23,49,186,189,129,159,111,175,163,92,209,79,114,31");
asm(" .byte 12,255,187,55,129,243,24,251,170,201,125,117,210,190,86,243");
asm(" .byte 125,85,156,164,211,171,34,164,95,9,212,198,106,40,118,137");
asm(" .byte 104,88,31,206,43,252,204,74,182,18,58,102,242,49,220,151");
asm(" .byte 250,104,224,236,101,250,239,204,111,99,46,242,155,100,227,239");
asm(" .byte 192,122,14,16,237,90,149,145,200,125,235,118,131,116,140,214");
asm(" .byte 58,136,125,26,65,155,122,35,214,237,15,132,225,147,199,168");
asm(" .byte 207,124,244,233,165,62,222,42,227,10,206,87,146,129,87,184");
asm(" .byte 76,80,108,161,242,171,96,208,193,202,77,58,3,80,220,207");
asm(" .byte 233,102,236,126,156,47,77,162,247,35,162,215,187,3,190,247");
asm(" .byte 118,65,163,254,87,83,19,119,1,201,9,78,210,171,80,228");
asm(" .byte 201,207,4,207,249,249,17,113,28,241,237,105,200,149,31,113");
asm(" .byte 173,250,123,248,228,174,23,207,60,60,204,216,48,100,227,121");
asm(" .byte 164,95,33,157,64,250,4,137,206,8,249,36,31,228,51,157");
asm(" .byte 226,28,64,113,59,221,119,252,244,51,225,247,90,176,79,249");
asm(" .byte 199,197,25,32,246,10,221,78,132,220,231,112,182,205,157,51");
asm(" .byte 96,222,249,71,17,179,229,35,222,35,191,119,103,97,40,234");
asm(" .byte 161,53,55,84,25,55,43,236,212,229,136,219,75,145,39,193");
asm(" .byte 15,37,84,117,135,19,244,149,176,53,43,161,51,11,13,181");
asm(" .byte 56,0,250,107,163,197,10,203,76,164,58,180,81,157,13,103");
asm(" .byte 21,37,169,51,170,148,4,84,244,83,243,209,174,72,219,139");
asm(" .byte 88,38,108,135,93,252,12,235,179,57,87,194,191,173,52,149");
asm(" .byte 220,238,136,10,88,193,121,70,161,60,31,101,236,235,12,244");
asm(" .byte 187,92,97,99,41,250,187,230,36,239,187,102,162,47,16,153");
asm(" .byte 12,216,182,121,71,100,10,229,91,118,68,18,189,225,200,154");
asm(" .byte 224,0,167,61,69,223,99,46,158,188,211,32,31,25,44,222");
asm(" .byte 105,46,199,58,124,197,123,76,37,116,191,97,243,94,139,179");
asm(" .byte 237,237,81,255,254,157,38,142,7,201,100,55,229,125,198,147");
asm(" .byte 173,192,225,176,30,80,91,139,119,58,124,185,123,204,148,140");
asm(" .byte 1,243,135,179,176,63,246,1,243,14,140,67,54,78,189,235");
asm(" .byte 21,236,245,229,228,107,147,207,216,16,107,167,15,152,190,220");
asm(" .byte 94,83,241,162,94,83,141,13,185,126,156,35,252,70,53,240");
asm(" .byte 127,14,124,55,112,201,175,28,4,238,48,225,22,247,242,118");
asm(" .byte 55,75,167,152,201,6,153,78,95,4,220,3,132,139,49,136");
asm(" .byte 62,200,123,242,174,217,3,230,163,192,255,249,164,128,65,178");
asm(" .byte 230,43,238,51,75,129,247,67,224,77,243,246,24,117,92,254");
asm(" .byte 223,54,21,231,219,130,119,56,43,46,152,41,238,138,208,246");
asm(" .byte 2,249,222,43,0,47,130,236,23,144,239,197,218,202,1,39");
asm(" .byte 33,206,156,128,126,249,160,71,205,13,240,126,59,65,19,245");
asm(" .byte 37,126,147,77,223,137,53,228,242,185,253,70,22,230,220,134");
asm(" .byte 57,167,162,79,6,250,94,206,222,230,251,123,25,202,42,206");
asm(" .byte 152,44,37,16,185,111,2,59,123,161,8,231,196,44,232,6");
asm(" .byte 228,253,123,183,224,124,199,2,225,199,18,217,217,163,168,223");
asm(" .byte 149,75,231,136,154,200,46,123,96,234,52,140,113,5,245,197");
asm(" .byte 220,153,128,109,216,239,43,41,199,126,239,1,110,111,6,221");
asm(" .byte 219,84,71,122,115,3,83,113,6,31,35,61,153,74,177,179");
asm(" .byte 156,223,58,247,229,148,151,6,34,142,137,252,238,166,15,124");
asm(" .byte 222,155,138,58,167,196,75,147,120,233,95,194,131,158,6,242");
asm(" .byte 115,6,204,254,226,110,83,165,185,53,129,111,243,238,8,59");
asm(" .byte 160,163,116,71,114,73,61,104,120,28,54,233,2,249,47,244");
asm(" .byte 123,90,235,227,124,190,135,98,4,231,187,102,138,246,174,89");
asm(" .byte 114,184,143,239,195,100,148,207,3,143,228,120,10,96,46,199");
asm(" .byte 206,252,176,77,87,194,36,95,69,92,238,96,99,232,252,16");
asm(" .byte 62,105,218,157,39,205,201,25,93,145,4,192,147,167,117,69");
asm(" .byte 18,41,159,222,21,177,133,143,143,82,236,175,56,219,194,217");
asm(" .byte 35,116,190,56,106,58,194,71,121,92,149,125,6,231,254,233");
asm(" .byte 176,139,40,39,162,254,61,186,83,204,62,28,117,107,135,163");
asm(" .byte 41,90,119,36,39,35,20,73,209,94,71,204,246,186,153,51");
asm(" .byte 77,220,23,76,6,60,5,117,118,167,223,160,123,57,199,16");
asm(" .byte 230,214,78,154,10,187,203,76,1,221,9,40,167,128,230,68");
asm(" .byte 202,137,102,205,31,206,177,83,156,138,121,217,81,51,39,73");
asm(" .byte 140,147,0,56,17,117,138,94,107,252,57,22,187,110,4,54");
asm(" .byte 166,184,168,143,235,206,12,138,149,112,78,156,134,121,174,64");
asm(" .byte 154,128,24,60,19,249,149,136,215,167,35,93,133,52,3,41");
asm(" .byte 11,137,108,137,205,249,198,232,85,236,93,115,58,248,71,107");
asm(" .byte 101,160,99,6,202,78,186,139,3,45,89,128,25,167,165,43");
asm(" .byte 76,107,154,142,177,174,178,172,105,6,224,44,212,157,197,250");
asm(" .byte 167,53,132,162,171,221,194,6,220,119,139,184,115,112,130,199");
asm(" .byte 56,27,34,94,12,69,248,89,27,229,107,62,165,24,184,42");
asm(" .byte 82,169,15,76,133,254,165,63,163,5,83,225,27,199,200,63");
asm(" .byte 251,247,211,58,190,21,245,249,250,166,142,125,17,203,140,224");
asm(" .byte 156,251,247,241,51,145,188,155,155,68,54,24,103,222,169,200");
asm(" .byte 175,66,202,67,42,65,74,249,34,22,187,6,74,231,198,255");
asm(" .byte 100,87,61,210,166,254,8,250,227,92,1,121,128,31,9,218");
asm(" .byte 131,102,10,206,236,54,248,139,135,16,47,127,62,25,177,29");
asm(" .byte 218,47,32,191,48,99,192,140,130,182,243,128,207,3,14,3");
asm(" .byte 62,3,248,61,192,239,2,62,13,120,4,240,41,192,195,128");
asm(" .byte 67,87,245,155,111,3,62,9,248,9,192,39,1,31,3,188");
asm(" .byte 31,240,9,192,117,116,183,40,206,137,201,207,163,254,121,244");
asm(" .byte 125,13,245,207,2,126,22,240,48,224,65,192,131,128,95,2");
asm(" .byte 124,0,240,1,192,100,163,123,105,28,192,191,66,125,136,230");
asm(" .byte 2,124,16,240,146,229,3,230,30,148,247,160,252,60,202,253");
asm(" .byte 128,55,3,254,87,203,124,100,119,252,168,247,163,254,167,168");
asm(" .byte 247,204,192,89,140,238,36,249,94,100,155,173,55,35,230,176");
asm(" .byte 236,141,157,238,237,0,211,254,252,234,19,30,87,68,43,209");
asm(" .byte 119,245,148,1,179,100,85,143,57,242,9,237,87,40,218,12");
asm(" .byte 62,109,117,192,30,20,137,121,156,164,71,90,63,244,183,41");
asm(" .byte 172,21,82,172,95,75,103,223,189,211,102,12,240,123,156,223");
asm(" .byte 201,126,171,208,175,4,125,138,215,6,77,197,21,136,150,20");
asm(" .byte 5,205,247,64,131,134,242,51,13,193,136,111,25,120,239,18");
asm(" .byte 99,46,36,255,158,214,15,30,84,71,179,232,28,141,60,87");
asm(" .byte 198,231,115,32,219,159,167,6,205,236,201,65,238,91,30,193");
asm(" .byte 248,41,24,215,165,151,27,152,155,206,251,247,187,176,142,20");
asm(" .byte 140,127,90,142,247,233,45,161,103,232,190,106,141,228,205,167");
asm(" .byte 179,66,71,118,219,108,67,155,231,15,188,101,7,222,133,82");
asm(" .byte 196,93,57,125,102,138,19,54,15,50,110,207,9,240,51,33");
asm(" .byte 67,142,88,99,236,115,57,206,101,160,97,4,176,14,56,93");
asm(" .byte 198,110,241,54,85,198,36,116,183,5,127,54,70,60,177,131");
asm(" .byte 46,27,236,45,124,212,94,226,69,202,85,3,230,110,208,139");
asm(" .byte 152,55,154,6,126,232,146,78,218,19,186,235,242,130,142,11");
asm(" .byte 217,220,70,83,76,159,73,62,231,244,116,200,27,234,6,208");
asm(" .byte 207,129,62,149,192,25,70,25,227,70,213,191,12,60,66,177");
asm(" .byte 157,182,159,177,247,144,166,253,8,105,63,61,200,12,69,135");
asm(" .byte 179,3,38,249,170,233,149,144,37,37,157,251,190,254,148,1");
asm(" .byte 115,41,100,129,104,35,90,111,155,213,109,148,98,222,99,160");
asm(" .byte 249,104,118,144,235,62,157,203,226,124,155,11,158,13,73,120");
asm(" .byte 50,127,14,18,138,214,129,30,55,100,100,1,232,169,156,46");
asm(" .byte 246,248,142,79,248,125,85,148,244,131,238,248,154,49,206,81");
asm(" .byte 140,137,152,49,58,140,60,87,242,131,100,228,211,41,181,81");
asm(" .byte 13,252,160,123,176,211,196,43,57,70,37,198,192,25,34,122");
asm(" .byte 12,99,92,24,231,119,230,251,40,103,208,93,11,252,81,213");
asm(" .byte 245,180,150,84,138,19,146,83,156,187,65,243,110,238,163,192");
asm(" .byte 239,121,180,230,99,88,51,61,39,25,74,237,51,115,175,167");
asm(" .byte 152,186,154,63,35,177,226,66,175,246,218,36,127,216,223,15");
asm(" .byte 96,237,143,152,137,20,111,128,151,79,211,60,224,225,30,36");
asm(" .byte 247,143,25,107,70,234,71,26,70,58,142,116,142,224,236,71");
asm(" .byte 204,225,121,3,124,143,254,25,99,251,224,83,221,44,141,63");
asm(" .byte 163,194,184,233,33,140,177,167,248,17,243,209,220,71,204,208");
asm(" .byte 173,66,31,136,94,127,110,63,239,243,15,34,222,25,59,41");
asm(" .byte 215,248,176,148,27,186,255,165,251,150,147,224,73,235,223,16");
asm(" .byte 131,128,199,231,51,197,61,225,232,133,216,217,239,163,207,18");
asm(" .byte 216,214,190,132,132,240,106,212,199,159,121,208,185,107,209,39");
asm(" .byte 226,188,81,76,247,76,242,222,18,202,242,42,141,185,32,43");
asm(" .byte 104,28,254,76,158,19,21,236,9,221,231,58,133,238,60,137");
asm(" .byte 113,73,239,215,200,187,130,56,207,19,26,130,225,161,220,223");
asm(" .byte 153,197,160,151,226,243,185,200,97,187,109,7,10,130,14,156");
asm(" .byte 97,29,36,75,5,162,141,159,193,105,12,180,153,103,174,195");
asm(" .byte 122,245,224,40,217,174,201,159,138,123,136,71,151,194,158,21");
asm(" .byte 136,181,211,57,221,15,90,87,163,189,122,114,143,49,13,235");
asm(" .byte 160,123,170,51,160,131,202,4,63,246,137,144,161,95,220,28");
asm(" .byte 48,146,164,60,166,207,223,110,80,251,123,87,136,51,160,77");
asm(" .byte 15,134,9,247,20,245,187,185,135,227,37,106,1,227,248,21");
asm(" .byte 66,150,78,162,126,222,167,227,99,238,146,99,126,239,22,41");
asm(" .byte 151,160,55,139,246,26,123,244,121,201,0,223,63,90,211,129");
asm(" .byte 2,161,123,14,185,151,20,175,29,40,120,196,124,167,100,64");
asm(" .byte 214,119,171,51,185,236,245,65,158,232,121,202,218,48,215,85");
asm(" .byte 45,244,120,14,98,215,28,126,110,166,187,152,29,230,113,200");
asm(" .byte 21,233,19,217,164,227,174,29,156,239,101,160,99,23,198,58");
asm(" .byte 174,4,200,23,70,253,37,244,28,148,141,221,132,250,227,202");
asm(" .byte 14,51,99,194,128,217,200,105,125,68,210,186,195,196,217,38");
asm(" .byte 122,12,249,68,73,111,165,164,151,228,205,241,53,180,128,199");
asm(" .byte 47,36,72,93,208,128,155,162,245,153,239,64,31,232,254,35");
asm(" .byte 15,122,114,58,181,143,235,5,104,61,245,61,232,255,59,192");
asm(" .byte 243,44,239,49,72,142,119,194,94,224,252,109,84,204,125,221");
asm(" .byte 204,37,155,4,121,115,187,154,77,5,117,238,107,122,204,133");
asm(" .byte 235,94,55,23,33,93,49,159,238,47,17,131,104,111,142,54");
asm(" .byte 79,131,239,1,191,149,134,237,56,203,173,138,50,156,223,113");
asm(" .byte 206,202,4,79,34,215,241,243,249,225,145,100,158,31,53,171");
asm(" .byte 225,131,118,221,32,158,39,145,47,114,43,100,111,27,195,179");
asm(" .byte 46,136,179,171,55,113,192,172,193,154,90,139,197,115,93,241");
asm(" .byte 76,84,216,86,138,183,146,165,188,249,100,59,236,203,51,144");
asm(" .byte 193,39,109,150,58,27,221,27,98,190,214,98,191,121,236,90");
asm(" .byte 113,174,30,70,30,151,5,232,200,216,133,106,194,93,25,157");
asm(" .byte 113,167,95,237,174,247,167,217,27,222,130,77,95,21,189,14");
asm(" .byte 182,97,54,114,236,241,253,117,71,123,205,254,27,168,255,154");
asm(" .byte 40,245,221,133,49,118,93,78,119,64,161,84,26,231,38,146");
asm(" .byte 61,233,83,93,232,147,164,61,108,50,119,23,226,86,122,166");
asm(" .byte 90,35,236,142,254,48,167,73,199,89,70,209,43,28,189,151");
asm(" .byte 209,51,144,135,177,231,15,155,206,161,135,205,186,186,135,204");
asm(" .byte 210,209,1,78,75,62,198,158,236,12,165,218,88,207,159,136");
asm(" .byte 183,51,81,102,67,61,102,54,230,225,207,212,32,243,76,59");
asm(" .byte 10,221,10,24,206,107,201,174,8,29,58,115,141,208,33,6");
asm(" .byte 252,203,44,122,244,91,234,7,156,160,228,75,50,247,211,189");
asm(" .byte 23,109,48,206,54,123,253,117,126,51,136,52,124,153,56,99");
asm(" .byte 156,253,152,206,168,161,232,119,33,131,35,11,33,103,57,66");
asm(" .byte 111,97,19,78,249,165,239,132,223,75,206,208,47,141,235,30");
asm(" .byte 202,10,242,56,154,158,187,12,98,12,55,247,123,85,145,186");
asm(" .byte 132,30,122,6,151,252,231,143,105,127,123,205,157,200,27,46");
asm(" .byte 19,207,29,72,79,199,80,174,194,92,155,49,23,197,27,223");
asm(" .byte 4,237,75,96,155,252,88,175,187,38,200,215,53,4,28,125");
asm(" .byte 238,128,89,58,183,199,108,146,186,254,205,25,61,124,126,226");
asm(" .byte 125,5,230,164,251,54,186,155,58,250,49,157,215,197,188,116");
asm(" .byte 150,252,45,205,171,247,242,88,125,149,141,101,230,98,174,186");
asm(" .byte 185,251,204,164,149,226,93,1,138,1,161,24,167,220,202,195");
asm(" .byte 220,102,26,128,61,133,226,206,122,40,13,231,230,212,238,240");
asm(" .byte 144,51,21,177,45,51,170,115,232,30,41,45,250,123,133,238");
asm(" .byte 43,202,113,174,239,49,212,124,248,67,208,181,5,243,40,232");
asm(" .byte 79,247,211,39,48,198,144,75,236,59,217,71,178,1,167,93");
asm(" .byte 219,192,223,109,70,235,84,248,255,143,69,60,148,6,90,84");
asm(" .byte 238,107,5,238,17,133,189,224,45,134,110,184,43,12,210,217");
asm(" .byte 33,232,101,134,229,121,202,191,41,242,121,10,19,119,56,148");
asm(" .byte 199,159,163,92,224,119,44,95,125,70,18,191,155,115,232,13");
asm(" .byte 97,58,59,253,88,161,187,33,126,119,6,255,127,28,186,86");
asm(" .byte 27,253,33,206,245,113,57,214,192,247,55,227,107,105,8,69");
asm(" .byte 254,17,248,211,230,236,131,190,190,105,174,198,88,143,0,87");
asm(" .byte 245,210,115,148,219,163,253,128,167,203,231,42,229,24,167,138");
asm(" .byte 238,251,160,55,20,27,245,40,108,222,108,113,223,179,183,12");
asm(" .byte 249,139,104,91,128,124,7,250,204,199,154,23,2,126,27,117");
asm(" .byte 199,97,143,234,0,195,221,38,175,66,190,85,17,250,93,253");
asm(" .byte 84,143,177,246,129,30,35,37,116,191,97,247,239,132,157,96");
asm(" .byte 99,42,104,186,7,188,215,36,61,36,151,157,192,167,249,105");
asm(" .byte 78,47,230,116,201,57,191,133,188,21,109,26,206,238,101,144");
asm(" .byte 169,6,216,54,146,37,178,59,54,77,69,57,96,174,206,128");
asm(" .byte 15,197,90,79,227,252,253,19,204,121,30,231,201,186,140,221");
asm(" .byte 220,54,222,129,121,228,88,153,205,197,187,249,254,172,70,93");
asm(" .byte 191,132,87,2,62,175,34,54,199,217,252,29,140,17,194,24");
asm(" .byte 23,210,6,204,183,0,47,7,173,239,160,237,29,148,79,160");
asm(" .byte 28,164,187,9,192,31,199,98,124,13,139,228,26,84,238,239");
asm(" .byte 223,52,93,206,106,156,45,67,233,237,240,209,183,160,111,43");
asm(" .byte 206,222,39,93,226,28,167,122,67,54,248,202,228,27,21,225");
asm(" .byte 87,170,32,107,170,190,216,72,146,241,74,5,232,158,249,20");
asm(" .byte 99,58,82,41,82,5,82,37,82,29,210,179,72,199,145,216");
asm(" .byte 79,25,179,207,22,118,144,33,167,49,230,146,13,196,60,159");
asm(" .byte 129,166,243,200,33,35,99,231,138,4,141,116,215,120,166,136");
asm(" .byte 199,29,145,98,199,62,142,111,124,36,238,234,232,89,195,49");
asm(" .byte 156,169,235,196,61,2,197,209,233,201,116,119,130,241,62,157");
asm(" .byte 194,125,207,171,252,25,181,170,134,203,164,29,162,181,23,59");
asm(" .byte 122,204,247,49,198,52,254,238,68,40,90,226,216,199,199,63");
asm(" .byte 134,186,45,24,111,181,61,96,166,162,205,219,133,177,229,253");
asm(" .byte 15,221,25,19,110,3,104,41,149,248,68,203,175,208,167,217");
asm(" .byte 190,211,116,200,177,220,178,237,223,80,175,99,172,53,24,171");
asm(" .byte 42,97,159,41,159,195,36,95,140,15,82,233,61,35,30,15");
asm(" .byte 38,243,251,116,254,14,14,75,222,64,242,238,221,97,36,176");
asm(" .byte 219,35,63,138,197,94,120,79,23,49,215,73,232,223,31,48");
asm(" .byte 7,173,35,156,42,108,236,227,31,145,110,136,56,141,206,139");
asm(" .byte 199,80,95,130,189,222,247,145,136,127,175,130,205,141,175,187");
asm(" .byte 29,227,190,198,132,220,254,154,137,61,167,247,153,92,90,141");
asm(" .byte 241,18,99,183,82,252,126,4,252,218,130,249,118,30,162,117");
asm(" .byte 247,113,89,37,186,236,90,119,26,127,151,9,227,120,233,46");
asm(" .byte 88,59,61,10,190,218,166,232,161,116,154,171,65,129,92,75");
asm(" .byte 157,109,196,158,149,232,116,118,18,50,131,45,207,204,1,29");
asm(" .byte 7,232,92,128,250,86,185,87,144,125,181,53,55,224,104,45");
asm(" .byte 166,179,3,205,17,178,61,14,220,71,128,71,231,233,91,135");
asm(" .byte 197,29,56,221,133,111,0,188,17,233,175,72,237,72,119,33");
asm(" .byte 237,64,250,7,164,255,137,116,0,233,151,72,135,145,78,34");
asm(" .byte 253,30,233,29,44,242,47,200,207,35,37,2,158,128,180,4");
asm(" .byte 112,22,242,98,164,74,164,124,204,115,59,242,89,155,24,187");
asm(" .byte 243,215,226,185,156,7,121,59,210,22,164,32,210,118,164,239");
asm(" .byte 35,245,33,61,140,20,66,122,26,233,223,144,142,252,154,238");
asm(" .byte 75,235,82,153,82,151,234,198,216,253,40,31,71,90,8,248");
asm(" .byte 20,181,169,117,169,195,200,255,132,244,87,234,187,93,97,159");
asm(" .byte 35,183,189,202,216,229,72,87,34,205,66,186,5,105,13,146");
asm(" .byte 15,233,251,72,3,72,207,34,189,130,244,27,164,253,216,160");
asm(" .byte 68,140,249,230,171,52,223,223,85,140,34,127,31,233,35,164");
asm(" .byte 207,94,21,252,81,177,185,241,103,53,244,156,102,10,202,151");
asm(" .byte 33,205,64,202,67,122,8,73,71,186,17,233,102,164,197,72");
asm(" .byte 85,72,116,55,65,207,245,136,231,171,94,19,119,24,118,246");
asm(" .byte 130,73,231,115,122,110,157,136,253,173,130,143,178,179,39,46");
asm(" .byte 62,179,190,145,252,8,100,84,213,119,24,54,126,47,89,123");
asm(" .byte 130,158,59,225,220,248,204,251,144,89,170,211,193,130,233,121");
asm(" .byte 251,204,6,127,150,201,26,222,48,109,222,80,100,228,177,61");
asm(" .byte 230,72,227,78,115,109,92,7,24,233,64,85,244,140,178,211");
asm(" .byte 65,122,208,111,99,103,207,56,179,204,171,237,200,149,157,38");
asm(" .byte 206,19,134,19,99,145,205,253,11,98,184,3,44,11,114,37");
asm(" .byte 226,135,20,189,119,234,231,46,255,84,58,31,196,227,51,186");
asm(" .byte 211,160,184,41,149,159,133,107,17,227,209,157,173,223,65,237");
asm(" .byte 14,249,108,156,206,101,231,245,44,200,97,128,98,129,177,88");
asm(" .byte 40,217,72,102,171,248,253,102,67,106,22,191,19,162,216,99");
asm(" .byte 244,195,216,217,101,164,139,85,219,12,242,115,139,190,136,205");
asm(" .byte 35,95,77,243,188,142,181,149,84,103,153,195,76,248,105,108");
asm(" .byte 233,255,213,79,15,177,113,63,77,239,63,144,159,222,253,225");
asm(" .byte 184,159,126,30,99,104,69,194,247,138,179,246,54,227,61,208");
asm(" .byte 243,228,135,194,63,95,7,90,14,184,178,120,252,172,229,92");
asm(" .byte 234,139,127,28,127,183,65,62,79,113,52,92,234,139,19,254");
asm(" .byte 19,95,28,247,191,14,125,29,247,201,125,244,206,137,10,62");
asm(" .byte 192,166,252,241,67,113,78,123,128,236,22,228,193,198,182,115");
asm(" .byte 29,79,2,15,255,252,33,217,24,126,174,75,38,156,109,232");
asm(" .byte 55,13,182,111,79,90,150,249,178,236,231,39,158,22,101,153");
asm(" .byte 37,57,136,247,179,15,147,255,26,155,230,217,103,62,15,28");
asm(" .byte 122,134,114,80,226,229,145,61,202,35,223,253,134,57,71,174");
asm(" .byte 127,143,236,71,119,151,37,88,239,70,105,251,200,6,157,71");
asm(" .byte 255,173,232,171,73,127,95,44,99,242,185,200,155,100,76,158");
asm(" .byte 49,59,203,60,157,147,101,78,75,207,66,140,188,123,148,98");
asm(" .byte 155,247,48,102,46,202,131,56,87,93,0,124,142,137,247,88");
asm(" .byte 96,143,142,172,164,61,45,162,179,115,122,20,49,127,102,33");
asm(" .byte 198,175,146,207,122,234,208,231,33,73,235,109,152,167,184,9");
asm(" .byte 180,21,253,128,227,46,6,174,91,167,251,163,237,166,134,245");
asm(" .byte 231,78,221,103,254,43,236,157,182,105,159,185,211,150,197,159");
asm(" .byte 129,238,250,240,226,243,246,100,106,159,132,88,11,58,52,70");
asm(" .byte 109,79,0,39,248,161,120,94,71,62,230,89,148,253,18,159");
asm(" .byte 222,223,104,200,253,1,183,195,227,177,194,118,110,251,223,1");
asm(" .byte 77,185,240,99,243,129,123,53,198,26,145,115,221,37,251,146");
asm(" .byte 62,53,23,255,128,203,211,149,128,175,34,25,177,11,156,13");
asm(" .byte 18,135,238,207,51,100,93,211,135,226,185,14,197,211,185,118");
asm(" .byte 177,255,119,74,60,122,238,144,164,255,206,108,230,62,128,101");
asm(" .byte 22,23,103,113,159,84,156,35,116,168,74,226,215,88,214,25");
asm(" .byte 231,197,179,31,142,235,208,234,216,184,14,65,71,79,109,201");
asm(" .byte 205,50,243,171,119,243,243,198,36,210,33,172,243,165,28,122");
asm(" .byte 238,229,55,212,108,127,120,72,75,51,179,170,28,70,205,74");
asm(" .byte 200,115,213,84,195,33,117,136,116,71,65,124,94,199,239,25");
asm(" .byte 224,119,9,150,227,110,193,56,39,155,177,239,202,15,248,222");
asm(" .byte 102,255,23,116,115,154,101,92,58,67,84,34,167,51,101,10");
asm(" .byte 108,84,35,206,162,116,166,160,115,40,215,221,124,113,47,201");
asm(" .byte 220,61,230,59,231,133,14,16,79,50,116,49,255,234,188,44");
asm(" .byte 199,211,250,118,254,252,158,206,138,83,88,127,210,125,237,219");
asm(" .byte 213,41,108,123,82,221,53,251,184,239,155,2,154,54,207,201");
asm(" .byte 50,237,210,71,78,148,107,63,146,38,214,174,164,250,5,141");
asm(" .byte 186,205,168,158,77,52,166,115,158,210,251,12,117,56,247,238");
asm(" .byte 251,88,156,27,168,239,53,232,123,212,3,89,103,98,189,57");
asm(" .byte 255,133,245,94,33,215,75,62,251,138,130,30,190,110,210,107");
asm(" .byte 90,59,245,91,130,242,93,231,197,243,93,232,210,171,147,184");
asm(" .byte 125,9,69,114,177,151,231,216,118,115,2,202,241,57,232,174");
asm(" .byte 48,69,207,49,43,47,203,50,55,158,23,251,79,239,49,18");
asm(" .byte 254,142,243,227,116,210,153,173,117,81,214,197,247,210,210,254");
asm(" .byte 11,116,78,248,26,155,185,240,252,69,25,123,149,198,218,195");
asm(" .byte 105,10,154,195,139,200,207,208,216,171,162,127,255,105,44,249");
asm(" .byte 36,232,57,153,144,101,126,235,188,176,159,167,128,247,136,229");
asm(" .byte 206,158,222,183,249,85,1,246,145,158,157,195,94,82,204,76");
asm(" .byte 241,219,181,192,223,131,184,212,175,5,248,115,110,187,243,113");
asm(" .byte 126,103,134,216,52,115,11,226,107,255,108,122,55,181,203,220");
asm(" .byte 130,124,63,242,205,200,159,64,78,113,24,189,235,21,251,64");
asm(" .byte 240,141,222,221,250,13,191,139,122,134,222,211,65,172,245,134");
asm(" .byte 73,119,84,224,125,38,197,150,173,242,217,251,61,200,115,103");
asm(" .byte 15,56,156,90,31,189,215,108,115,106,1,186,207,26,163,59");
asm(" .byte 14,122,214,155,64,207,34,233,125,93,122,55,78,249,193,232");
asm(" .byte 213,160,47,9,115,158,203,161,119,188,186,71,237,58,197,155");
asm(" .byte 1,131,230,80,164,127,30,165,251,14,215,246,112,124,140,119");
asm(" .byte 225,147,41,254,223,56,11,113,36,226,195,87,64,227,209,195");
asm(" .byte 3,230,1,196,178,195,168,31,206,161,119,250,2,252,153,98");
asm(" .byte 138,222,55,58,19,115,60,0,126,61,171,10,158,252,66,174");
asm(" .byte 169,16,123,182,31,248,83,160,251,216,100,254,78,224,100,224");
asm(" .byte 246,22,209,123,2,66,119,232,61,162,73,88,103,50,171,73");
asm(" .byte 103,252,61,249,67,14,187,182,143,223,177,8,26,107,225,71");
asm(" .byte 216,60,187,147,226,207,19,92,31,254,10,122,171,48,238,18");
asm(" .byte 208,81,137,57,147,48,102,27,230,167,88,150,222,237,163,56");
asm(" .byte 196,252,64,248,202,11,174,129,151,192,139,35,117,185,129,151");
asm(" .byte 232,78,113,51,183,247,251,248,249,177,16,107,247,81,108,66");
asm(" .byte 247,15,160,155,250,189,253,1,127,174,19,109,3,237,7,203");
asm(" .byte 192,55,196,228,211,244,160,169,194,102,208,189,193,179,213,71");
asm(" .byte 35,83,88,223,153,226,161,160,145,27,166,247,63,119,157,153");
asm(" .byte 194,30,56,67,231,138,231,92,71,35,106,105,185,241,92,209");
asm(" .byte 209,136,163,52,96,60,55,251,104,228,121,196,186,244,94,201");
asm(" .byte 221,31,8,57,116,43,65,147,158,171,86,103,5,141,31,127");
asm(" .byte 32,108,40,252,70,102,53,127,151,244,224,153,132,63,83,254");
asm(" .byte 212,25,130,173,121,34,123,244,76,158,78,239,135,63,112,198");
asm(" .byte 239,10,154,187,170,131,252,30,155,236,104,138,182,203,248,133");
asm(" .byte 28,127,62,249,23,212,147,220,187,49,119,151,202,206,230,126");
asm(" .byte 36,218,40,14,251,8,245,255,252,33,221,51,132,34,233,51");
asm(" .byte 130,198,119,101,27,189,51,90,40,223,89,59,230,12,26,30");
asm(" .byte 89,79,239,158,206,148,245,139,65,115,189,172,159,6,124,216");
asm(" .byte 142,23,72,142,195,192,255,166,172,167,119,73,167,83,61,248");
asm(" .byte 249,45,208,228,212,233,57,136,176,71,207,50,236,23,234,232");
asm(" .byte 29,164,84,244,123,46,245,100,228,151,249,39,35,39,207,95");
asm(" .byte 244,95,153,75,220,3,102,47,226,126,222,158,118,50,242,140");
asm(" .byte 214,31,57,118,254,162,95,161,247,47,162,207,165,159,140,60");
asm(" .byte 93,120,50,242,178,172,167,159,65,206,134,109,15,124,32,238");
asm(" .byte 105,196,207,234,196,59,104,20,167,86,34,221,129,88,117,29");
asm(" .byte 82,59,210,243,5,140,149,129,160,205,128,119,188,38,222,63");
asm(" .byte 237,71,94,89,171,176,39,100,76,187,149,49,101,234,179,177");
asm(" .byte 216,32,202,23,122,17,191,83,108,204,127,142,81,29,237,34");
asm(" .byte 59,51,164,133,153,179,113,129,162,85,27,94,126,38,27,140");
asm(" .byte 48,189,97,1,253,22,160,29,103,120,149,165,70,232,253,153");
asm(" .byte 244,89,237,145,212,41,237,17,85,115,135,179,180,14,163,91");
asm(" .byte 235,118,210,123,30,234,95,30,164,60,210,53,165,150,126,139");
asm(" .byte 17,238,186,165,92,83,36,172,0,238,154,44,235,111,70,189");
asm(" .byte 132,21,192,93,147,100,253,124,212,75,88,1,220,149,34,235");
asm(" .byte 75,81,47,97,5,112,215,68,89,127,19,234,37,172,0,238");
asm(" .byte 74,150,245,243,80,47,97,5,112,215,4,89,127,35,234,37");
asm(" .byte 172,0,238,74,146,245,37,168,151,176,2,184,43,81,214,223");
asm(" .byte 128,122,9,43,128,187,18,100,253,92,212,75,88,1,220,229");
asm(" .byte 144,245,115,80,47,97,5,112,151,93,214,23,163,94,194,10");
asm(" .byte 224,46,155,172,159,141,122,9,43,128,187,84,89,95,132,122");
asm(" .byte 9,43,128,187,20,89,239,66,189,132,21,192,93,76,214,235");
asm(" .byte 168,151,48,124,133,70,247,163,140,53,44,16,191,137,41,141");
asm(" .byte 140,195,161,217,199,166,208,187,255,126,27,149,83,177,143,89");
asm(" .byte 218,90,254,251,146,217,106,200,130,7,223,162,132,143,168,74");
asm(" .byte 232,25,47,221,115,136,247,142,146,109,240,23,155,98,176,37");
asm(" .byte 238,23,207,236,102,182,176,205,89,22,38,124,85,209,12,146");
asm(" .byte 79,53,53,219,240,107,131,145,46,125,48,213,198,138,82,23");
asm(" .byte 28,59,151,90,142,84,10,153,161,103,210,87,151,118,132,109");
asm(" .byte 90,247,144,226,212,112,20,200,22,185,222,244,244,21,244,124");
asm(" .byte 60,54,171,75,85,156,81,213,225,143,198,236,67,68,127,154");
asm(" .byte 170,164,70,213,169,93,209,88,198,33,156,239,27,211,48,102");
asm(" .byte 228,106,25,3,84,219,122,195,154,222,100,164,166,35,158,119");
asm(" .byte 49,3,98,252,45,230,188,11,116,132,13,213,81,69,241,196");
asm(" .byte 60,197,206,223,195,155,23,98,131,145,70,39,253,38,168,219");
asm(" .byte 248,2,182,182,128,222,97,80,170,163,221,19,193,47,54,100");
asm(" .byte 179,49,63,253,78,40,210,229,28,228,60,84,29,206,40,205");
asm(" .byte 241,33,108,178,243,183,237,145,164,161,74,35,69,171,9,59");
asm(" .byte 83,66,145,238,137,254,16,211,214,62,141,62,143,233,106,224");
asm(" .byte 103,130,95,69,160,51,27,243,106,42,157,233,177,7,105,111");
asm(" .byte 197,98,99,93,56,115,128,7,17,226,1,197,148,68,63,127");
asm(" .byte 38,9,124,216,24,133,250,42,76,167,57,211,1,167,198,236");
asm(" .byte 105,252,125,84,63,232,85,21,255,213,156,183,185,187,49,174");
asm(" .byte 191,8,176,139,214,185,213,30,224,60,162,182,5,249,231,208");
asm(" .byte 63,148,90,94,122,78,129,173,79,245,231,130,247,197,131,216");
asm(" .byte 95,93,81,28,254,116,249,91,40,186,3,26,83,93,78,131");
asm(" .byte 246,151,13,13,210,90,149,219,138,3,252,206,132,244,118,10");
asm(" .byte 255,237,72,47,175,231,239,94,88,202,170,43,85,240,150,121");
asm(" .byte 225,79,34,134,130,254,108,40,68,243,161,58,100,251,54,198");
asm(" .byte 129,140,170,234,156,114,99,58,228,41,73,171,49,186,111,42");
asm(" .byte 15,211,187,194,169,69,129,176,243,112,40,162,65,198,84,101");
asm(" .byte 77,84,87,31,231,252,38,186,52,236,69,170,108,219,197,122");
asm(" .byte 35,46,180,29,152,50,196,121,18,27,10,69,67,215,79,15");
asm(" .byte 63,118,211,116,103,10,228,40,133,205,78,189,153,248,167,117");
asm(" .byte 135,201,70,249,237,88,103,198,96,42,61,231,86,103,3,87");
asm(" .byte 13,68,30,75,11,68,122,93,93,97,85,73,1,191,96,207");
asm(" .byte 156,33,215,227,89,93,174,128,30,114,65,222,35,91,181,195");
asm(" .byte 198,223,68,220,121,191,170,212,94,220,123,177,127,122,84,61");
asm(" .byte 148,98,116,101,132,148,144,125,87,228,49,59,225,215,24,49");
asm(" .byte 189,227,48,246,251,16,112,14,17,207,57,47,81,63,7,62");
asm(" .byte 247,10,214,1,29,97,111,160,237,4,228,193,240,67,118,92");
asm(" .byte 105,98,61,180,230,70,198,101,201,216,165,247,69,226,235,226");
asm(" .byte 114,114,136,25,126,236,61,100,204,6,25,83,17,83,37,167");
asm(" .byte 128,62,198,102,163,77,49,64,55,218,82,109,144,33,209,230");
asm(" .byte 236,138,12,65,150,30,187,41,37,12,94,68,136,23,20,51");
asm(" .byte 169,135,142,3,255,68,250,159,248,111,230,52,172,155,25,23");
asm(" .byte 215,162,232,233,36,95,140,185,210,85,197,149,62,137,63,223");
asm(" .byte 44,226,239,228,146,140,208,123,187,24,135,100,49,19,242,194");
asm(" .byte 101,136,255,166,78,142,179,213,238,191,40,99,4,219,36,252");
asm(" .byte 117,251,50,143,239,203,2,190,47,216,147,136,223,30,138,76");
asm(" .byte 39,29,45,10,69,179,177,39,90,81,32,66,123,66,246,137");
asm(" .byte 246,226,123,218,15,13,245,37,146,201,218,8,217,42,172,61");
asm(" .byte 251,49,192,143,1,62,0,56,27,246,45,27,182,45,95,65");
asm(" .byte 61,224,199,0,31,0,108,43,26,140,228,31,30,76,61,135");
asm(" .byte 245,210,62,238,163,247,32,209,207,186,143,180,135,180,127,91");
asm(" .byte 181,221,255,225,222,61,72,247,169,255,143,251,112,212,178,15");
asm(" .byte 14,201,203,231,232,119,83,224,229,79,197,111,23,233,189,196");
asm(" .byte 49,42,219,36,111,31,225,239,99,134,34,7,232,174,89,238");
asm(" .byte 217,31,129,171,102,31,143,254,144,114,215,137,232,3,200,247");
asm(" .byte 163,221,143,248,118,171,189,134,206,82,243,254,9,244,230,228");
asm(" .byte 215,134,19,233,185,124,49,217,56,232,61,236,163,180,211,105");
asm(" .byte 214,223,59,54,66,22,227,123,75,123,10,60,232,91,216,102");
asm(" .byte 197,81,97,187,233,57,210,183,254,18,59,171,34,134,224,239");
asm(" .byte 48,251,97,19,189,24,27,177,255,254,208,128,249,233,185,216");
asm(" .byte 217,144,115,192,76,14,61,8,158,62,24,102,189,223,9,47");
asm(" .byte 125,104,48,114,110,199,139,73,244,27,66,166,239,53,233,125");
asm(" .byte 76,58,43,211,29,145,82,181,212,160,223,117,78,160,247,88");
asm(" .byte 171,28,244,46,241,76,58,199,210,243,37,251,208,165,184,42");
asm(" .byte 250,151,74,92,194,179,187,246,142,94,58,86,144,143,165,82");
asm(" .byte 188,126,232,210,182,120,191,191,224,236,76,239,6,55,239,100");
asm(" .byte 140,18,173,107,26,253,102,72,19,191,91,85,233,119,171,78");
asm(" .byte 122,207,142,126,211,182,123,234,123,231,98,99,7,129,23,98");
asm(" .byte 226,183,206,244,251,83,69,252,138,93,73,77,115,166,179,250");
asm(" .byte 165,21,181,243,243,54,229,231,228,117,230,140,255,51,169,172");
asm(" .byte 238,186,242,21,203,23,45,93,60,159,181,120,125,141,107,91");
asm(" .byte 61,157,90,65,149,182,116,121,213,202,58,173,172,188,124,97");
asm(" .byte 85,157,118,147,54,222,180,232,146,146,79,107,107,244,161,146");
asm(" .byte 249,154,188,236,43,213,101,90,85,205,194,154,21,43,235,150");
asm(" .byte 46,95,172,21,120,181,188,78,45,91,43,232,212,92,69,55");
asm(" .byte 20,234,248,207,165,21,20,52,123,219,59,124,90,94,179,86");
asm(" .byte 240,29,173,98,121,89,29,170,124,237,218,188,188,102,150,119");
asm(" .byte 195,166,121,90,94,225,117,157,201,172,98,105,109,121,126,89");
asm(" .byte 101,205,178,252,188,38,182,112,245,210,58,78,59,179,179,54");
asm(" .byte 143,175,211,71,83,181,182,105,91,52,207,250,14,143,87,203");
asm(" .byte 185,115,83,179,119,203,157,160,40,7,117,157,158,102,45,167");
asm(" .byte 115,190,246,109,237,142,235,230,107,243,215,83,93,211,38,244");
asm(" .byte 104,214,174,197,127,5,235,52,87,126,241,56,158,206,9,211");
asm(" .byte 231,205,159,159,163,221,172,177,89,190,141,94,74,121,250,220");
asm(" .byte 102,203,250,202,36,127,10,90,180,70,223,6,93,174,237,226");
asm(" .byte 98,58,249,98,106,86,84,177,242,219,230,204,153,51,183,62");
asm(" .byte 239,107,59,123,53,16,122,177,23,33,82,191,202,21,224,85");
asm(" .byte 65,107,251,250,2,111,135,103,93,203,102,13,75,213,114,88");
asm(" .byte 243,70,79,231,122,208,41,150,72,85,40,248,26,91,90,181");
asm(" .byte 2,215,69,74,243,58,11,49,36,43,104,108,109,101,77,155");
asm(" .byte 58,125,237,27,231,117,122,58,59,91,218,219,234,91,154,89");
asm(" .byte 50,122,205,7,117,172,209,235,109,109,233,100,222,142,246,117");
asm(" .byte 45,32,137,73,156,78,54,171,185,209,215,56,75,244,44,108");
asm(" .byte 106,111,91,215,178,190,19,131,206,18,96,97,94,97,73,103");
asm(" .byte 97,75,91,11,163,93,153,199,42,23,86,204,207,207,107,22");
asm(" .byte 255,179,133,181,117,85,148,151,45,173,89,64,121,249,178,178");
asm(" .byte 219,40,175,40,175,173,203,215,55,231,233,37,155,101,198,86");
asm(" .byte 45,40,171,163,166,250,101,101,139,89,190,158,175,179,170,101");
asm(" .byte 43,42,22,206,207,47,154,180,12,59,141,60,191,72,199,255");
asm(" .byte 186,158,63,27,255,176,242,186,154,74,54,151,21,196,215,196");
asm(" .byte 233,23,31,153,160,117,201,90,185,28,170,185,187,165,217,211");
asm(" .byte 62,175,169,189,217,211,84,191,206,219,201,138,230,200,42,254");
asm(" .byte 111,61,111,96,174,162,27,101,237,198,198,205,245,107,91,124");
asm(" .byte 29,141,62,15,115,205,193,132,162,90,86,213,55,249,58,90");
asm(" .byte 235,55,162,15,155,91,204,154,61,62,79,147,111,158,200,234");
asm(" .byte 125,247,122,61,108,54,125,57,195,215,209,222,58,79,230,245");
asm(" .byte 173,158,187,61,173,108,189,167,205,211,209,216,58,175,173,241");
asm(" .byte 110,226,107,125,179,103,99,251,87,42,219,189,62,206,122,125");
asm(" .byte 179,78,11,214,117,23,142,59,43,170,242,93,108,105,197,194");
asm(" .byte 85,96,13,151,253,202,178,229,21,2,170,89,88,190,130,180");
asm(" .byte 160,124,197,178,219,43,160,180,42,43,175,175,45,103,149,75");
asm(" .byte 151,223,150,95,224,98,10,103,22,223,22,240,77,40,246,184");
asm(" .byte 182,99,51,22,45,134,0,8,156,57,64,201,211,139,86,179");
asm(" .byte 154,133,139,208,99,19,171,42,95,86,145,215,57,31,115,95");
asm(" .byte 220,86,252,15,124,78,81,129,43,91,0,124,36,62,161,235");
asm(" .byte 82,182,110,104,108,107,195,186,93,179,117,182,169,211,211,177");
asm(" .byte 182,125,243,60,153,215,55,109,108,190,200,165,246,77,190,230");
asm(" .byte 246,246,142,139,229,117,173,45,235,55,248,234,239,105,241,109");
asm(" .byte 64,83,125,231,6,15,137,175,108,244,108,106,245,116,212,55");
asm(" .byte 182,173,199,206,98,159,190,194,234,187,255,238,107,171,239,109");
asm(" .byte 188,231,98,93,99,171,175,197,183,169,89,244,183,18,12,241");
asm(" .byte 217,212,185,150,45,170,171,89,186,108,62,163,127,191,76,83");
asm(" .byte 99,91,203,70,182,8,194,73,44,231,27,226,98,100,49,37");
asm(" .byte 251,69,21,113,51,159,184,202,217,29,23,71,57,19,27,231");
asm(" .byte 205,69,166,176,245,94,168,79,92,18,88,92,20,24,125,57");
asm(" .byte 165,85,240,88,103,186,46,197,97,238,186,181,122,211,156,27");
asm(" .byte 139,152,126,67,73,81,81,209,218,38,161,235,141,190,162,206");
asm(" .byte 246,66,216,9,182,96,197,10,97,16,73,167,102,109,242,98");
asm(" .byte 48,207,172,187,61,29,164,211,133,190,205,62,86,177,104,217");
asm(" .byte 237,68,224,172,102,207,221,179,124,190,123,255,51,75,38,13");
asm(" .byte 23,75,250,153,210,53,56,200,130,252,251,76,28,174,231,240");
asm(" .byte 45,234,120,253,225,167,9,254,9,63,85,239,224,240,147,28");
asm(" .byte 142,9,152,62,127,194,6,56,124,224,22,130,63,37,184,62");
asm(" .byte 200,113,126,32,96,142,83,252,203,216,86,170,220,133,124,112");
asm(" .byte 112,208,31,66,238,143,197,182,14,203,178,251,87,148,63,225");
asm(" .byte 175,28,18,101,31,242,86,180,135,145,63,105,251,153,191,238");
asm(" .byte 176,104,63,122,88,180,15,35,95,141,246,153,71,98,91,159");
asm(" .byte 64,123,51,242,193,39,191,225,191,112,68,224,185,143,82,254");
asm(" .byte 164,127,245,75,177,173,165,192,43,121,37,182,213,141,124,195");
asm(" .byte 176,24,47,229,215,162,222,254,26,225,61,229,223,114,12,56");
asm(" .byte 40,135,126,19,219,202,130,201,254,227,200,139,81,214,78,138");
asm(" .byte 250,253,103,69,191,253,239,139,117,244,126,44,250,85,94,16");
asm(" .byte 244,120,47,136,250,11,247,51,118,250,254,248,119,93,198,83");
asm(" .byte 97,252,219,95,50,15,200,239,99,29,216,41,190,225,181,231");
asm(" .byte 37,241,157,43,250,94,217,149,242,187,89,244,225,132,98,148");
asm(" .byte 167,91,190,61,182,161,75,124,115,43,119,155,248,118,214,254");
asm(" .byte 110,241,205,173,167,20,241,221,41,167,252,46,22,147,223,241");
asm(" .byte 42,93,174,240,111,212,104,138,248,22,88,170,252,22,216,174");
asm(" .byte 29,2,14,238,16,223,48,74,146,223,216,162,252,139,88,172");
asm(" .byte 61,23,245,49,228,52,6,98,212,246,55,182,179,255,246,95");
asm(" .byte 233,178,241,47,126,61,191,243,255,175,68,127,35,203,199,233");
asm(" .byte 115,207,188,250,27,26,215,55,77,47,44,42,210,138,116,87");
asm(" .byte 177,126,67,145,206,202,174,67,43,255,230,18,83,232,227,61");
asm(" .byte 142,57,117,11,89,130,61,73,153,160,164,217,51,148,169,202");
asm(" .byte 229,182,105,202,21,74,166,154,175,44,46,47,159,167,205,172");
asm(" .byte 109,223,212,209,228,233,184,87,91,124,253,245,90,101,139,207");
asm(" .byte 67,99,33,222,184,177,96,142,254,13,173,184,112,78,161,11");
asm(" .byte 226,208,185,129,62,176,212,184,150,193,217,250,60,29,94,86");
asm(" .byte 216,214,238,243,20,150,45,88,90,224,107,92,207,10,55,52");
asm(" .byte 118,110,96,133,205,247,182,117,222,187,81,228,190,14,86,184");
asm(" .byte 190,109,83,161,212,250,75,10,245,104,235,240,180,18,158,0");
asm(" .byte 188,173,62,26,185,5,255,250,60,48,15,133,235,200,167,23");
asm(" .byte 118,180,147,21,98,133,101,53,203,10,81,77,211,11,176,165");
asm(" .byte 121,179,192,175,111,236,232,104,188,87,224,199,225,239,52,117");
asm(" .byte 112,18,26,55,182,52,97,218,118,12,39,134,89,219,217,41");
asm(" .byte 6,160,15,51,181,172,221,228,67,160,129,152,98,227,70,79");
asm(" .byte 155,239,191,33,39,19,165,78,144,156,242,111,254,41,151,126");
asm(" .byte 239,45,190,75,233,242,59,119,132,199,191,205,167,140,127,187");
asm(" .byte 142,89,190,53,151,45,191,75,167,74,125,217,0,188,82,101");
asm(" .byte 188,61,174,75,215,202,185,85,169,71,251,1,248,85,209,87");
asm(" .byte 97,227,223,236,155,37,117,73,149,122,151,235,16,250,246,101");
asm(" .byte 250,110,96,66,103,8,143,244,229,13,40,104,149,133,46,85");
asm(" .byte 166,10,169,99,124,60,232,89,46,22,213,32,105,182,89,240");
asm(" .byte 107,229,247,254,84,169,167,193,68,161,159,214,117,80,121,141");
asm(" .byte 5,143,244,122,87,162,208,119,187,92,95,28,207,35,105,77");
asm(" .byte 144,118,231,64,146,176,65,95,230,95,131,5,143,235,11,240");
asm(" .byte 142,42,151,226,81,250,174,5,143,190,7,185,5,134,104,250");
asm(" .byte 77,95,29,239,46,11,30,217,185,61,11,5,61,95,198,187");
asm(" .byte 199,34,7,33,224,133,128,119,83,194,87,241,182,90,240,78");
asm(" .byte 191,134,116,219,165,223,108,139,227,221,47,240,188,221,76,126");
asm(" .byte 139,178,82,124,135,50,229,75,120,15,203,239,29,218,88,252");
asm(" .byte 155,145,227,109,86,188,199,228,119,21,109,210,46,23,255,7");
asm(" .byte 120,255,34,233,35,60,250,62,80,201,127,128,247,180,228,137");
asm(" .byte 141,197,191,69,57,254,29,76,235,254,254,194,50,30,217,243");
asm(" .byte 210,229,144,85,246,213,253,24,178,224,133,129,23,94,62,254");
asm(" .byte 61,72,235,188,47,203,113,185,172,173,80,152,178,2,251,59");
asm(" .byte 235,171,120,175,9,28,239,69,57,7,222,245,95,163,151,127");
asm(" .byte 144,176,46,203,249,192,187,238,75,120,138,133,182,248,223,42");
asm(" .byte 224,141,124,205,120,255,7,208,53,90,193,116,84,0,0,0");
asm("eod_flash_pgb:");
asm(" pop r25"); /* pull return address from stack */
asm(" pop r24");
asm(" clc"); /* multiply by 2 to get data address */
asm(" rol r24");
asm(" rol r25");
asm(" ret");
}

#endif


