

#ifndef ESP8266_PERI_H_INCLUDED
#define ESP8266_PERI_H_INCLUDED


#define ESP8266_REG(addr) *((volatile uint32_t *)(0x60000000+(addr)))
#define ESP8266_DREG(addr) *((volatile uint32_t *)(0x3FF00000+(addr)))
#define ESP8266_CLOCK 80000000UL


//SPI_READY
#define SPIRDY    ESP8266_DREG(0x0C)
#define SPI_BUSY   9 //wait SPI idle

//SPI0 Registers (SPI0 is used for the flash)
#define SPI0CMD		ESP8266_REG(0x200)
#define SPI0A 		ESP8266_REG(0x204)
#define SPI0C 		ESP8266_REG(0x208)
#define SPI0C1 		ESP8266_REG(0x20C)
#define SPI0RS 		ESP8266_REG(0x210)
#define SPI0C2 		ESP8266_REG(0x214)
#define SPI0CLK 	ESP8266_REG(0x218)
#define SPI0U 		ESP8266_REG(0x21C)
#define SPI0U1 		ESP8266_REG(0x220)
#define SPI0U2 		ESP8266_REG(0x224)
#define SPI0WS 		ESP8266_REG(0x228)
#define SPI0P 		ESP8266_REG(0x22C)
#define SPI0S 		ESP8266_REG(0x230)
#define SPI0S1 		ESP8266_REG(0x234)
#define SPI0S2 		ESP8266_REG(0x238)
#define SPI0S3 		ESP8266_REG(0x23C)
#define SPI0W0 		ESP8266_REG(0x240)
#define SPI0W1 		ESP8266_REG(0x244)
#define SPI0W2 		ESP8266_REG(0x248)
#define SPI0W3 		ESP8266_REG(0x24C)
#define SPI0W4 		ESP8266_REG(0x250)
#define SPI0W5 		ESP8266_REG(0x254)
#define SPI0W6 		ESP8266_REG(0x258)
#define SPI0W7 		ESP8266_REG(0x25C)
#define SPI0W8 		ESP8266_REG(0x260)
#define SPI0W9 		ESP8266_REG(0x264)
#define SPI0W10 	ESP8266_REG(0x268)
#define SPI0W11 	ESP8266_REG(0x26C)
#define SPI0W12 	ESP8266_REG(0x270)
#define SPI0W13 	ESP8266_REG(0x274)
#define SPI0W14 	ESP8266_REG(0x278)
#define SPI0W15 	ESP8266_REG(0x27C)
#define SPI0E3 		ESP8266_REG(0x2FC)
#define SPI0W(p)  ESP8266_REG(0x240 + ((p & 0xF) * 4))

//SPI1 Registers
#define SPI1CMD		ESP8266_REG(0x100)
#define SPI1A 		ESP8266_REG(0x104)
#define SPI1C 		ESP8266_REG(0x108)
#define SPI1C1 		ESP8266_REG(0x10C)
#define SPI1RS 		ESP8266_REG(0x110)
#define SPI1C2 		ESP8266_REG(0x114)
#define SPI1CLK 	ESP8266_REG(0x118)
#define SPI1U 		ESP8266_REG(0x11C)
#define SPI1U1 		ESP8266_REG(0x120)
#define SPI1U2 		ESP8266_REG(0x124)
#define SPI1WS 		ESP8266_REG(0x128)
#define SPI1P 		ESP8266_REG(0x12C)
#define SPI1S 		ESP8266_REG(0x130)
#define SPI1S1 		ESP8266_REG(0x134)
#define SPI1S2 		ESP8266_REG(0x138)
#define SPI1S3 		ESP8266_REG(0x13C)
#define SPI1W0 		ESP8266_REG(0x140)
#define SPI1W1 		ESP8266_REG(0x144)
#define SPI1W2 		ESP8266_REG(0x148)
#define SPI1W3 		ESP8266_REG(0x14C)
#define SPI1W4 		ESP8266_REG(0x150)
#define SPI1W5 		ESP8266_REG(0x154)
#define SPI1W6 		ESP8266_REG(0x158)
#define SPI1W7 		ESP8266_REG(0x15C)
#define SPI1W8 		ESP8266_REG(0x160)
#define SPI1W9 		ESP8266_REG(0x164)
#define SPI1W10 	ESP8266_REG(0x168)
#define SPI1W11 	ESP8266_REG(0x16C)
#define SPI1W12 	ESP8266_REG(0x170)
#define SPI1W13 	ESP8266_REG(0x174)
#define SPI1W14 	ESP8266_REG(0x178)
#define SPI1W15 	ESP8266_REG(0x17C)
#define SPI1E0 		ESP8266_REG(0x1F0)
#define SPI1E1 		ESP8266_REG(0x1F4)
#define SPI1E2 		ESP8266_REG(0x1F8)
#define SPI1E3 		ESP8266_REG(0x1FC)
#define SPI1W(p)  ESP8266_REG(0x140 + ((p & 0xF) * 4))

//SPI0, SPI1 & I2S Interupt Register
#define SPIIR     ESP8266_DREG(0x20)
#define SPII0     4 //SPI0 Interrupt
#define SPII1     7 //SPI1 Interrupt
#define SPII2     9 //I2S Interrupt

//SPI CMD
#define SPICMDREAD (1 << 31) //SPI_FLASH_READ
#define SPICMDWREN (1 << 30) //SPI_FLASH_WREN
#define SPICMDWRDI (1 << 29) //SPI_FLASH_WRDI
#define SPICMDRDID (1 << 28) //SPI_FLASH_RDID
#define SPICMDRDSR (1 << 27) //SPI_FLASH_RDSR
#define SPICMDWRSR (1 << 26) //SPI_FLASH_WRSR
#define SPICMDPP  (1 << 25) //SPI_FLASH_PP
#define SPICMDSE  (1 << 24) //SPI_FLASH_SE
#define SPICMDBE  (1 << 23) //SPI_FLASH_BE
#define SPICMDCE  (1 << 22) //SPI_FLASH_CE
#define SPICMDDP  (1 << 21) //SPI_FLASH_DP
#define SPICMDRES (1 << 20) //SPI_FLASH_RES
#define SPICMDHPM (1 << 19) //SPI_FLASH_HPM
#define SPICMDUSR (1 << 18) //SPI_FLASH_USR
#define SPIBUSY 	(1 << 18) //SPI_USR

//SPI CTRL (SPIxC)
#define SPICWBO       (1 << 26) //SPI_WR_BIT_ODER
#define SPICRBO       (1 << 25) //SPI_RD_BIT_ODER
#define SPICQIO       (1 << 24) //SPI_QIO_MODE
#define SPICDIO       (1 << 23) //SPI_DIO_MODE
#define SPIC2BSE      (1 << 22) //SPI_TWO_BYTE_STATUS_EN
#define SPICWPR       (1 << 21) //SPI_WP_REG
#define SPICQOUT      (1 << 20) //SPI_QOUT_MODE
#define SPICSHARE     (1 << 19) //SPI_SHARE_BUS
#define SPICHOLD      (1 << 18) //SPI_HOLD_MODE
#define SPICAHB       (1 << 17) //SPI_ENABLE_AHB
#define SPICSSTAAI    (1 << 16) //SPI_SST_AAI
#define SPICRESANDRES (1 << 15) //SPI_RESANDRES
#define SPICDOUT      (1 << 14) //SPI_DOUT_MODE
#define SPICFASTRD    (1 << 13) //SPI_FASTRD_MODE

//SPI CTRL1 (SPIxC1)
#define SPIC1TCSH   0xF //SPI_T_CSH
#define SPIC1TCSH_S 28 //SPI_T_CSH_S
#define SPIC1TRES   0xFFF //SPI_T_RES
#define SPIC1TRES_S 16 //SPI_T_RES_S
#define SPIC1BTL    0xFFFF //SPI_BUS_TIMER_LIMIT
#define SPIC1BTL_S  0 //SPI_BUS_TIMER_LIMIT_S

//SPI Status (SPIxRS)
#define SPIRSEXT    0xFF //SPI_STATUS_EXT
#define SPIRSEXT_S  24 //SPI_STATUS_EXT_S
#define SPIRSWB     0xFF //SPI_WB_MODE
#define SPIRSWB_S   16 //SPI_WB_MODE_S
#define SPIRSSP     (1 << 7) //SPI_FLASH_STATUS_PRO_FLAG
#define SPIRSTBP    (1 << 5) //SPI_FLASH_TOP_BOT_PRO_FLAG
#define SPIRSBP2    (1 << 4) //SPI_FLASH_BP2
#define SPIRSBP1    (1 << 3) //SPI_FLASH_BP1
#define SPIRSBP0    (1 << 2) //SPI_FLASH_BP0
#define SPIRSWRE    (1 << 1) //SPI_FLASH_WRENABLE_FLAG
#define SPIRSBUSY   (1 << 0) //SPI_FLASH_BUSY_FLAG

//SPI CTRL2 (SPIxC2)
#define SPIC2CSDN     0xF //SPI_CS_DELAY_NUM
#define SPIC2CSDN_S   28  //SPI_CS_DELAY_NUM_S
#define SPIC2CSDM     0x3 //SPI_CS_DELAY_MODE
#define SPIC2CSDM_S   26  //SPI_CS_DELAY_MODE_S
#define SPIC2MOSIDN   0x7 //SPI_MOSI_DELAY_NUM
#define SPIC2MOSIDN_S 23  //SPI_MOSI_DELAY_NUM_S
#define SPIC2MOSIDM   0x3 //SPI_MOSI_DELAY_MODE
#define SPIC2MOSIDM_S 21  //SPI_MOSI_DELAY_MODE_S
#define SPIC2MISODN   0x7 //SPI_MISO_DELAY_NUM
#define SPIC2MISODN_S 18  //SPI_MISO_DELAY_NUM_S
#define SPIC2MISODM   0x3 //SPI_MISO_DELAY_MODE
#define SPIC2MISODM_S 16  //SPI_MISO_DELAY_MODE_S
#define SPIC2CKOHM    0xF //SPI_CK_OUT_HIGH_MODE
#define SPIC2CKOHM_S  12  //SPI_CK_OUT_HIGH_MODE_S
#define SPIC2CKOLM    0xF //SPI_CK_OUT_LOW_MODE
#define SPIC2CKOLM_S  8   //SPI_CK_OUT_LOW_MODE_S
#define SPIC2HT       0xF //SPI_HOLD_TIME
#define SPIC2HT_S     4   //SPI_HOLD_TIME_S
#define SPIC2ST       0xF //SPI_SETUP_TIME
#define SPIC2ST_S     0   //SPI_SETUP_TIME_S

//SPI CLK (SPIxCLK)
#define SPICLK_EQU_SYSCLK (1 << 31) //SPI_CLK_EQU_SYSCLK
#define SPICLKDIVPRE 0x1FFF //SPI_CLKDIV_PRE
#define SPICLKDIVPRE_S 18 //SPI_CLKDIV_PRE_S
#define SPICLKCN 0x3F //SPI_CLKCNT_N
#define SPICLKCN_S 12 //SPI_CLKCNT_N_S
#define SPICLKCH 0x3F //SPI_CLKCNT_H
#define SPICLKCH_S 6 //SPI_CLKCNT_H_S
#define SPICLKCL 0x3F //SPI_CLKCNT_L
#define SPICLKCL_S 0 //SPI_CLKCNT_L_S

//SPI Phases (SPIxU)
#define SPIUCOMMAND	(1 << 31) //COMMAND pahse, SPI_USR_COMMAND
#define SPIUADDR	(1 << 30) //ADDRESS phase, SPI_FLASH_USR_ADDR
#define SPIUDUMMY	(1 << 29) //DUMMY phase, SPI_FLASH_USR_DUMMY
#define SPIUMISO	(1 << 28) //MISO phase, SPI_FLASH_USR_DIN
#define SPIUMOSI	(1 << 27) //MOSI phase, SPI_FLASH_DOUT
#define SPIUDUMMYIDLE (1 << 26) //SPI_USR_DUMMY_IDLE
#define SPIUMOSIH (1 << 25) //MOSI phase uses W8-W15, SPI_USR_DOUT_HIGHPART
#define SPIUMISOH (1 << 24) //MISO pahse uses W8-W15, SPI_USR_DIN_HIGHPART
#define SPIUPREPHOLD (1 << 23) //SPI_USR_PREP_HOLD
#define SPIUCMDHOLD (1 << 22) //SPI_USR_CMD_HOLD
#define SPIUADDRHOLD (1 << 21) //SPI_USR_ADDR_HOLD
#define SPIUDUMMYHOLD (1 << 20) //SPI_USR_DUMMY_HOLD
#define SPIUMISOHOLD (1 << 19) //SPI_USR_DIN_HOLD
#define SPIUMOSIHOLD (1 << 18) //SPI_USR_DOUT_HOLD
#define SPIUHOLDPOL (1 << 17) //SPI_USR_HOLD_POL
#define SPIUSIO (1 << 16) //SPI_SIO
#define SPIUFWQIO (1 << 15) //SPI_FWRITE_QIO
#define SPIUFWDIO (1 << 14) //SPI_FWRITE_DIO
#define SPIUFWQUAD (1 << 13) //SPI_FWRITE_QUAD
#define SPIUFWDUAL (1 << 12) //SPI_FWRITE_DUAL
#define SPIUWRBYO (1 << 11) //SPI_WR_BYTE_ORDER
#define SPIURDBYO (1 << 10) //SPI_RD_BYTE_ORDER
#define SPIUAHBEM 0x3 //SPI_AHB_ENDIAN_MODE
#define SPIUAHBEM_S 8 //SPI_AHB_ENDIAN_MODE_S
#define SPIUSME (1 << 7) //SPI Master Edge (0:falling, 1:rising), SPI_CK_OUT_EDGE
#define SPIUSSE   (1 << 6) //SPI Slave Edge (0:falling, 1:rising), SPI_CK_I_EDGE
#define SPIUCSSETUP (1 << 5) //SPI_CS_SETUP
#define SPIUCSHOLD (1 << 4) //SPI_CS_HOLD
#define SPIUAHBUCMD (1 << 3) //SPI_AHB_USR_COMMAND
#define SPIUAHBUCMD4B (1 << 1) //SPI_AHB_USR_COMMAND_4BYTE
#define SPIUDUPLEX (1 << 0) //SPI_DOUTDIN

//SPI Phase Length Locations
#define SPILCOMMAND	28 //4 bit in SPIxU2 default 7 (8bit)
#define SPILADDR	26 //6 bit in SPIxU1 default:23 (24bit)
#define SPILDUMMY	0  //8 bit in SPIxU1 default:0 (0 cycles)
#define SPILMISO	8  //9 bit in SPIxU1 default:0 (1bit)
#define SPILMOSI	17 //9 bit in SPIxU1 default:0 (1bit)
//SPI Phase Length Masks
#define SPIMCOMMAND	0xF
#define SPIMADDR	0x3F
#define SPIMDUMMY	0xFF
#define SPIMMISO	0x1FF
#define SPIMMOSI	0x1FF

//SPI Slave (SPIxS)
#define SPISSRES    (1 << 31) //SYNC RESET, SPI_SYNC_RESET
#define SPISE       (1 << 30) //Slave Enable, SPI_SLAVE_MODE
#define SPISBE      (1 << 29) //WR/RD BUF enable, SPI_SLV_WR_RD_BUF_EN
#define SPISSE      (1 << 28) //STA enable, SPI_SLV_WR_RD_STA_EN
#define SPISCD      (1 << 27) //CMD define, SPI_SLV_CMD_DEFINE
#define SPISTRCNT   0xF //SPI_TRANS_CNT
#define SPISTRCNT_S 23 //SPI_TRANS_CNT_S
#define SPISSLS     0x7 //SPI_SLV_LAST_STATE
#define SPISSLS_S   20 //SPI_SLV_LAST_STATE_S
#define SPISSLC     0x7 //SPI_SLV_LAST_COMMAND
#define SPISSLC_S   17 //SPI_SLV_LAST_COMMAND_S
#define SPISCSIM    0x3 //SPI_CS_I_MODE
#define SPIDCSIM_S  10 //SPI_CS_I_MODE_S
#define SPISTRIE    (1 << 9)  //TRANS interrupt enable
#define SPISWSIE    (1 << 8)  //WR_STA interrupt enable
#define SPISRSIE    (1 << 7)  //RD_STA interrupt enable
#define SPISWBIE    (1 << 6)  //WR_BUF interrupt enable
#define SPISRBIE    (1 << 5)  //RD_BUF interrupt enable
#define SPISTRIS    (1 << 4)  //TRANS interrupt status
#define SPISWSIS    (1 << 3)  //WR_STA interrupt status
#define SPISRSIS    (1 << 2)  //RD_STA interrupt status
#define SPISWBIS    (1 << 1)  //WR_BUF interrupt status
#define SPISRBIS    (1 << 0)  //RD_BUF interrupt status

//SPI Slave1 (SPIxS1)
#define SPIS1LSTA   27 //5 bit in SPIxS1 default:0 (1bit), SPI_SLV_STATUS_BITLEN
#define SPIS1FE     (1 << 26) //SPI_SLV_STATUS_FAST_EN
#define SPIS1RSTA   (1 << 25) //default:0 enable STA read from Master, SPI_SLV_STATUS_READBACK
#define SPIS1LBUF   16 //9 bit in SPIxS1 default:0 (1bit), SPI_SLV_BUF_BITLEN
#define SPIS1LRBA   10 //6 bit in SPIxS1 default:0 (1bit), SPI_SLV_RD_ADDR_BITLEN
#define SPIS1LWBA   4  //6 bit in SPIxS1 default:0 (1bit), SPI_SLV_WR_ADDR_BITLEN
#define SPIS1WSDE   (1 << 3) //SPI_SLV_WRSTA_DUMMY_EN
#define SPIS1RSDE   (1 << 2) //SPI_SLV_RDSTA_DUMMY_EN
#define SPIS1WBDE   (1 << 1) //SPI_SLV_WRBUF_DUMMY_EN
#define SPIS1RBDE   (1 << 0) //SPI_SLV_RDBUF_DUMMY_EN

//SPI Slave2 (SPIxS2)
#define SPIS2WBDL 0xFF //SPI_SLV_WRBUF_DUMMY_CYCLELEN
#define SPIS2WBDL_S 24 //SPI_SLV_WRBUF_DUMMY_CYCLELEN_S
#define SPIS2RBDL 0xFF //SPI_SLV_RDBUF_DUMMY_CYCLELEN
#define SPIS2RBDL_S 16 //SPI_SLV_RDBUF_DUMMY_CYCLELEN_S
#define SPIS2WSDL 0xFF //SPI_SLV_WRSTA_DUMMY_CYCLELEN
#define SPIS2WSDL_S 8 //SPI_SLV_WRSTA_DUMMY_CYCLELEN_S
#define SPIS2RSDL 0xFF //SPI_SLV_RDSTA_DUMMY_CYCLELEN
#define SPIS2RSDL_S 0 //SPI_SLV_RDSTA_DUMMY_CYCLELEN_S

//SPI Slave3 (SPIxS3)
#define SPIS3WSCV 0xFF //SPI_SLV_WRSTA_CMD_VALUE
#define SPIS3WSCV_S 24 //SPI_SLV_WRSTA_CMD_VALUE_S
#define SPIS3RSCV 0xFF //SPI_SLV_RDSTA_CMD_VALUE
#define SPIS3RSCV_S 16 //SPI_SLV_RDSTA_CMD_VALUE_S
#define SPIS3WBCV 0xFF //SPI_SLV_WRBUF_CMD_VALUE
#define SPIS3WBCV_S 8 //SPI_SLV_WRBUF_CMD_VALUE_S
#define SPIS3RBCV 0xFF //SPI_SLV_RDBUF_CMD_VALUE
#define SPIS3RBCV_S 0 //SPI_SLV_RDBUF_CMD_VALUE_S

//SPI EXT0 (SPIxE0)
#define SPIE0TPPEN (1 << 31) //SPI_T_PP_ENA
#define SPIE0TPPS 0xF //SPI_T_PP_SHIFT
#define SPIE0TPPS_S 16 //SPI_T_PP_SHIFT_S
#define SPIE0TPPT 0xFFF //SPI_T_PP_TIME
#define SPIE0TPPT_S 0 //SPI_T_PP_TIME_S

//SPI EXT1 (SPIxE1)
#define SPIE1TEREN (1 << 31) //SPI_T_ERASE_ENA
#define SPIE1TERS 0xF //SPI_T_ERASE_SHIFT
#define SPIE1TERS_S 16 //SPI_T_ERASE_SHIFT_S
#define SPIE1TERT 0xFFF //SPI_T_ERASE_TIME
#define SPIE1TERT_S 0 //SPI_T_ERASE_TIME_S

//SPI EXT2 (SPIxE2)
#define SPIE2ST 0x7 //SPI_ST
#define SPIE2ST_S 0 //SPI_ST_S

//SPI EXT3 (SPIxE3)
#define SPIE2IHEN 0x3 //SPI_INT_HOLD_ENA
#define SPIE2IHEN_S 0 //SPI_INT_HOLD_ENA_S

//SPI PIN (SPIxP)
#define SPIPCS2DIS (1 << 2)
#define SPIPCS1DIS (1 << 1)
#define SPIPCS0DIS (1 << 0)


#endif
