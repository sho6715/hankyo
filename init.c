// *************************************************************************
//   ���{�b�g��		�F half
//   �T�v		�F init�t�@�C���i���W�X�^�ݒ�t�@�C�����j
//   ����		�F �Ȃ�
//   ����		�F WORD = BYTE��2 LONG ��BYTE���S
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.06.30			sato			�V�K�i�t�@�C���̃C���N���[�h�j
// *************************************************************************/

//**************************************************
// �C���N���[�h�t�@�C���iinclude�j
//**************************************************
#include <typedefine.h>						// ��`
#include <iodefine.h>						// I/O
#include <init.h>							//���W�X�^


//**************************************************
// ��`�idefine�j
//**************************************************




//**************************************************
// �񋓑́ienum�j
//**************************************************


//**************************************************
// �\���́istruct�j
//**************************************************


//**************************************************
// �O���[�o���ϐ�
//**************************************************




//**************************************************
// �v���g�^�C�v�錾�i�t�@�C�����ŕK�v�Ȃ��̂����L�q�j
//**************************************************

// *************************************************************************
//   �@�\		�F �N���b�N�̃��W�X�^�ݒ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.06.30			sato			�V�K
// *************************************************************************/
PRIVATE void init_clock(void)
{


	SYSTEM.PRCR.WORD = 0xa50b;		//�N���b�N�\�[�X�I���̕ی�̉���

	SYSTEM.PLLCR.WORD = 0x0F00;		/* PLL ���{�~16 ����1���� (12.000MHz * 16 = 192MHz)*/
	SYSTEM.PLLCR2.BYTE = 0x00;		/* PLL ENABLE */
	
	SYSTEM.PLLWTCR.BYTE     = 0x0F;		/* 4194304cycle(Default) */

	
	// ICK   : 192/2 = 96MHz 		// �V�X�e���N���b�N CPU DMAC DTC ROM RAM
	// PCLKA : 192/2 = 96MHz	 	// ���Ӄ��W���[���N���b�NA ETHERC�AEDMAC�ADEU
	// PCLKB : 192/4 = 48MHz 		// ���Ӄ��W���[���N���b�NB ��L�ȊO PCLKB=PCLK
/*	
	SYSTEM.SCKCR.BIT.FCK=0x02;		//FCLK MAX 50MHz  192/4
	SYSTEM.SCKCR.BIT.ICK=0x01;		//ICLK MAX 100MHz 192/2
	SYSTEM.SCKCR.BIT.PSTOP1=0x01;		//BCLK �o�͒�~
	SYSTEM.SCKCR.BIT.PSTOP0=0x01;		//SDCLK �o�͒�~
	SYSTEM.SCKCR.BIT.BCK=0x02;		//BCLK MAX 100MHz ICLK�ȉ��ɂ���K�v������192/4
	SYSTEM.SCKCR.BIT.PCKA=0x01;		//PCLKA MAX 100MHz 192/2
	SYSTEM.SCKCR.BIT.PCKB=0x02;		//PCLKB MAX 50MHz 192/4
	//��L�̐ݒ�ł͐�����clock�ݒ肪�ł��Ȃ����߉��L�̂悤�Ɉꊇ�Őݒ肷�邱��
*/
	SYSTEM.SCKCR.LONG = 0x21C21211;		//FCK1/4 ICK1/2 BCLK��~ SDCLK��~ BCK1/4 PCLKA1/2 PCLKB1/4
/*
	SYSTEM.SCKCR2.BIT.UCK=0x03;		//UCLK MAX 48MHz 192/4
	SYSTEM.SCKCR2.BIT.IEBCK=0x02;		//IECLK MAX 50MHz 192/4
*/
	SYSTEM.SCKCR2.WORD = 0x0032;		/* UCLK1/4 IEBCK1/4 */
	SYSTEM.BCKCR.BYTE = 0x01;		/* BCLK = 1/2 */
	
	SYSTEM.SCKCR3.WORD = 0x0400;		//PLL��H�I��

}

// *************************************************************************
//   �@�\		�F IO�̃��W�X�^�ݒ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.06.30			sato			�V�K
// *************************************************************************/
PRIVATE void init_io(void)
{
	/* ================== */
	/*  GPIO(�ėp���o��)  */
	/* ================== */
	// �o�͒l 
	PORTA.PODR.BIT.B1			= 0;		//�|�[�gA-1�̏����o��0[V]�i�f�o�b�O�pLED�j
	PORTA.PODR.BIT.B3			= 0;		//�|�[�gA-3�̏����o��0[V]�i�f�o�b�O�pLED�j
	PORTA.PODR.BIT.B4			= 0;		//�|�[�gA-4�̏����o��0[V]�i�f�o�b�O�pLED�j
	PORTA.PODR.BIT.B6			= 0;		//�|�[�gA-6�̏����o��0[V]�i�f�o�b�O�pLED�j
	
	PORT3.PODR.BIT.B7			= 0;		//�|�[�g3-7�̏����o��0[V]�i�d���p��LED�j
//	PORT1.PODR.BIT.B4			= 0;		//�|�[�g1-4�̏����o��0[V]�i�d���p��LED�j
	//�Z���T
	PORTB.PODR.BIT.B0			= 0;		//�|�[�gB-0�̏����o��0[V]�i�Z���TLED�j
	PORTB.PODR.BIT.B1			= 0;		//�|�[�gB-1�̏����o��0[V]�i�Z���TLED�j
	PORTB.PODR.BIT.B3			= 0;		//�|�[�gB-3�̏����o��0[V]�i�Z���TLED�j
	PORTB.PODR.BIT.B5			= 0;		//�|�[�gB-5�̏����o��0[V]�i�Z���TLED�j
	//DCM
	PORT1.PODR.BIT.B7			= 0;		//PWM(TPU0)
	PORT1.PODR.BIT.B6			= 0;		//PWM(TPU3)
	PORT3.PODR.BIT.B1			= 0;		//AIN1
	PORT1.PODR.BIT.B5			= 0;		//AIN2
//	PORTB.PODR.BIT.B5			= 0;		//BIN1
//	PORTB.PODR.BIT.B6			= 0;		//BIN2
//	PORTB.PODR.BIT.B7			= 0;		//STBY
	
	// ���o�͐ݒ� 
	PORTA.PDR.BIT.B1			= 1;		//�|�[�gA-1���o�͂ɐݒ�
	PORTA.PDR.BIT.B3			= 1;		//�|�[�gA-3���o�͂ɐݒ�
	PORTA.PDR.BIT.B4			= 1;		//�|�[�gA-4���o�͂ɐݒ�
	PORTA.PDR.BIT.B6			= 1;		//�|�[�gA-6���o�͂ɐݒ�
	
	PORT3.PDR.BIT.B7			= 1;		//�|�[�g3-7���o�͂ɐݒ�
//	PORT1.PDR.BIT.B4			= 1;		//�|�[�g1-4���o�͂ɐݒ�
	//�Z���T
	PORTB.PDR.BIT.B0			= 1;		//�|�[�gB-0���o�͂ɐݒ�
	PORTB.PDR.BIT.B1			= 1;		//�|�[�gB-1���o�͂ɐݒ�
	PORTB.PDR.BIT.B3			= 1;		//�|�[�gB-3���o�͂ɐݒ�
	PORTB.PDR.BIT.B5			= 1;		//�|�[�gB-5���o�͂ɐݒ�
	//DCM
	PORT1.PDR.BIT.B7			= 1;		//PWM(TPU0)
	PORT1.PDR.BIT.B6			= 1;		//PWM(TPU3)
	PORT3.PDR.BIT.B1			= 1;		//AIN1
	PORT1.PDR.BIT.B5			= 1;		//AIN2
//	PORTB.PDR.BIT.B5			= 1;		//BIN1
//	PORTB.PDR.BIT.B6			= 1;		//BIN2
//	PORTB.PDR.BIT.B7			= 1;		//STBY
	
	PORT3.PDR.BIT.B1			= 1;		//AIN2
	// ���̓v���A�b�v�ݒ� 
	PORT2.PCR.BIT.B7			= 1;		// �|�[�g2-7�̓v���A�b�v���g�p	(�v�b�V���X�C�b�`�p)
	PORT4.PCR.BIT.B6			= 1;		// �|�[�g4-6�̓v���A�b�v���g�p	(�v�b�V���X�C�b�`�p)
	
	// �|�[�g���̓f�[�^���W�X�^ 
	PORT2.PIDR.BIT.B7			= 1;		// �|�[�g2-7����̓|�[�g�Ɛڑ�����
	PORT4.PIDR.BIT.B6			= 1;		// �|�[�g4-6����̓|�[�g�Ɛڑ�����
	
	
	/* ========= */
	/*  A/D�ϊ�  */
	/* ========= */
	SYSTEM.PRCR.WORD = 0xA502;
	SYSTEM.MSTPCRA.BIT.MSTPA17 	= 0;		// AD(12bit)�X�^���o�C����
	SYSTEM.PRCR.WORD = 0xA500;
	
	S12AD.ADCSR.BIT.CKS 		= 3;		// PCLK�ŕϊ�
	
	
	//TPUPWM�p
	MPC.PWPR.BIT.B0WI  = 0;
	MPC.PWPR.BIT.PFSWE = 1;
	MPC.P17PFS.BIT.PSEL		= 3;	//TIOCB0
	MPC.P16PFS.BIT.PSEL		= 3;	//TIOCB1
	MPC.PWPR.BIT.B0WI  = 1;
	MPC.PWPR.BIT.PFSWE = 0;
	PORT1.PMR.BIT.B7		= 1;
	PORT1.PMR.BIT.B6		= 1;
	
	
}

// *************************************************************************
//   �@�\		�F MTU�̃��W�X�^�ݒ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.06.30			sato			�V�K
// *************************************************************************/
PRIVATE void init_mtu(void)
{
	SYSTEM.PRCR.WORD = 0xA502;
	MSTP(MTU0) 	= 0;						// 
	MSTP(MTU1) 	= 0;
	MSTP(MTU3) 	= 0;
	MSTP(MTU2)	= 0;
	SYSTEM.PRCR.WORD = 0xA500;
	
	MTU.TSTR.BYTE = 0;						//�^�C�}����X�g�b�v
	
	// -----------------------
	//  �V�X�e���p(MTU0)
	// -----------------------
	// �^�C�}���荞�݂̐ݒ� 
	MTU0.TCR.BIT.CCLR 			= 1;		// TGRA�̃R���y�A�}�b�`��TCNT�N���A
	MTU0.TCR.BIT.TPSC 			= 2;		// PCLK(48MHz)/16 ��1�J�E���g
	MTU0.TIER.BIT.TGIEA			= 1;		// TGRA�Ƃ̃R���y�A�}�b�`�Ŋ��荞�݋���
	MTU0.TGRA 					= 750 * 4;	// 1msec���Ɋ��荞��
	MTU0.TCNT 					= 0;		// �^�C�}�N���A
	
	IEN(MTU0,TGIA0) = 1;	//���荞�ݗv�������� 
	IPR(MTU0,TGIA0) = 7;	//���荞�ݗD��x�����_�ɐݒ�
	IR(MTU0,TGIA0)	= 0;	//���荞�݃X�e�[�^�X�t���O���N���A
	
	MTU.TSTR.BIT.CST0 = 0;	//�^�C�}�X�g�b�v
	
	// -----------------------
	//  �o�b�e���[�p(MTU1)
	// -----------------------
	// �^�C�}���荞�݂̐ݒ� 
	MTU1.TCR.BIT.CCLR 			= 1;		// TGRA�̃R���y�A�}�b�`��TCNT�N���A
	MTU1.TCR.BIT.TPSC 			= 2;		// PCLK(48MHz)/16 ��1�J�E���g
	MTU1.TIER.BIT.TGIEA			= 1;		// TGRA�Ƃ̃R���y�A�}�b�`�Ŋ��荞�݋���
	MTU1.TGRA 					= 7500 * 4;	// 10msec���Ɋ��荞��
	MTU1.TCNT 					= 0;		// �^�C�}�N���A
	
	IEN(MTU1,TGIA1) = 1;	//���荞�ݗv�������� 
	IPR(MTU1,TGIA1) = 6;	//���荞�ݗD��x�����_�ɐݒ�
	IR(MTU1,TGIA1)	= 0;	//���荞�݃X�e�[�^�X�t���O���N���A
	
	MTU.TSTR.BIT.CST1 = 0;	//�^�C�}�X�g�b�v
	
	// -----------------------
	//  �Z���T�p(MTU3)
	// -----------------------
	// �^�C�}���荞�݂̐ݒ� 
	MTU3.TCR.BIT.CCLR 			= 1;		// TGRA�̃R���y�A�}�b�`��TCNT�N���A
	MTU3.TCR.BIT.TPSC 			= 2;		// PCLK(48MHz)/16 ��1�J�E���g
	MTU3.TIER.BIT.TGIEA			= 1;		// TGRA�Ƃ̃R���y�A�}�b�`�Ŋ��荞�݋���
	MTU3.TGRA 					= 750;	// 250usec���Ɋ��荞��
	MTU3.TCNT 					= 0;		// �^�C�}�N���A
	
	IEN(MTU3,TGIA3) = 1;	//���荞�ݗv�������� 
	IPR(MTU3,TGIA3) = 8;	//���荞�ݗD��x�����_�ɐݒ�
	IR(MTU3,TGIA3)	= 0;	//���荞�݃X�e�[�^�X�t���O���N���A
	
	MTU.TSTR.BIT.CST3 = 0;	//�^�C�}�X�g�b�v
	
}

// *************************************************************************
//   �@�\		�F TPU�̃��W�X�^�ݒ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.07.03			sato			�V�K
// *************************************************************************/
PRIVATE void init_tpu(void)
{
	SYSTEM.PRCR.WORD 	= 0xA502;
	MSTP(TPU0) 			= 0;
	MSTP(TPU1) 			= 0;
	SYSTEM.PRCR.WORD 	= 0xA500;
	
	// -----------------------
	//  �E���[�^PWM�o�͗p(TPU0)
	// -----------------------
	TPU0.TCR.BIT.CCLR 			= 1;		// TGRA�̃R���y�A�}�b�`��TCNT�N���A
	TPU0.TCR.BIT.CKEG			= 1;		// CKEG�����オ��G�b�W�ŃJ�E���g
	TPU0.TCR.BIT.TPSC 			= 1;		// PCLK(48MHz)/4 ��1�J�E���g
	TPU0.TMDR.BIT.MD			= 3;		// PWM ���[�h 2
//	TPU0.TIORH.BIT.IOA			= 2;		// TIOCA �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 1 �o��
	TPU0.TIORH.BIT.IOB			= 2;		// TIOCB �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 1 �o��
	TPU0.TGRA 				= 240;		// ����(usec)
	TPU0.TGRB 				= 120;		// onDuty
	TPU0.TCNT 				= 0;		// �^�C�}�N���A
	
	// -----------------------
	//  �����[�^PWM�o�͗p(TPU1)
	// -----------------------
	TPU1.TCR.BIT.CCLR 			= 1;		// TGRA�̃R���y�A�}�b�`��TCNT�N���A
	TPU1.TCR.BIT.CKEG			= 1;		// CKEG�����オ��G�b�W�ŃJ�E���g
	TPU1.TCR.BIT.TPSC 			= 1;		// PCLK(48MHz)/4 ��1�J�E���g
	TPU1.TMDR.BIT.MD			= 3;		// PWM ���[�h 2
//	TPU1.TIORH.BIT.IOA			= 2;		// TIOCA �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 1 �o��
	TPU1.TIOR.BIT.IOB			= 2;		// TIOCB �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 1 �o��
	TPU1.TGRA 				= 240;		// ����(usec)
	TPU1.TGRB 				= 120;		// onDuty
	TPU1.TCNT 				= 0;		// �^�C�}�N���A
	
}

// *************************************************************************
//   �@�\		�F SCI�̃��W�X�^�ݒ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.07.03			sato			�V�K
// *************************************************************************/
PRIVATE void init_sci(void)
{	
	SYSTEM.PRCR.WORD 	= 0xA502;
	MSTP(SCI1) 			= 0;
	SYSTEM.PRCR.WORD 	= 0xA500;
	SCI1.SCR.BYTE	= 0x00;
	while(0x00 != (SCI1.SCR.BYTE & 0xF0));
	SCI1.SMR.BYTE	= 0x00;
//	PORT2.PODR.BIT.B6 = 1;			//TXD��Dirction�̐؂�ւ���̒l��high
//	PORT2.PDR.BIT.B6 = 1;			//�o�͂ɐݒ�
//	PORT3.PDR.BIT.B0 = 0;			//���͂ɐݒ�
//	PORT2.PMR.BIT.B6 = 0;			//�ėp�|�[�g�ɐݒ�
//	PORT3.PMR.BIT.B0 = 0;			//�ėp�|�[�g�ɐݒ�
	MPC.PWPR.BIT.B0WI  = 0;
	MPC.PWPR.BIT.PFSWE = 1;
	MPC.P26PFS.BIT.PSEL = 0x0A;		//TXD1
	MPC.P30PFS.BIT.PSEL = 0x0A;		//RXD1
	MPC.PWPR.BIT.PFSWE = 0;
	MPC.PWPR.BIT.B0WI  = 1;
	PORT3.PMR.BIT.B0 = 1;			//���Ӌ@�\(RXD1)�Ƃ��Ďg�p
	PORT2.PMR.BIT.B6 = 1;			//���Ӌ@�\(TXD1)�Ƃ��Ďg�p
	SCI1.BRR	= 78;
	SCI1.SEMR.BIT.ABCS = 1;
	SCI1.SCR.BYTE	=0x30;

	IEN(SCI1,RXI1) 		= 1;
	IEN(SCI1,TXI1) 		= 1;
	ICU.IPR[217].BIT.IPR	= 5;

}

// *************************************************************************
//   �@�\		�F SPI�ݒ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.06.30			sato			�V�K
// *************************************************************************/
PUBLIC void init_spi(void)
{
	SYSTEM.PRCR.WORD 	= 0xA502;
	MSTP(RSPI0) 		= 0;
	SYSTEM.PRCR.WORD 	= 0xA500;
	
	RSPI0.SPCR.BYTE = 0x00;
	
//	PORTC.PMR.BIT.B4	= 1;
	PORTC.PMR.BIT.B5	= 1;
	PORTC.PMR.BIT.B6	= 1;
	PORTC.PMR.BIT.B7	= 1;
	MPC.PWPR.BIT.B0WI = 0;
	MPC.PWPR.BIT.PFSWE = 1;
//	MPC.PC4PFS.BYTE = 0x0D;
	MPC.PC5PFS.BYTE = 0x0D;  /* RSPI RSPCKA */       
	MPC.PC6PFS.BYTE = 0x0D;  /* RSPI MOSIA  */
	MPC.PC7PFS.BYTE = 0x0D;  /* RSPI MISOA  */
	MPC.PWPR.BIT.PFSWE = 0;
	MPC.PWPR.BIT.B0WI = 1;
	PORTC.PODR.BIT.B4 = 1;
	PORTC.PDR.BIT.B4 = 1;
//�G���R�[�_�p
	PORT4.PODR.BIT.B2 = 1;
	PORT4.PDR.BIT.B2 = 1;
	PORT4.PODR.BIT.B1 = 1;
	PORT4.PDR.BIT.B1 = 1;
	
	RSPI0.SPBR		= 2;
	RSPI0.SPCMD0.WORD	= 0x0f83;	
	
	RSPI0.SPCR.BYTE = 0xF8;

}

// *************************************************************************
//   �@�\		�F ���W�X�^�ݒ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.06.30			sato			�V�K
// *************************************************************************/
PUBLIC void CPU_init(void)
{
	init_clock();
	init_io();
	init_mtu();
	init_tpu();
	init_sci();
	init_spi();
}
