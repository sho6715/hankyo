// *************************************************************************
//   ���{�b�g��	�F �d���x�[�V�b�NDC�}�E�X�A�T���V���C��
//   �T�v		�F �T���V���C����HAL�i�n�[�h�E�G�A���ۑw�j�t�@�C��
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.06.28			sato			�V�K�i�t�@�C���̃C���N���[�h�j
// *************************************************************************/

//**************************************************
// �C���N���[�h�t�@�C���iinclude�j
//**************************************************
#include <typedefine.h>						// ��`
#include <iodefine.h>						// I/O
#include <hal.h>							// HAL
#include <stdio.h>
#include <math.h>
#include <init.h>
#include <parameters.h>
//#include <hal_dist.h>

//**************************************************
// ��`�idefine�j
//**************************************************
#define		BAT_GOOD			(3060)			// �c�ʌ����Ă����i���F�j�A1�Z��3.7V�ȏ�F 3060 = ( 3700mV * 1�Z�� ) / 1.5(����) / 3300 * 4096 - 1
#define		BAT_LOW				(2812)			// �c�ʂ�΂��I�I�i�ԐF�j�A1�Z��3.4V�ȏ�F 2812 = ( 3400mV * 1�Z�� ) / 1.5(����) / 3300 * 4096 - 1
#define		GYRO_REF_NUM		(200)		//�W���C���̃��t�@�����X�l���T���v�����O���鐔
#define		ENC_RESET_VAL		(32768)			// �G���R�[�_�̒��Ԓl
#define		DCM_R_IN			(PORT3.PODR.BIT.B1)		// DCM�EIN
#define		DCM_L_IN			(PORT1.PODR.BIT.B5)		// DCM��IN
#define		DCM_R_TIMER			(TPUA.TSTR.BIT.CST0)	// DCM�E�^�C�}�J�n
#define		DCM_L_TIMER			(TPUA.TSTR.BIT.CST1)	// DCM���^�C�}�J�n
#define		DCM_R_TIORB			(TPU0.TIORH.BIT.IOB)	// DCM�E�s���o�͐ݒ�B
#define		DCM_L_TIORB			(TPU1.TIOR.BIT.IOB)	// DCM���s���o�͐ݒ�B
#define		DCM_R_TCNT			(TPU0.TCNT)				// DCM�E�J�E���g�l
#define		DCM_L_TCNT			(TPU1.TCNT)				// DCM���J�E���g�l
#define		DCM_R_GRA			(TPU0.TGRA)				// DCM�E����
#define		DCM_R_GRB			(TPU0.TGRB)				// DCM�EDuty��
#define		DCM_L_GRA			(TPU1.TGRA)				// DCM������
#define		DCM_L_GRB			(TPU1.TGRB)				// DCM��Duty��

/* �񒲐��p�����[�^ */
#define PI							( 3.14159f )								// ��

/* �����p�����[�^ */
#define VCC_MAX						( 8.4f )									// �o�b�e���ő�d��[V]�A4.2[V]�~2[�Z��]
#define TIRE_R						( 22.25f )									// �^�C�����a [mm]
#define GEAR_RATIO					( 36 / 8 )									// �M�A��(�X�p�[/�s�j�I��)
//#define ROTATE_PULSE					( 2048 )									// 1���̃^�C���p���X��
#define DIST_1STEP					( PI * TIRE_R / GEAR_RATIO / ROTATE_PULSE )				// 1�p���X�Ői�ދ��� [mm]
#define F_CNT2MM(cnt)					( (FLOAT)cnt * DIST_1STEP )				// [�J�E���g�l]����[mm]�֊��Z
#define MOT_MOVE_ST_THRESHOLD				( 25 )							// ���i�ړ�������臒l[mm]
#define MOT_MOVE_ST_MIN					( 20 )							// ���i�ړ������̍Œ�ړ���[mm]
//#define MOT_ACC						( 1800 )						// ���i�ړ��̉����x[mm/s2]
//#define MOT_DEC						( 1800 )						// ���i�ړ��̌����x[mm/s2]
//#define MOT_ACC_ANGLE					( 1800 )		//����̊p�����x[mm/s2]
//#define MOT_DEC_ANGLE					( 1800 )		//����̊p�����x[mm/s2]

//20170815 ���M�n����������ɒǉ�
#define A1_MIN					( 25 )						// ��1�Œ�ړ��p�x
#define A2_MIN					( 30 )						// ��2�Œ�ړ��p�x
#define A3_MIN					( 20 )						// ��3�Œ�ړ��p�x

#define ANGLE_OFFSET1_R				( 0 )	//-12					// �p�x�̃I�t�Z�b�g�l�i�o�b�t�@�����O�ɂ��덷�𖄂߂邽�߂̒l�j
#define ANGLE_OFFSET1				( 0 )	//-12					// �p�x�̃I�t�Z�b�g�l�i�o�b�t�@�����O�ɂ��덷�𖄂߂邽�߂̒l�j
#define ANGLE_OFFSET2_R				( 0 )	//3
#define ANGLE_OFFSET2				( 0 )						// �p�x�̃I�t�Z�b�g�l�i�o�b�t�@�����O�ɂ��덷�𖄂߂邽�߂̒l�j
#define ANGLE_OFFSET3				( 0 )					// �p�x�̃I�t�Z�b�g�l�i�o�b�t�@�����O�ɂ��덷�𖄂߂邽�߂̒l�j

//#define MOT_MOVE_ST_THRESHOLD			( 25 )						// ���i�ړ�������臒l[mm]
//#define MOT_MOVE_ST_MIN				( 20 )						// ���i�ړ������̍Œ�ړ���[mm]


#define log_num			(1000)					//���O�擾���i�ύX���͂������ύX�j

//**************************************************
// �񋓑́ienum�j
//**************************************************
/* ���䓮��^�C�v */
typedef enum{
	CTRL_ACC,				// [00] ������(���i)
	CTRL_CONST,				// [01] ������(���i)
	CTRL_DEC,				// [02] ������(���i)

	CTRL_SKEW_ACC,			// [03] �΂߉�����(���i)
	CTRL_SKEW_CONST,		// [04] �΂ߓ�����(���i)
	CTRL_SKEW_DEC,			// [05] �΂ߌ�����(���i)
	
	CTRL_HIT_WALL,			// [06]�Ǔ��ē���
	
	CTRL_ACC_TRUN,			// [07] ������(���M�n����)
	CTRL_CONST_TRUN,		// [08] ������(���M�n����)
	CTRL_DEC_TRUN,			// [09] ������(���M�n����)
	
	CTRL_ENTRY_SURA,		// [10]�X�����[���O�O�i
	CTRL_ACC_SURA,			// [11] ������(�X��)
	CTRL_CONST_SURA,		// [12] ������(�X��)
	CTRL_DEC_SURA,			// [13] ������(�X��)
	CTRL_EXIT_SURA,			// [14] �X�����[����O�i

	CTRL_MAX,

}enCTRL_TYPE;


/* ����^�C�v */
typedef enum{
	MOT_ST_NC    =  0,
	MOT_ACC_CONST_DEC,			// [01] ��`����
	MOT_ACC_CONST_DEC_CUSTOM,	// [02] ��`�����i�����l�ύX�j
	MOT_ACC_CONST,				// [03] �����{����
	MOT_ACC_CONST_CUSTOM,		// [04] �����{�����i�����l�ύX�j
	MOT_CONST_DEC,				// [05] �����{����
	MOT_CONST_DEC_CUSTOM,		// [06] �����{�����i�����l�ύX�j
	MOT_ST_MAX,
}enMOT_ST_TYPE;

/* ���i�^�C�v */
typedef enum{
	MOT_GO_ST_NORMAL    =  0,	// �ʏ�̒��i
	MOT_GO_ST_SKEW,				// �΂߂̒��i
	MOT_GO_ST_MAX,
}enMOT_GO_ST_TYPE;


//**************************************************
// �\���́istruct�j
//**************************************************
/* ������ */
typedef struct{

	FLOAT			f_time;			// ����					[msec]

	/* ���x���� */
	FLOAT			f_acc1;			// �����x1				[mm/s2]
	FLOAT			f_acc3;			// �����x3				[mm/s2]
	FLOAT			f_now;			// ���ݑ��x				[mm/s]
	FLOAT			f_trgt;			// ������̖ڕW���x		[mm/s]
	FLOAT			f_last;			// ������̍ŏI���x		[mm/s]

	/* �������� */
	FLOAT			f_dist;			// �ړ�����				[mm]
	FLOAT			f_l1;			// ��1�ړ�����			[mm]
	FLOAT			f_l1_2;			// ��1+2�ړ�����		[mm]

	/* �p���x���� */
	FLOAT			f_accAngleS1;	// �p�����x1			[rad/s2]
	FLOAT			f_accAngleS3;	// �p�����x3			[rad/s2]
	FLOAT			f_nowAngleS;	// ���݊p���x			[rad/s]
	FLOAT			f_trgtAngleS;	// ������̖ڕW�p���x	[rad/s]
	FLOAT			f_lastAngleS;	// ������̍ŏI�p���x	[rad/s]

	/* �p�x���� */
	FLOAT			f_angle;		// �ړ��p�x				[rad]
	FLOAT			f_angle1;		// ��1�ړ��p�x			[rad]
	FLOAT			f_angle1_2;		// ��1+2�ړ��p�x		[rad]
}stMOT_DATA;

/* ����f�[�^ */
typedef struct{
	enCTRL_TYPE		en_type;		// ����^�C�v
	FLOAT			f_time;			// �ڕW���� [sec]
	FLOAT			f_acc;			// [���x����]   �����x[mm/s2]
	FLOAT			f_now;			// [���x����]   ���ݑ��x[mm/s]
	FLOAT			f_trgt;			// [���x����]   �ŏI���x[mm/s]
	FLOAT			f_nowDist;		// [��������]   ���݋���[mm]
	FLOAT			f_dist;			// [��������]   �ŏI����[mm]
	FLOAT			f_accAngleS;	// [�p���x����] �p�����x[rad/s2]
	FLOAT			f_nowAngleS;	// [�p���x����] ���݊p���x[rad/s]
	FLOAT			f_trgtAngleS;	// [�p���x����] �ŏI�p���x[rad/s]
	FLOAT			f_nowAngle;		// [�p�x����]   ���݊p�x[rad]
	FLOAT			f_angle;		// [�p������]   �ŏI�p�x[rad]
}stCTRL_DATA;


//**************************************************
// �O���[�o���ϐ�
//**************************************************
/* �o�b�e���Ď� */
PRIVATE USHORT	us_BatLvAve = 4095;							// �o�b�e�����ϒl�iAD�ϊ��̍ő�l�ŏ������j

/*�W���C���Z���T*/
PRIVATE SHORT s_GyroVal; 					  				// �W���C���Z���T�̌��ݒl
PRIVATE SHORT s_GyroValBuf[8];								// �W���C���Z���T�̃o�b�t�@�l
PUBLIC FLOAT  f_GyroNowAngle;		 						// �W���C���Z���T�̌��݊p�x
PRIVATE LONG  l_GyroRef; 									// �W���C���Z���T�̊�l

/* ����  */
PRIVATE enCTRL_TYPE		en_Type;						// �������
PRIVATE UCHAR 			uc_CtrlFlag			= FALSE;	// �t�B�[�h�o�b�N or �t�B�[�h�t�H���[�h ����L���t���O�iFALSE:�����A1�F�L���j
PRIVATE LONG			l_CntR;							// �E���[�^�̃J�E���g�ω���						�i1[msec]���ɍX�V�����j
PRIVATE LONG			l_CntL;							// �����[�^�̃J�E���g�ω���						�i1[msec]���ɍX�V�����j
// ����
PUBLIC  FLOAT			f_Time 				= 0;		// ���쎞��[sec]								�i1[msec]���ɍX�V�����j
PUBLIC  FLOAT			f_TrgtTime 			= 1000;		// ����ڕW���� [msec]							�i�ݒ�l�j
// ���x����//////////////////////////////////////////
PRIVATE FLOAT 			f_Acc			= 0;		// [���x����]   �����x							�i�ݒ�l�j
PRIVATE FLOAT			f_BaseSpeed		= 0;		// [���x����]   �����x							�i�ݒ�l�j
PRIVATE FLOAT			f_LastSpeed 		= 0;		// [���x����]   �ŏI�ڕW���x					�i�ݒ�l�j
PRIVATE FLOAT			f_NowSpeed		= 0;		// [���x����]   ���݂̑��x [mm/s]				�i1[msec]���ɍX�V�����j
PUBLIC FLOAT			f_TrgtSpeed 		= 0;		// [���x����]   �ڕW�ړ����x [mm/s]				�i1[msec]���ɍX�V�����j
PRIVATE FLOAT			f_ErrSpeedBuf		= 0;		// [���x����] �@���x�G���[�l�̃o�b�t�@	�i1[msec]���ɍX�V�����j
PUBLIC FLOAT			f_SpeedErrSum 		= 0;		// [���x����]   ���x�ϕ�����̃T���l			�i1[msec]���ɍX�V�����j
// ��������
PRIVATE FLOAT			f_BaseDist		= 0;		// [��������]   �����ʒu						�i�ݒ�l�j
PRIVATE FLOAT			f_LastDist 		= 0;		// [��������]   �ŏI�ړ�����					�i�ݒ�l�j
PUBLIC FLOAT			f_TrgtDist 		= 0;		// [��������]   �ڕW�ړ�����					�i1[msec]���ɍX�V�����j
PUBLIC volatile FLOAT 		f_NowDist		= 0;		// [��������]   ���݋���						�i1[msec]���ɍX�V�����j
PRIVATE FLOAT			f_NowDistR		= 0;		// [��������]   ���݋����i�E�j					�i1[msec]���ɍX�V�����j
PRIVATE FLOAT 			f_NowDistL		= 0;		// [��������]   ���݋����i���j					�i1[msec]���ɍX�V�����j
PUBLIC FLOAT			f_DistErrSum 		= 0;		// [��������]   �����ϕ�����̃T���l			�i1[msec]���ɍX�V�����j
// �p���x����
PRIVATE FLOAT 			f_AccAngleS		= 0;		// [�p���x����] �p�����x						�i�ݒ�l�j
PRIVATE FLOAT			f_BaseAngleS		= 0;		// [�p���x����] �����p���x						�i�ݒ�l�j
PRIVATE FLOAT			f_LastAngleS 		= 0;		// [�p���x����] �ŏI�ڕW�p���x					�i�ݒ�l�j
PUBLIC FLOAT			f_TrgtAngleS 		= 0;		// [�p���x����] �ڕW�p���x [rad/s]				�i1[msec]���ɍX�V�����j
PRIVATE FLOAT			f_ErrAngleSBuf		= 0;		// [�p���x����] �p���x�G���[�l�̃o�b�t�@	�i1[msec]���ɍX�V�����j
PUBLIC FLOAT			f_AngleSErrSum 		= 0;		// [�p���x����]   �p�x�ϕ�����̃T���l			�i1[msec]���ɍX�V�����j
// �p�x����
PRIVATE FLOAT			f_BaseAngle		= 0;		// [�p�x����]   �����p�x						�i�ݒ�l�j
PRIVATE FLOAT			f_LastAngle 		= 0;		// [�p�x����]   �ŏI�ڕW�p�x					�i�ݒ�l�j
PUBLIC volatile FLOAT 		f_NowAngle		= 0;		// [�p�x����]   ���݊p�x�@	volatile�����Ȃ���while���甲�����Ȃ��Ȃ�i�œK���̂����j�i1[msec]���ɍX�V�����j
PUBLIC FLOAT			f_TrgtAngle 		= 0;		// [�p�x����]   �ڕW�p�x						�i1[msec]���ɍX�V�����j
PUBLIC FLOAT			f_AngleErrSum 		= 0;		// [�p�x����]   �p�x�ϕ�����̃T���l			�i1[msec]���ɍX�V�����j
// �ǐ���
PRIVATE LONG 			l_WallErr 		= 0;		// [�ǐ���]     �ǂƂ̕΍�						�i1[msec]���ɍX�V�����j
PRIVATE FLOAT			f_ErrDistBuf		= 0;		// [�ǐ���]     �����Z���T�[�G���[�l�̃o�b�t�@	�i1[msec]���ɍX�V�����j

/* ���� */
PRIVATE FLOAT 			f_MotNowSpeed 		= 0.0f;		// ���ݑ��x
PRIVATE FLOAT 			f_MotTrgtSpeed 		= 0.0f;		// �ڕW���x
PRIVATE	stMOT_DATA 		st_Info;				// �V�[�P���X�f�[�^
PRIVATE FLOAT			f_MotSuraStaSpeed	= 0.0f;
PRIVATE FLOAT			sliplengs		= 0.0f;		//�X���b�v����

PRIVATE enMOT_WALL_EDGE_TYPE	en_WallEdge = MOT_WALL_EDGE_NONE;	// �ǐ؂�␳
PRIVATE BOOL			bl_IsWallEdge = FALSE;				// �ǐ؂ꌟ�m�iTRUE:���m�AFALSE�F�񌟒m�j
PRIVATE FLOAT			f_WallEdgeAddDist = 0;				// �ǐ؂�␳�̈ړ�����

//�t�F�C���Z�[�t
PUBLIC FLOAT  f_ErrChkAngle; 			  // �W���C���Z���T�̃G���[���o�p�̊p�x
PUBLIC BOOL   bl_ErrChk; 				  // �W���C���Z���T�̃G���[���o�iFALSE�F���m���Ȃ��ATRUE�F���m����j
PRIVATE BOOL			bl_failsafe		= FALSE;	// �}�E�X���̐���s�\�iTRUE�F����s�\�AFALSE�F����\�j

//���O�v���O�����Q�i�擾���ύX��define�ցj
PRIVATE	FLOAT	Log_1[log_num];
PRIVATE FLOAT	Log_2[log_num];
PRIVATE FLOAT	Log_3[log_num];
PRIVATE FLOAT	Log_4[log_num];
PRIVATE FLOAT	Log_5[log_num];
PRIVATE FLOAT	Log_6[log_num];
PRIVATE FLOAT	Log_7[log_num];
PRIVATE FLOAT	Log_8[log_num];
PRIVATE FLOAT	Log_9[log_num];
PRIVATE FLOAT	Log_10[log_num];
PRIVATE FLOAT	Log_11[log_num];
PRIVATE FLOAT	Log_12[log_num];

PRIVATE	USHORT	log_count = 0;
PUBLIC	BOOL	b_logflag = FALSE;

PRIVATE FLOAT	templog1	= 0;
PRIVATE FLOAT	templog2	= 0;

//���O�p�f���[�e�B�[
PRIVATE	FLOAT	f_Duty_R;
PRIVATE	FLOAT	f_Duty_L;

PRIVATE CHAR	i;

//**************************************************
// �v���g�^�C�v�錾�i�t�@�C�����ŕK�v�Ȃ��̂����L�q�j
//**************************************************


// *************************************************************************
//   �@�\		�F HAL������������B
//   ����		�F �Ȃ�
//   ����		�F �����ϐ��Ȃǂ��N���A����B
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
//   ���̑�     �F�N�����ɓ���
// **************************    ��    ��    *******************************
// 		v1.0		2013.11.27			�O��			�V�K
// 		v1.1		2013.11.27			�O��			�Z���T�̊�l�ƃ��~�b�g�l��ݒ�
// *************************************************************************/
PUBLIC void HAL_init( void )
{
	/* �W���C���Z���T */
	f_GyroNowAngle = 0;			// �W���C���Z���T�̌��݊p�x(0�ɂ��Ă��T�����͓������A����Ƃ�testrun�Ƃ��͓����Ȃ�)�C���ς݂Ǝv����
	l_GyroRef  = 0;				// �W���C���Z���T�̊�l
	
	f_ErrChkAngle = 0;
	bl_ErrChk = FALSE;
	
	/* �G���R�[�_ */
//	ENC_Sta();	//�����̃s����LEFT(�ˋN�͑O������)
	
	/* DCM*/
//	DCM_ENA = ON;
	
	/* [�b��] warning�����������i�폜����OK�j */
	f_AccAngleS = f_AccAngleS;
	f_BaseAngleS = f_BaseAngleS;
	f_LastAngleS = f_LastAngleS;
	f_BaseAngle = f_BaseAngle;
	f_LastAngle = f_LastAngle;

}

// *************************************************************************
//   �@�\		�F 1�����o��
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.11.13			�O��			�V�K
// *************************************************************************/
PUBLIC void SCI_putch(char data)
{
	while(SCI1.SSR.BIT.TEND == 0) {};
//		if (SCI1.SSR.BYTE & 0x80) {			// ���M�o�b�t�@�̋󂫃`�F�b�N
			SCI1.TDR = data;
//			SCI1.SSR.BYTE = SCI1.SSR.BYTE & 0x40;
			SCI1.SSR.BIT.TEND = 0;
//			break;
//		}
//	}
}


// *************************************************************************
//   �@�\		�F ������o��
//   ����		�F �Ȃ�
//   ����		�F "\n"�݂̂�"CR+LF"�o�͂��s���B
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.11.13			�O��			�V�K
// *************************************************************************/
PUBLIC void SCI_puts(char *buffer)
{
	char data;
	
	/* null�܂ŏo�� */
	while( (data = *( buffer++ ) ) != 0 ){
		
		/* �f�[�^�̒l�ɉ����ďo�͂�ς��� */
		if (data == 0x0a) {
			SCI_putch(0x0d);		// CR�o��
			SCI_putch(0x0a);		// LF�o��
		} else {
			SCI_putch(data);		// 1�����o��
		}
	}
}


// *************************************************************************
//   �@�\		�F ������o��
//   ����		�F �Ȃ�
//   ����		�F �����񒷂��w��t��"/\n"�̂� "CR+LF"�o�͂��s���B
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.11.13			�O��			�V�K
// *************************************************************************/
PUBLIC void SCI_putsL(char *buffer, int len)
{
	int i;
	char data;
	
	for( i=0; i<len; i++ ){
		data=*(buffer++);
		
		/* �f�[�^�̒l�ɉ����ďo�͂�ς��� */
		if (data == 0x0a) {
			SCI_putch(0x0d);		// CR�o��
			SCI_putch(0x0a);		// LF�o��
		} else {
			SCI_putch(data);		// 1�����o��
		}
	}
}


// *************************************************************************
//   �@�\		�F 1�����o�͗p���b�p�[�֐�
//   ����		�F �Ȃ�
//   ����		�F printf�Ȃǂ̒჌�x���o�͂Ɏg�p�����
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.11.13			�O��			�V�K
// *************************************************************************/
PUBLIC void charput(unsigned char data)
{
	SCI_putch(data);
}


// *************************************************************************
//   �@�\		�F ���̓o�b�t�@�`�F�b�N
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.11.13			�O��			�V�K
// *************************************************************************/
PUBLIC int SCI_chkRecv(void)
{
	/* �f�[�^�̎�M�`�F�b�N */
	if (IR(SCI1,RXI1) == 1) {
		return 1;		// ��M�f�[�^����
	}
	else {
		return 0;		// ��M�f�[�^�Ȃ�
	}
}


// *************************************************************************
//   �@�\		�F 1�������́ichar�^�Łj
//   ����		�F �Ȃ�
//   ����		�F �G�R�[�o�b�N�Ȃ��B
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.11.13			�O��			�V�K
// *************************************************************************/
PUBLIC char SCI_getch(void)
{
	char data;
	
	while(1){
		/* �f�[�^�̎�M�`�F�b�N */
		if ( SCI_chkRecv() ) {
			data = SCI1.RDR;						// �f�[�^��M
//			SCI1.SSR.BYTE = SCI1.SSR.BYTE & 0x80;
			IR(SCI1,RXI1) = 0;
			break;
		}
	}
	return data;
}


// *************************************************************************
//   �@�\		�F 1�������́iunsigned char�^�Łj
//   ����		�F �Ȃ�
//   ����		�F �G�R�[�o�b�N�Ȃ��B
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.11.13			�O��			�V�K
// *************************************************************************/
PUBLIC unsigned char SCI_getch_uc(void)
{
	unsigned char data;
	while(1){
		/* �f�[�^�̎�M�`�F�b�N */
		if ( SCI_chkRecv() ) {
			data = SCI1.RDR;						// �f�[�^��M
//			SCI1.SSR.BYTE = SCI1.SSR.BYTE & 0x80;
			IR(SCI1,RXI1) = 0;
			break;
		}
	}
	return data;
}


// *************************************************************************
//   �@�\		�F ���������
//   ����		�F �Ȃ�
//   ����		�F CR�R�[�h�܂�/�ő�255����/�G�R�[�o�b�N����
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.11.13			�O��			�V�K
// *************************************************************************/
PUBLIC int SCI_gets(char *buffer)
{
	char data;
	int  i = 0;

	while(1){
		data = SCI_getch();		// 1��������
		*buffer = data;
		SCI_putch(data);		// 1�����o��(�G�R�[�o�b�N)
		buffer++;
		i++;
		if (i > 255)      break;	// �ő啶�����ɓ��B
		if (data == 0x0D) break;	// CR�R�[�h�܂Ŏ�M����
	}
	*buffer = (unsigned char)0;		// null
	
	return i;						// ���͕�������ԋp
}


// *************************************************************************
//   �@�\		�F 1�������͗p���b�p�[�֐�
//   ����		�F �Ȃ�
//   ����		�F scanf�Ȃǂ̒჌�x���o�͂Ɏg�p�����
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.11.13			�O��			�V�K
// *************************************************************************/
PUBLIC unsigned char charget(void)
{
	return SCI_getch_uc();
}


// *************************************************************************
//   �@�\		�F �o�b�e���d�����擾����
//   ����		�F �Ȃ�
//   ����		�F ���O5��̕��ϒl
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �d��[mV]
// **************************    ��    ��    *******************************
// 		v1.0		2013.11.13			�O��			�V�K
// *************************************************************************/
PUBLIC FLOAT BAT_getLv(void)
{
	FLOAT f_val = (FLOAT)( us_BatLvAve + 1 );		// �l��0����n�܂邩��1�����Z
	
	return ( f_val / 4096 * 3300 / 2 * 3 );
}


// *************************************************************************
//   �@�\		�F �o�b�e���Ď��p�|�[�����O�֐�
//   ����		�F �Ȃ�
//   ����		�F ���荞�݂�����s�����
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.11.16			�O��			�V�K
// *************************************************************************/
PUBLIC void BAT_Pol( void )
{
	static USHORT 	us_batLv[5] = { 4095, 4095, 4095, 4095, 4095 };		// �o�b�e�����x���iAD�ϊ��̍ő�l�ŏ������j

	/* ================================================== */
	/*  ���ϒl���擾���邽�߁A�f�[�^�̃o�b�t�@�������s��  */
	/* ================================================== */
	/* �o�b�t�@���V�t�g */
	us_batLv[4] = us_batLv[3];
	us_batLv[3] = us_batLv[2];
	us_batLv[2] = us_batLv[1];
	us_batLv[1] = us_batLv[0];

	/* �ŐV�̒l���i�[ */
	S12AD.ADANS0.WORD 		= 0x0001;		// AN0 �ϊ��Ώېݒ�
	S12AD.ADCSR.BIT.ADST 		= 1;		// AD�ϊ��J�n
	while( S12AD.ADCSR.BIT.ADST == 1);		// AD�ϊ��҂�
	us_batLv[0] = S12AD.ADDR0;				// AN0 �ϊ��f�[�^�擾

	/* �d�����ω� */
	us_BatLvAve = ( us_batLv[0] + us_batLv[1] + us_batLv[2] + us_batLv[3] + us_batLv[4] ) / 5;
	
	/*  �c�ʂɉ�����LED��\��  */
	/* ======================= */
	if( us_BatLvAve < BAT_LOW ) {			// �c�ʂ�΂��I�I�i�ԐF�j
		LEDG = OFF;
//		LEDR = ON;
	}
	else if( us_BatLvAve < BAT_GOOD ) {		// �c�ʌ����Ă����i���F�j
//		LEDG = ON;
//		LEDR = ON;
		
		if( i==50 ){
			LEDG = ~LEDG;	//L�`�J
			i=0;
		}
		else{
			i++;
		}
		
	}
	else{									// �c�ʖ��Ȃ��i�ΐF�j
		LEDG = ON;
//		LEDR = OFF;
	}
}

// *************************************************************************
//   �@�\		�F �W���C���Z���T�̃��t�@�����X�l�i��̒l�j��ݒ肷��
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
//   ���̑��@�@�F�@�N�����ɓ���
// **************************    ��    ��    *******************************
// 		v1.0		2018.08.07			sato			�V�K
//		v1.1		2018.08.25			sato			Ref�̎擾�����m�ɂł��Ă��Ȃ��悤�Ɍ��������ߋ�����Ref��ݒ肵�Ă���i�ً}�[�u�j
// *************************************************************************/
PUBLIC void GYRO_SetRef( void )
{
	USHORT i;
	ULONG ul_ref = 0;
	
	/* �f�[�^�T���v�����O */
	for( i=0; i<GYRO_REF_NUM; i++){			// 100��T���v�����O�������ϒl����̒l�Ƃ���B
		ul_ref += (ULONG)s_GyroVal;
		TIME_wait(1);
	}
	
	/* ��l�Z�o�i���ϒl�j */
	l_GyroRef = ul_ref / GYRO_REF_NUM * 100;		// ���x��100�{�ɂ���
//	l_GyroRef = 0x1304*100;
}

// *************************************************************************
//   �@�\		�F �W���C���̊p���x�Ɋւ��鐧��΍����擾����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
//
// **************************    ��    ��    *******************************
// 		v1.0		2013.11.26		�O��			�V�K
// *************************************************************************/
PUBLIC FLOAT GYRO_getSpeedErr( void )
{
	LONG  l_val = (LONG)s_GyroVal * 100 ;				// 100�{�̐��x�ɂ���
	LONG  l_err = l_val - l_GyroRef ;
	FLOAT f_res;
	
	/* �p���x�̕΍��Z�o */
	if( ( l_err < -20 * 100 ) || ( 20 * 100 < l_err ) ){
		f_res = (FLOAT)l_err /32.768 / 100;		//32.768 = 2^16(16bit)/2000(+-1000�x) LSB/(��/s)
													// 100�{�̐��x
	}
	else{
		f_res = 0;									// [deg/s]
	}
	
	return f_res;
}

// *************************************************************************
//   �@�\		�F �W���C���̌��݂̊p�x���擾����
//   ����		�F �Ȃ�
//   ����		�F ��
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.11.26			�O��			�V�K
// *************************************************************************/
PUBLIC FLOAT GYRO_getNowAngle( void )
{
	return f_GyroNowAngle;
}

// *************************************************************************
//   �@�\		�F �W���C���̌��݂̊p�x���擾����
//   ����		�F �Ȃ�
//   ����		�F ��
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.11.26			�O��			�V�K
// *************************************************************************/
PUBLIC FLOAT GYRO_getRef( void )
{
	return l_GyroRef;
}

// *************************************************************************
//   �@�\		�F �W���C���Z���T�p�|�[�����O�֐�
//   ����		�F �Ȃ�
//   ����		�F ���荞�݂�����s�����
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.11.26			�O��			�V�K
//		v2.0		2018.08.16			sato		SPI�ɂ��W���C���擾�ݒ�
// *************************************************************************/
PUBLIC void GYRO_Pol( void )
{
	FLOAT f_speed;
	
	/* �o�b�t�@�V�t�g�i[7]�ɐV�����f�[�^�����邽�߁A[0]�̃f�[�^���̂ĂāA1���l�߂�j */
/*	s_GyroValBuf[0]	= s_GyroValBuf[1];
	s_GyroValBuf[1]	= s_GyroValBuf[2];
	s_GyroValBuf[2]	= s_GyroValBuf[3];
	s_GyroValBuf[3]	= s_GyroValBuf[4];
	s_GyroValBuf[4]	= s_GyroValBuf[5];
	s_GyroValBuf[5]	= s_GyroValBuf[6];
	s_GyroValBuf[6]	= s_GyroValBuf[7];
*/	
	/* �ŐV�̃W���C���Z���T�l���擾 */
//	s_GyroValBuf[7] = (SHORT)recv_spi_gyro();
	
	/* �W���C���̒l�𕽊�����i��������8�j */
	s_GyroVal = (SHORT)recv_spi_gyro();//( s_GyroValBuf[0] + s_GyroValBuf[1] + s_GyroValBuf[2] + s_GyroValBuf[3] +
				//  s_GyroValBuf[4] + s_GyroValBuf[5] + s_GyroValBuf[6] + s_GyroValBuf[7] ) / 8;
	
	/* ���݂̊p�x���X�V���� */
	f_speed = GYRO_getSpeedErr();			// �p���x�擾 (0.001sec���̊p���x)
	f_GyroNowAngle += f_speed / 1000;		// �p�x�ݒ�   (0.001sec���ɉ��Z���邽��)

	/* �G���[�`�F�b�N */
	if( bl_ErrChk == TRUE ){
		
		f_ErrChkAngle += f_speed/1000;		// �p�x�ݒ�   (0.001sec���ɉ��Z���邽��)
		
		if( ( f_ErrChkAngle < -500 ) || ( 500 < f_ErrChkAngle )||(f_speed <-1000)||(1000<f_speed) ){
			
//			Failsafe_flag();
//			printf("fail\n\r");
		}
	}
}

// *************************************************************************
//   �@�\		�F �G���[���o�p���J�n����
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.10.10			sato			�V�K
// *************************************************************************/
PUBLIC void GYRO_staErrChkAngle( void )
{
	f_ErrChkAngle = 0;
	bl_ErrChk = TRUE;

}


// *************************************************************************
//   �@�\		�F �G���[���o�p���I������
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.10.10			sato			�V�K
// *************************************************************************/
PUBLIC void GYRO_endErrChkAngle( void )
{
	f_ErrChkAngle = 0;
	bl_ErrChk = FALSE;

}

// *************************************************************************
//   �@�\		�F SPI�֐�
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.08.06			sato			�V�K
// *************************************************************************/
PUBLIC USHORT recv_spi(USHORT spi_ad)
{
	USHORT recv;
	RSPI0.SPDR.WORD.H = spi_ad;
	
	while(!RSPI0.SPSR.BIT.IDLNF);	//���M�J�n���m�F
	while(RSPI0.SPSR.BIT.IDLNF);		//RSPI0����ُ�Ԃ��m�F
		recv = RSPI0.SPDR.WORD.H ;

	return(recv);
}

// *************************************************************************
//   �@�\		�F SPI(whoami)
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.08.04			�O��			�V�K
// *************************************************************************/
PUBLIC USHORT recv_spi_who(void)
{
	USHORT recv;
	USHORT whoami = (0x75|0x80);
	PORTC.PODR.BIT.B4 = 0;
	recv = recv_spi(whoami);
	recv = recv_spi(0x00);
	PORTC.PODR.BIT.B4 = 1;
	return(recv);
		
}

// *************************************************************************
//   �@�\		�F SPI_init
//   ����		�F �Ȃ�
//   ����		�F ������s
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.08.05			sato		�V�K
// *************************************************************************/
PUBLIC void recv_spi_init(void)
{
	USHORT recv;
	USHORT register107 = (0x6B|0x00);		//power management1
	USHORT register106 = (0x6A|0x00);
	USHORT register112 = (0x70|0x00);
	USHORT register27 = (0x1B|0x00);
	
	PORTC.PODR.BIT.B4 = 0;
	recv = recv_spi(register107);
	recv = recv_spi(0x80);
	PORTC.PODR.BIT.B4 = 1;
	TIME_wait(100);
	
	PORTC.PODR.BIT.B4 = 0;
	recv = recv_spi(register106);
	recv = recv_spi(0x01);
	PORTC.PODR.BIT.B4 = 1;
	TIME_wait(100);
	
	PORTC.PODR.BIT.B4 = 0;
	recv = recv_spi(register112);
	recv = recv_spi(0x40);
	PORTC.PODR.BIT.B4 = 1;
	TIME_wait(1);
	
	PORTC.PODR.BIT.B4 = 0;
	recv = recv_spi(register27);
	recv = recv_spi(0x30);
	PORTC.PODR.BIT.B4 = 1;
	TIME_wait(1);
	
	PORTC.PODR.BIT.B4 = 0;
	recv = recv_spi(register107);
	recv = recv_spi(0x00);
	PORTC.PODR.BIT.B4 = 1;
	TIME_wait(1);
	
	/*read*/
/*
	PORTC.PODR.BIT.B4 = 0;
	recv = recv_spi(register27|0x80);
	recv = recv_spi(0x00);
	PORTC.PODR.BIT.B4 = 1;
	
	return(recv);
*/
}

// *************************************************************************
//   �@�\		�F SPI_gyro_read
//   ����		�F �Ȃ�
//   ����		�F ������s
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.08.05			sato		�V�K
// *************************************************************************/
PUBLIC USHORT recv_spi_gyro(void)
{
	USHORT recv = 0;
	USHORT recv1;
	USHORT recv2;
	USHORT gyro_H = (0x47|0x80);	//register71
	USHORT gyro_L = (0x48|0x80);	//register72
	
	PORTC.PODR.BIT.B4 = 0;
	TIME_waitFree(50);
	recv1 = recv_spi(gyro_H);
	recv1 = recv_spi(0x00);
	PORTC.PODR.BIT.B4 = 1;
	
	PORTC.PODR.BIT.B4 = 0;
	TIME_waitFree(50);
	recv2 = recv_spi(gyro_L);
	recv2 = recv_spi(0x00);
	PORTC.PODR.BIT.B4 = 1;
	
	RSPI0.SPSR.BYTE = 0xA0;
	
	recv = (recv1<<8)+(recv2&0xFF);
	
	return(recv);
		
}

// *************************************************************************
//   �@�\		�F SPI_gyro_read
//   ����		�F �Ȃ�
//   ����		�F ������s
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.08.25			sato		�V�K
// *************************************************************************/
PUBLIC USHORT recv_spi_gyrooffset(void)
{
	USHORT recv = 0;
	USHORT recv1;
	USHORT recv2;
	USHORT gyro_H = (0x23|0x80);	//register23
	USHORT gyro_L = (0x24|0x80);	//register24
	
	PORTC.PODR.BIT.B4 = 0;
	TIME_waitFree(50);
	recv1 = recv_spi(gyro_H);
	recv1 = recv_spi(0x00);
	PORTC.PODR.BIT.B4 = 1;
	
	PORTC.PODR.BIT.B4 = 0;
	TIME_waitFree(50);
	recv2 = recv_spi(gyro_L);
	recv2 = recv_spi(0x00);
	PORTC.PODR.BIT.B4 = 1;
	
	RSPI0.SPSR.BYTE = 0xA0;
	
	recv = (recv1<<8)+(recv2&0xFF);
	
	return(recv);
		
}


// *************************************************************************
//   �@�\		�F DCM�̉�]������CW�i���v���j�ɂ���
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F ���[�^ID
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.08.19			sato	��]�����͌�Ń`�F�b�N	�V�K
// *************************************************************************/
PUBLIC void DCM_setDirCw( enDCM_ID en_id )
{
	/* ��]�����ݒ� */
	if( en_id == DCM_R ){			// �E
		DCM_R_IN = OFF;				// BIN1
	}
	else{							// ��
		DCM_L_IN = ON;			// AIN1

	}
}


// *************************************************************************
//   �@�\		�F DCM�̉�]������CCW�i�����v���j�ɂ���
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F ���[�^ID
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.03			�O��			�V�K
// *************************************************************************/
PUBLIC void DCM_setDirCcw( enDCM_ID en_id )
{
	/* ��]�����ݒ� */
	if( en_id == DCM_R ){			// �E
		DCM_R_IN = ON;			// BIN1
	}
	else{							// ��
		DCM_L_IN = OFF;				// AIN1
	}
}


// *************************************************************************
//   �@�\		�F DCM���~����
//   ����		�F �Ȃ�
//   ����		�F PWM��HI�o�͒��ɖ{�֐������s����ƁA�s����100%�o�͏�ԂȂ邽�߁A�֐����Ńs�����N���A�iLo�j����B
//   ����		�F ���[�^ID
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.03			�O��			�V�K
// *************************************************************************/
PUBLIC void DCM_stopMot( enDCM_ID en_id )
{
	/* ��~�ݒ� */
	if( en_id == DCM_R ){			// �E
		DCM_R_IN = OFF;			// BIN1
		DCM_R_TIMER = OFF;			// �^�C�}��~
		DCM_R_TIORB = 1;			// TIOCA �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 0 �o��
	}
	else{							// ��
		DCM_L_IN = OFF;			// AIN1
		DCM_L_TIMER = OFF;			// �^�C�}��~
		DCM_L_TIORB = 1;			// TIOCA �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 0 �o��
	}
}


// *************************************************************************
//   �@�\		�F DCM���u���[�L���O����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F ���[�^ID
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.03			�O��			�V�K
// *************************************************************************/
PUBLIC void DCM_brakeMot( enDCM_ID en_id )
{
	/* ��~�ݒ� */
	if( en_id == DCM_R ){			// �E
		DCM_R_IN = ON;				// BIN1
		DCM_R_TIMER = OFF;			// �^�C�}��~
		DCM_R_TIORB = 1;			// TIOCB �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 0 �o��
	}
	else{							// ��
		DCM_L_IN = ON;				// AIN1
		DCM_L_TIMER = OFF;			// �^�C�}��~
	    	DCM_L_TIORB = 1;			// TIOCB �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 0 �o��
	}
}


// *************************************************************************
//   �@�\		�F DCM�𓮍�J�n����
//   ����		�F �Ȃ�
//   ����		�F ����J�n�O��PWM�Ɖ�]�������w�肵�Ă�������
//   ����		�F ���[�^ID
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.03			�O��			�V�K
// *************************************************************************/
PUBLIC void DCM_staMot( enDCM_ID en_id )
{
	/* �^�C�}�X�^�[�g */
	if( en_id == DCM_R ){			// �E
		DCM_R_TIORB = 2;			// TIOCB �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 1 �o��
		DCM_R_TIMER = ON;			// �^�C�}�J�n
	}
	else{							// ��
	    	DCM_L_TIORB = 2;			// TIOCB �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 1 �o��
		DCM_L_TIMER = ON;			// �^�C�}�J�n
	}
}


// *************************************************************************
//   �@�\		�F �SDCM�𓮍�J�n����
//   ����		�F �Ȃ�
//   ����		�F ����J�n�O��PWM�Ɖ�]�������w�肵�Ă�������
//   ����		�F ���[�^ID
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.03			�O��			�V�K
// *************************************************************************/
PUBLIC void DCM_staMotAll( void )
{
	DCM_staMot(DCM_R);									// �E���[�^ON
	DCM_staMot(DCM_L);									// �����[�^ON
}


// *************************************************************************
//   �@�\		�F DCM��PWM-Duty��ݒ肷��
//   ����		�F ���荞�݊O����ݒ肷��ƁA�_�u���o�b�t�@�łȂ��Ǝ������ɂȂ�ꍇ������B
//   ����		�F ���荞�݃n���h��������s���邱�ƁBDuty0%�̏ꍇ���[�^���~������iPWM�ɂЂ����o��j
//   ����		�F ���[�^ID�A�H�H�H�H�H
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.03			�O��			�V�K
// *************************************************************************/
PUBLIC void DCM_setPwmDuty( enDCM_ID en_id, USHORT us_duty10 )
{
	USHORT	us_cycle;							// ����
	USHORT	us_onReg;							// �ݒ肷��ON-duty
	
	/* PWM�ݒ� */
	if( en_id == DCM_R ){				// �E
	
		if( 0 == us_duty10 ){			// Duty0%�ݒ�
			DCM_brakeMot( en_id );
		}
		else if( 1000 <= us_duty10 ){	// Duty100%

			DCM_R_TIMER = OFF;			// �^�C�}��~
			DCM_R_TCNT = 0;				// TCNT �J�E���^���N���A
			DCM_R_GRB = 5000;			// �^�C�}�l�ύX
		    	DCM_R_TIORB = 6;			// TIOCB �[�q�̋@�\ : �����o�͂� 1 �o�́B�R���y�A�}�b�`�� 1 �o��
			DCM_R_TIMER = ON;			// �^�C�}�J�n
			us_duty10 = 1000;
		}
		else{
			us_cycle = DCM_R_GRA;		// ����
			us_onReg = (USHORT)( (ULONG)us_cycle * (ULONG)us_duty10 / (ULONG)1000 );	// Duty2Reg �v�Z��
			DCM_R_TIMER = OFF;			// �^�C�}��~
			DCM_R_TCNT = 0;				// TCNT �J�E���^���N���A
			DCM_R_GRB = us_onReg;		// onDuty
			DCM_staMot( en_id );		// ��]�J�n
			printf("%f\r\n",us_cycle);
		}
	}
	else{								// ��

		if( 0 == us_duty10 ){			// Duty0%
			DCM_brakeMot( en_id );
		}
		else if( 1000 <= us_duty10 ){	// Duty100%

			DCM_L_TIMER = OFF;			// �^�C�}��~
			DCM_L_TCNT = 0;				// TCNT �J�E���^���N���A
			DCM_L_GRB = 5000;			// �^�C�}�l�ύX
		    	DCM_L_TIORB = 6;			// TIOCB �[�q�̋@�\ : �����o�͂� 1 �o�́B�R���y�A�}�b�`�� 1 �o��
			DCM_L_TIMER = ON;			// �^�C�}�J�n
			us_duty10 = 1000;
		}
		else{
			us_cycle = DCM_L_GRA;		// ����
			us_onReg = (USHORT)( (ULONG)us_cycle * (ULONG)us_duty10 / (ULONG)1000 );	// Duty2Reg �v�Z��
			DCM_L_TIMER = OFF;			// �^�C�}��~
			DCM_L_TCNT = 0;				// TCNT �J�E���^���N���A
			DCM_L_GRB = us_onReg;		// �^�C�}�l�ύX
			DCM_staMot( en_id );		// ��]�J�n
			printf("%f\r\n",us_cycle);
		}
	}
}

// *************************************************************************
//   �@�\		�F ������J�n����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.14			�O��			�V�K
// *************************************************************************/
PUBLIC void CTRL_sta( void )
{
	uc_CtrlFlag = TRUE;
}

// *************************************************************************
//   �@�\		�F ������~����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.14			�O��			�V�K
// *************************************************************************/
PUBLIC void CTRL_stop( void )
{
	uc_CtrlFlag = FALSE;
	DCM_brakeMot( DCM_R );		// �u���[�L
	DCM_brakeMot( DCM_L );		// �u���[�L
}
